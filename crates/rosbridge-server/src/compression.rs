//! Outgoing message compression: CBOR, CBOR-raw, and PNG, matching rosbridge's
//! subscribe-time encodings.

use base64::Engine;
use serde_json::{Map, Value};

/// The selected compression scheme for a subscription. Precedence when several
/// clients share a topic is `cbor-raw > cbor > png > none`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Compression {
    #[default]
    None,
    Png,
    Cbor,
    CborRaw,
}

impl Compression {
    pub fn parse(s: &str) -> Compression {
        match s {
            "png" => Compression::Png,
            "cbor" => Compression::Cbor,
            "cbor-raw" => Compression::CborRaw,
            _ => Compression::None,
        }
    }

    /// Precedence rank for multi-client coalescing (higher wins).
    pub fn rank(self) -> u8 {
        match self {
            Compression::None => 0,
            Compression::Png => 1,
            Compression::Cbor => 2,
            Compression::CborRaw => 3,
        }
    }
}

/// Serialize a JSON value to a CBOR byte buffer.
pub fn to_cbor(value: &Value) -> Vec<u8> {
    let mut buf = Vec::with_capacity(256);
    // serde_json::Value implements Serialize; ciborium maps it to CBOR.
    ciborium::into_writer(value, &mut buf).expect("cbor serialization of JSON value cannot fail");
    buf
}

/// Build a `cbor-raw` publish frame: `{op, topic, msg:{secs,nsecs,bytes}}`
/// CBOR-encoded, carrying the untouched raw CDR payload.
pub fn cbor_raw_frame(topic: &str, raw_cdr: &[u8], secs: i64, nsecs: u32) -> Vec<u8> {
    let mut msg = Map::new();
    msg.insert("secs".into(), Value::from(secs));
    msg.insert("nsecs".into(), Value::from(nsecs));
    // CBOR byte string is produced via serde_bytes-style encoding; ciborium
    // encodes a Vec<u8> as an array of ints, so we encode the envelope
    // manually to emit a CBOR byte string for `bytes`.
    let mut buf = Vec::with_capacity(raw_cdr.len() + 64);
    write_cbor_raw_envelope(&mut buf, topic, raw_cdr, secs, nsecs);
    buf
}

/// Manually emit `{"op":"publish","topic":..,"msg":{"secs":..,"nsecs":..,"bytes":<bstr>}}`
/// as CBOR, using a CBOR byte string for the raw payload.
fn write_cbor_raw_envelope(out: &mut Vec<u8>, topic: &str, raw: &[u8], secs: i64, nsecs: u32) {
    // map(3): op, topic, msg
    out.push(0xA3);
    cbor_text(out, "op");
    cbor_text(out, "publish");
    cbor_text(out, "topic");
    cbor_text(out, topic);
    cbor_text(out, "msg");
    // map(3): secs, nsecs, bytes
    out.push(0xA3);
    cbor_text(out, "secs");
    cbor_int(out, secs);
    cbor_text(out, "nsecs");
    cbor_int(out, nsecs as i64);
    cbor_text(out, "bytes");
    cbor_bytes(out, raw);
}

fn cbor_head(out: &mut Vec<u8>, major: u8, val: u64) {
    let m = major << 5;
    if val < 24 {
        out.push(m | val as u8);
    } else if val < 0x100 {
        out.push(m | 24);
        out.push(val as u8);
    } else if val < 0x1_0000 {
        out.push(m | 25);
        out.extend_from_slice(&(val as u16).to_be_bytes());
    } else if val < 0x1_0000_0000 {
        out.push(m | 26);
        out.extend_from_slice(&(val as u32).to_be_bytes());
    } else {
        out.push(m | 27);
        out.extend_from_slice(&val.to_be_bytes());
    }
}

fn cbor_text(out: &mut Vec<u8>, s: &str) {
    cbor_head(out, 3, s.len() as u64);
    out.extend_from_slice(s.as_bytes());
}

fn cbor_bytes(out: &mut Vec<u8>, b: &[u8]) {
    cbor_head(out, 2, b.len() as u64);
    out.extend_from_slice(b);
}

fn cbor_int(out: &mut Vec<u8>, v: i64) {
    if v >= 0 {
        cbor_head(out, 0, v as u64);
    } else {
        cbor_head(out, 1, (-1 - v) as u64);
    }
}

/// PNG-compress a JSON string the way rosbridge does: pack the UTF-8 bytes as
/// RGB pixels of a near-square image (padded with `\n`), PNG-encode, base64.
pub fn png_encode(json: &str) -> Result<String, png::EncodingError> {
    let data = json.as_bytes();
    let length = data.len().max(1);
    let pixels = (length as f64 / 3.0).sqrt().ceil() as usize;
    let width = pixels.max(1);
    let height = ((length as f64 / 3.0) / width as f64).ceil() as usize;
    let height = height.max(1);
    let needed = width * height * 3;

    let mut buf = Vec::with_capacity(needed);
    buf.extend_from_slice(data);
    buf.resize(needed, b'\n');

    let mut png_bytes = Vec::new();
    {
        let mut encoder = png::Encoder::new(&mut png_bytes, width as u32, height as u32);
        encoder.set_color(png::ColorType::Rgb);
        encoder.set_depth(png::BitDepth::Eight);
        let mut writer = encoder.write_header()?;
        writer.write_image_data(&buf)?;
    }
    Ok(base64::engine::general_purpose::STANDARD.encode(&png_bytes))
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn compression_precedence() {
        assert!(Compression::CborRaw.rank() > Compression::Cbor.rank());
        assert!(Compression::Cbor.rank() > Compression::Png.rank());
        assert!(Compression::Png.rank() > Compression::None.rank());
    }

    #[test]
    fn cbor_roundtrips_via_ciborium() {
        let v = json!({"op":"publish","topic":"/t","msg":{"data":42}});
        let bytes = to_cbor(&v);
        let back: Value = ciborium::from_reader(&bytes[..]).unwrap();
        assert_eq!(back, v);
    }

    #[test]
    fn cbor_raw_has_byte_string() {
        let frame = cbor_raw_frame("/t", &[1, 2, 3, 4], 5, 6);
        let back: ciborium::value::Value = ciborium::from_reader(&frame[..]).unwrap();
        // Navigate to msg.bytes and confirm it's a CBOR byte string.
        let map = back.as_map().unwrap();
        let msg = map
            .iter()
            .find(|(k, _)| k.as_text() == Some("msg"))
            .map(|(_, v)| v)
            .unwrap();
        let msg_map = msg.as_map().unwrap();
        let bytes = msg_map
            .iter()
            .find(|(k, _)| k.as_text() == Some("bytes"))
            .map(|(_, v)| v)
            .unwrap();
        assert_eq!(bytes.as_bytes().unwrap(), &[1, 2, 3, 4]);
    }

    #[test]
    fn png_encodes_valid_image() {
        let s = r#"{"op":"publish","topic":"/t","msg":{"data":"hello"}}"#;
        let b64 = png_encode(s).unwrap();
        let raw = base64::engine::general_purpose::STANDARD.decode(&b64).unwrap();
        // PNG signature.
        assert_eq!(&raw[..8], &[137, 80, 78, 71, 13, 10, 26, 10]);
    }
}
