//! Minimal alignment-aware CDR reader/writer (the DDS/ROS2 wire format).
//!
//! CDR is not self-describing; the [`crate::codec`] module drives these
//! primitives using a parsed message schema. Both little- and big-endian
//! encapsulations are supported. Alignment is measured relative to the start
//! of the CDR body (i.e. *after* the 4-byte encapsulation header), matching
//! the RTPS rule used by ROS2.

/// Errors from CDR (de)serialization.
#[derive(Debug, thiserror::Error)]
pub enum CdrError {
    #[error("unexpected end of CDR buffer (needed {needed} bytes at offset {offset})")]
    Eof { needed: usize, offset: usize },
    #[error("invalid encapsulation header")]
    BadEncapsulation,
    #[error("invalid UTF-8 in CDR string")]
    Utf8,
    #[error("sequence length {len} exceeds remaining buffer")]
    BadLength { len: usize },
}

/// Byte order of a CDR stream.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Endian {
    Little,
    Big,
}

/// Writer that accumulates a CDR body with correct inter-field padding.
pub struct CdrWriter {
    buf: Vec<u8>,
    endian: Endian,
}

impl CdrWriter {
    /// Create a writer and emit the 4-byte encapsulation header for `endian`.
    pub fn new(endian: Endian) -> Self {
        let mut buf = Vec::with_capacity(64);
        // Encapsulation header: [0x00, scheme, options(2 bytes)].
        buf.push(0x00);
        buf.push(match endian {
            Endian::Big => 0x00,   // CDR_BE
            Endian::Little => 0x01, // CDR_LE
        });
        buf.push(0x00);
        buf.push(0x00);
        CdrWriter { buf, endian }
    }

    /// Body length so far (excludes the 4-byte encapsulation header).
    #[inline]
    fn body_len(&self) -> usize {
        self.buf.len() - 4
    }

    /// Pad with zero bytes so the next write of `align` bytes is aligned.
    #[inline]
    pub fn align(&mut self, align: usize) {
        if align <= 1 {
            return;
        }
        let rem = self.body_len() % align;
        if rem != 0 {
            for _ in 0..(align - rem) {
                self.buf.push(0);
            }
        }
    }

    pub fn write_u8(&mut self, v: u8) {
        self.buf.push(v);
    }

    pub fn write_bool(&mut self, v: bool) {
        self.buf.push(v as u8);
    }

    pub fn write_bytes(&mut self, b: &[u8]) {
        self.buf.extend_from_slice(b);
    }

    pub fn write_f32(&mut self, v: f32) {
        self.align(4);
        match self.endian {
            Endian::Little => self.buf.extend_from_slice(&v.to_le_bytes()),
            Endian::Big => self.buf.extend_from_slice(&v.to_be_bytes()),
        }
    }

    pub fn write_f64(&mut self, v: f64) {
        self.align(8);
        match self.endian {
            Endian::Little => self.buf.extend_from_slice(&v.to_le_bytes()),
            Endian::Big => self.buf.extend_from_slice(&v.to_be_bytes()),
        }
    }

    /// Write a CDR string: u32 length (incl. NUL) + UTF-8 bytes + NUL.
    pub fn write_string(&mut self, s: &str) {
        let bytes = s.as_bytes();
        self.write_u32(bytes.len() as u32 + 1);
        self.buf.extend_from_slice(bytes);
        self.buf.push(0);
    }

    /// Finish and return the full buffer (header + body).
    pub fn into_bytes(self) -> Vec<u8> {
        self.buf
    }

    pub fn endian(&self) -> Endian {
        self.endian
    }
}

/// Generates the integer write_* methods (all align to their size).
macro_rules! macro_int_impl {
    ($($name:ident => $ty:ty),* $(,)?) => {
        impl CdrWriter {
            $(
                pub fn $name(&mut self, v: $ty) {
                    let sz = std::mem::size_of::<$ty>();
                    self.align(sz);
                    match self.endian {
                        Endian::Little => self.buf.extend_from_slice(&v.to_le_bytes()),
                        Endian::Big => self.buf.extend_from_slice(&v.to_be_bytes()),
                    }
                }
            )*
        }
    };
}

macro_int_impl!(
    write_i8 => i8,
    write_i16 => i16,
    write_u16 => u16,
    write_i32 => i32,
    write_u32 => u32,
    write_i64 => i64,
    write_u64 => u64,
);

/// Reader that walks a CDR body honoring the same alignment rules.
pub struct CdrReader<'a> {
    data: &'a [u8],
    /// Cursor position relative to the body start (after the 4-byte header).
    pos: usize,
    endian: Endian,
}

impl<'a> CdrReader<'a> {
    /// Parse the encapsulation header and create a reader over the body.
    pub fn new(data: &'a [u8]) -> Result<Self, CdrError> {
        if data.len() < 4 {
            return Err(CdrError::BadEncapsulation);
        }
        let endian = match data[1] {
            0x00 | 0x02 => Endian::Big,    // CDR_BE / PL_CDR_BE
            0x01 | 0x03 => Endian::Little, // CDR_LE / PL_CDR_LE
            _ => return Err(CdrError::BadEncapsulation),
        };
        Ok(CdrReader {
            data: &data[4..],
            pos: 0,
            endian,
        })
    }

    /// Reader over a raw body with an explicit endian (no header).
    pub fn from_body(data: &'a [u8], endian: Endian) -> Self {
        CdrReader { data, pos: 0, endian }
    }

    #[inline]
    fn align(&mut self, align: usize) {
        if align > 1 {
            let rem = self.pos % align;
            if rem != 0 {
                self.pos += align - rem;
            }
        }
    }

    #[inline]
    fn take(&mut self, n: usize) -> Result<&'a [u8], CdrError> {
        if self.pos + n > self.data.len() {
            return Err(CdrError::Eof {
                needed: n,
                offset: self.pos,
            });
        }
        let s = &self.data[self.pos..self.pos + n];
        self.pos += n;
        Ok(s)
    }

    pub fn read_u8(&mut self) -> Result<u8, CdrError> {
        Ok(self.take(1)?[0])
    }

    pub fn read_bool(&mut self) -> Result<bool, CdrError> {
        Ok(self.read_u8()? != 0)
    }

    pub fn read_f32(&mut self) -> Result<f32, CdrError> {
        self.align(4);
        let b = self.take(4)?;
        let arr = [b[0], b[1], b[2], b[3]];
        Ok(match self.endian {
            Endian::Little => f32::from_le_bytes(arr),
            Endian::Big => f32::from_be_bytes(arr),
        })
    }

    pub fn read_f64(&mut self) -> Result<f64, CdrError> {
        self.align(8);
        let b = self.take(8)?;
        let mut arr = [0u8; 8];
        arr.copy_from_slice(b);
        Ok(match self.endian {
            Endian::Little => f64::from_le_bytes(arr),
            Endian::Big => f64::from_be_bytes(arr),
        })
    }

    /// Read a CDR string (u32 length incl. NUL, then bytes, drop trailing NUL).
    pub fn read_string(&mut self) -> Result<String, CdrError> {
        let len = self.read_u32()? as usize;
        if len == 0 {
            return Ok(String::new());
        }
        let bytes = self.take(len)?;
        // Drop trailing NUL terminator if present.
        let end = if bytes.last() == Some(&0) { len - 1 } else { len };
        std::str::from_utf8(&bytes[..end])
            .map(|s| s.to_string())
            .map_err(|_| CdrError::Utf8)
    }

    /// Read a length prefix (u32) used by sequences, validating it fits.
    pub fn read_seq_len(&mut self) -> Result<usize, CdrError> {
        let len = self.read_u32()? as usize;
        Ok(len)
    }

    pub fn read_raw(&mut self, n: usize) -> Result<&'a [u8], CdrError> {
        self.take(n)
    }

    pub fn endian(&self) -> Endian {
        self.endian
    }
}

/// Generates the integer read_* methods.
macro_rules! macro_read_int {
    ($($name:ident => $ty:ty),* $(,)?) => {
        impl<'a> CdrReader<'a> {
            $(
                pub fn $name(&mut self) -> Result<$ty, CdrError> {
                    const N: usize = std::mem::size_of::<$ty>();
                    self.align(N);
                    let b = self.take(N)?;
                    let mut arr = [0u8; N];
                    arr.copy_from_slice(b);
                    Ok(match self.endian {
                        Endian::Little => <$ty>::from_le_bytes(arr),
                        Endian::Big => <$ty>::from_be_bytes(arr),
                    })
                }
            )*
        }
    };
}

macro_read_int!(
    read_i8 => i8,
    read_i16 => i16,
    read_u16 => u16,
    read_i32 => i32,
    read_u32 => u32,
    read_i64 => i64,
    read_u64 => u64,
);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip_primitives_le() {
        let mut w = CdrWriter::new(Endian::Little);
        w.write_u8(7);
        w.write_u32(0xDEADBEEF);
        w.write_f64(1234.5678);
        w.write_string("hello");
        w.write_i16(-3);
        let bytes = w.into_bytes();

        let mut r = CdrReader::new(&bytes).unwrap();
        assert_eq!(r.read_u8().unwrap(), 7);
        assert_eq!(r.read_u32().unwrap(), 0xDEADBEEF);
        assert!((r.read_f64().unwrap() - 1234.5678).abs() < 1e-9);
        assert_eq!(r.read_string().unwrap(), "hello");
        assert_eq!(r.read_i16().unwrap(), -3);
    }

    #[test]
    fn roundtrip_primitives_be() {
        let mut w = CdrWriter::new(Endian::Big);
        w.write_u16(0x1234);
        w.write_i64(-1234567);
        w.write_f32(2.5);
        let bytes = w.into_bytes();
        assert_eq!(bytes[1], 0x00);

        let mut r = CdrReader::new(&bytes).unwrap();
        assert_eq!(r.read_u16().unwrap(), 0x1234);
        assert_eq!(r.read_i64().unwrap(), -1234567);
        assert_eq!(r.read_f32().unwrap(), 2.5);
    }

    #[test]
    fn alignment_padding() {
        // u8 then u32: the u32 must be 4-aligned, so 3 pad bytes appear.
        let mut w = CdrWriter::new(Endian::Little);
        w.write_u8(1);
        w.write_u32(2);
        let bytes = w.into_bytes();
        // header(4) + u8(1) + pad(3) + u32(4) = 12
        assert_eq!(bytes.len(), 12);
    }
}
