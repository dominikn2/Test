//! Schema-driven conversion between `serde_json::Value` and CDR bytes.
//!
//! This is the rosbridge serialization boundary: ROS2 carries opaque CDR over
//! DDS, while rosbridge clients speak JSON. Because CDR is not self-describing,
//! every conversion is driven by a parsed [`MessageSpec`] from the [`Registry`].

use base64::Engine;
use serde_json::{Map, Value};

use crate::cdr::{CdrReader, CdrWriter, Endian};
use crate::datatype::{BaseType, FieldType, Multiplicity, Primitive};
use crate::registry::Registry;
use crate::spec::MessageSpec;

/// A ROS time/duration value: seconds and nanoseconds.
pub type RosTime = (i32, u32);

/// Errors during JSON<->CDR conversion.
#[derive(Debug, thiserror::Error)]
pub enum CodecError {
    #[error("field '{path}': expected {expected}, found {found}")]
    TypeMismatch {
        path: String,
        expected: String,
        found: String,
    },
    #[error("field '{path}': value out of range for {ty}")]
    OutOfRange { path: String, ty: String },
    #[error("unknown nested type '{0}'")]
    UnknownType(String),
    #[error("fixed array '{path}' expects {expected} elements, got {got}")]
    ArrayLen {
        path: String,
        expected: usize,
        got: usize,
    },
    #[error("invalid base64 in byte field '{path}'")]
    Base64 { path: String },
    #[error(transparent)]
    Cdr(#[from] crate::cdr::CdrError),
    #[error(transparent)]
    Registry(#[from] crate::registry::RegistryError),
}

/// Context shared across a single encode/decode operation.
pub struct Codec<'a> {
    pub registry: &'a Registry,
    /// Current ROS time, used for `"now"` substitution and header auto-fill.
    pub now: Option<RosTime>,
}

impl<'a> Codec<'a> {
    pub fn new(registry: &'a Registry) -> Self {
        Codec { registry, now: None }
    }

    pub fn with_now(registry: &'a Registry, now: Option<RosTime>) -> Self {
        Codec { registry, now }
    }

    // ---- Encoding: JSON -> CDR ------------------------------------------

    /// Encode a JSON object into a CDR buffer for the given message type.
    pub fn encode(&self, spec: &MessageSpec, value: &Value) -> Result<Vec<u8>, CodecError> {
        let mut w = CdrWriter::new(Endian::Little);
        self.encode_message(spec, value, &mut w, "")?;
        Ok(w.into_bytes())
    }

    fn encode_message(
        &self,
        spec: &MessageSpec,
        value: &Value,
        w: &mut CdrWriter,
        path: &str,
    ) -> Result<(), CodecError> {
        let empty = Map::new();
        let obj = value.as_object().unwrap_or(&empty);
        let is_header = spec.name == "std_msgs/msg/Header";

        for field in &spec.fields {
            let fpath = if path.is_empty() {
                field.name.clone()
            } else {
                format!("{path}.{}", field.name)
            };
            let fval = obj.get(&field.name);

            // Header.stamp auto-fill with current time if omitted.
            let auto_now =
                is_header && field.name == "stamp" && fval.map(|v| v.is_null()).unwrap_or(true);

            match fval {
                Some(v) if !auto_now => self.encode_field(&field.ty, v, w, &fpath)?,
                _ => {
                    // Use written default or zero-value.
                    self.encode_default(&field.ty, field.default.as_deref(), w, &fpath, auto_now)?;
                }
            }
        }
        Ok(())
    }

    fn encode_field(
        &self,
        ty: &FieldType,
        value: &Value,
        w: &mut CdrWriter,
        path: &str,
    ) -> Result<(), CodecError> {
        match ty.multiplicity {
            Multiplicity::Scalar => self.encode_scalar(&ty.base, value, w, path),
            Multiplicity::FixedArray(n) => {
                // Byte blob fixed arrays may arrive base64-encoded.
                if ty.is_byte_blob() {
                    let bytes = decode_byte_blob(value, path)?;
                    if bytes.len() != n {
                        return Err(CodecError::ArrayLen {
                            path: path.into(),
                            expected: n,
                            got: bytes.len(),
                        });
                    }
                    w.write_bytes(&bytes);
                    return Ok(());
                }
                let arr = value.as_array().ok_or_else(|| mismatch(path, "array", value))?;
                if arr.len() != n {
                    return Err(CodecError::ArrayLen {
                        path: path.into(),
                        expected: n,
                        got: arr.len(),
                    });
                }
                for (i, el) in arr.iter().enumerate() {
                    self.encode_scalar(&ty.base, el, w, &format!("{path}[{i}]"))?;
                }
                Ok(())
            }
            Multiplicity::BoundedSequence(_) | Multiplicity::UnboundedSequence => {
                if ty.is_byte_blob() {
                    let bytes = decode_byte_blob(value, path)?;
                    w.write_u32(bytes.len() as u32);
                    w.write_bytes(&bytes);
                    return Ok(());
                }
                let arr = value.as_array().ok_or_else(|| mismatch(path, "array", value))?;
                w.write_u32(arr.len() as u32);
                for (i, el) in arr.iter().enumerate() {
                    self.encode_scalar(&ty.base, el, w, &format!("{path}[{i}]"))?;
                }
                Ok(())
            }
        }
    }

    fn encode_scalar(
        &self,
        base: &BaseType,
        value: &Value,
        w: &mut CdrWriter,
        path: &str,
    ) -> Result<(), CodecError> {
        match base {
            BaseType::Primitive(p) => encode_primitive(*p, value, w, path),
            BaseType::String(_) | BaseType::WString(_) => {
                let s = value.as_str().ok_or_else(|| mismatch(path, "string", value))?;
                w.write_string(s);
                Ok(())
            }
            BaseType::Message(name) => {
                // `"now"` for time/duration types.
                let nested = self.registry.message(name)?;
                let resolved = self.resolve_time_value(name, value);
                self.encode_message(nested, resolved.as_ref().unwrap_or(value), w, path)
            }
        }
    }

    /// If the type is a Time/Duration and value is `"now"`, return the
    /// substituted `{sec,nanosec}` object.
    fn resolve_time_value(&self, name: &str, value: &Value) -> Option<Value> {
        if is_time_type(name) && value.as_str() == Some("now") {
            let (sec, nsec) = self.now.unwrap_or((0, 0));
            let mut m = Map::new();
            m.insert("sec".into(), Value::from(sec));
            m.insert("nanosec".into(), Value::from(nsec));
            Some(Value::Object(m))
        } else if is_time_type(name) {
            // Accept ROS1 secs/nsecs aliases.
            value.as_object().map(|o| {
                let mut m = o.clone();
                if !m.contains_key("sec") {
                    if let Some(v) = o.get("secs") {
                        m.insert("sec".into(), v.clone());
                    }
                }
                if !m.contains_key("nanosec") {
                    if let Some(v) = o.get("nsecs") {
                        m.insert("nanosec".into(), v.clone());
                    }
                }
                Value::Object(m)
            })
        } else {
            None
        }
    }

    fn encode_default(
        &self,
        ty: &FieldType,
        default: Option<&str>,
        w: &mut CdrWriter,
        path: &str,
        auto_now: bool,
    ) -> Result<(), CodecError> {
        // Arrays/sequences default to empty (or written default literal).
        match ty.multiplicity {
            Multiplicity::Scalar => {}
            Multiplicity::FixedArray(n) => {
                if ty.is_byte_blob() {
                    w.write_bytes(&vec![0u8; n]);
                    return Ok(());
                }
                for i in 0..n {
                    self.encode_default_scalar(&ty.base, None, w, &format!("{path}[{i}]"), false)?;
                }
                return Ok(());
            }
            Multiplicity::BoundedSequence(_) | Multiplicity::UnboundedSequence => {
                // Empty sequence by default.
                w.write_u32(0);
                return Ok(());
            }
        }
        self.encode_default_scalar(&ty.base, default, w, path, auto_now)
    }

    fn encode_default_scalar(
        &self,
        base: &BaseType,
        default: Option<&str>,
        w: &mut CdrWriter,
        path: &str,
        auto_now: bool,
    ) -> Result<(), CodecError> {
        match base {
            BaseType::Primitive(p) => {
                let v = default_primitive_value(*p, default);
                encode_primitive(*p, &v, w, path)
            }
            BaseType::String(_) | BaseType::WString(_) => {
                w.write_string(default.unwrap_or(""));
                Ok(())
            }
            BaseType::Message(name) => {
                if auto_now || (is_time_type(name) && self.now.is_some() && auto_now) {
                    let (sec, nsec) = self.now.unwrap_or((0, 0));
                    let mut m = Map::new();
                    m.insert("sec".into(), Value::from(sec));
                    m.insert("nanosec".into(), Value::from(nsec));
                    let nested = self.registry.message(name)?;
                    return self.encode_message(nested, &Value::Object(m), w, path);
                }
                let nested = self.registry.message(name)?;
                self.encode_message(nested, &Value::Object(Map::new()), w, path)
            }
        }
    }

    // ---- Decoding: CDR -> JSON ------------------------------------------

    /// Decode a CDR buffer into a JSON object for the given message type.
    pub fn decode(&self, spec: &MessageSpec, bytes: &[u8]) -> Result<Value, CodecError> {
        let mut r = CdrReader::new(bytes)?;
        self.decode_message(spec, &mut r)
    }

    fn decode_message(&self, spec: &MessageSpec, r: &mut CdrReader) -> Result<Value, CodecError> {
        let mut obj = Map::with_capacity(spec.fields.len());
        for field in &spec.fields {
            obj.insert(field.name.clone(), self.decode_field(&field.ty, r)?);
        }
        Ok(Value::Object(obj))
    }

    fn decode_field(&self, ty: &FieldType, r: &mut CdrReader) -> Result<Value, CodecError> {
        match ty.multiplicity {
            Multiplicity::Scalar => self.decode_scalar(&ty.base, r),
            Multiplicity::FixedArray(n) => {
                if ty.is_byte_blob() {
                    let bytes = r.read_raw(n)?;
                    return Ok(Value::String(
                        base64::engine::general_purpose::STANDARD.encode(bytes),
                    ));
                }
                let mut arr = Vec::with_capacity(n);
                for _ in 0..n {
                    arr.push(self.decode_scalar(&ty.base, r)?);
                }
                Ok(Value::Array(arr))
            }
            Multiplicity::BoundedSequence(_) | Multiplicity::UnboundedSequence => {
                let len = r.read_seq_len()?;
                if ty.is_byte_blob() {
                    let bytes = r.read_raw(len)?;
                    return Ok(Value::String(
                        base64::engine::general_purpose::STANDARD.encode(bytes),
                    ));
                }
                let mut arr = Vec::with_capacity(len.min(1024));
                for _ in 0..len {
                    arr.push(self.decode_scalar(&ty.base, r)?);
                }
                Ok(Value::Array(arr))
            }
        }
    }

    fn decode_scalar(&self, base: &BaseType, r: &mut CdrReader) -> Result<Value, CodecError> {
        match base {
            BaseType::Primitive(p) => decode_primitive(*p, r),
            BaseType::String(_) | BaseType::WString(_) => Ok(Value::String(r.read_string()?)),
            BaseType::Message(name) => {
                let nested = self.registry.message(name)?;
                self.decode_message(nested, r)
            }
        }
    }
}

// ---- Free helpers --------------------------------------------------------

fn is_time_type(name: &str) -> bool {
    name == "builtin_interfaces/msg/Time" || name == "builtin_interfaces/msg/Duration"
}

fn mismatch(path: &str, expected: &str, found: &Value) -> CodecError {
    CodecError::TypeMismatch {
        path: path.into(),
        expected: expected.into(),
        found: type_name_of(found).into(),
    }
}

fn type_name_of(v: &Value) -> &'static str {
    match v {
        Value::Null => "null",
        Value::Bool(_) => "bool",
        Value::Number(_) => "number",
        Value::String(_) => "string",
        Value::Array(_) => "array",
        Value::Object(_) => "object",
    }
}

/// Decode a JSON byte blob that may be base64 string or array of ints.
fn decode_byte_blob(value: &Value, path: &str) -> Result<Vec<u8>, CodecError> {
    match value {
        Value::String(s) => base64::engine::general_purpose::STANDARD
            .decode(s)
            .map_err(|_| CodecError::Base64 { path: path.into() }),
        Value::Array(arr) => arr
            .iter()
            .map(|v| {
                v.as_u64()
                    .and_then(|n| u8::try_from(n).ok())
                    .ok_or_else(|| CodecError::OutOfRange {
                        path: path.into(),
                        ty: "uint8".into(),
                    })
            })
            .collect(),
        _ => Err(mismatch(path, "base64 string or array", value)),
    }
}

/// Produce a JSON default for a primitive from an optional literal.
fn default_primitive_value(p: Primitive, default: Option<&str>) -> Value {
    if let Some(d) = default {
        match p {
            Primitive::Bool => return Value::Bool(d == "true" || d == "1"),
            Primitive::Float32 | Primitive::Float64 => {
                if let Ok(f) = d.parse::<f64>() {
                    if let Some(n) = serde_json::Number::from_f64(f) {
                        return Value::Number(n);
                    }
                }
            }
            _ => {
                if let Ok(i) = d.parse::<i64>() {
                    return Value::from(i);
                }
                if let Ok(u) = d.parse::<u64>() {
                    return Value::from(u);
                }
            }
        }
    }
    match p {
        Primitive::Bool => Value::Bool(false),
        Primitive::Float32 | Primitive::Float64 => Value::from(0.0),
        _ => Value::from(0),
    }
}

fn encode_primitive(
    p: Primitive,
    value: &Value,
    w: &mut CdrWriter,
    path: &str,
) -> Result<(), CodecError> {
    macro_rules! int {
        ($ty:ty, $write:ident) => {{
            let n = as_i128(value).ok_or_else(|| mismatch(path, stringify!($ty), value))?;
            let v = <$ty>::try_from(n).map_err(|_| CodecError::OutOfRange {
                path: path.into(),
                ty: p.to_string(),
            })?;
            w.$write(v);
        }};
    }
    macro_rules! uint {
        ($ty:ty, $write:ident) => {{
            let n = as_i128(value).ok_or_else(|| mismatch(path, stringify!($ty), value))?;
            let v = <$ty>::try_from(n).map_err(|_| CodecError::OutOfRange {
                path: path.into(),
                ty: p.to_string(),
            })?;
            w.$write(v);
        }};
    }
    match p {
        Primitive::Bool => {
            let b = value
                .as_bool()
                .or_else(|| value.as_i64().map(|n| n != 0))
                .ok_or_else(|| mismatch(path, "bool", value))?;
            w.write_bool(b);
        }
        Primitive::Byte | Primitive::Uint8 | Primitive::Char => uint!(u8, write_u8),
        Primitive::Int8 => int!(i8, write_i8),
        Primitive::Int16 => int!(i16, write_i16),
        Primitive::Uint16 => uint!(u16, write_u16),
        Primitive::Int32 => int!(i32, write_i32),
        Primitive::Uint32 => uint!(u32, write_u32),
        Primitive::Int64 => int!(i64, write_i64),
        Primitive::Uint64 => uint!(u64, write_u64),
        Primitive::Float32 => {
            let f = as_f64_or_nan(value).ok_or_else(|| mismatch(path, "float32", value))?;
            w.write_f32(f as f32);
        }
        Primitive::Float64 => {
            let f = as_f64_or_nan(value).ok_or_else(|| mismatch(path, "float64", value))?;
            w.write_f64(f);
        }
    }
    Ok(())
}

fn decode_primitive(p: Primitive, r: &mut CdrReader) -> Result<Value, CodecError> {
    Ok(match p {
        Primitive::Bool => Value::Bool(r.read_bool()?),
        Primitive::Byte | Primitive::Uint8 | Primitive::Char => Value::from(r.read_u8()?),
        Primitive::Int8 => Value::from(r.read_i8()?),
        Primitive::Int16 => Value::from(r.read_i16()?),
        Primitive::Uint16 => Value::from(r.read_u16()?),
        Primitive::Int32 => Value::from(r.read_i32()?),
        Primitive::Uint32 => Value::from(r.read_u32()?),
        Primitive::Int64 => Value::from(r.read_i64()?),
        Primitive::Uint64 => Value::from(r.read_u64()?),
        Primitive::Float32 => float_to_json(r.read_f32()? as f64),
        Primitive::Float64 => float_to_json(r.read_f64()?),
    })
}

/// Non-finite floats become JSON `null` (rosbridge behavior).
fn float_to_json(f: f64) -> Value {
    if f.is_finite() {
        serde_json::Number::from_f64(f)
            .map(Value::Number)
            .unwrap_or(Value::Null)
    } else {
        Value::Null
    }
}

/// Interpret a JSON number as an integer (accepts integral floats too).
fn as_i128(v: &Value) -> Option<i128> {
    if let Some(i) = v.as_i64() {
        Some(i as i128)
    } else if let Some(u) = v.as_u64() {
        Some(u as i128)
    } else if let Some(f) = v.as_f64() {
        if f.fract() == 0.0 {
            Some(f as i128)
        } else {
            None
        }
    } else {
        None
    }
}

/// JSON number as f64, mapping `null` to NaN (rosbridge round-trips non-finite
/// floats as null).
fn as_f64_or_nan(v: &Value) -> Option<f64> {
    match v {
        Value::Null => Some(f64::NAN),
        Value::Number(_) => v.as_f64(),
        _ => None,
    }
}
