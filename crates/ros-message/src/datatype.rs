//! The data-type model for ROS2 interface fields.

use std::fmt;

/// A ROS2 primitive (built-in) scalar type.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Primitive {
    Bool,
    /// `byte` / `octet`: 8-bit unsigned, treated as raw byte.
    Byte,
    /// `char`: 8-bit unsigned in ROS2.
    Char,
    Int8,
    Uint8,
    Int16,
    Uint16,
    Int32,
    Uint32,
    Int64,
    Uint64,
    Float32,
    Float64,
}

impl Primitive {
    /// Map a textual base type to a primitive, if it is one.
    pub fn from_token(tok: &str) -> Option<Primitive> {
        Some(match tok {
            "bool" | "boolean" => Primitive::Bool,
            "byte" | "octet" => Primitive::Byte,
            "char" => Primitive::Char,
            "int8" => Primitive::Int8,
            "uint8" => Primitive::Uint8,
            "int16" => Primitive::Int16,
            "uint16" => Primitive::Uint16,
            "int32" => Primitive::Int32,
            "uint32" => Primitive::Uint32,
            "int64" => Primitive::Int64,
            "uint64" => Primitive::Uint64,
            "float32" | "float" => Primitive::Float32,
            "float64" | "double" => Primitive::Float64,
            _ => return None,
        })
    }

    /// Size and alignment (equal for all CDR primitives) in bytes.
    pub fn size(self) -> usize {
        match self {
            Primitive::Bool
            | Primitive::Byte
            | Primitive::Char
            | Primitive::Int8
            | Primitive::Uint8 => 1,
            Primitive::Int16 | Primitive::Uint16 => 2,
            Primitive::Int32 | Primitive::Uint32 | Primitive::Float32 => 4,
            Primitive::Int64 | Primitive::Uint64 | Primitive::Float64 => 8,
        }
    }

    /// Whether a sequence/array of this primitive is a byte blob that
    /// rosbridge encodes as base64 in JSON (`uint8[]`, `byte[]`, `char[]`).
    pub fn is_byte_like(self) -> bool {
        matches!(self, Primitive::Uint8 | Primitive::Byte | Primitive::Char)
    }
}

impl fmt::Display for Primitive {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            Primitive::Bool => "bool",
            Primitive::Byte => "byte",
            Primitive::Char => "char",
            Primitive::Int8 => "int8",
            Primitive::Uint8 => "uint8",
            Primitive::Int16 => "int16",
            Primitive::Uint16 => "uint16",
            Primitive::Int32 => "int32",
            Primitive::Uint32 => "uint32",
            Primitive::Int64 => "int64",
            Primitive::Uint64 => "uint64",
            Primitive::Float32 => "float32",
            Primitive::Float64 => "float64",
        };
        f.write_str(s)
    }
}

/// The base (element) type of a field, ignoring array-ness.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum BaseType {
    Primitive(Primitive),
    /// `string` with optional upper bound.
    String(Option<usize>),
    /// `wstring` with optional upper bound (UTF-16 on the wire).
    WString(Option<usize>),
    /// A nested message type, normalized to `package/msg/Type`.
    Message(String),
}

/// How a field repeats.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Multiplicity {
    /// A single value.
    Scalar,
    /// A fixed-length array `type[N]` — no length prefix on the wire.
    FixedArray(usize),
    /// An upper-bounded sequence `sequence<type, N>` / `type[<=N]`.
    BoundedSequence(usize),
    /// An unbounded sequence `sequence<type>` / `type[]`.
    UnboundedSequence,
}

/// A fully-resolved field type: base type plus multiplicity.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct FieldType {
    pub base: BaseType,
    pub multiplicity: Multiplicity,
}

impl FieldType {
    pub fn is_array(&self) -> bool {
        !matches!(self.multiplicity, Multiplicity::Scalar)
    }

    /// True when this is a byte blob (`uint8[]`/`byte[]`/`char[]`, any array
    /// form) which rosbridge base64-encodes in JSON.
    pub fn is_byte_blob(&self) -> bool {
        self.is_array()
            && matches!(self.base, BaseType::Primitive(p) if p.is_byte_like())
    }
}
