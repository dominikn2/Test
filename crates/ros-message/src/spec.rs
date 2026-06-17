//! Parsed representation of a ROS2 interface definition.

use crate::datatype::FieldType;

/// A single named field of a message.
#[derive(Debug, Clone, PartialEq)]
pub struct Field {
    pub name: String,
    pub ty: FieldType,
    /// Default value text (as written in the `.msg`), if any.
    pub default: Option<String>,
}

/// A `TYPE NAME=value` constant.
#[derive(Debug, Clone, PartialEq)]
pub struct Constant {
    pub name: String,
    /// The primitive/string type token of the constant.
    pub ty: String,
    /// The literal value text.
    pub value: String,
}

/// A parsed message specification: the fields and constants of one message
/// type. The fully-qualified name is the registry key, kept here for context.
#[derive(Debug, Clone, PartialEq)]
pub struct MessageSpec {
    /// Fully-qualified `package/msg/Type`.
    pub name: String,
    pub fields: Vec<Field>,
    pub constants: Vec<Constant>,
}

/// A parsed service specification (request + response).
#[derive(Debug, Clone, PartialEq)]
pub struct ServiceSpec {
    pub name: String,
    pub request: MessageSpec,
    pub response: MessageSpec,
}

/// A parsed action specification (goal + result + feedback).
#[derive(Debug, Clone, PartialEq)]
pub struct ActionSpec {
    pub name: String,
    pub goal: MessageSpec,
    pub result: MessageSpec,
    pub feedback: MessageSpec,
}
