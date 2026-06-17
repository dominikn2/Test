//! Parser for ROS2 `.msg`, `.srv`, and `.action` interface definitions.

use crate::datatype::{BaseType, FieldType, Multiplicity, Primitive};
use crate::spec::{ActionSpec, Constant, Field, MessageSpec, ServiceSpec};

/// Errors raised while parsing an interface definition.
#[derive(Debug, thiserror::Error, PartialEq)]
pub enum ParseError {
    #[error("invalid field/constant line: {0:?}")]
    InvalidLine(String),
    #[error("invalid type token: {0:?}")]
    InvalidType(String),
    #[error("invalid array specifier: {0:?}")]
    InvalidArray(String),
}

/// Normalize a referenced type name to `package/msg/Type` form.
///
/// * `Header` (no slash) is special-cased to `std_msgs/msg/Header`.
/// * A bare `Type` resolves within `current_pkg`.
/// * `pkg/Type` is expanded to `pkg/msg/Type`.
/// * `pkg/msg/Type` (already qualified) is returned as-is.
pub fn normalize_type_name(raw: &str, current_pkg: &str) -> String {
    let parts: Vec<&str> = raw.split('/').collect();
    match parts.as_slice() {
        [name] => {
            if *name == "Header" {
                "std_msgs/msg/Header".to_string()
            } else {
                format!("{current_pkg}/msg/{name}")
            }
        }
        [pkg, name] => format!("{pkg}/msg/{name}"),
        [pkg, kind, name] => format!("{pkg}/{kind}/{name}"),
        _ => raw.to_string(),
    }
}

/// Parse just the type portion of a field declaration (no array suffix logic
/// applied yet — this resolves the base type only).
fn parse_base_type(tok: &str, current_pkg: &str) -> Result<BaseType, ParseError> {
    // Bounded string: string<=N / wstring<=N
    if let Some(rest) = tok.strip_prefix("string") {
        return Ok(BaseType::String(parse_string_bound(rest)?));
    }
    if let Some(rest) = tok.strip_prefix("wstring") {
        return Ok(BaseType::WString(parse_string_bound(rest)?));
    }
    if let Some(p) = Primitive::from_token(tok) {
        return Ok(BaseType::Primitive(p));
    }
    if tok.is_empty() {
        return Err(ParseError::InvalidType(tok.to_string()));
    }
    Ok(BaseType::Message(normalize_type_name(tok, current_pkg)))
}

fn parse_string_bound(rest: &str) -> Result<Option<usize>, ParseError> {
    if rest.is_empty() {
        Ok(None)
    } else if let Some(n) = rest.strip_prefix("<=") {
        n.parse::<usize>()
            .map(Some)
            .map_err(|_| ParseError::InvalidType(format!("string{rest}")))
    } else {
        Err(ParseError::InvalidType(format!("string{rest}")))
    }
}

/// Parse a complete type token including array/sequence suffixes.
pub fn parse_field_type(token: &str, current_pkg: &str) -> Result<FieldType, ParseError> {
    // IDL sequence<T> / sequence<T, N>
    if let Some(inner) = token.strip_prefix("sequence<").and_then(|s| s.strip_suffix('>')) {
        let mut parts = inner.rsplitn(2, ',');
        let first = parts.next().unwrap().trim();
        if let Some(elem) = parts.next() {
            // sequence<T, N>
            let bound: usize = first
                .parse()
                .map_err(|_| ParseError::InvalidArray(token.to_string()))?;
            let base = parse_base_type(elem.trim(), current_pkg)?;
            return Ok(FieldType {
                base,
                multiplicity: Multiplicity::BoundedSequence(bound),
            });
        } else {
            let base = parse_base_type(first, current_pkg)?;
            return Ok(FieldType {
                base,
                multiplicity: Multiplicity::UnboundedSequence,
            });
        }
    }

    // Array suffix form: T[], T[N], T[<=N]
    if let Some(open) = token.find('[') {
        if !token.ends_with(']') {
            return Err(ParseError::InvalidArray(token.to_string()));
        }
        let base_tok = &token[..open];
        let spec = &token[open + 1..token.len() - 1];
        let base = parse_base_type(base_tok, current_pkg)?;
        let multiplicity = if spec.is_empty() {
            Multiplicity::UnboundedSequence
        } else if let Some(n) = spec.strip_prefix("<=") {
            Multiplicity::BoundedSequence(
                n.parse().map_err(|_| ParseError::InvalidArray(token.to_string()))?,
            )
        } else {
            Multiplicity::FixedArray(
                spec.parse().map_err(|_| ParseError::InvalidArray(token.to_string()))?,
            )
        };
        return Ok(FieldType { base, multiplicity });
    }

    Ok(FieldType {
        base: parse_base_type(token, current_pkg)?,
        multiplicity: Multiplicity::Scalar,
    })
}

/// Strip a trailing `# comment`, respecting nothing fancy (callers handle
/// string-constant values separately).
fn strip_comment(line: &str) -> &str {
    match line.find('#') {
        Some(idx) => &line[..idx],
        None => line,
    }
}

/// Parse a `.msg` body into a [`MessageSpec`]. `name` is the fully-qualified
/// `package/msg/Type`; `current_pkg` is the package used to resolve relative
/// references.
pub fn parse_message(name: &str, current_pkg: &str, body: &str) -> Result<MessageSpec, ParseError> {
    let mut fields = Vec::new();
    let mut constants = Vec::new();

    for raw_line in body.lines() {
        // Determine whether this is a constant before stripping comments, so a
        // `string` constant value containing '#' survives.
        let trimmed_full = raw_line.trim();
        if trimmed_full.is_empty() || trimmed_full.starts_with('#') {
            continue;
        }

        // Tokenize "type rest"
        let mut it = trimmed_full.splitn(2, char::is_whitespace);
        let type_tok = it.next().unwrap_or("");
        let rest = it.next().unwrap_or("").trim();
        if rest.is_empty() {
            // A lone token isn't valid; but skip gracefully (e.g. stray text).
            return Err(ParseError::InvalidLine(raw_line.to_string()));
        }

        // Constant? "NAME=value" appears in `rest` with an '='. Constants use
        // a primitive/string type and an uppercase-ish name. We treat any
        // line whose remainder contains '=' (before whitespace) as constant.
        if let Some(eq) = rest.find('=') {
            // Make sure the '=' belongs to a constant (NAME=val), not a default
            // value in a field (field defaults appear after the name + space).
            let name_part = rest[..eq].trim();
            if !name_part.contains(char::is_whitespace) && !name_part.is_empty() {
                let value_part = rest[eq + 1..].trim().to_string();
                // For non-string constants, a trailing comment is stripped.
                let value = if type_tok.starts_with("string") || type_tok.starts_with("wstring") {
                    value_part
                } else {
                    strip_comment(&value_part).trim().to_string()
                };
                constants.push(Constant {
                    name: name_part.to_string(),
                    ty: type_tok.to_string(),
                    value,
                });
                continue;
            }
        }

        // Field: "type name [default]"
        let cleaned = strip_comment(trimmed_full);
        let mut parts = cleaned.split_whitespace();
        let type_tok = parts.next().ok_or_else(|| ParseError::InvalidLine(raw_line.to_string()))?;
        let field_name = parts
            .next()
            .ok_or_else(|| ParseError::InvalidLine(raw_line.to_string()))?;
        let default_text = {
            let collected: Vec<&str> = parts.collect();
            if collected.is_empty() {
                None
            } else {
                Some(collected.join(" "))
            }
        };
        let ty = parse_field_type(type_tok, current_pkg)?;
        fields.push(Field {
            name: field_name.to_string(),
            ty,
            default: default_text,
        });
    }

    Ok(MessageSpec {
        name: name.to_string(),
        fields,
        constants,
    })
}

/// Split a `.srv` body into request and response on the `---` separator and
/// parse each half.
pub fn parse_service(name: &str, current_pkg: &str, body: &str) -> Result<ServiceSpec, ParseError> {
    let (req, resp) = split_sections(body, 2).map(|mut v| (v.remove(0), v.remove(0)))?;
    Ok(ServiceSpec {
        name: name.to_string(),
        request: parse_message(&format!("{name}_Request"), current_pkg, &req)?,
        response: parse_message(&format!("{name}_Response"), current_pkg, &resp)?,
    })
}

/// Split a `.action` body into goal/result/feedback on `---` separators.
pub fn parse_action(name: &str, current_pkg: &str, body: &str) -> Result<ActionSpec, ParseError> {
    let mut sections = split_sections(body, 3)?;
    let feedback = sections.remove(2);
    let result = sections.remove(1);
    let goal = sections.remove(0);
    Ok(ActionSpec {
        name: name.to_string(),
        goal: parse_message(&format!("{name}_Goal"), current_pkg, &goal)?,
        result: parse_message(&format!("{name}_Result"), current_pkg, &result)?,
        feedback: parse_message(&format!("{name}_Feedback"), current_pkg, &feedback)?,
    })
}

/// Split a body on lines consisting solely of `---`. Pads with empty sections
/// up to `expected` so missing trailing sections parse as empty messages.
fn split_sections(body: &str, expected: usize) -> Result<Vec<String>, ParseError> {
    let mut sections: Vec<String> = vec![String::new()];
    for line in body.lines() {
        if line.trim() == "---" {
            sections.push(String::new());
        } else {
            let last = sections.last_mut().unwrap();
            last.push_str(line);
            last.push('\n');
        }
    }
    while sections.len() < expected {
        sections.push(String::new());
    }
    Ok(sections)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn normalize_names() {
        assert_eq!(normalize_type_name("Header", "foo"), "std_msgs/msg/Header");
        assert_eq!(normalize_type_name("Point", "geometry_msgs"), "geometry_msgs/msg/Point");
        assert_eq!(normalize_type_name("std_msgs/String", "x"), "std_msgs/msg/String");
        assert_eq!(normalize_type_name("std_msgs/msg/String", "x"), "std_msgs/msg/String");
    }

    #[test]
    fn parse_array_forms() {
        let p = "geometry_msgs";
        assert_eq!(parse_field_type("float64", p).unwrap().multiplicity, Multiplicity::Scalar);
        assert_eq!(
            parse_field_type("float64[]", p).unwrap().multiplicity,
            Multiplicity::UnboundedSequence
        );
        assert_eq!(
            parse_field_type("float64[3]", p).unwrap().multiplicity,
            Multiplicity::FixedArray(3)
        );
        assert_eq!(
            parse_field_type("uint8[<=10]", p).unwrap().multiplicity,
            Multiplicity::BoundedSequence(10)
        );
        assert_eq!(
            parse_field_type("sequence<int32>", p).unwrap().multiplicity,
            Multiplicity::UnboundedSequence
        );
        assert_eq!(
            parse_field_type("sequence<int32, 5>", p).unwrap().multiplicity,
            Multiplicity::BoundedSequence(5)
        );
    }

    #[test]
    fn parse_bounded_string() {
        let p = "x";
        assert_eq!(parse_field_type("string<=20", p).unwrap().base, BaseType::String(Some(20)));
        assert_eq!(parse_field_type("string", p).unwrap().base, BaseType::String(None));
    }

    #[test]
    fn parse_msg_fields_and_constants() {
        let body = "uint8 FOO=1\nstring NAME=hello # not a comment value\nfloat64 x\nfloat64 y 3.5\nHeader header\n";
        let spec = parse_message("p/msg/M", "p", body).unwrap();
        assert_eq!(spec.constants.len(), 2);
        assert_eq!(spec.constants[0].name, "FOO");
        assert_eq!(spec.constants[0].value, "1");
        assert_eq!(spec.constants[1].value, "hello # not a comment value");
        assert_eq!(spec.fields.len(), 3);
        assert_eq!(spec.fields[1].default.as_deref(), Some("3.5"));
        assert_eq!(spec.fields[2].ty.base, BaseType::Message("std_msgs/msg/Header".into()));
    }

    #[test]
    fn parse_srv_sections() {
        let s = parse_service("p/srv/S", "p", "int64 a\nint64 b\n---\nint64 sum\n").unwrap();
        assert_eq!(s.request.fields.len(), 2);
        assert_eq!(s.response.fields.len(), 1);
    }
}
