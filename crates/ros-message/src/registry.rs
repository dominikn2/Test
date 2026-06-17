//! A registry of interface definitions, resolving nested types on demand.

use std::collections::HashMap;
use std::path::Path;

use crate::parser::{self, ParseError};
use crate::spec::{ActionSpec, MessageSpec, ServiceSpec};

/// Errors raised by the registry.
#[derive(Debug, thiserror::Error)]
pub enum RegistryError {
    #[error("unknown message type: {0}")]
    UnknownType(String),
    #[error("unknown service type: {0}")]
    UnknownService(String),
    #[error("unknown action type: {0}")]
    UnknownAction(String),
    #[error("parse error in {name}: {source}")]
    Parse { name: String, source: ParseError },
    #[error("io error: {0}")]
    Io(#[from] std::io::Error),
}

/// Holds all known message/service/action specs, keyed by `package/msg/Type`,
/// `package/srv/Type`, `package/action/Type`.
#[derive(Default)]
pub struct Registry {
    messages: HashMap<String, MessageSpec>,
    services: HashMap<String, ServiceSpec>,
    actions: HashMap<String, ActionSpec>,
}

/// Split a fully-qualified name into `(package, kind, Type)`.
fn split_fqn(name: &str) -> Option<(&str, &str, &str)> {
    let mut it = name.split('/');
    match (it.next(), it.next(), it.next(), it.next()) {
        (Some(pkg), Some(kind), Some(ty), None) => Some((pkg, kind, ty)),
        _ => None,
    }
}

/// Normalize a possibly category-omitted type name to `pkg/kind/Type`.
pub fn normalize(name: &str, default_kind: &str) -> String {
    let parts: Vec<&str> = name.split('/').collect();
    match parts.as_slice() {
        [pkg, ty] => format!("{pkg}/{default_kind}/{ty}"),
        _ => name.to_string(),
    }
}

impl Registry {
    /// Create a registry preloaded with the bundled standard interfaces.
    pub fn with_standard_types() -> Self {
        let mut reg = Registry::default();
        crate::defs::load_into(&mut reg);
        reg
    }

    /// Register a `.msg` body under its fully-qualified `pkg/msg/Type` name.
    pub fn add_message(&mut self, fqn: &str, body: &str) -> Result<(), RegistryError> {
        let (pkg, _kind, _ty) = split_fqn(fqn).ok_or_else(|| RegistryError::UnknownType(fqn.into()))?;
        let spec = parser::parse_message(fqn, pkg, body)
            .map_err(|e| RegistryError::Parse { name: fqn.into(), source: e })?;
        self.messages.insert(fqn.to_string(), spec);
        Ok(())
    }

    /// Register a `.srv` body. Also registers the synthesized request/response
    /// messages as `pkg/srv/Type_Request` / `_Response`.
    pub fn add_service(&mut self, fqn: &str, body: &str) -> Result<(), RegistryError> {
        let (pkg, _kind, _ty) =
            split_fqn(fqn).ok_or_else(|| RegistryError::UnknownService(fqn.into()))?;
        let spec = parser::parse_service(fqn, pkg, body)
            .map_err(|e| RegistryError::Parse { name: fqn.into(), source: e })?;
        self.messages
            .insert(format!("{fqn}_Request"), spec.request.clone());
        self.messages
            .insert(format!("{fqn}_Response"), spec.response.clone());
        self.services.insert(fqn.to_string(), spec);
        Ok(())
    }

    /// Register a `.action` body, synthesizing the wrapped goal/result/feedback
    /// service and message types ROS2 generates.
    pub fn add_action(&mut self, fqn: &str, body: &str) -> Result<(), RegistryError> {
        let (pkg, _kind, _ty) =
            split_fqn(fqn).ok_or_else(|| RegistryError::UnknownAction(fqn.into()))?;
        let spec = parser::parse_action(fqn, pkg, body)
            .map_err(|e| RegistryError::Parse { name: fqn.into(), source: e })?;
        self.messages.insert(format!("{fqn}_Goal"), spec.goal.clone());
        self.messages.insert(format!("{fqn}_Result"), spec.result.clone());
        self.messages
            .insert(format!("{fqn}_Feedback"), spec.feedback.clone());
        self.actions.insert(fqn.to_string(), spec);
        Ok(())
    }

    /// Look up a message spec (accepts `pkg/Type` or `pkg/msg/Type`).
    pub fn message(&self, name: &str) -> Result<&MessageSpec, RegistryError> {
        let norm = normalize(name, "msg");
        self.messages
            .get(&norm)
            .ok_or(RegistryError::UnknownType(norm))
    }

    pub fn service(&self, name: &str) -> Result<&ServiceSpec, RegistryError> {
        let norm = normalize(name, "srv");
        self.services
            .get(&norm)
            .ok_or(RegistryError::UnknownService(norm))
    }

    pub fn action(&self, name: &str) -> Result<&ActionSpec, RegistryError> {
        let norm = normalize(name, "action");
        self.actions
            .get(&norm)
            .ok_or(RegistryError::UnknownAction(norm))
    }

    pub fn has_message(&self, name: &str) -> bool {
        self.messages.contains_key(&normalize(name, "msg"))
    }

    /// Number of registered message specs (for diagnostics).
    pub fn message_count(&self) -> usize {
        self.messages.len()
    }

    /// Recursively load `.msg`/`.srv`/`.action` files from an ament-style
    /// install tree. Expects `<dir>/<pkg>/msg/*.msg` etc., as found under
    /// `share/` of a ROS2 install or `AMENT_PREFIX_PATH`.
    pub fn load_ament_prefix(&mut self, prefix: &Path) -> Result<usize, RegistryError> {
        let share = prefix.join("share");
        let root = if share.is_dir() { share } else { prefix.to_path_buf() };
        let mut count = 0;
        if !root.is_dir() {
            return Ok(0);
        }
        for pkg_entry in std::fs::read_dir(&root)? {
            let pkg_entry = pkg_entry?;
            if !pkg_entry.file_type()?.is_dir() {
                continue;
            }
            let pkg = pkg_entry.file_name().to_string_lossy().to_string();
            for kind in ["msg", "srv", "action"] {
                let dir = pkg_entry.path().join(kind);
                if !dir.is_dir() {
                    continue;
                }
                for f in std::fs::read_dir(&dir)? {
                    let f = f?;
                    let path = f.path();
                    let ext = path.extension().and_then(|e| e.to_str()).unwrap_or("");
                    if ext != kind {
                        continue;
                    }
                    let stem = path.file_stem().and_then(|s| s.to_str()).unwrap_or("");
                    let fqn = format!("{pkg}/{kind}/{stem}");
                    let body = std::fs::read_to_string(&path)?;
                    let res = match kind {
                        "msg" => self.add_message(&fqn, &body),
                        "srv" => self.add_service(&fqn, &body),
                        "action" => self.add_action(&fqn, &body),
                        _ => unreachable!(),
                    };
                    if res.is_ok() {
                        count += 1;
                    }
                }
            }
        }
        Ok(count)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn standard_types_present() {
        let reg = Registry::with_standard_types();
        assert!(reg.message("std_msgs/msg/String").is_ok());
        assert!(reg.message("std_msgs/String").is_ok());
        assert!(reg.message("geometry_msgs/msg/Twist").is_ok());
        assert!(reg.message("builtin_interfaces/msg/Time").is_ok());
        assert!(reg.service("std_srvs/srv/SetBool").is_ok());
    }
}
