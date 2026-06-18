//! Server configuration mirroring rosbridge_server (ros2 branch) parameters.
//!
//! Names, types, and defaults match the ROS node parameters so this server is a
//! drop-in replacement. See `rosbridge_websocket.py` SERVER_PARAMETERS and
//! PROTOCOL_PARAMETERS.

use std::sync::Arc;

/// Parsed glob list. `None` means "no filter, allow all"; `Some([])` means
/// "deny everything"; otherwise a list of `fnmatch`-style patterns.
pub type GlobList = Option<Vec<String>>;

/// Parse a glob string in the rosbridge format:
/// * `""` -> `None` (allow all)
/// * `"[]"` -> `Some([])` (deny all)
/// * `"['/a', '/b']"` -> `Some(["/a","/b"])`
pub fn parse_glob_string(s: &str) -> GlobList {
    let t = s.trim();
    if t.is_empty() {
        return None;
    }
    let inner = t.trim_start_matches('[').trim_end_matches(']').trim();
    if inner.is_empty() {
        return Some(Vec::new());
    }
    let items = inner
        .split(',')
        .map(|p| p.trim().trim_matches(|c| c == '\'' || c == '"').trim().to_string())
        .filter(|p| !p.is_empty())
        .collect();
    Some(items)
}

/// Full server configuration.
#[derive(Debug, Clone)]
pub struct Config {
    // --- server params ---
    pub port: u16,
    pub address: String,
    pub url_path: String,
    pub retry_startup_delay: f64,
    pub certfile: String,
    pub keyfile: String,
    pub websocket_ping_interval: f64,
    pub websocket_ping_timeout: f64,
    pub use_compression: bool,

    // --- protocol params ---
    pub fragment_timeout: u64,
    pub delay_between_messages: f64,
    pub max_message_size: usize,
    pub unregister_timeout: f64,
    pub topics_glob: GlobList,
    pub topics_pub_glob: GlobList,
    pub topics_sub_glob: GlobList,
    pub services_glob: GlobList,
    pub actions_glob: GlobList,
    pub call_services_in_new_thread: bool,
    pub default_call_service_timeout: f64,
    pub send_action_goals_in_new_thread: bool,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            port: 9090,
            address: String::new(),
            url_path: "/".to_string(),
            retry_startup_delay: 2.0,
            certfile: String::new(),
            keyfile: String::new(),
            websocket_ping_interval: 0.0,
            websocket_ping_timeout: 30.0,
            use_compression: false,
            fragment_timeout: 600,
            delay_between_messages: 0.0,
            max_message_size: 1_000_000,
            unregister_timeout: 10.0,
            topics_glob: None,
            topics_pub_glob: None,
            topics_sub_glob: None,
            services_glob: None,
            actions_glob: None,
            call_services_in_new_thread: true,
            default_call_service_timeout: 5.0,
            send_action_goals_in_new_thread: true,
        }
    }
}

impl Config {
    /// Merge the legacy `topics_glob` into both pub and sub globs, and append
    /// `/rosapi/*` to a non-None `services_glob`, matching node startup logic.
    pub fn finalize_globs(&mut self) {
        if let Some(legacy) = &self.topics_glob {
            merge_glob(&mut self.topics_pub_glob, legacy);
            merge_glob(&mut self.topics_sub_glob, legacy);
        }
        if let Some(svc) = &mut self.services_glob {
            let rosapi = "/rosapi/*".to_string();
            if !svc.contains(&rosapi) {
                svc.push(rosapi);
            }
        }
    }

    /// True when SSL should be enabled (both cert and key set).
    pub fn ssl_enabled(&self) -> bool {
        !self.certfile.is_empty() && !self.keyfile.is_empty()
    }
}

fn merge_glob(target: &mut GlobList, extra: &[String]) {
    match target {
        Some(list) => {
            for e in extra {
                if !list.contains(e) {
                    list.push(e.clone());
                }
            }
        }
        None => *target = Some(extra.to_vec()),
    }
}

/// Shared, read-only handle to the configuration.
pub type SharedConfig = Arc<Config>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn glob_parsing() {
        assert_eq!(parse_glob_string(""), None);
        assert_eq!(parse_glob_string("[]"), Some(vec![]));
        assert_eq!(
            parse_glob_string("['/foo/*', \"/bar\"]"),
            Some(vec!["/foo/*".to_string(), "/bar".to_string()])
        );
    }

    #[test]
    fn finalize_merges_legacy_and_rosapi() {
        let mut c = Config {
            topics_glob: Some(vec!["/a".into()]),
            topics_pub_glob: Some(vec!["/b".into()]),
            services_glob: Some(vec!["/svc".into()]),
            ..Default::default()
        };
        c.finalize_globs();
        assert!(c.topics_pub_glob.as_ref().unwrap().contains(&"/a".to_string()));
        assert!(c.topics_sub_glob.as_ref().unwrap().contains(&"/a".to_string()));
        assert!(c.services_glob.as_ref().unwrap().contains(&"/rosapi/*".to_string()));
    }

    #[test]
    fn defaults_match_parity() {
        let c = Config::default();
        assert_eq!(c.port, 9090);
        assert_eq!(c.max_message_size, 1_000_000);
        assert_eq!(c.default_call_service_timeout, 5.0);
        assert!(c.call_services_in_new_thread);
    }
}
