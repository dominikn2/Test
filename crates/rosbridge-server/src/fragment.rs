//! Outgoing fragmentation and incoming defragmentation (op `fragment`).

use std::collections::HashMap;
use std::time::Instant;

/// Split a serialized message into `fragment` op JSON strings if it exceeds
/// `fragment_size` (measured in characters, matching rosbridge's string
/// slicing). Returns `None` if no fragmentation is needed.
pub fn fragment(serialized: &str, fragment_size: usize, id: &str) -> Option<Vec<String>> {
    let chars: Vec<char> = serialized.chars().collect();
    if fragment_size == 0 || chars.len() <= fragment_size {
        return None;
    }
    let total = chars.len().div_ceil(fragment_size);
    let mut out = Vec::with_capacity(total);
    for (num, chunk) in chars.chunks(fragment_size).enumerate() {
        let data: String = chunk.iter().collect();
        let frag = serde_json::json!({
            "op": "fragment",
            "id": id,
            "data": data,
            "num": num,
            "total": total,
        });
        out.push(serde_json::to_string(&frag).unwrap());
    }
    Some(out)
}

/// State for a single in-progress reassembly.
struct Pending {
    total: usize,
    parts: HashMap<usize, String>,
    last_touched: Instant,
}

/// Hard limits guarding against malicious fragment streams.
const MAX_PENDING_GROUPS: usize = 1024;
const MAX_TOTAL_FRAGMENTS: usize = 1_000_000;
const MAX_BUFFERED_BYTES: usize = 256 * 1024 * 1024;

/// Reassembles fragmented inbound messages, expiring stale partial groups.
pub struct Defragmenter {
    pending: HashMap<String, Pending>,
    timeout_secs: u64,
    buffered_bytes: usize,
}

/// Result of feeding a fragment.
pub enum FragmentOutcome {
    /// More fragments needed.
    Incomplete,
    /// The full message was reconstructed.
    Complete(String),
    /// The fragment was invalid (duplicate/out-of-range/mismatched total).
    Invalid(&'static str),
}

impl Defragmenter {
    pub fn new(timeout_secs: u64) -> Self {
        Defragmenter {
            pending: HashMap::new(),
            timeout_secs,
            buffered_bytes: 0,
        }
    }

    /// Feed one fragment. On the final piece, returns the reassembled string.
    pub fn push(&mut self, id: &str, num: usize, total: usize, data: String) -> FragmentOutcome {
        self.sweep(id);
        if total == 0 || num >= total {
            return FragmentOutcome::Invalid("fragment index out of range");
        }
        if total > MAX_TOTAL_FRAGMENTS {
            return FragmentOutcome::Invalid("fragment total too large");
        }
        // Reject brand-new groups once we are tracking too many or buffering
        // too much (defends against memory-exhaustion via distinct ids).
        let is_new = !self.pending.contains_key(id);
        if is_new && self.pending.len() >= MAX_PENDING_GROUPS {
            return FragmentOutcome::Invalid("too many concurrent fragment groups");
        }
        if self.buffered_bytes.saturating_add(data.len()) > MAX_BUFFERED_BYTES {
            return FragmentOutcome::Invalid("fragment buffer limit exceeded");
        }
        let data_len = data.len();
        let entry = self.pending.entry(id.to_string()).or_insert_with(|| Pending {
            total,
            parts: HashMap::new(),
            last_touched: Instant::now(),
        });
        if entry.total != total {
            return FragmentOutcome::Invalid("inconsistent fragment total");
        }
        if entry.parts.contains_key(&num) {
            return FragmentOutcome::Invalid("duplicate fragment");
        }
        entry.parts.insert(num, data);
        entry.last_touched = Instant::now();
        self.buffered_bytes += data_len;

        if entry.parts.len() == entry.total {
            let entry = self.pending.remove(id).unwrap();
            let freed: usize = entry.parts.values().map(|p| p.len()).sum();
            self.buffered_bytes = self.buffered_bytes.saturating_sub(freed);
            let mut reconstructed = String::new();
            for i in 0..entry.total {
                match entry.parts.get(&i) {
                    Some(p) => reconstructed.push_str(p),
                    None => return FragmentOutcome::Invalid("missing fragment at completion"),
                }
            }
            FragmentOutcome::Complete(reconstructed)
        } else {
            FragmentOutcome::Incomplete
        }
    }

    /// Drop partial groups older than the timeout (except `keep`, the group
    /// currently being appended to).
    fn sweep(&mut self, keep: &str) {
        let timeout = self.timeout_secs;
        let now = Instant::now();
        let mut freed = 0usize;
        self.pending.retain(|id, p| {
            let live = id == keep || now.duration_since(p.last_touched).as_secs() < timeout;
            if !live {
                freed += p.parts.values().map(|s| s.len()).sum::<usize>();
            }
            live
        });
        self.buffered_bytes = self.buffered_bytes.saturating_sub(freed);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_fragmentation_when_small() {
        assert!(fragment("hello", 10, "x").is_none());
    }

    #[test]
    fn fragments_and_reassembles() {
        let msg = "abcdefghij"; // 10 chars
        let frags = fragment(msg, 3, "id1").unwrap();
        assert_eq!(frags.len(), 4); // 3+3+3+1
        let mut de = Defragmenter::new(600);
        let mut result = None;
        for f in &frags {
            let v: serde_json::Value = serde_json::from_str(f).unwrap();
            let outcome = de.push(
                v["id"].as_str().unwrap(),
                v["num"].as_u64().unwrap() as usize,
                v["total"].as_u64().unwrap() as usize,
                v["data"].as_str().unwrap().to_string(),
            );
            if let FragmentOutcome::Complete(s) = outcome {
                result = Some(s);
            }
        }
        assert_eq!(result.as_deref(), Some("abcdefghij"));
    }

    #[test]
    fn rejects_oversized_total() {
        let mut de = Defragmenter::new(600);
        assert!(matches!(
            de.push("a", 0, usize::MAX, "x".into()),
            FragmentOutcome::Invalid(_)
        ));
    }

    #[test]
    fn caps_concurrent_groups() {
        let mut de = Defragmenter::new(600);
        // Open the maximum number of distinct partial groups.
        for i in 0..MAX_PENDING_GROUPS {
            assert!(matches!(
                de.push(&format!("g{i}"), 0, 2, "x".into()),
                FragmentOutcome::Incomplete
            ));
        }
        // One more brand-new group must be rejected.
        assert!(matches!(
            de.push("overflow", 0, 2, "x".into()),
            FragmentOutcome::Invalid(_)
        ));
    }

    #[test]
    fn rejects_duplicate() {
        let mut de = Defragmenter::new(600);
        assert!(matches!(de.push("a", 0, 2, "x".into()), FragmentOutcome::Incomplete));
        assert!(matches!(
            de.push("a", 0, 2, "x".into()),
            FragmentOutcome::Invalid(_)
        ));
    }
}
