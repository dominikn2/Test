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

/// Reassembles fragmented inbound messages, expiring stale partial groups.
pub struct Defragmenter {
    pending: HashMap<String, Pending>,
    timeout_secs: u64,
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
        }
    }

    /// Feed one fragment. On the final piece, returns the reassembled string.
    pub fn push(&mut self, id: &str, num: usize, total: usize, data: String) -> FragmentOutcome {
        self.sweep(id);
        if total == 0 || num >= total {
            return FragmentOutcome::Invalid("fragment index out of range");
        }
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

        if entry.parts.len() == entry.total {
            let entry = self.pending.remove(id).unwrap();
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
        self.pending.retain(|id, p| {
            id == keep || now.duration_since(p.last_touched).as_secs() < timeout
        });
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
    fn rejects_duplicate() {
        let mut de = Defragmenter::new(600);
        assert!(matches!(de.push("a", 0, 2, "x".into()), FragmentOutcome::Incomplete));
        assert!(matches!(
            de.push("a", 0, 2, "x".into()),
            FragmentOutcome::Invalid(_)
        ));
    }
}
