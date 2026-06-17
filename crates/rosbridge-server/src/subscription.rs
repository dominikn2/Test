//! Per-subscription parameters and the multi-subscription coalescing rules used
//! when one client subscribes to the same topic more than once.

use crate::compression::Compression;

/// Parameters for a single `subscribe` request (one `id`/`sid`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SubParams {
    /// Minimum milliseconds between forwarded messages (0 = unthrottled).
    pub throttle_rate: u64,
    /// Buffer length when throttling (0 = drop instead of queue).
    pub queue_length: usize,
    /// Maximum bytes before fragmenting (None = never fragment).
    pub fragment_size: Option<usize>,
    pub compression: Compression,
}

impl Default for SubParams {
    fn default() -> Self {
        SubParams {
            throttle_rate: 0,
            queue_length: 0,
            fragment_size: None,
            compression: Compression::None,
        }
    }
}

/// Coalesce multiple subscriptions on the same topic to the "lowest common
/// denominator", matching rosbridge `Subscription.update_params`:
/// min throttle_rate, min queue_length, min (non-None) fragment_size, and the
/// highest-precedence compression.
pub fn coalesce<'a>(params: impl IntoIterator<Item = &'a SubParams>) -> SubParams {
    let mut it = params.into_iter();
    let first = match it.next() {
        Some(p) => *p,
        None => return SubParams::default(),
    };
    let mut out = first;
    for p in it {
        out.throttle_rate = out.throttle_rate.min(p.throttle_rate);
        out.queue_length = out.queue_length.min(p.queue_length);
        out.fragment_size = match (out.fragment_size, p.fragment_size) {
            (Some(a), Some(b)) => Some(a.min(b)),
            (Some(a), None) => Some(a),
            (None, Some(b)) => Some(b),
            (None, None) => None,
        };
        if p.compression.rank() > out.compression.rank() {
            out.compression = p.compression;
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn coalesce_picks_lcd() {
        let a = SubParams {
            throttle_rate: 100,
            queue_length: 10,
            fragment_size: Some(1000),
            compression: Compression::None,
        };
        let b = SubParams {
            throttle_rate: 50,
            queue_length: 5,
            fragment_size: None,
            compression: Compression::Cbor,
        };
        let c = coalesce([&a, &b]);
        assert_eq!(c.throttle_rate, 50);
        assert_eq!(c.queue_length, 5);
        assert_eq!(c.fragment_size, Some(1000));
        assert_eq!(c.compression, Compression::Cbor);
    }

    #[test]
    fn coalesce_empty_is_default() {
        let c = coalesce(std::iter::empty());
        assert_eq!(c, SubParams::default());
    }
}
