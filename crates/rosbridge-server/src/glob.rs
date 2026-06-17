//! Glob-based security filtering with `fnmatch` semantics (matching rosbridge's
//! use of Python `fnmatch.fnmatch`).
//!
//! In `fnmatch`, `*` matches any sequence (including `/`), `?` matches a single
//! character, and `[seq]`/`[!seq]` are character classes.

use crate::config::GlobList;

/// True if `name` matches `pattern` using `fnmatch` rules.
pub fn fnmatch(name: &str, pattern: &str) -> bool {
    let n: Vec<char> = name.chars().collect();
    let p: Vec<char> = pattern.chars().collect();
    matches_from(&n, 0, &p, 0)
}

fn matches_from(n: &[char], mut ni: usize, p: &[char], mut pi: usize) -> bool {
    // Iterative with backtracking for '*'.
    let mut star_pi: Option<usize> = None;
    let mut star_ni = 0;
    while ni < n.len() {
        if pi < p.len() {
            match p[pi] {
                '*' => {
                    star_pi = Some(pi);
                    star_ni = ni;
                    pi += 1;
                    continue;
                }
                '?' => {
                    pi += 1;
                    ni += 1;
                    continue;
                }
                '[' => {
                    if let Some((matched, next_pi)) = match_class(n[ni], p, pi) {
                        if matched {
                            pi = next_pi;
                            ni += 1;
                            continue;
                        }
                    } else {
                        // Malformed class: treat '[' literally.
                        if p[pi] == n[ni] {
                            pi += 1;
                            ni += 1;
                            continue;
                        }
                    }
                }
                c => {
                    if c == n[ni] {
                        pi += 1;
                        ni += 1;
                        continue;
                    }
                }
            }
        }
        // Mismatch: backtrack to last '*' if any.
        if let Some(sp) = star_pi {
            pi = sp + 1;
            star_ni += 1;
            ni = star_ni;
        } else {
            return false;
        }
    }
    // Consume trailing '*'.
    while pi < p.len() && p[pi] == '*' {
        pi += 1;
    }
    pi == p.len()
}

/// Match a `[...]` character class starting at `p[pi] == '['`. Returns
/// `(matched, index_after_class)` or `None` if malformed.
fn match_class(c: char, p: &[char], pi: usize) -> Option<(bool, usize)> {
    let mut i = pi + 1;
    let negate = p.get(i) == Some(&'!');
    if negate {
        i += 1;
    }
    let mut matched = false;
    let start = i;
    while i < p.len() {
        if p[i] == ']' && i > start {
            return Some((matched ^ negate, i + 1));
        }
        // Range a-b
        if i + 2 < p.len() && p[i + 1] == '-' && p[i + 2] != ']' {
            if c >= p[i] && c <= p[i + 2] {
                matched = true;
            }
            i += 3;
        } else {
            if p[i] == c {
                matched = true;
            }
            i += 1;
        }
    }
    None
}

/// Apply a glob list to `name`. `None` allows everything; an empty list denies
/// everything; otherwise the name must match at least one pattern.
pub fn allowed(globs: &GlobList, name: &str) -> bool {
    match globs {
        None => true,
        Some(list) => list.iter().any(|g| fnmatch(name, g)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn star_matches_slashes() {
        assert!(fnmatch("/rosapi/topics", "/rosapi/*"));
        assert!(fnmatch("/a/b/c", "/a/*"));
        assert!(fnmatch("anything", "*"));
    }

    #[test]
    fn exact_and_question() {
        assert!(fnmatch("/foo", "/foo"));
        assert!(!fnmatch("/foo", "/bar"));
        assert!(fnmatch("/foo", "/fo?"));
        assert!(!fnmatch("/foo", "/fo"));
    }

    #[test]
    fn char_classes() {
        assert!(fnmatch("/cam1", "/cam[0-9]"));
        assert!(!fnmatch("/camX", "/cam[0-9]"));
        assert!(fnmatch("/camX", "/cam[!0-9]"));
    }

    #[test]
    fn allowed_semantics() {
        assert!(allowed(&None, "/anything"));
        assert!(!allowed(&Some(vec![]), "/anything"));
        assert!(allowed(&Some(vec!["/ok/*".into()]), "/ok/x"));
        assert!(!allowed(&Some(vec!["/ok/*".into()]), "/no"));
    }
}
