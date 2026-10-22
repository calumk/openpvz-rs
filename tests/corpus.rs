//! Optional corpus regression tests against local PVZ samples.
//!
//! Sample files are not distributed with this crate. To run these tests, put
//! `.pvz` files in `test_files/` together with a `test_files/corpus.json`
//! manifest that lists each file and the values it must produce (see
//! `tests/corpus.example.json`). Without the manifest the tests pass
//! without checking anything.
//!
//! Entries marked `"heavy": true` only run with `cargo test -- --ignored`.
use openpvz::{diagnostics, export_glb, export_step, tools, PvzArchive};
use serde_json::{json, Value};
use std::path::{Path, PathBuf};

fn test_files() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("test_files")
}

/// Load the local manifest; `None` when no corpus is installed.
fn manifest() -> Option<Vec<Value>> {
    let path = test_files().join("corpus.json");
    let text = std::fs::read_to_string(&path).ok()?;
    let doc: Value = serde_json::from_str(&text)
        .unwrap_or_else(|e| panic!("invalid {}: {e}", path.display()));
    Some(doc["files"].as_array().expect("corpus.json needs a \"files\" array").clone())
}

fn entries(heavy: bool) -> Vec<(String, Value)> {
    let Some(files) = manifest() else {
        eprintln!("test_files/corpus.json not found; corpus tests skipped");
        return vec![];
    };
    files
        .into_iter()
        .filter(|e| e["heavy"].as_bool().unwrap_or(false) == heavy)
        .map(|e| {
            let file = e["file"].as_str().expect("entry needs \"file\"");
            let path = test_files().join(file);
            assert!(path.exists(), "corpus file missing: {}", path.display());
            (path.to_string_lossy().into_owned(), e)
        })
        .collect()
}

/// Every key in `expected` must equal the same key in `actual`.
fn assert_subset(context: &str, actual: &Value, expected: &Value) {
    let Some(map) = expected.as_object() else { return };
    for (key, want) in map {
        assert_eq!(&actual[key], want, "{context}: field `{key}`");
    }
}

fn tempdir(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("openpvz-tests-{}-{name}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

fn glb_json(path: &Path) -> Value {
    let data = std::fs::read(path).unwrap();
    assert_eq!(&data[0..4], b"glTF");
    let json_len = u32::from_le_bytes(data[12..16].try_into().unwrap()) as usize;
    serde_json::from_slice(data[20..20 + json_len].trim_ascii_end()).unwrap()
}

fn step_data_section(path: &Path) -> Vec<u8> {
    let data = std::fs::read(path).unwrap();
    let marker = b"\nDATA;\n";
    let pos = data.windows(marker.len()).position(|w| w == marker).unwrap() + marker.len();
    data[pos..].to_vec()
}

fn check_entry(path: &str, e: &Value) {
    let name = e["file"].as_str().unwrap();
    let archive = PvzArchive::open(path).unwrap();

    let stats = serde_json::to_value(diagnostics::model_stats(&archive).unwrap()).unwrap();
    assert!(stats["missing_ol_members"].as_array().unwrap().is_empty(), "{name}: missing OL members");
    assert_subset(&format!("{name} info"), &stats, &e["info"]);

    if !e["validate"].is_null() {
        let report = diagnostics::validate_archive(&archive, 2e-5, None);
        assert_subset(&format!("{name} validate"), &report, &json!({ "ok": e["validate"]["ok"] }));
        assert_subset(&format!("{name} validate.geometry"), &report["geometry"], &e["validate"]["geometry"]);
    }

    if let Some(spots) = e["exact"].as_array() {
        for spot in spots {
            let member = spot["member"].as_str().expect("exact spot needs \"member\"");
            let info = tools::exact_ol_info(path, member, false, 2e-5).unwrap();
            assert_subset(&format!("{name} exact {member}"), &info, &spot["expect"]);
        }
    }

    if !e["glb"].is_null() {
        let dir = tempdir("glb");
        let out = dir.join("out.glb");
        let glb = export_glb(&archive, out.to_str().unwrap(), false, None).unwrap();
        assert_subset(&format!("{name} glb stats"), &serde_json::to_value(&glb).unwrap(), &e["glb"]["stats"]);
        let doc = glb_json(&out);
        for key in ["meshes", "nodes", "materials"] {
            if let Some(want) = e["glb"][key].as_u64() {
                assert_eq!(doc[key].as_array().unwrap().len() as u64, want, "{name} glb {key}");
            }
        }
        std::fs::remove_dir_all(&dir).ok();
    }

    if !e["step"].is_null() {
        let dir = tempdir("step");
        let a = dir.join("a.step");
        let totals = export_step(&archive, a.to_str().unwrap(), 2e-5, 5e-6, 1e-6, None).unwrap();
        assert!(totals.failed_parts.is_empty(), "{name}: failed parts {:?}", totals.failed_parts);
        assert_subset(&format!("{name} step totals"), &serde_json::to_value(&totals).unwrap(), &e["step"]["totals"]);

        let text = std::fs::read_to_string(&a).unwrap();
        assert!(text.starts_with("ISO-10303-21;"));
        assert!(text.trim_end().ends_with("END-ISO-10303-21;"));
        assert_eq!(text.matches("PCURVE(").count(), totals.emitted_pcurves, "{name}: PCURVE count");
        if let Some(counts) = e["step"]["entity_counts"].as_object() {
            for (entity, want) in counts {
                let got = text.matches(&format!("{entity}(")).count() as u64;
                assert_eq!(Some(got), want.as_u64(), "{name}: {entity} count");
            }
        }
        // entity ids are dense from 1
        let mut ids: Vec<usize> = text
            .lines()
            .filter_map(|line| line.strip_prefix('#')?.split_once(' ').and_then(|(n, _)| n.parse().ok()))
            .collect();
        ids.sort_unstable();
        assert_eq!(ids.len(), totals.entities, "{name}: entity count");
        assert_eq!(ids.last().copied().unwrap_or(0), totals.entities, "{name}: entity ids not dense");

        // DATA section is byte-deterministic across runs (header holds a timestamp)
        let b = dir.join("b.step");
        let archive = PvzArchive::open(path).unwrap();
        export_step(&archive, b.to_str().unwrap(), 2e-5, 5e-6, 1e-6, None).unwrap();
        assert!(step_data_section(&a) == step_data_section(&b), "{name}: STEP DATA not deterministic");
        std::fs::remove_dir_all(&dir).ok();
    }
}

fn check_heavy_entry(path: &str, e: &Value) {
    let name = e["file"].as_str().unwrap();
    if !e["probe_exact"].is_null() {
        let report = tools::probe_exact_archive(path, None).unwrap();
        assert_subset(&format!("{name} probe-exact"), &report, &e["probe_exact"]);
    }
    if !e["validate_exact"].is_null() {
        let report = tools::validate_exact_archive(path, 2e-5, None).unwrap();
        assert_subset(&format!("{name} validate-exact"), &report, &e["validate_exact"]);
    }
    if !e["info"].is_null() || !e["validate"].is_null() {
        check_entry(path, e);
    }
}

#[test]
fn corpus_fast() {
    for (path, e) in entries(false) {
        check_entry(&path, &e);
    }
}

#[test]
#[ignore] // large corpora; run with: cargo test -- --ignored
fn corpus_heavy() {
    for (path, e) in entries(true) {
        check_heavy_entry(&path, &e);
    }
}
