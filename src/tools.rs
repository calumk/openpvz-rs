//! CLI-oriented inspection, extraction and reverse-engineering utilities.
use crate::binary::hex32;
use crate::constants::DEFAULT_PCURVE_TOLERANCE;
use crate::ol::{extract_ol_surface_meshes, parse_ol_appearances, parse_ol_streams};
use crate::ol_exact::{exact_geometry_summary, parse_ol_exact_geometry, to_occ_intermediate, validate_exact_geometry};
use crate::pvs::parse_pvs_container;
use crate::zip::ZipArchive;
use crate::{Result, openpvz_error};
use std::collections::BTreeMap;

fn inc(map: &mut BTreeMap<String, usize>, key: String, amount: usize) {
    *map.entry(key).or_insert(0) += amount;
}

/// Export one OL tessellation member as OBJ.
pub fn export_ol_obj(pvz_path: &str, ol_member: &str, output: &str) -> Result<serde_json::Value> {
    let zip = ZipArchive::open(pvz_path)?;
    let raw = zip.read(ol_member)?;
    let surfaces = extract_ol_surface_meshes(&raw)?;
    let mut lines = vec![format!("# {ol_member} extracted from Creo View PVZ"), String::new()];
    let (mut base, mut verts, mut tris) = (1usize, 0usize, 0usize);
    for surface in &surfaces {
        for v in &surface.vertices { lines.push(format!("v {} {} {}", v[0], v[1], v[2])); }
        for v in &surface.vertices { lines.push(format!("vn {} {} {}", v[3], v[4], v[5])); }
        for [a, b, c] in &surface.triangles {
            lines.push(format!("f {aa}//{aa} {bb}//{bb} {cc}//{cc}", aa = base + *a as usize, bb = base + *b as usize, cc = base + *c as usize));
        }
        base += surface.vertices.len();
        verts += surface.vertices.len();
        tris += surface.triangles.len();
    }
    std::fs::write(output, format!("{}\n", lines.join("\n")))
        .map_err(|e| openpvz_error(format!("cannot write {output}: {e}")))?;
    Ok(serde_json::json!({ "ol": ol_member, "output": output,
        "surface_blocks": surfaces.len(), "vertices": verts, "triangles": tris }))
}

/// Batch-probe every OL member without requiring PVS references.
pub fn probe_ol_archive(pvz_path: &str, progress: Option<&dyn Fn(usize, usize)>) -> Result<serde_json::Value> {
    let zip = ZipArchive::open(pvz_path)?;
    let ol_members: Vec<String> = zip.names().into_iter().filter(|n| n.to_lowercase().ends_with(".ol")).collect();
    let mut sigs: BTreeMap<String, usize> = BTreeMap::new();
    let mut streams: BTreeMap<String, usize> = BTreeMap::new();
    let mut errors = vec![];
    let (mut blocks, mut verts, mut tris) = (0, 0, 0);
    let mut unpaired = 0usize;
    for (i, name) in ol_members.iter().enumerate() {
        let r: Result<()> = (|| {
            let raw = zip.read(name)?;
            let parsed = parse_ol_streams(&raw)?;
            inc(&mut sigs, parsed.signature, 1);
            for s in &parsed.streams { inc(&mut streams, hex32(s.stream_type), 1); }
            unpaired += parse_ol_appearances(&parsed.streams).map(|(_, u)| u).unwrap_or(0);
            let surfaces = extract_ol_surface_meshes(&raw)?;
            blocks += surfaces.len();
            for s in &surfaces { verts += s.vertices.len(); tris += s.triangles.len(); }
            Ok(())
        })();
        if let Err(e) = r { errors.push(serde_json::json!({ "ol": name, "error": e.to_string() })); }
        if let Some(cb) = progress { cb(i + 1, ol_members.len()); }
    }
    Ok(serde_json::json!({ "ol_files": ol_members.len(), "signatures": sigs,
        "stream_occurrences": streams, "surface_blocks": blocks,
        "vertices": verts, "triangles": tris, "failures": errors.len(), "errors": errors }))
}

/// Container/stream metadata without decoding full geometry.
pub fn inspect_containers(pvz_path: &str, sample_ol: usize) -> Result<serde_json::Value> {
    let zip = ZipArchive::open(pvz_path)?;
    let names = zip.names();
    let pvs_members: Vec<_> = names.iter().filter(|n| n.to_lowercase().ends_with(".pvs")).collect();
    let ol_members: Vec<_> = names.iter().filter(|n| n.to_lowercase().ends_with(".ol")).collect();
    let mut pvs_info = vec![];
    for name in pvs_members {
        let (sig, streams) = parse_pvs_container(&zip.read(name)?)?;
        pvs_info.push(serde_json::json!({ "member": name, "signature": sig,
            "streams": streams.iter().map(|s| serde_json::json!({
                "type": s.stream_type, "name": s.name,
                "decompressed_bytes": s.data.len(), "compressed_bytes": s.compressed.len() })).collect::<Vec<_>>() }));
    }
    let mut samples = vec![];
    for name in ol_members.into_iter().take(sample_ol) {
        let parsed = parse_ol_streams(&zip.read(name)?)?;
        samples.push(serde_json::json!({ "member": name, "signature": parsed.signature,
            "version": parsed.version,
            "streams": parsed.streams.iter().map(|s| serde_json::json!({
                "type": hex32(s.stream_type), "decompressed_bytes": s.data.len() })).collect::<Vec<_>>() }));
    }
    Ok(serde_json::json!({ "pvz": pvz_path, "members": names.len(),
        "pvs": pvs_info, "ol_count": zip.names().iter().filter(|n| n.to_lowercase().ends_with(".ol")).count(),
        "ol_samples": samples }))
}

/// Dump decompressed PVS/selected OL streams for manual binary analysis.
pub fn dump_streams(pvz_path: &str, output_dir: &str, ol_member: Option<&str>) -> Result<serde_json::Value> {
    let zip = ZipArchive::open(pvz_path)?;
    std::fs::create_dir_all(output_dir).map_err(|e| openpvz_error(format!("cannot create {output_dir}: {e}")))?;
    let mut written = vec![];
    for name in zip.names().iter().filter(|n| n.to_lowercase().ends_with(".pvs")) {
        let (_, streams) = parse_pvs_container(&zip.read(name)?)?;
        for (i, s) in streams.iter().enumerate() {
            let base = std::path::Path::new(name).file_name().map(|s| s.to_string_lossy().into_owned()).unwrap_or_else(|| name.clone());
            let filename = format!("{base}.stream_{i}_{}_{}.bin", s.stream_type, if s.name.is_empty() { "unnamed" } else { &s.name })
                .replace('/', "_");
            let path = std::path::Path::new(output_dir).join(filename);
            std::fs::write(&path, &s.data).map_err(|e| openpvz_error(format!("cannot write dump: {e}")))?;
            written.push(path.to_string_lossy().into_owned());
        }
    }
    if let Some(ol) = ol_member {
        let parsed = parse_ol_streams(&zip.read(ol)?)?;
        for (i, s) in parsed.streams.iter().enumerate() {
            let base = std::path::Path::new(ol).file_name().map(|s| s.to_string_lossy().into_owned()).unwrap_or_else(|| ol.to_string());
            let path = std::path::Path::new(output_dir).join(format!("{base}.stream_{i}_{}.bin", hex32(s.stream_type)));
            std::fs::write(&path, &s.data).map_err(|e| openpvz_error(format!("cannot write dump: {e}")))?;
            written.push(path.to_string_lossy().into_owned());
        }
    }
    Ok(serde_json::json!({ "written": written }))
}

pub fn probe_exact_archive(pvz_path: &str, progress: Option<&dyn Fn(usize, usize)>) -> Result<serde_json::Value> {
    let zip = ZipArchive::open(pvz_path)?;
    let ol_members: Vec<String> = zip.names().into_iter().filter(|n| n.to_lowercase().ends_with(".ol")).collect();
    let (mut parsed, mut failures) = (0, 0);
    let (mut faces, mut topo, mut loops, mut coedges, mut singular, mut edges, mut pcurves) = (0, 0, 0, 0, 0, 0, 0);
    let (mut shells, mut closed) = (0, 0);
    let mut surface_types: BTreeMap<String, usize> = BTreeMap::new();
    let mut curve_types: BTreeMap<String, usize> = BTreeMap::new();
    let mut versions: BTreeMap<String, usize> = BTreeMap::new();
    let mut precisions: BTreeMap<String, usize> = BTreeMap::new();
    let mut errors = vec![];
    for (i, name) in ol_members.iter().enumerate() {
        let r: Result<()> = (|| {
            let model = parse_ol_exact_geometry(&zip.read(name)?)?
                .ok_or_else(|| openpvz_error("no recognized exact-geometry stream"))?;
            let s = exact_geometry_summary(&model);
            parsed += 1;
            faces += s.faces; topo += s.topological_faces; loops += s.loops; coedges += s.coedges;
            singular += s.singular_uses; edges += s.edges; pcurves += s.pcurves;
            shells += s.shell_candidates; closed += s.closed_shell_candidates;
            for (k, v) in s.surface_types { inc(&mut surface_types, k, v); }
            for (k, v) in s.curve_types { inc(&mut curve_types, k, v); }
            inc(&mut versions, model.exact_version.to_string(), 1);
            inc(&mut precisions, model.precision.clone(), 1);
            Ok(())
        })();
        if let Err(e) = r {
            failures += 1;
            if errors.len() < 100 { errors.push(serde_json::json!({ "ol": name, "error": e.to_string() })); }
        }
        if let Some(cb) = progress { cb(i + 1, ol_members.len()); }
    }
    Ok(serde_json::json!({ "ol_files": ol_members.len(), "parsed": parsed, "failures": failures,
        "faces": faces, "topological_faces": topo, "loops": loops, "coedges": coedges,
        "singular_uses": singular, "edges": edges, "pcurves": pcurves,
        "shell_candidates": shells, "closed_shell_candidates": closed,
        "surface_types": surface_types, "curve_types": curve_types,
        "exact_versions": versions, "precisions": precisions,
        "errors": errors, "ok": failures == 0 }))
}

pub fn validate_exact_archive(pvz_path: &str, tolerance: f64, progress: Option<&dyn Fn(usize, usize)>) -> Result<serde_json::Value> {
    let zip = ZipArchive::open(pvz_path)?;
    let ol_members: Vec<String> = zip.names().into_iter().filter(|n| n.to_lowercase().ends_with(".ol")).collect();
    let (mut parsed, mut parse_failures) = (0, 0);
    let (mut tested, mut skipped, mut val_failures) = (0, 0, 0);
    let mut max_error: f64 = 0.0;
    let mut surface_types: BTreeMap<String, usize> = BTreeMap::new();
    let mut curve_types: BTreeMap<String, usize> = BTreeMap::new();
    let mut errors = vec![];
    for (i, name) in ol_members.iter().enumerate() {
        let r: Result<()> = (|| {
            let model = parse_ol_exact_geometry(&zip.read(name)?)?
                .ok_or_else(|| openpvz_error("no recognized exact-geometry stream"))?;
            parsed += 1;
            let v = validate_exact_geometry(&model, tolerance);
            tested += v.tested; skipped += v.skipped; val_failures += v.failures;
            max_error = max_error.max(v.max_error);
            for (k, val) in v.surface_types { inc(&mut surface_types, k, val); }
            for (k, val) in v.curve_types { inc(&mut curve_types, k, val); }
            if !v.ok && errors.len() < 100 {
                errors.push(serde_json::json!({ "ol": name, "failures": v.failures,
                    "max_error": v.max_error, "sample": &v.errors[..v.errors.len().min(5)] }));
            }
            Ok(())
        })();
        if let Err(e) = r {
            parse_failures += 1;
            if errors.len() < 100 { errors.push(serde_json::json!({ "ol": name, "error": e.to_string() })); }
        }
        if let Some(cb) = progress { cb(i + 1, ol_members.len()); }
    }
    Ok(serde_json::json!({ "ol_files": ol_members.len(), "parsed": parsed,
        "parse_failures": parse_failures, "tolerance": tolerance,
        "tested_coedges": tested, "skipped_coedges": skipped,
        "validation_failures": val_failures, "max_error": max_error,
        "surface_types": surface_types, "curve_types": curve_types,
        "errors": errors, "ok": parse_failures == 0 && val_failures == 0 }))
}

pub fn export_occ_intermediate(pvz_path: &str, ol_member: &str, output: &str, include_normalized: bool) -> Result<serde_json::Value> {
    let zip = ZipArchive::open(pvz_path)?;
    let model = parse_ol_exact_geometry(&zip.read(ol_member)?)?
        .ok_or_else(|| openpvz_error(format!("{ol_member} has no recognized exact-geometry stream")))?;
    let doc = to_occ_intermediate(&model, include_normalized);
    std::fs::write(output, format!("{}\n", serde_json::to_string_pretty(&doc).unwrap()))
        .map_err(|e| openpvz_error(format!("cannot write {output}: {e}")))?;
    let s = exact_geometry_summary(&model);
    Ok(serde_json::json!({ "ol": ol_member, "output": output,
        "faces": s.faces, "loops": s.loops, "coedges": s.coedges, "edges": s.edges, "pcurves": s.pcurves }))
}

pub fn exact_ol_info(pvz_path: &str, ol_member: &str, validate: bool, tolerance: f64) -> Result<serde_json::Value> {
    let _ = DEFAULT_PCURVE_TOLERANCE;
    let zip = ZipArchive::open(pvz_path)?;
    let model = parse_ol_exact_geometry(&zip.read(ol_member)?)?
        .ok_or_else(|| openpvz_error(format!("{ol_member} has no recognized exact-geometry stream")))?;
    let mut out = serde_json::to_value(exact_geometry_summary(&model)).unwrap();
    if validate {
        out["validation"] = serde_json::to_value(validate_exact_geometry(&model, tolerance)).unwrap();
    }
    out["shells"] = serde_json::to_value(&model.shells).unwrap();
    Ok(out)
}
