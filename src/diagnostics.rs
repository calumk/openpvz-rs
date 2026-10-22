//! Structural and numerical validation helpers for PVZ/PVS/OL corpora.
use crate::binary::hex32;
use crate::constants::{pvs_placement, DEFAULT_PCURVE_TOLERANCE};
use crate::ol::{exact_geometry_catalog, extract_ol_position_bounds, extract_ol_surface_meshes, parse_ol_appearances, parse_ol_streams};
use crate::pvz::PvzArchive;
use crate::Result;
use std::collections::BTreeMap;

fn bbox_error(a: &[f64; 6], b: &[f64; 6]) -> f64 {
    a.iter().zip(b.iter()).map(|(x, y)| (x - y).abs()).fold(0.0, f64::max)
}

fn rotation_metrics(rotation: &[f64; 9]) -> (f64, f64) {
    let b = [[rotation[0],rotation[1],rotation[2]],[rotation[3],rotation[4],rotation[5]],[rotation[6],rotation[7],rotation[8]]];
    let mut err: f64 = 0.0;
    for i in 0..3 { for j in 0..3 {
        let d = b[i][0]*b[j][0]+b[i][1]*b[j][1]+b[i][2]*b[j][2];
        err = err.max((d - if i==j {1.0} else {0.0}).abs());
    }}
    let det = rotation[0]*(rotation[4]*rotation[8]-rotation[5]*rotation[7])
        - rotation[1]*(rotation[3]*rotation[8]-rotation[5]*rotation[6])
        + rotation[2]*(rotation[3]*rotation[7]-rotation[4]*rotation[6]);
    (err, det)
}

#[derive(Debug, serde::Serialize)]
pub struct ModelStats {
    pub pvs_signature: String,
    pub pvs_member: String,
    pub definitions: usize,
    pub part_definitions: usize,
    pub assembly_definitions: usize,
    pub definitions_with_ol: usize,
    pub assembly_links: usize,
    pub root_index: usize,
    pub root_name: String,
    pub occurrences: usize,
    pub part_occurrences: usize,
    pub max_depth: usize,
    pub missing_ol_members: Vec<String>,
}

/// High-level hierarchy and member statistics.
pub fn model_stats(archive: &PvzArchive) -> Result<ModelStats> {
    let definitions = &archive.pvs.definitions;
    let mut occurrences: Vec<(usize, usize)> = vec![];
    fn walk(definitions: &[crate::pvs::Definition], idx: usize, depth: usize, stack: &mut Vec<usize>, out: &mut Vec<(usize, usize)>) -> Result<()> {
        if stack.contains(&idx) { return Err("cycle in PVS assembly graph".into()); }
        out.push((idx, depth));
        stack.push(idx);
        for child in &definitions[idx].children {
            walk(definitions, child.definition_index, depth + 1, stack, out)?;
        }
        stack.pop();
        Ok(())
    }
    walk(definitions, archive.pvs.root_index, 0, &mut vec![], &mut occurrences)?;
    Ok(ModelStats {
        pvs_signature: archive.pvs.signature.clone(),
        pvs_member: archive.pvs_member.clone(),
        definitions: definitions.len(),
        part_definitions: definitions.iter().filter(|d| d.kind == "part").count(),
        assembly_definitions: definitions.iter().filter(|d| d.kind == "assembly").count(),
        definitions_with_ol: definitions.iter().filter(|d| d.ol_name.is_some()).count(),
        assembly_links: definitions.iter().map(|d| d.children.len()).sum(),
        root_index: archive.pvs.root_index,
        root_name: definitions[archive.pvs.root_index].source_name.clone(),
        occurrences: occurrences.len(),
        part_occurrences: occurrences.iter().filter(|(i, _)| definitions[*i].kind == "part").count(),
        max_depth: occurrences.iter().map(|(_, d)| *d).max().unwrap_or(0),
        missing_ol_members: archive.validate_members(),
    })
}

fn inc(map: &mut BTreeMap<String, usize>, key: String, amount: usize) {
    *map.entry(key).or_insert(0) += amount;
}

/// Cross-check PVS metadata, OL tessellation and exact-stream catalogues.
pub fn validate_archive(
    archive: &PvzArchive,
    bbox_tolerance: f64,
    progress: Option<&dyn Fn(usize, usize)>,
) -> serde_json::Value {
    let definitions = &archive.pvs.definitions;
    let mut ol_sigs: BTreeMap<String, usize> = BTreeMap::new();
    let mut ol_streams: BTreeMap<String, usize> = BTreeMap::new();
    let mut exact_tags: BTreeMap<String, usize> = BTreeMap::new();
    let mut surf_pre: BTreeMap<String, usize> = BTreeMap::new();
    let mut edge_pre: BTreeMap<String, usize> = BTreeMap::new();
    let mut bbox_mismatches = vec![];
    let mut ol_errors = vec![];
    let (mut surface_blocks, mut vertices, mut triangles) = (0, 0, 0);
    let mut unpaired_appearances = 0usize;
    let (mut winding_good, mut winding_bad, mut winding_degen) = (0, 0, 0);

    let geometry: Vec<_> = definitions.iter().filter(|d| d.ol_name.is_some()).collect();
    for (n, definition) in geometry.iter().enumerate() {
        let result: Result<()> = (|| {
            let raw = archive.read_ol(definition.ol_name.as_ref().unwrap())?;
            let parsed = parse_ol_streams(&raw)?;
            inc(&mut ol_sigs, parsed.signature.clone(), 1);
            for s in &parsed.streams { inc(&mut ol_streams, hex32(s.stream_type), 1); }
            if let (Some(pvs_bbox), Some(ol_bounds)) = (&definition.bbox, extract_ol_position_bounds(&raw)?) {
                let err = bbox_error(pvs_bbox, &ol_bounds);
                if err > bbox_tolerance {
                    bbox_mismatches.push(serde_json::json!({
                        "definition": definition.index, "source": definition.source_name,
                        "ol": definition.ol_name, "max_abs_error": err,
                        "pvs_bbox": definition.bbox, "ol_vertex_bbox": ol_bounds }));
                }
            }
            unpaired_appearances += parse_ol_appearances(&parsed.streams).map(|(_, u)| u).unwrap_or(0);
            let surfaces = extract_ol_surface_meshes(&raw)?;
            surface_blocks += surfaces.len();
            for surface in &surfaces {
                vertices += surface.vertices.len();
                triangles += surface.triangles.len();
                for [a, b, c] in &surface.triangles {
                    let (pa, pb, pc) = (&surface.vertices[*a as usize], &surface.vertices[*b as usize], &surface.vertices[*c as usize]);
                    let u = [pb[0]-pa[0], pb[1]-pa[1], pb[2]-pa[2]];
                    let v = [pc[0]-pa[0], pc[1]-pa[1], pc[2]-pa[2]];
                    let n = [u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0]];
                    let area2 = n[0]*n[0]+n[1]*n[1]+n[2]*n[2];
                    if area2 < 1e-24 { winding_degen += 1; continue; }
                    let s = [pa[3]+pb[3]+pc[3], pa[4]+pb[4]+pc[4], pa[5]+pb[5]+pc[5]];
                    if n[0]*s[0]+n[1]*s[1]+n[2]*s[2] >= 0.0 { winding_good += 1; } else { winding_bad += 1; }
                }
            }
            let catalog = exact_geometry_catalog(&raw)?;
            for (k, v) in catalog.tag_counts { inc(&mut exact_tags, k, v); }
            for (k, v) in catalog.tag_0x13_prefixes { inc(&mut surf_pre, k, v); }
            for (k, v) in catalog.tag_0x14_prefixes { inc(&mut edge_pre, k, v); }
            Ok(())
        })();
        if let Err(e) = result {
            ol_errors.push(serde_json::json!({
                "definition": definition.index, "source": definition.source_name,
                "ol": definition.ol_name, "error": e.to_string() }));
        }
        if let Some(cb) = progress { cb(n + 1, geometry.len()); }
    }

    let mut placement_counts: BTreeMap<String, usize> = BTreeMap::new();
    let mut flag_counts: BTreeMap<String, usize> = BTreeMap::new();
    let mut unknown_blocks: BTreeMap<String, usize> = BTreeMap::new();
    let (mut max_orth, mut min_det, mut max_det, mut full_count): (f64, f64, f64, usize) = (0.0, f64::INFINITY, f64::NEG_INFINITY, 0);
    for definition in definitions {
        for child in &definition.children {
            inc(&mut placement_counts, format!("0x{:02x}", child.placement_kind), 1);
            if child.placement_flags != 0 {
                inc(&mut flag_counts, format!("0x{:02x}", child.placement_flags), 1);
            }
            if child.placement_encoding == pvs_placement::FULL {
                let (o, d) = rotation_metrics(&child.placement.rotation);
                max_orth = max_orth.max(o);
                min_det = min_det.min(d);
                max_det = max_det.max(d);
                full_count += 1;
            }
        }
        for block in &definition.extra_blocks {
            if block.len() >= 2 {
                inc(&mut unknown_blocks, format!("{:02x}{:02x}", block[0], block[1]), 1);
            }
        }
    }
    let nondegen = winding_good + winding_bad;
    let missing = archive.validate_members();
    let structure = model_stats(archive).ok().and_then(|s| serde_json::to_value(s).ok()).unwrap_or(serde_json::Value::Null);
    serde_json::json!({
        "format": { "pvs_signature": archive.pvs.signature, "ol_signatures": ol_sigs, "ol_stream_occurrences": ol_streams },
        "structure": structure,
        "geometry": {
            "surface_blocks": surface_blocks, "vertices": vertices, "triangles": triangles,
            "triangle_winding": {
                "agree_with_stored_normals": winding_good,
                "disagree_with_stored_normals": winding_bad,
                "degenerate": winding_degen,
                "agreement_fraction_nondegenerate": if nondegen > 0 { Some(winding_good as f64 / nondegen as f64) } else { None },
            },
            "bbox_tolerance": bbox_tolerance,
            "bbox_mismatch_count": bbox_mismatches.len(),
            "bbox_mismatches": bbox_mismatches,
        },
        "placements": {
            "kind_counts": placement_counts, "high_flag_counts": flag_counts,
            "full_rotation_count": full_count,
            "max_orthonormality_error": if full_count > 0 { Some(max_orth) } else { None },
            "determinant_range": if full_count > 0 { Some(vec![min_det, max_det]) } else { None },
        },
        "unknown_pvs_subrecords": unknown_blocks,
        "exact_geometry_catalog": { "tag_counts": exact_tags, "tag_0x13_prefixes": surf_pre, "tag_0x14_prefixes": edge_pre },
        "ol_error_count": ol_errors.len(), "ol_errors": ol_errors,
        "unpaired_appearance_value_records": unpaired_appearances,
        "ok": ol_errors.is_empty() && bbox_mismatches.is_empty() && missing.is_empty(),
    })
}

/// Print the occurrence tree.
pub fn print_tree(archive: &PvzArchive, max_depth: Option<usize>) {
    fn visit(archive: &PvzArchive, idx: usize, indent: &str, feature: Option<&str>, placement: Option<&crate::pvs::Placement>, depth: usize, max_depth: Option<usize>) {
        let d = &archive.pvs.definitions[idx];
        let mut suffix = String::new();
        if let Some(f) = feature { suffix.push_str(&format!(" feature={f}")); }
        if let Some(p) = placement {
            if !p.is_identity() {
                suffix.push_str(&format!(" t=({:.6},{:.6},{:.6})", p.translation[0], p.translation[1], p.translation[2]));
            }
        }
        let geo = d.ol_name.as_ref().map(|o| format!(" ol={o}")).unwrap_or_default();
        println!("{indent}[{idx}] {}{}{suffix}", d.source_name, geo);
        if max_depth.map(|m| depth >= m).unwrap_or(false) { return; }
        for child in &d.children {
            visit(archive, child.definition_index, &format!("{indent}  "), Some(&child.feature_id), Some(&child.placement), depth + 1, max_depth);
        }
    }
    visit(archive, archive.pvs.root_index, "", None, None, 0, max_depth);
}

pub fn default_bbox_tolerance() -> f64 { DEFAULT_PCURVE_TOLERANCE }
