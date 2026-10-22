//! Exact-part B-rep emission and AP214 assembly export.
//! Pipeline stages: implicit-trim materialization,
//! p-curve preparation, region selection, edge/face/shell emission,
//! appearance presentation, product/assembly structure.
use crate::constants::DEFAULT_IMPLICIT_EDGE_COLLAPSE_TOLERANCE;
use crate::ol::{extract_ol_surface_meshes, Appearance};
use crate::ol_exact::{
    build_shell_candidates, classify_exact_coedge, classify_exact_loop,
    evaluate_exact_surface, parse_ol_exact_geometry,
    ExactCurve, ExactModel,
};
use crate::step_geom::{
    closest_exact_curve_parameter, curve_entity_3d, map_native_pcurve_to_step, pcurve_entity,
    placement_axis, refine_surface_uv_to_point, surface_entity,
};
use crate::step_text::{encode_step_string, fnum, refs, step_appearance_style, step_bool, StepWriter};
use crate::{Result, openpvz_error};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, HashMap, HashSet};

fn sha256_hex(data: &[u8]) -> String {
    let mut h = Sha256::new();
    h.update(data);
    format!("{:x}", h.finalize())
}

// ---------- STEP annotations (side tables; the model stays immutable) ----------

#[derive(Default)]
struct StepAnnot {
    traversal: HashMap<(usize, usize), String>, // (face, loop) -> stored|reversed|either
    step_pc: HashMap<(usize, usize, usize), usize>, // (face, loop, coedge_pos) -> pcurve index
    step_rev: HashMap<(usize, usize, usize), bool>,
    sense: HashMap<(usize, usize, usize), Option<String>>, // curveSense
}

fn loop_key(face: usize, aloop: usize) -> String {
    format!("{face}:{aloop}")
}

// ---------- implicit trim edges (OL08 edge-less uses) ----------

pub struct ImplicitTrimStats { pub synthetic_count: usize, pub collapsed_count: usize }

pub fn materialize_implicit_trim_edges(
    model: &mut ExactModel,
    collapse_tolerance: f64,
) -> Result<ImplicitTrimStats> {
    let mut synthetic_count = 0;
    let mut collapsed_count = 0;
    // collect work list first to satisfy borrow checker
    let mut work: Vec<(usize, usize, usize)> = Vec::new();
    for fi in 0..model.faces.len() {
        for li in 0..model.faces[fi].loops.len() {
            for ci in 0..model.faces[fi].loops[li].coedges.len() {
                if model.faces[fi].loops[li].coedges[ci].edge_index.is_none() {
                    work.push((fi, li, ci));
                }
            }
        }
    }
    for (fi, li, ci) in work {
        // resolve context immutably
        let (uv_points, support) = {
            let face = &model.faces[fi];
            let coedge = &face.loops[li].coedges[ci];
            let ctx = crate::ol_exact::exact_coedge_pcurve_context(model, face, coedge)
                .ok_or_else(|| openpvz_error(format!("cannot materialize implicit trim edge on face {fi}")))?;
            if ctx.pcurve.points.is_empty() {
                return Err(format!("cannot materialize implicit trim edge on face {fi}").into());
            }
            (ctx.pcurve.points.clone(), ctx.support.clone())
        };
        // dense sampling
        let mut dense: Vec<[f64; 2]> = Vec::new();
        if uv_points.len() >= 2 {
            for s in 0..uv_points.len() - 1 {
                let (a, b) = (uv_points[s], uv_points[s + 1]);
                let subdiv = if uv_points.len() == 2 { 32 } else { 4 };
                for k in 0..subdiv {
                    let t = k as f64 / subdiv as f64;
                    dense.push([a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t]);
                }
            }
            dense.push(*uv_points.last().unwrap());
        } else {
            dense = uv_points.clone();
        }
        let mut points3: Vec<[f64; 3]> = Vec::with_capacity(dense.len());
        for uv in &dense {
            match evaluate_exact_surface(&support, uv[0], uv[1]) {
                Some(p) if p.iter().all(|x| x.is_finite()) => points3.push(p),
                _ => return Err(format!("implicit trim edge on face {fi} produced invalid 3D points").into()),
            }
        }
        let (mut mins, mut maxs) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
        for p in &points3 {
            for a in 0..3 {
                mins[a] = mins[a].min(p[a]);
                maxs[a] = maxs[a].max(p[a]);
            }
        }
        let span = (maxs[0]-mins[0]).hypot(maxs[1]-mins[1]).hypot(maxs[2]-mins[2]);
        let coedge = &mut model.faces[fi].loops[li].coedges[ci];
        if span <= collapse_tolerance {
            coedge.collapsed_implicit = true;
            collapsed_count += 1;
            continue;
        }
        let edge_index = model.edges.len();
        model.edges.push(crate::ol_exact::ExactEdge {
            index: edge_index,
            curve: ExactCurve::Polyline { flags: 0, points: points3, trailing_bytes: 0 },
            source_offset: 0,
        });
        let coedge = &mut model.faces[fi].loops[li].coedges[ci];
        coedge.edge_index = Some(edge_index);
        synthetic_count += 1;
    }
    Ok(ImplicitTrimStats { synthetic_count, collapsed_count })
}

// ---------- p-curve preparation ----------

fn repair_pcurve_against_edge(
    model: &ExactModel,
    coedge_pos: (usize, usize, usize),
    source_error: f64,
    tolerance: f64,
) -> Option<(Vec<[f64; 2]>, f64, usize)> {
    if !(source_error > tolerance) { return None; }
    let (fi, li, ci) = coedge_pos;
    let face = model.faces.get(fi)?;
    let coedge = face.loops.get(li)?.coedges.get(ci)?;
    let ctx = crate::ol_exact::exact_coedge_pcurve_context(model, face, coedge)?;
    let edge_index = coedge.edge_index?;
    let edge = model.edges.get(edge_index)?;
    if ctx.pcurve.points.is_empty() { return None; }
    let mut repaired: Vec<[f64; 2]> = Vec::with_capacity(ctx.pcurve.points.len());
    let mut max_residual: f64 = 0.0;
    for uv in &ctx.pcurve.points {
        let approx = evaluate_exact_surface(ctx.support, uv[0], uv[1])?;
        if !approx.iter().all(|x| x.is_finite()) { return None; }
        let t = closest_exact_curve_parameter(&edge.curve, &approx)?;
        let target = crate::ol_exact::evaluate_exact_curve(&edge.curve, t)?;
        if !target.iter().all(|x| x.is_finite()) { return None; }
        let (ruv, residual) = refine_surface_uv_to_point(ctx.support, &target, *uv);
        if !residual.is_finite() { return None; }
        max_residual = max_residual.max(residual);
        repaired.push(ruv);
    }
    if max_residual >= source_error || max_residual > tolerance { return None; }
    let n = repaired.len();
    Some((repaired, max_residual, n))
}

struct PrepOutput {
    // loop key -> pcurve index -> aligned STEP UV points
    loop_pcurve_maps: HashMap<String, HashMap<usize, Vec<[f64; 2]>>>,
    max_pcurve_residual: f64,
    repaired_pcurves: usize,
    repaired_pcurve_points: usize,
    max_source_edge_residual: f64,
    max_repaired_edge_residual: f64,
}

fn surface_periods_step(surface: &crate::ol_exact::ExactSurface) -> (Option<f64>, Option<f64>) {
    use crate::ol_exact::ExactSurface::*;
    match surface {
        Cylinder { .. } | Cone { .. } | Sphere { .. } => (Some(std::f64::consts::PI * 2.0), None),
        Torus { .. } => {
            if crate::step_geom::torus_is_sphere_like(surface) {
                (Some(std::f64::consts::PI * 2.0), None)
            } else {
                (Some(std::f64::consts::PI * 2.0), Some(std::f64::consts::PI * 2.0))
            }
        }
        _ => (None, None),
    }
}

fn align_uv_segment(points: &[[f64; 2]], target: Option<[f64; 2]>, periods: (Option<f64>, Option<f64>)) -> Vec<[f64; 2]> {
    if target.is_none() || points.is_empty() { return points.to_vec(); }
    let t = target.unwrap();
    let mut uo: f64 = 0.0;
    let mut vo: f64 = 0.0;
    if let Some(p) = periods.0 { uo = ((t[0] - points[0][0]) / p).round() * p; }
    if let Some(p) = periods.1 { vo = ((t[1] - points[0][1]) / p).round() * p; }
    points.iter().map(|pt| [pt[0] + uo, pt[1] + vo]).collect()
}

fn prepare_step_pcurves(model: &ExactModel, annot: &mut StepAnnot, tolerance: f64) -> Result<PrepOutput> {
    let mut loop_pcurve_maps: HashMap<String, HashMap<usize, Vec<[f64; 2]>>> = HashMap::new();
    let mut max_pcurve_residual: f64 = 0.0;
    let mut repaired_pcurves = 0;
    let mut repaired_pcurve_points = 0;
    let mut max_source_edge_residual: f64 = 0.0;
    let mut max_repaired_edge_residual: f64 = 0.0;

    for face in &model.faces {
        let mut mapped_by_pc: HashMap<usize, Vec<[f64; 2]>> = HashMap::new();
        for aloop in &face.loops {
            let class = classify_exact_loop(model, face, aloop);
            let (traversal, rev_by) = match &class {
                Some(c) => (c.traversal.clone(), c.pcurve_reversed_by_original_index.clone()),
                None => ("stored".to_string(), vec![]),
            };
            annot.traversal.insert((face.index, aloop.index), traversal);
            for pos in 0..aloop.coedges.len() {
                annot.step_rev.insert((face.index, aloop.index, pos), rev_by.get(pos).copied().unwrap_or(false));
            }
            for (pos, coedge) in aloop.coedges.iter().enumerate() {
                let ctx = crate::ol_exact::exact_coedge_pcurve_context(model, face, coedge)
                    .ok_or_else(|| openpvz_error(format!("missing pcurve context face={} edge={:?}", face.index, coedge.edge_index)))?;
                annot.step_pc.insert((face.index, aloop.index, pos), ctx.index);
                let classification = classify_exact_coedge(model, face, coedge);
                match &classification {
                    None => {
                        if !coedge.collapsed_implicit {
                            return Err(format!("cannot classify coedge face={} edge={:?}", face.index, coedge.edge_index).into());
                        }
                        annot.sense.insert((face.index, aloop.index, pos), None);
                    }
                    Some(c) => {
                        let rev = rev_by.get(pos).copied().unwrap_or(false);
                        let sense = if rev {
                            if c.curve_sense == "forward" { "reversed" } else { "forward" }
                        } else {
                            c.curve_sense.as_str()
                        };
                        annot.sense.insert((face.index, aloop.index, pos), Some(sense.to_string()));
                    }
                }
                if mapped_by_pc.contains_key(&ctx.index) { continue; }
                let mut pcurve_points = ctx.pcurve.points.clone();
                if let Some(c) = &classification {
                    max_source_edge_residual = max_source_edge_residual.max(c.error);
                    if let Some(edge_index) = coedge.edge_index {
                        let edge = &model.edges[edge_index];
                        if let Some((repaired, max_res, count)) = repair_pcurve_against_edge(
                            model, (face.index, aloop.index, pos), c.error, tolerance,
                        ) {
                            let _ = edge;
                            pcurve_points = repaired;
                            repaired_pcurves += 1;
                            repaired_pcurve_points += count;
                            max_repaired_edge_residual = max_repaired_edge_residual.max(max_res);
                        }
                    }
                }
                let (mapped, residual) = map_native_pcurve_to_step(&face.surface, &pcurve_points, ctx.support)?;
                max_pcurve_residual = max_pcurve_residual.max(residual);
                if residual > tolerance {
                    return Err(format!("STEP pcurve mapping residual {residual} exceeds tolerance on face {}", face.index).into());
                }
                mapped_by_pc.insert(ctx.index, mapped);
            }
        }
        // align periodic branches loop-by-loop
        for aloop in &face.loops {
            let periods = surface_periods_step(&face.surface);
            let trav = annot.traversal.get(&(face.index, aloop.index)).cloned().unwrap_or_else(|| "stored".to_string());
            let mut order: Vec<usize> = (0..aloop.coedges.len()).collect();
            if trav == "reversed" { order.reverse(); }
            let mut aligned: HashMap<usize, Vec<[f64; 2]>> = HashMap::new();
            let mut prev_end: Option<[f64; 2]> = None;
            for &pos in &order {
                let pc_idx = *annot.step_pc.get(&(face.index, aloop.index, pos)).unwrap();
                let mut raw = mapped_by_pc.get(&pc_idx).cloned().unwrap_or_default();
                if raw.is_empty() { continue; }
                if annot.step_rev.get(&(face.index, aloop.index, pos)).copied().unwrap_or(false) {
                    raw.reverse();
                }
                let pts = align_uv_segment(&raw, prev_end, periods);
                prev_end = pts.last().copied();
                aligned.insert(pc_idx, pts);
            }
            loop_pcurve_maps.insert(loop_key(face.index, aloop.index), aligned);
        }
    }
    Ok(PrepOutput { loop_pcurve_maps, max_pcurve_residual, repaired_pcurves,
        repaired_pcurve_points, max_source_edge_residual, max_repaired_edge_residual })
}

// ---------- loop polygon / sliver ----------

fn ordered_positions(n: usize, traversal: &str) -> Vec<usize> {
    let mut o: Vec<usize> = (0..n).collect();
    if traversal == "reversed" { o.reverse(); }
    o
}

fn polygon_area(points: &[[f64; 2]]) -> f64 {
    if points.len() < 3 { return 0.0; }
    let mut t: f64 = 0.0;
    for i in 0..points.len() {
        let (a, b) = (points[i], points[(i + 1) % points.len()]);
        t += a[0] * b[1] - b[0] * a[1];
    }
    t / 2.0
}
fn polygon_perimeter(points: &[[f64; 2]]) -> f64 {
    if points.len() < 2 { return 0.0; }
    let mut p: f64 = 0.0;
    for i in 0..points.len() {
        let (a, b) = (points[i], points[(i + 1) % points.len()]);
        p += (b[0]-a[0]).hypot(b[1]-a[1]);
    }
    p
}

/// Narrow sanitizer for the observed ~0.6µm two-edge planar sliver.
fn planar_two_edge_sliver_width(
    model: &ExactModel, region: &crate::ol_exact::FaceRegion,
    annot: &StepAnnot, maps: &HashMap<String, HashMap<usize, Vec<[f64; 2]>>>,
) -> f64 {
    let face = &model.faces[region.face_index];
    if !matches!(face.surface, crate::ol_exact::ExactSurface::Plane { .. }) { return f64::INFINITY; }
    if region.loop_refs.len() != 1 { return f64::INFINITY; }
    let (rfi, rli) = region.loop_refs[0];
    let aloop = match model.faces.get(rfi).and_then(|f| f.loops.get(rli)) { Some(l) => l, None => return f64::INFINITY };
    if aloop.coedges.len() != 2 { return f64::INFINITY; }
    let map = match maps.get(&loop_key(rfi, rli)) { Some(m) => m, None => return f64::INFINITY };
    // rebuild polygon from map
    let trav = annot.traversal.get(&(rfi, rli)).cloned().unwrap_or_else(|| "stored".to_string());
    let mut poly: Vec<[f64; 2]> = Vec::new();
    for pos in ordered_positions(aloop.coedges.len(), &trav) {
        let pc_idx = match annot.step_pc.get(&(rfi, rli, pos)) { Some(i) => *i, None => continue };
        let pts = match map.get(&pc_idx) { Some(p) => p, None => continue };
        if poly.is_empty() { poly.extend(pts.iter().copied()); }
        else {
            let prev = *poly.last().unwrap();
            let dup = (pts[0][0]-prev[0]).hypot(pts[0][1]-prev[1]) < 1e-10;
            poly.extend(pts.iter().skip(if dup {1} else {0}).copied());
        }
    }
    if poly.len() > 2 {
        let (f, l) = (poly[0], *poly.last().unwrap());
        if (f[0]-l[0]).hypot(f[1]-l[1]) < 1e-10 { poly.pop(); }
    }
    let per = polygon_perimeter(&poly);
    if per <= 0.0 { return 0.0; }
    2.0 * polygon_area(&poly).abs() / per
}

// ---------- orientation solving ----------

fn collect_singular_edge_indices(model: &ExactModel) -> HashSet<usize> {
    model.faces.iter().flat_map(|f| f.singular_uses.iter().map(|s| s.edge_index)).collect()
}

fn solve_region_orientations(
    model: &ExactModel,
    regions: &[crate::ol_exact::FaceRegion], singular: &HashSet<usize>,
) -> (HashMap<usize, bool>, usize) {
    let mut orientation: HashMap<usize, bool> = HashMap::new();
    // BTreeMap keeps edge iteration (and hence adjacency insertion) deterministic across runs.
    let mut uses_by_edge: BTreeMap<usize, Vec<(usize, bool)>> = BTreeMap::new();
    for region in regions {
        let face = &model.faces[region.face_index];
        for (rfi, rli) in &region.loop_refs {
            let aloop = &model.faces[*rfi].loops[*rli];
            for coedge in &aloop.coedges {
                let Some(ei) = coedge.edge_index else { continue };
                if singular.contains(&ei) { continue; }
                let class = classify_exact_coedge(model, face, coedge);
                let Some(c) = class else { continue };
                uses_by_edge.entry(ei).or_default().push((region.index, c.curve_sense == "forward"));
            }
        }
    }
    let mut adj: BTreeMap<usize, Vec<(usize, bool)>> = regions.iter().map(|r| (r.index, vec![])).collect();
    for uses in uses_by_edge.values() {
        if uses.len() != 2 || uses[0].0 == uses[1].0 { continue; }
        let (a, b) = (uses[0], uses[1]);
        let must_flip = (1 ^ (a.1 as u8) ^ (b.1 as u8)) != 0;
        adj.get_mut(&a.0).unwrap().push((b.0, must_flip));
        adj.get_mut(&b.0).unwrap().push((a.0, must_flip));
    }
    let mut contradictions = 0;
    for seed in regions {
        if orientation.contains_key(&seed.index) { continue; }
        orientation.insert(seed.index, true);
        let mut stack = vec![seed.index];
        while let Some(ri) = stack.pop() {
            let cur = orientation[&ri];
            for (nb, flip) in adj.get(&ri).cloned().unwrap_or_default() {
                let wanted = if flip { !cur } else { cur };
                if let Some(prev) = orientation.get(&nb) {
                    if *prev != wanted { contradictions += 1; }
                    continue;
                }
                orientation.insert(nb, wanted);
                stack.push(nb);
            }
        }
    }
    (orientation, contradictions / 2)
}

// ---------- appearance mapping (tessellation appearance -> exact faces) ----------

pub struct AppearanceMapping {
    pub appearances: Vec<Option<Appearance>>, // per face index
    pub mode: String,
    pub unmatched_face_indices: Vec<usize>,
    pub unmatched_mesh_indices: Vec<usize>,
    pub inferred_face_indices: Vec<usize>,
    pub remaining_unstyled: Vec<usize>,
    pub max_match_cost: f64,
}

fn mesh_bounds(m: &crate::ol::SurfaceMesh) -> Option<([f64; 3], [f64; 3])> {
    if m.vertices.is_empty() { return None; }
    let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
    for v in &m.vertices {
        for a in 0..3 { lo[a] = lo[a].min(v[a]); hi[a] = hi[a].max(v[a]); }
    }
    Some((lo, hi))
}
fn exact_face_bounds(model: &ExactModel, face_index: usize) -> Option<([f64; 3], [f64; 3])> {
    let face = &model.faces[face_index];
    let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
    let mut n = 0;
    for aloop in &face.loops {
        for coedge in &aloop.coedges {
            let Some(ctx) = crate::ol_exact::exact_coedge_pcurve_context(model, face, coedge) else { continue; };
            if ctx.pcurve.points.is_empty() { continue; }
            for uv in &ctx.pcurve.points {
                let Some(p) = evaluate_exact_surface(ctx.support, uv[0], uv[1]) else { continue; };
                if !p.iter().all(|x| x.is_finite()) { continue; }
                for a in 0..3 { lo[a] = lo[a].min(p[a]); hi[a] = hi[a].max(p[a]); }
                n += 1;
            }
        }
    }
    if n == 0 { None } else { Some((lo, hi)) }
}
fn bounds_cost(a: Option<([f64; 3], [f64; 3])>, b: Option<([f64; 3], [f64; 3])>) -> f64 {
    let (Some((alo, ahi)), Some((blo, bhi))) = (a, b) else { return 5.0; };
    let (mut scale, mut delta): (f64, f64) = (0.0, 0.0);
    for ax in 0..3 {
        scale = scale.max(ahi[ax]-alo[ax]).max(bhi[ax]-blo[ax]);
        delta = delta.max((alo[ax]-blo[ax]).abs()).max((ahi[ax]-bhi[ax]).abs());
    }
    delta / scale.max(1e-9)
}

pub fn map_ol_appearances_to_exact_faces(ol_data: &[u8], model: &ExactModel) -> Result<AppearanceMapping> {
    let meshes = extract_ol_surface_meshes(ol_data)?;
    let (mapping, mode, unmatched_f, unmatched_m, max_cost) = if meshes.len() == model.faces.len() {
        ((0..model.faces.len()).map(Some).collect::<Vec<_>>(), "ordered".to_string(), vec![], vec![], 0.0)
    } else {
        // monotonic DP alignment
        let fb: Vec<_> = (0..model.faces.len()).map(|i| exact_face_bounds(model, i)).collect();
        let tb: Vec<_> = meshes.iter().map(mesh_bounds).collect();
        let (ec, tc) = (model.faces.len(), meshes.len());
        let stride = tc + 1;
        let cells = (ec + 1) * stride;
        if cells > 20_000_000 {
            return Err(format!("appearance alignment too large ({ec} exact faces x {tc} tess surfaces)").into());
        }
        let mut cost = vec![f64::INFINITY; cells];
        let mut bt = vec![0u8; cells];
        cost[0] = 0.0;
        let mesh_skip = 0.32;
        let face_skip = |i: usize| {
            let f = &model.faces[i];
            if !f.loops.is_empty() || !f.singular_uses.is_empty() { 0.32 } else { 0.03 }
        };
        for ei in 0..=ec {
            for mi in 0..=tc {
                let cur = ei * stride + mi;
                let base = cost[cur];
                if !base.is_finite() { continue; }
                if ei < ec && mi < tc {
                    let nx = (ei + 1) * stride + mi + 1;
                    let c = base + bounds_cost(fb[ei], tb[mi]);
                    if c < cost[nx] { cost[nx] = c; bt[nx] = 1; }
                }
                if ei < ec {
                    let nx = (ei + 1) * stride + mi;
                    let c = base + face_skip(ei);
                    if c < cost[nx] { cost[nx] = c; bt[nx] = 2; }
                }
                if mi < tc {
                    let nx = ei * stride + mi + 1;
                    let c = base + mesh_skip;
                    if c < cost[nx] { cost[nx] = c; bt[nx] = 3; }
                }
            }
        }
        let mut mapping: Vec<Option<usize>> = vec![None; ec];
        let (mut uf, mut um) = (vec![], vec![]);
        let (mut ei, mut mi) = (ec, tc);
        let mut maxc: f64 = 0.0;
        while ei > 0 || mi > 0 {
            match bt[ei * stride + mi] {
                1 => {
                    let c = bounds_cost(fb[ei-1], tb[mi-1]);
                    maxc = maxc.max(c);
                    mapping[ei-1] = Some(mi-1);
                    ei -= 1; mi -= 1;
                }
                2 => { uf.push(ei-1); ei -= 1; }
                3 => { um.push(mi-1); mi -= 1; }
                _ => return Err(format!("appearance alignment backtrace failed at {ei},{mi}").into()),
            }
        }
        uf.reverse(); um.reverse();
        (mapping, "geometric_alignment".to_string(), uf, um, maxc)
    };
    let mut appearances: Vec<Option<Appearance>> = mapping.iter().map(|mi| mi.and_then(|i| meshes.get(i).and_then(|m| m.appearance.clone()))).collect();
    // topology inference: fill unstyled face when all styled neighbours agree
    let mut edge_faces: HashMap<usize, HashSet<usize>> = HashMap::new();
    for face in &model.faces {
        for aloop in &face.loops {
            for c in &aloop.coedges {
                if let Some(ei) = c.edge_index {
                    edge_faces.entry(ei).or_default().insert(face.index);
                }
            }
        }
    }
    let mut inferred = vec![];
    let mut changed = true;
    while changed {
        changed = false;
        for face in &model.faces {
            if appearances[face.index].is_some() || face.loops.is_empty() { continue; }
            let mut cands: HashMap<String, Appearance> = HashMap::new();
            for aloop in &face.loops {
                for c in &aloop.coedges {
                    let Some(ei) = c.edge_index else { continue };
                    if let Some(neighs) = edge_faces.get(&ei) {
                        for ni in neighs {
                            if *ni == face.index { continue; }
                            if let Some(a) = &appearances[*ni] {
                                cands.entry(crate::appearance::appearance_key(Some(a))).or_insert_with(|| a.clone());
                            }
                        }
                    }
                }
            }
            if cands.len() == 1 {
                appearances[face.index] = cands.into_values().next();
                inferred.push(face.index);
                changed = true;
            }
        }
    }
    let remaining: Vec<usize> = model.faces.iter()
        .filter(|f| !f.loops.is_empty() && appearances[f.index].is_none())
        .map(|f| f.index).collect();
    Ok(AppearanceMapping { appearances, mode, unmatched_face_indices: unmatched_f,
        unmatched_mesh_indices: unmatched_m, inferred_face_indices: inferred,
        remaining_unstyled: remaining, max_match_cost: max_cost })
}

// ---------- B-rep emission ----------

pub struct PartStats {
    pub shape_items: Vec<u32>,
    pub face_count: usize,
    pub edge_count: usize,
    pub vertex_count: usize,
    pub pcurve_count: usize,
    pub max_pcurve_residual: f64,
    pub repaired_pcurves: usize,
    pub repaired_pcurve_points: usize,
    pub max_source_edge_residual: f64,
    pub max_repaired_edge_residual: f64,
    pub closed_shells: usize,
    pub open_shells: usize,
    pub vertex_max_spread: f64,
    pub orientation_contradictions: usize,
    pub omitted_degenerate: Vec<serde_json::Value>,
    pub styled_items: usize,
    pub styled_faces: usize,
    pub unstyled_faces: usize,
    pub unique_appearances: usize,
    pub source_face_count: usize,
    pub face_region_count: usize,
    pub split_region_count: usize,
}

fn point3w(writer: &mut StepWriter, p: &[f64; 3]) -> Result<u32> {
    writer.ent(&format!("CARTESIAN_POINT('',({},{},{}))", fnum(p[0])?, fnum(p[1])?, fnum(p[2])?))
}

#[allow(clippy::too_many_arguments)]
pub fn emit_exact_part(
    writer: &mut StepWriter,
    model: &mut ExactModel,
    parametric_context: u32,
    geom_context: Option<u32>,
    face_appearances: Option<Vec<Option<Appearance>>>,
    pcurve_tolerance: f64,
    min_sliver_width: f64,
) -> Result<PartStats> {
    let implicit = materialize_implicit_trim_edges(model, DEFAULT_IMPLICIT_EDGE_COLLAPSE_TOLERANCE)?;
    if implicit.synthetic_count > 0 || implicit.collapsed_count > 0 {
        model.shells = build_shell_candidates(model);
    }
    let vertex_topology = crate::ol_exact::build_exact_vertex_topology(model);
    let singular = collect_singular_edge_indices(model);

    // base geometry
    let mut vertex_refs: Vec<u32> = Vec::with_capacity(vertex_topology.vertices.len());
    for v in &vertex_topology.vertices {
        let p = point3w(writer, &v.point)?;
        vertex_refs.push(writer.ent(&format!("VERTEX_POINT('',#{p})"))?);
    }
    let mut surface_refs: Vec<u32> = Vec::with_capacity(model.faces.len());
    for face in &model.faces {
        surface_refs.push(surface_entity(writer, &face.surface)?);
    }
    let mut curve_refs: Vec<Option<u32>> = Vec::with_capacity(model.edges.len());
    for edge in &model.edges {
        curve_refs.push(curve_entity_3d(writer, &edge.curve)?);
    }

    let mut annot = StepAnnot::default();
    let prep = prepare_step_pcurves(model, &mut annot, pcurve_tolerance)?;

    // region selection (sliver filter)
    let regions = model.face_regions.clone();
    let mut omitted: HashSet<usize> = HashSet::new();
    let mut omitted_degenerate: Vec<serde_json::Value> = vec![];
    for region in &regions {
        let w = planar_two_edge_sliver_width(model, region, &annot, &prep.loop_pcurve_maps);
        if w.is_finite() && w < min_sliver_width {
            omitted.insert(region.index);
            omitted_degenerate.push(serde_json::json!({
                "regionIndex": region.index, "faceIndex": region.face_index,
                "reason": "planar_two_edge_sliver", "characteristicWidth": w }));
        }
    }
    let active: Vec<_> = regions.iter().filter(|r| !omitted.contains(&r.index)).collect();
    let mut active_loop_keys: HashSet<String> = HashSet::new();
    for r in &active {
        for (rfi, rli) in &r.loop_refs {
            active_loop_keys.insert(loop_key(*rfi, *rli));
        }
    }

    // p-curve uses per edge
    let mut uses_by_edge: Vec<Vec<(usize, usize, usize, Vec<[f64; 2]>)>> = vec![vec![]; model.edges.len()];
    let mut pcurve_count = 0;
    for face in &model.faces {
        for aloop in &face.loops {
            let key = loop_key(face.index, aloop.index);
            if !active_loop_keys.contains(&key) { continue; }
            let aligned = prep.loop_pcurve_maps.get(&key);
            for (pos, coedge) in aloop.coedges.iter().enumerate() {
                let pc_idx = match annot.step_pc.get(&(face.index, aloop.index, pos)) { Some(i) => *i, None => continue };
                let mapped = match aligned.and_then(|m| m.get(&pc_idx)) {
                    Some(m) => m.clone(),
                    None => return Err(format!("missing mapped pcurve face={} loop={} pcurve={pc_idx}", face.index, aloop.index).into()),
                };
                let Some(ei) = coedge.edge_index else { continue };
                if coedge.collapsed_implicit || singular.contains(&ei) { continue; }
                if ei >= uses_by_edge.len() { continue; }
                uses_by_edge[ei].push((face.index, aloop.index, pos, mapped));
                pcurve_count += 1;
            }
        }
    }

    // edges
    let mut edge_refs: Vec<Option<u32>> = vec![None; model.edges.len()];
    for (ei, edge) in model.edges.iter().enumerate() {
        if matches!(edge.curve, ExactCurve::DegeneratePoint { .. }) || singular.contains(&ei) { continue; }
        let topo = &vertex_topology.edges[ei];
        let (Some(sv), Some(ev)) = (topo.start_vertex, topo.end_vertex) else {
            return Err(format!("edge {ei} has incomplete vertex topology").into());
        };
        let mut assoc: Vec<u32> = vec![];
        let mut seen: HashSet<String> = HashSet::new();
        // uses tuple = (face_index, loop_index, coedge_pos, mapped_points)
        for (fi, li, pos, mapped) in &uses_by_edge[ei] {
            let sense = annot.sense.get(&(*fi, *li, *pos)).cloned().flatten();
            let aligned: Vec<[f64; 2]> = if sense.as_deref() == Some("reversed") {
                mapped.iter().rev().copied().collect()
            } else {
                mapped.clone()
            };
            let key = format!("{fi}:{}", aligned.iter().map(|p| format!("{},{}", p[0], p[1])).collect::<Vec<_>>().join(";"));
            if !seen.insert(key) { continue; }
            assoc.push(pcurve_entity(writer, surface_refs[*fi], &aligned, parametric_context)?);
        }
        if assoc.len() > 2 {
            return Err(format!("edge {ei} has {} pcurve associations; STEP SURFACE_CURVE supports at most 2", assoc.len()).into());
        }
        let geom_ref = if assoc.is_empty() {
            curve_refs[ei].ok_or_else(|| openpvz_error(format!("edge {ei} has no 3D curve")))?
        } else {
            writer.ent(&format!("SURFACE_CURVE('',#{},{},.CURVE_3D.)",
                curve_refs[ei].ok_or_else(|| openpvz_error(format!("edge {ei} has no 3D curve")))?,
                refs(&assoc)))?
        };
        edge_refs[ei] = Some(writer.ent(&format!(
            "EDGE_CURVE('',#{},#{},#{geom_ref},.T.)", vertex_refs[sv], vertex_refs[ev]))?);
    }

    // faces
    let (orientation, contradictions) = solve_region_orientations(model, &active.iter().map(|r| (*r).clone()).collect::<Vec<_>>(), &singular);
    let mut face_refs: Vec<Option<u32>> = vec![None; regions.len()];
    let mut split_region_count = 0;
    for region in &active {
        let face = &model.faces[region.face_index];
        if region.loop_refs.is_empty() && region.singular_uses.is_empty() { continue; }
        if region.local_index > 0 { split_region_count += 1; }
        let face_sense = orientation.get(&region.index).copied().unwrap_or(true);
        // loop refs (first = outer)
        let mut bounds: Vec<u32> = vec![];
        // region.loop_refs[0] is the outer loop, followed by its holes
        let mut loop_ref_by_idx: HashMap<usize, Option<u32>> = HashMap::new();
        for (order, (rfi, rli)) in region.loop_refs.iter().enumerate() {
            if loop_ref_by_idx.contains_key(rli) { continue; }
            let aloop = &model.faces[*rfi].loops[*rli];
            let trav = annot.traversal.get(&(*rfi, *rli)).cloned().unwrap_or_else(|| "stored".to_string());
            let mut oes: Vec<u32> = vec![];
            for pos in ordered_positions(aloop.coedges.len(), &trav) {
                let coedge = &aloop.coedges[pos];
                let Some(ei) = coedge.edge_index else { continue };
                if singular.contains(&ei) { continue; }
                let Some(er) = edge_refs[ei] else { continue };
                let sense = annot.sense.get(&(*rfi, *rli, pos)).cloned().flatten();
                oes.push(writer.ent(&format!("ORIENTED_EDGE('',*,*,#{er},{})",
                    step_bool(sense.as_deref() == Some("forward"))))?);
            }
            let lr: Option<u32> = if oes.is_empty() { None }
                else { Some(writer.ent(&format!("EDGE_LOOP('',{}", refs(&oes)))?) };
            loop_ref_by_idx.insert(*rli, lr);
            if let Some(lr) = lr {
                let bound_type = if order == 0 { "FACE_OUTER_BOUND" } else { "FACE_BOUND" };
                bounds.push(writer.ent(&format!("{bound_type}('',#{lr},{})", step_bool(face_sense)))?);
            }
            let _ = face;
        }
        if bounds.is_empty() { continue; }
        face_refs[region.index] = Some(writer.ent(&format!(
            "ADVANCED_FACE('',{},#{},{})",
            refs(&bounds), surface_refs[face.index], step_bool(face_sense)
        ))?);
    }

    // shells
    let mut shape_items: Vec<u32> = vec![];
    let (mut closed, mut open) = (0, 0);
    for shell in &model.shells {
        let faces: Vec<u32> = shell.face_region_indices.iter().filter_map(|i| face_refs.get(*i).copied().flatten()).collect();
        if faces.is_empty() { continue; }
        let omitted_from_shell = shell.face_region_indices.iter().any(|i| omitted.contains(i));
        let can_solid = shell.closed_candidate && !omitted_from_shell && faces.len() == shell.face_region_indices.len();
        if can_solid {
            let cs = writer.ent(&format!("CLOSED_SHELL('',{}", refs(&faces)))?;
            shape_items.push(writer.ent(&format!("MANIFOLD_SOLID_BREP('',#{cs})"))?);
            closed += 1;
        } else {
            let os = writer.ent(&format!("OPEN_SHELL('',{}", refs(&faces)))?;
            shape_items.push(writer.ent(&format!("SHELL_BASED_SURFACE_MODEL('',(#{os}))"))?);
            open += 1;
        }
    }

    // appearance presentation
    let (styled_items, styled_faces, unstyled_faces, unique_apps, presentation) =
        emit_appearance_presentation(writer, &regions, &face_refs, face_appearances.as_deref(), &shape_items, geom_context)?;
    let _ = presentation;

    Ok(PartStats {
        shape_items,
        face_count: face_refs.iter().flatten().count(),
        edge_count: edge_refs.iter().flatten().count(),
        vertex_count: vertex_refs.len(),
        pcurve_count,
        max_pcurve_residual: prep.max_pcurve_residual,
        repaired_pcurves: prep.repaired_pcurves,
        repaired_pcurve_points: prep.repaired_pcurve_points,
        max_source_edge_residual: prep.max_source_edge_residual,
        max_repaired_edge_residual: prep.max_repaired_edge_residual,
        closed_shells: closed,
        open_shells: open,
        vertex_max_spread: vertex_topology.max_spread,
        orientation_contradictions: contradictions,
        omitted_degenerate,
        styled_items, styled_faces, unstyled_faces, unique_appearances: unique_apps,
        source_face_count: model.faces.iter().filter(|f| !f.loops.is_empty() || !f.singular_uses.is_empty()).count(),
        face_region_count: regions.len(),
        split_region_count,
    })
}

fn emit_appearance_presentation(
    writer: &mut StepWriter,
    regions: &[crate::ol_exact::FaceRegion],
    face_refs: &[Option<u32>],
    face_appearances: Option<&[Option<Appearance>]>,
    shape_items: &[u32],
    geom_context: Option<u32>,
) -> Result<(usize, usize, usize, usize, Option<u32>)> {
    let emitted: Vec<&crate::ol_exact::FaceRegion> = regions.iter()
        .filter(|r| r.index < face_refs.len() && face_refs[r.index].is_some()).collect();
    if geom_context.is_none() || face_appearances.is_none() {
        return Ok((0, 0, emitted.len(), 0, None));
    }
    let apps = face_appearances.unwrap();
    let known: Vec<Option<&Appearance>> = emitted.iter().map(|r| apps.get(r.face_index).and_then(|o| o.as_ref())).collect();
    let mut uniq: HashSet<String> = HashSet::new();
    for k in known.iter().flatten() { uniq.insert(crate::appearance::appearance_key(Some(k))); }
    let all_known = !known.is_empty() && known.iter().all(|k| k.is_some());
    let mut styled: Vec<u32> = vec![];
    let mut styled_faces = 0;
    if all_known && uniq.len() == 1 && !shape_items.is_empty() {
        let (assignment, n) = step_appearance_style(writer, known[0].unwrap())?;
        let label = if n.name.is_empty() { "color".to_string() } else { n.name.clone() };
        for item in shape_items {
            styled.push(writer.ent(&format!("STYLED_ITEM({},(#{assignment}),#{item})", encode_step_string(&label)))?);
        }
        styled_faces = emitted.len();
    } else {
        for region in &emitted {
            let Some(app) = apps.get(region.face_index).and_then(|o| o.as_ref()) else { continue };
            let (assignment, n) = step_appearance_style(writer, app)?;
            let label = if n.name.is_empty() { "color".to_string() } else { n.name.clone() };
            styled.push(writer.ent(&format!("STYLED_ITEM({},(#{assignment}),#{})",
                encode_step_string(&label), face_refs[region.index].unwrap()))?);
            styled_faces += 1;
        }
    }
    let presentation = if styled.is_empty() { None } else {
        Some(writer.ent(&format!("MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION('',{},#{})",
            refs(&styled), geom_context.unwrap()))?)
    };
    Ok((styled.len(), styled_faces, emitted.len() - styled_faces, uniq.len(), presentation))
}

// ---------- contexts / products / assembly ----------

pub struct StepContexts { pub app: u32, pub product_ctx: u32, pub pd_ctx: u32, pub geom: u32, pub param: u32 }

pub fn emit_contexts(writer: &mut StepWriter, uncertainty: f64) -> Result<StepContexts> {
    let app = writer.ent("APPLICATION_CONTEXT('core data for automotive mechanical design processes')")?;
    writer.ent(&format!("APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,#{app})"))?;
    let product_ctx = writer.ent(&format!("PRODUCT_CONTEXT('',#{app},'mechanical')"))?;
    let pd_ctx = writer.ent(&format!("PRODUCT_DEFINITION_CONTEXT('part definition',#{app},'design')"))?;
    let length_unit = writer.ent("( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT($,.METRE.) )")?;
    let angle_unit = writer.ent("( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) )")?;
    let solid_unit = writer.ent("( NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT() )")?;
    let unc = writer.ent(&format!("UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE({}),#{length_unit},'distance_accuracy_value','confusion accuracy')", fnum(uncertainty)?))?;
    let geom = writer.ent(&format!("( GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#{unc})) GLOBAL_UNIT_ASSIGNED_CONTEXT((#{length_unit},#{angle_unit},#{solid_unit})) REPRESENTATION_CONTEXT('Context #1','3D Context with UNIT and UNCERTAINTY') )"))?;
    let param = writer.ent("( GEOMETRIC_REPRESENTATION_CONTEXT(2) PARAMETRIC_REPRESENTATION_CONTEXT() REPRESENTATION_CONTEXT('2D SPACE','') )")?;
    Ok(StepContexts { app, product_ctx, pd_ctx, geom, param })
}

pub struct ProductDef { pub product: u32, pub formation: u32, pub pd: u32, pub pds: u32 }

pub fn emit_product_definition(writer: &mut StepWriter, name: &str, ctx: &StepContexts) -> Result<ProductDef> {
    let enc = encode_step_string(name);
    let product = writer.ent(&format!("PRODUCT({enc},{enc},'',(#{}))", ctx.product_ctx))?;
    let formation = writer.ent(&format!("PRODUCT_DEFINITION_FORMATION('','',#{product})"))?;
    let pd = writer.ent(&format!("PRODUCT_DEFINITION('design','',#{formation},#{})", ctx.pd_ctx))?;
    let pds = writer.ent(&format!("PRODUCT_DEFINITION_SHAPE('','',#{pd})"))?;
    writer.ent(&format!("PRODUCT_RELATED_PRODUCT_CATEGORY('part',$,(#{product}))"))?;
    Ok(ProductDef { product, formation, pd, pds })
}

pub fn emit_identity_axis(writer: &mut StepWriter) -> Result<u32> {
    crate::step_geom::placement_axis(writer, &[0.0, 0.0, 0.0], &[1.0,0.0,0.0, 0.0,1.0,0.0, 0.0,0.0,1.0])
}

pub fn emit_shape_representation(writer: &mut StepWriter, ctx: &StepContexts, pds: u32, items: &[u32]) -> Result<u32> {
    let rep = writer.ent(&format!("SHAPE_REPRESENTATION('',{},#{})", refs(items), ctx.geom))?;
    writer.ent(&format!("SHAPE_DEFINITION_REPRESENTATION(#{pds},#{rep})"))?;
    Ok(rep)
}

// ---------- public export API ----------

pub struct StepPartOutcome {
    pub stats: PartStats,
    pub appearance_mode: String,
    pub unmatched_faces: usize,
    pub unmatched_meshes: usize,
    pub inferred_faces: usize,
    pub remaining_unstyled: usize,
}

fn decode_emit_unique(
    writer: &mut StepWriter, ctx: &StepContexts,
    definition_source: &str, ol_name: &str, ol_data: &[u8],
    pcurve_tolerance: f64, min_sliver_width: f64,
) -> Result<(PartStats, AppearanceMapping)> {
    let mut exact = parse_ol_exact_geometry(ol_data)?
        .ok_or_else(|| openpvz_error(format!("{ol_name}: exact geometry stream not found")))?;
    let mapping = map_ol_appearances_to_exact_faces(ol_data, &exact)
        .map_err(|e| openpvz_error(format!("{ol_name} ({definition_source}): {e}")))?;
    let stats = emit_exact_part(writer, &mut exact, ctx.param, Some(ctx.geom),
        Some(mapping.appearances.clone()), pcurve_tolerance, min_sliver_width)
        .map_err(|e| openpvz_error(format!("{ol_name} ({definition_source}): {e}")))?;
    Ok((stats, mapping))
}

/// Export one already-decoded exact model as a standalone STEP file.
pub fn export_step_exact_model(
    model: &mut ExactModel,
    output: &str,
    name: &str,
    face_appearances: Option<Vec<Option<Appearance>>>,
    pcurve_tolerance: f64,
    step_uncertainty: f64,
    min_sliver_width: f64,
) -> Result<PartStats> {
    let mut writer = StepWriter::new(output, "openpvz-rs", name)?;
    let ctx = emit_contexts(&mut writer, step_uncertainty)?;
    let product = emit_product_definition(&mut writer, name, &ctx)?;
    let identity = emit_identity_axis(&mut writer)?;
    let stats = emit_exact_part(&mut writer, model, ctx.param, Some(ctx.geom), face_appearances, pcurve_tolerance, min_sliver_width)?;
    let mut items = vec![identity];
    items.extend(stats.shape_items.iter().copied());
    emit_shape_representation(&mut writer, &ctx, product.pds, &items)?;
    let entities = writer.entity_count;
    writer.finish()?;
    let _ = entities;
    Ok(stats)
}

/// Export one OL member from a PVZ archive as a standalone STEP part.
pub fn export_step_ol(
    archive: &crate::pvz::PvzArchive,
    ol_member: &str,
    output: &str,
    pcurve_tolerance: f64,
    step_uncertainty: f64,
    min_sliver_width: f64,
) -> Result<(PartStats, BTreeMap<String, serde_json::Value>)> {
    let ol_data = archive.read_ol(ol_member)?;
    let mut writer = StepWriter::new(output, "openpvz-rs", ol_member)?;
    let ctx = emit_contexts(&mut writer, step_uncertainty)?;
    let product = emit_product_definition(&mut writer, ol_member, &ctx)?;
    let identity = emit_identity_axis(&mut writer)?;
    let (stats, mapping) = decode_emit_unique(&mut writer, &ctx, ol_member, ol_member, &ol_data, pcurve_tolerance, min_sliver_width)?;
    let mut items = vec![identity];
    items.extend(stats.shape_items.iter().copied());
    emit_shape_representation(&mut writer, &ctx, product.pds, &items)?;
    writer.finish()?;
    let mut map_info = BTreeMap::new();
    map_info.insert("mode".into(), serde_json::Value::String(mapping.mode));
    map_info.insert("unmatchedFaceCount".into(), mapping.unmatched_face_indices.len().into());
    map_info.insert("unmatchedMeshCount".into(), mapping.unmatched_mesh_indices.len().into());
    map_info.insert("inferredFaceCount".into(), mapping.inferred_face_indices.len().into());
    map_info.insert("remainingUnstyledFaceCount".into(), mapping.remaining_unstyled.len().into());
    map_info.insert("maxMatchCost".into(), serde_json::json!(mapping.max_match_cost));
    Ok((stats, map_info))
}

#[derive(Debug, Default, serde::Serialize, Clone)]
pub struct FailedPart {
    pub definition_index: usize,
    pub source_name: String,
    pub ol_name: String,
    pub error: String,
}

#[derive(Debug, Default, serde::Serialize)]
pub struct AssemblyTotals {
    pub definitions: usize,
    pub geometry_definitions: usize,
    pub unique_geometry_payloads: usize,
    pub reused_geometry_definitions: usize,
    pub faces: usize,
    pub edges: usize,
    pub vertices: usize,
    pub pcurves: usize,
    pub emitted_faces: usize,
    pub emitted_edges: usize,
    pub emitted_vertices: usize,
    pub emitted_pcurves: usize,
    pub closed_shells: usize,
    pub open_shells: usize,
    pub omitted_degenerate_faces: usize,
    pub max_pcurve_residual: f64,
    pub repaired_pcurves: usize,
    pub repaired_pcurve_points: usize,
    pub max_source_edge_residual: f64,
    pub max_repaired_edge_residual: f64,
    pub step_styled_items: usize,
    pub step_styled_faces: usize,
    pub step_unstyled_faces: usize,
    pub appearance_alignment_fallbacks: usize,
    pub appearance_unmatched_faces: usize,
    pub appearance_unmatched_meshes: usize,
    pub appearance_inferred_faces: usize,
    pub appearance_remaining_unstyled_faces: usize,
    pub step_unique_appearances: usize,
    pub entities: usize,
    pub occurrences: usize,
    /// Parts whose exact geometry failed to decode/emit. The export continues
    /// past them (product emitted without shape items) instead of aborting a
    /// multi-thousand-part assembly on one bad member. Empty on full success.
    pub failed_parts: Vec<FailedPart>,
}

/// Export the complete PVS assembly hierarchy as AP214 (with OL-hash dedup).
pub fn export_step(
    archive: &crate::pvz::PvzArchive,
    output: &str,
    pcurve_tolerance: f64,
    step_uncertainty: f64,
    min_sliver_width: f64,
    progress: Option<&dyn Fn(usize, usize)>,
) -> Result<AssemblyTotals> {
    let base = std::path::Path::new(&archive.path).file_name()
        .map(|s| s.to_string_lossy().into_owned()).unwrap_or_else(|| "PVZ model".into());
    let mut writer = StepWriter::new(output, "openpvz-rs", &base)?;
    let ctx = emit_contexts(&mut writer, step_uncertainty)?;
    let definitions = &archive.pvs.definitions;
    let mut totals = AssemblyTotals { definitions: definitions.len(), ..Default::default() };
    struct DefState { product: ProductDef, rep: u32, identity: u32, child_axes: Vec<u32> }
    let mut states: Vec<Option<DefState>> = Vec::with_capacity(definitions.len());
    let mut cache: HashMap<String, PartStats> = HashMap::new();
    let mut mapping_cache: HashMap<String, (String, usize, usize, usize, usize)> = HashMap::new();
    // Digests that already failed: skip re-emitting, still record each affected definition.
    let mut failed_digests: HashMap<String, String> = HashMap::new();

    for (index, definition) in definitions.iter().enumerate() {
        let product = emit_product_definition(&mut writer, &definition.source_name, &ctx)?;
        let identity = emit_identity_axis(&mut writer)?;
        let mut cache_key: Option<String> = None;
        if let Some(ol_name) = &definition.ol_name {
            // Quarantine: one undecodable member records a failed part and the
            // export continues, instead of aborting a whole assembly.
            let ol_data_opt: Option<Vec<u8>> = match archive.read_ol(ol_name) {
                Ok(data) => Some(data),
                // Quarantined: tail below emits a geometry-less product.
                Err(e) => {
                    totals.failed_parts.push(FailedPart {
                        definition_index: index,
                        source_name: definition.source_name.clone(),
                        ol_name: ol_name.clone(),
                        error: e.to_string(),
                    });
                    None
                }
            };
            if let Some(ol_data) = ol_data_opt {
            let digest = sha256_hex(&ol_data);
            if let Some(prev_error) = failed_digests.get(&digest) {
                totals.failed_parts.push(FailedPart {
                    definition_index: index,
                    source_name: definition.source_name.clone(),
                    ol_name: ol_name.clone(),
                    error: prev_error.clone(),
                });
            } else {
                cache_key = Some(digest.clone());
                if !cache.contains_key(&digest) {
                    match decode_emit_unique(&mut writer, &ctx, &definition.source_name, ol_name, &ol_data, pcurve_tolerance, min_sliver_width) {
                        Err(e) => {
                            failed_digests.insert(digest.clone(), e.to_string());
                            totals.failed_parts.push(FailedPart {
                                definition_index: index,
                                source_name: definition.source_name.clone(),
                                ol_name: ol_name.clone(),
                                error: e.to_string(),
                            });
                            cache_key = None;
                        }
                        Ok((stats, mapping)) => {
                            totals.unique_geometry_payloads += 1;
                            totals.emitted_faces += stats.face_count;
                            totals.emitted_edges += stats.edge_count;
                            totals.emitted_vertices += stats.vertex_count;
                            totals.emitted_pcurves += stats.pcurve_count;
                            totals.step_styled_items += stats.styled_items;
                            totals.step_styled_faces += stats.styled_faces;
                            totals.step_unstyled_faces += stats.unstyled_faces;
                            if mapping.mode == "geometric_alignment" { totals.appearance_alignment_fallbacks += 1; }
                            totals.appearance_unmatched_faces += mapping.unmatched_face_indices.len();
                            totals.appearance_unmatched_meshes += mapping.unmatched_mesh_indices.len();
                            totals.appearance_inferred_faces += mapping.inferred_face_indices.len();
                            totals.appearance_remaining_unstyled_faces += mapping.remaining_unstyled.len();
                            mapping_cache.insert(digest.clone(), (mapping.mode, mapping.unmatched_face_indices.len(),
                                mapping.unmatched_mesh_indices.len(), mapping.inferred_face_indices.len(), mapping.remaining_unstyled.len()));
                            cache.insert(digest.clone(), stats);
                        }
                    }
                } else {
                    totals.reused_geometry_definitions += 1;
                }
                if cache.contains_key(&digest) {
                    let stats = &cache[&digest];
                    totals.geometry_definitions += 1;
                    totals.faces += stats.face_count;
                    totals.edges += stats.edge_count;
                    totals.vertices += stats.vertex_count;
                    totals.pcurves += stats.pcurve_count;
                    totals.closed_shells += stats.closed_shells;
                    totals.open_shells += stats.open_shells;
                    totals.omitted_degenerate_faces += stats.omitted_degenerate.len();
                    totals.max_pcurve_residual = totals.max_pcurve_residual.max(stats.max_pcurve_residual);
                    totals.repaired_pcurves += stats.repaired_pcurves;
                    totals.repaired_pcurve_points += stats.repaired_pcurve_points;
                    totals.max_source_edge_residual = totals.max_source_edge_residual.max(stats.max_source_edge_residual);
                    totals.max_repaired_edge_residual = totals.max_repaired_edge_residual.max(stats.max_repaired_edge_residual);
                }
            }
            } // end if let Some(ol_data)
        }
        // shape items for this definition: need owned copy since cache borrow
        let shape_items: Vec<u32> = cache_key.as_ref().and_then(|k| cache.get(k)).map(|s| s.shape_items.clone()).unwrap_or_default();
        let mut child_axes = Vec::with_capacity(definition.children.len());
        for child in &definition.children {
            child_axes.push(placement_axis(&mut writer, &child.placement.translation, &child.placement.rotation)?);
        }
        let mut items = vec![identity];
        items.extend(shape_items);
        items.extend(child_axes.iter().copied());
        let rep = emit_shape_representation(&mut writer, &ctx, product.pds, &items)?;
        states.push(Some(DefState { product, rep, identity, child_axes }));
        if let Some(cb) = progress { cb(index + 1, definitions.len()); }
    }

    let mut occurrence_id = 1u32;
    for parent in definitions {
        let (parent_pd, parent_rep) = {
            let st = states[parent.index].as_ref().unwrap();
            (st.product.pd, st.rep)
        };
        for (ci, child) in parent.children.iter().enumerate() {
            let child_state = states[child.definition_index].as_ref().unwrap();
            let parent_state = states[parent.index].as_ref().unwrap();
            let transform = writer.ent(&format!("ITEM_DEFINED_TRANSFORMATION('','',#{},#{})",
                child_state.identity, parent_state.child_axes[ci]))?;
            let relationship = writer.ent(&format!("( REPRESENTATION_RELATIONSHIP('','',#{},#{}) REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#{transform}) SHAPE_REPRESENTATION_RELATIONSHIP() )",
                child_state.rep, parent_rep))?;
            let occurrence_name = if child.feature_id.is_empty() {
                archive.pvs.definitions[child.definition_index].source_name.clone()
            } else { child.feature_id.clone() };
            let usage = writer.ent(&format!("NEXT_ASSEMBLY_USAGE_OCCURRENCE({},{},'',#{parent_pd},#{},$)",
                encode_step_string(&occurrence_id.to_string()),
                encode_step_string(&occurrence_name),
                child_state.product.pd))?;
            occurrence_id += 1;
            let placement_shape = writer.ent(&format!("PRODUCT_DEFINITION_SHAPE('Placement','Placement of an item',#{usage})"))?;
            writer.ent(&format!("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#{relationship},#{placement_shape})"))?;
        }
    }
    totals.step_unique_appearances = writer.appearance_style_cache.len();
    totals.entities = writer.entity_count;
    totals.occurrences = (occurrence_id - 1) as usize;
    writer.finish()?;
    Ok(totals)
}
