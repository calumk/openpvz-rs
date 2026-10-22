//! Reverse-engineered OL exact-geometry and topology decoder.
//! Stops at a format-neutral CAD model; STEP syntax lives in `step.rs`.
use crate::binary::hex32;
use crate::constants::{exact_curve_type, exact_surface_type, ol_stream, ol_tag, NULL_INDEX};
use crate::ol::{iter_ol_tlv, parse_ol_streams};
use crate::{Result, openpvz_error};
use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};

pub use crate::ol::{OL_STREAM_EXACT_V3, OL_STREAM_EXACT_V4};

pub fn surface_type_name(code: u8) -> String {
    match code {
        x if x == exact_surface_type::CONE => "cone".into(),
        x if x == exact_surface_type::CYLINDER => "cylinder".into(),
        x if x == exact_surface_type::RATIONAL_BSPLINE => "rational_bspline".into(),
        x if x == exact_surface_type::PLANE => "plane".into(),
        x if x == exact_surface_type::SPHERE => "sphere".into(),
        x if x == exact_surface_type::TORUS => "torus".into(),
        _ => format!("unknown_{code}"),
    }
}
pub fn curve_type_name(code: u8) -> String {
    match code {
        x if x == exact_curve_type::CIRCLE => "circle".into(),
        x if x == exact_curve_type::DEGENERATE_POINT => "degenerate_point".into(),
        x if x == exact_curve_type::ELLIPSE => "ellipse".into(),
        x if x == exact_curve_type::LINE => "line".into(),
        x if x == exact_curve_type::RATIONAL_BSPLINE => "rational_bspline".into(),
        x if x == exact_curve_type::POLYLINE => "polyline".into(),
        _ => format!("unknown_{code}"),
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Precision { F32, F64 }
impl Precision {
    pub fn scalar_size(&self) -> usize { match self { Precision::F32 => 4, Precision::F64 => 8 } }
}
fn read_scalar(data: &[u8], off: usize, p: Precision) -> Result<f64> {
    match p {
        Precision::F64 => {
            if off + 8 > data.len() { return Err("truncated f64".into()); }
            Ok(f64::from_le_bytes(data[off..off+8].try_into().unwrap()))
        }
        Precision::F32 => {
            if off + 4 > data.len() { return Err("truncated f32".into()); }
            Ok(f32::from_le_bytes(data[off..off+4].try_into().unwrap()) as f64)
        }
    }
}
fn read_scalars(data: &[u8], off: usize, count: usize, p: Precision) -> Result<Vec<f64>> {
    (0..count).map(|i| read_scalar(data, off + i * p.scalar_size(), p)).collect()
}
fn read_u32le(b: &[u8], off: usize) -> Result<u32> {
    if off + 4 > b.len() { return Err("truncated u32".into()); }
    Ok(u32::from_le_bytes([b[off],b[off+1],b[off+2],b[off+3]]))
}
fn read_i32le(b: &[u8], off: usize) -> Result<i32> {
    if off + 4 > b.len() { return Err("truncated i32".into()); }
    Ok(i32::from_le_bytes([b[off],b[off+1],b[off+2],b[off+3]]))
}

pub type V3 = [f64; 3];
fn v3(v: &[f64], off: usize) -> V3 { [v[off], v[off+1], v[off+2]] }
fn norm3(v: V3) -> f64 { v[0].hypot(v[1]).hypot(v[2]) }
fn add3(a: V3, b: V3) -> V3 { [a[0]+b[0], a[1]+b[1], a[2]+b[2]] }
fn sub3(a: V3, b: V3) -> V3 { [a[0]-b[0], a[1]-b[1], a[2]-b[2]] }
fn combine3(terms: &[(&V3, f64)]) -> V3 {
    let mut o = [0.0; 3];
    for (v, s) in terms { o[0]+=v[0]*s; o[1]+=v[1]*s; o[2]+=v[2]*s; }
    o
}
fn distance3(a: V3, b: V3) -> f64 { norm3(sub3(a,b)) }

// ---------- surfaces / curves ----------

#[derive(Debug, Clone)]
pub struct BsplinePole3 { pub point: V3, pub weight: f64, pub homogeneous: [f64; 4] }
#[derive(Debug, Clone)]
pub struct BsplinePole2 { pub point: [f64; 2], pub weight: f64, pub homogeneous: [f64; 3] }

#[derive(Debug, Clone)]
pub enum ExactSurface {
    Plane { flags: u8, marker: u8, origin: V3, u_vector: V3, v_vector: V3, normal: V3, u_range: [f64;2], v_range: [f64;2], normalized: Option<Box<ExactSurface>>, trailing_bytes: usize },
    Cylinder { flags: u8, marker: u8, origin: V3, cos_vector: V3, sin_vector: V3, axis_step: V3, u_range: [f64;2], v_range: [f64;2], normalized: Option<Box<ExactSurface>>, trailing_bytes: usize },
    Cone { flags: u8, marker: u8, origin: V3, cos_growth: V3, sin_growth: V3, axis_step: V3, radial_offset: f64, radial_slope: f64, u_range: [f64;2], v_range: [f64;2], normalized: Option<Box<ExactSurface>>, trailing_bytes: usize },
    Sphere { flags: u8, marker: u8, center: V3, cos_vector: V3, sin_vector: V3, axis_vector: V3, u_range: [f64;2], v_range: [f64;2], normalized: Option<Box<ExactSurface>>, trailing_bytes: usize },
    Torus { flags: u8, marker: u8, origin: V3, major_cos: V3, major_sin: V3, axis_unit: V3, minor_cos: V3, minor_axis: V3, minor_sin_dir: V3, u_range: [f64;2], v_range: [f64;2], normalized: Option<Box<ExactSurface>>, trailing_bytes: usize },
    RationalBspline { flags: u8, marker: u8, reserved: u8, u_order: u32, v_order: u32, u_knots: Vec<f64>, v_knots: Vec<f64>, u_pole_count: usize, v_pole_count: usize, poles: Vec<BsplinePole3>, trailing_bytes: usize },
    Unknown { type_code: u8, flags: Option<u8>, marker: Option<u8>, raw_bytes: usize },
}

impl ExactSurface {
    pub fn type_name(&self) -> String {
        match self {
            ExactSurface::Plane{..} => "plane".into(),
            ExactSurface::Cylinder{..} => "cylinder".into(),
            ExactSurface::Cone{..} => "cone".into(),
            ExactSurface::Sphere{..} => "sphere".into(),
            ExactSurface::Torus{..} => "torus".into(),
            ExactSurface::RationalBspline{..} => "rational_bspline".into(),
            ExactSurface::Unknown{type_code,..} => surface_type_name(*type_code),
        }
    }
    pub fn normalized_surface(&self) -> Option<&ExactSurface> {
        match self {
            ExactSurface::Plane{normalized,..} | ExactSurface::Cylinder{normalized,..}
            | ExactSurface::Cone{normalized,..} | ExactSurface::Sphere{normalized,..}
            | ExactSurface::Torus{normalized,..} => normalized.as_deref(),
            ExactSurface::RationalBspline{..} => Some(self),
            ExactSurface::Unknown{..} => None,
        }
    }
}

#[derive(Debug, Clone)]
pub enum ExactCurve {
    Line { flags: u8, origin: V3, vector: V3, trailing_bytes: usize },
    Circle { flags: u8, center: V3, cos_vector: V3, sin_vector: V3, parameter_range: [f64;2], full_circle: bool, trailing_bytes: usize },
    Ellipse { flags: u8, center: V3, cos_vector: V3, sin_vector: V3, parameter_range: [f64;2], trailing_bytes: usize },
    DegeneratePoint { flags: u8, point: V3, trailing_bytes: usize },
    RationalBspline { flags: u8, reserved: u8, order: u32, knots: Vec<f64>, poles: Vec<BsplinePole3>, trailing_bytes: usize },
    Polyline { flags: u8, points: Vec<V3>, trailing_bytes: usize },
    Unknown { type_code: u8, flags: Option<u8>, raw_bytes: usize },
}

impl ExactCurve {
    pub fn type_name(&self) -> String {
        match self {
            ExactCurve::Line{..} => "line".into(),
            ExactCurve::Circle{..} => "circle".into(),
            ExactCurve::Ellipse{..} => "ellipse".into(),
            ExactCurve::DegeneratePoint{..} => "degenerate_point".into(),
            ExactCurve::RationalBspline{..} => "rational_bspline".into(),
            ExactCurve::Polyline{..} => "polyline".into(),
            ExactCurve::Unknown{type_code,..} => curve_type_name(*type_code),
        }
    }
}

#[derive(Debug, Clone)]
pub struct ExactPcurve {
    pub index: usize,
    pub type_code: u8,
    pub type_name: String,
    pub flags: u8,
    pub points: Vec<[f64; 2]>,
    // analytic detail (retained)
    pub origin: Option<[f64;2]>,
    pub vector: Option<[f64;2]>,
    pub center: Option<[f64;2]>,
    pub cos_vector: Option<[f64;2]>,
    pub sin_vector: Option<[f64;2]>,
    pub parameter_range: Option<[f64;2]>,
    pub order: Option<u32>,
    pub knots: Option<Vec<f64>>,
    pub poles: Option<Vec<BsplinePole2>>,
    pub terminator: Option<u8>,
    pub source_offset: usize,
}

#[derive(Debug, Clone)]
pub struct Coedge {
    pub edge_index: Option<usize>, // None == NULL_INDEX
    pub orientation: u8,
    pub pcurve_index: Option<usize>,
    pub normalized_pcurve_index: Option<usize>,
    pub collapsed_implicit: bool,
    pub curve_sense: Option<String>,
}

#[derive(Debug, Clone)]
pub struct ExactLoop {
    pub index: usize,
    pub marker: i32,
    pub coedges: Vec<Coedge>,
    pub degenerate_marker: bool,
}

#[derive(Debug, Clone)]
pub struct SingularUse { pub edge_index: usize, pub parameter_range: [f64; 2] }

#[derive(Debug, Clone)]
pub struct ExactFace {
    pub index: usize,
    pub surface: ExactSurface,
    pub loops: Vec<ExactLoop>,
    pub singular_uses: Vec<SingularUse>,
    pub source_offset: usize,
}

#[derive(Debug, Clone)]
pub struct ExactEdge { pub index: usize, pub curve: ExactCurve, pub source_offset: usize }

#[derive(Debug, Clone)]
pub struct FaceRegion {
    pub index: usize,
    pub face_index: usize,
    pub local_index: usize,
    pub outer_loop_global_index: Option<usize>,
    pub nesting_depth: usize,
    pub loop_refs: Vec<(usize, usize)>, // (face_index, loop_index) resolved at use
    pub singular_uses: Vec<SingularUse>,
}

#[derive(Debug, Clone)]
pub struct ShellCandidate {
    pub index: usize,
    pub face_region_indices: Vec<usize>,
    pub face_indices: Vec<usize>,
    pub edge_count: usize,
    pub boundary_edges: usize,
    pub non_manifold_edges: usize,
    pub degenerate_edges: usize,
    pub closed_candidate: bool,
}

#[derive(Debug, Clone)]
pub struct ExactModel {
    pub signature: String,
    pub exact_version: u32,
    pub precision: String,
    pub precision_enum: Precision,
    pub stream_type: u32,
    pub faces: Vec<ExactFace>,
    pub edges: Vec<ExactEdge>,
    pub pcurves: Vec<ExactPcurve>,
    pub unknown_tags: BTreeMap<String, usize>,
    pub face_regions: Vec<FaceRegion>,
    pub shells: Vec<ShellCandidate>,
}

// ---------- stream location ----------

pub struct FoundExact {
    pub signature: String,
    pub stream_type: u32,
    pub stream_data: Vec<u8>,
    pub exact_version: u32,
    pub precision: Precision,
}

pub fn find_ol_exact_stream(data: &[u8]) -> Result<Option<FoundExact>> {
    let parsed = parse_ol_streams(data)?;
    for s in parsed.streams.iter().filter(|s| s.stream_type == ol_stream::EXACT_V3 || s.stream_type == ol_stream::EXACT_V4) {
        let tlvs = iter_ol_tlv(&s.data)?;
        if let Some(first) = tlvs.first() {
            if first.tag == ol_tag::EXACT_VERSION && first.payload.len() == 4 {
                let v = u32::from_le_bytes([first.payload[0],first.payload[1],first.payload[2],first.payload[3]]);
                if v == 3 || v == 4 {
                    return Ok(Some(FoundExact {
                        signature: parsed.signature.clone(),
                        stream_type: s.stream_type,
                        stream_data: s.data.clone(),
                        exact_version: v,
                        precision: if v == 4 { Precision::F64 } else { Precision::F32 },
                    }));
                }
            }
        }
    }
    Ok(None)
}

// ---------- surface / curve parsing ----------

fn euclidean_pole3(h: &[f64]) -> BsplinePole3 {
    let (x,y,z,w) = (h[0],h[1],h[2],h[3]);
    let point = if w.abs() < 1e-30 { [x,y,z] } else { [x/w,y/w,z/w] };
    BsplinePole3 { point, weight: w, homogeneous: [x,y,z,w] }
}

fn parse_bspline_surface(payload: &[u8], p: Precision) -> Result<(ExactSurface, usize)> {
    let size = p.scalar_size();
    if payload.len() < 12 { return Err("short B-spline surface".into()); }
    let mut off = 4usize;
    let u_order = read_u32le(payload, off)?; off += 4;
    let u_knot_count = read_u32le(payload, off)? as usize; off += 4;
    if off + u_knot_count*size > payload.len() { return Err("truncated B-spline surface U knots".into()); }
    let u_knots = read_scalars(payload, off, u_knot_count, p)?; off += u_knot_count*size;
    if off + 8 > payload.len() { return Err("truncated B-spline surface V header".into()); }
    let v_order = read_u32le(payload, off)?; off += 4;
    let v_knot_count = read_u32le(payload, off)? as usize; off += 4;
    if off + v_knot_count*size > payload.len() { return Err("truncated B-spline surface V knots".into()); }
    let v_knots = read_scalars(payload, off, v_knot_count, p)?; off += v_knot_count*size;
    if off + 8 > payload.len() { return Err("truncated B-spline surface pole counts".into()); }
    let u_poles = read_u32le(payload, off)? as usize; off += 4;
    let v_poles = read_u32le(payload, off)? as usize; off += 4;
    if u_poles != u_knot_count - u_order as usize || v_poles != v_knot_count - v_order as usize {
        return Err(format!("unexpected B-spline surface dimensions: poles {u_poles}x{v_poles}, knots/orders {u_knot_count}/{u_order} x {v_knot_count}/{v_order}").into());
    }
    let pole_count = u_poles * v_poles;
    if off + pole_count*4*size > payload.len() { return Err("truncated B-spline surface poles".into()); }
    let mut poles = Vec::with_capacity(pole_count);
    for i in 0..pole_count {
        poles.push(euclidean_pole3(&read_scalars(payload, off + i*4*size, 4, p)?));
    }
    off += pole_count*4*size;
    Ok((ExactSurface::RationalBspline {
        flags: payload[1], marker: payload[2], reserved: payload[3],
        u_order, v_order, u_knots, v_knots, u_pole_count: u_poles, v_pole_count: v_poles,
        poles, trailing_bytes: payload.len() - off,
    }, off))
}

pub fn parse_exact_surface(payload: &[u8], p: Precision) -> Result<ExactSurface> {
    if payload.is_empty() { return Err("empty exact surface payload".into()); }
    let ty = payload[0];
    if ty == exact_surface_type::RATIONAL_BSPLINE {
        return Ok(parse_bspline_surface(payload, p)?.0);
    }
    let size = p.scalar_size();
    let parse_prim = |count: usize| -> Result<(Vec<f64>, &[u8])> {
        let expected = 3 + count*size;
        if payload.len() < expected {
            return Err(format!("short exact surface type {ty}: {} < {expected}", payload.len()).into());
        }
        Ok((read_scalars(payload, 3, count, p)?, &payload[expected..]))
    };
    // optional embedded normalized B-spline
    let embedded = |trailing: &[u8]| -> Option<Box<ExactSurface>> {
        if !trailing.is_empty() && trailing[0] == exact_surface_type::RATIONAL_BSPLINE {
            if let Ok((s, _)) = parse_bspline_surface(trailing, p) {
                // only accept if it consumes everything
                if let ExactSurface::RationalBspline{trailing_bytes: 0, ..} = s {
                    return Some(Box::new(s));
                }
            }
        }
        None
    };
    match ty {
        x if x == exact_surface_type::PLANE => {
            let (v, t) = parse_prim(16)?;
            Ok(ExactSurface::Plane { flags: payload[1], marker: payload[2],
                origin: v3(&v,0), u_vector: v3(&v,3), v_vector: v3(&v,6), normal: v3(&v,9),
                u_range: [v[12],v[13]], v_range: [v[14],v[15]],
                normalized: embedded(t), trailing_bytes: t.len() })
        }
        x if x == exact_surface_type::CYLINDER => {
            let (v, t) = parse_prim(16)?;
            Ok(ExactSurface::Cylinder { flags: payload[1], marker: payload[2],
                origin: v3(&v,0), cos_vector: v3(&v,3), sin_vector: v3(&v,6), axis_step: v3(&v,9),
                u_range: [v[12],v[13]], v_range: [v[14],v[15]],
                normalized: embedded(t), trailing_bytes: t.len() })
        }
        x if x == exact_surface_type::CONE => {
            let (v, t) = parse_prim(18)?;
            Ok(ExactSurface::Cone { flags: payload[1], marker: payload[2],
                origin: v3(&v,0), cos_growth: v3(&v,3), sin_growth: v3(&v,6), axis_step: v3(&v,9),
                radial_offset: v[12], radial_slope: v[13],
                u_range: [v[14],v[15]], v_range: [v[16],v[17]],
                normalized: embedded(t), trailing_bytes: t.len() })
        }
        x if x == exact_surface_type::SPHERE => {
            let (v, t) = parse_prim(16)?;
            Ok(ExactSurface::Sphere { flags: payload[1], marker: payload[2],
                center: v3(&v,0), cos_vector: v3(&v,3), sin_vector: v3(&v,6), axis_vector: v3(&v,9),
                u_range: [v[12],v[13]], v_range: [v[14],v[15]],
                normalized: embedded(t), trailing_bytes: t.len() })
        }
        x if x == exact_surface_type::TORUS => {
            let (v, t) = parse_prim(25)?;
            Ok(ExactSurface::Torus { flags: payload[1], marker: payload[2],
                origin: v3(&v,0), major_cos: v3(&v,3), major_sin: v3(&v,6), axis_unit: v3(&v,9),
                minor_cos: v3(&v,12), minor_axis: v3(&v,15), minor_sin_dir: v3(&v,18),
                u_range: [v[21],v[22]], v_range: [v[23],v[24]],
                normalized: embedded(t), trailing_bytes: t.len() })
        }
        _ => Ok(ExactSurface::Unknown { type_code: ty, flags: payload.get(1).copied(), marker: payload.get(2).copied(), raw_bytes: payload.len() }),
    }
}

fn parse_bspline_curve(payload: &[u8], p: Precision) -> Result<ExactCurve> {
    let size = p.scalar_size();
    if payload.len() < 11 { return Err("short B-spline curve".into()); }
    let order = read_u32le(payload, 3)?;
    let knot_count = read_u32le(payload, 7)? as usize;
    let mut off = 11usize;
    if off + knot_count*size + 4 > payload.len() { return Err("truncated B-spline curve knots".into()); }
    let knots = read_scalars(payload, off, knot_count, p)?; off += knot_count*size;
    let pole_count = read_u32le(payload, off)? as usize; off += 4;
    if pole_count != knot_count - order as usize {
        return Err(format!("unexpected B-spline curve pole count {pole_count}; knots/order imply {}", knot_count - order as usize).into());
    }
    if off + pole_count*4*size > payload.len() { return Err("truncated B-spline curve poles".into()); }
    let mut poles = Vec::with_capacity(pole_count);
    for i in 0..pole_count {
        poles.push(euclidean_pole3(&read_scalars(payload, off + i*4*size, 4, p)?));
    }
    off += pole_count*4*size;
    Ok(ExactCurve::RationalBspline { flags: payload[1], reserved: payload[2], order, knots, poles, trailing_bytes: payload.len() - off })
}

pub fn parse_exact_curve(payload: &[u8], p: Precision) -> Result<ExactCurve> {
    if payload.is_empty() { return Err("empty exact curve payload".into()); }
    let ty = payload[0];
    let size = p.scalar_size();
    if ty == exact_curve_type::LINE {
        let expected = 2 + 6*size;
        if payload.len() < expected { return Err("short line curve".into()); }
        let v = read_scalars(payload, 2, 6, p)?;
        return Ok(ExactCurve::Line { flags: payload[1], origin: v3(&v,0), vector: v3(&v,3), trailing_bytes: payload.len()-expected });
    }
    if ty == exact_curve_type::CIRCLE || ty == exact_curve_type::ELLIPSE {
        let compact = 2 + 9*size + 1;
        if ty == exact_curve_type::CIRCLE && payload.len() == compact {
            let v = read_scalars(payload, 2, 9, p)?;
            return Ok(ExactCurve::Circle { flags: payload[1], center: v3(&v,0), cos_vector: v3(&v,3), sin_vector: v3(&v,6), parameter_range: [0.0, std::f64::consts::PI*2.0], full_circle: true, trailing_bytes: 1 });
        }
        if payload.len() < 2 + 11*size { return Err(format!("short {} curve", if ty==1 {"circle"} else {"ellipse"}).into()); }
        let v = read_scalars(payload, 2, 11, p)?;
        let is_circle = ty == exact_curve_type::CIRCLE;
        if is_circle {
            return Ok(ExactCurve::Circle { flags: payload[1], center: v3(&v,0), cos_vector: v3(&v,3), sin_vector: v3(&v,6), parameter_range: [v[9],v[10]], full_circle: false, trailing_bytes: payload.len()-(2+11*size) });
        } else {
            return Ok(ExactCurve::Ellipse { flags: payload[1], center: v3(&v,0), cos_vector: v3(&v,3), sin_vector: v3(&v,6), parameter_range: [v[9],v[10]], trailing_bytes: payload.len()-(2+11*size) });
        }
    }
    if ty == exact_curve_type::DEGENERATE_POINT {
        let expected = 2 + 3*size;
        if payload.len() < expected { return Err("short degenerate-point edge".into()); }
        let v = read_scalars(payload, 2, 3, p)?;
        return Ok(ExactCurve::DegeneratePoint { flags: payload[1], point: [v[0],v[1],v[2]], trailing_bytes: payload.len()-expected });
    }
    if ty == exact_curve_type::RATIONAL_BSPLINE { return parse_bspline_curve(payload, p); }
    if ty == exact_curve_type::POLYLINE {
        if payload.len() < 7 { return Err("short polyline edge".into()); }
        let n = read_u32le(payload, 2)? as usize;
        let expected = 7 + n*3*size;
        if payload.len() != expected {
            return Err(format!("unexpected polyline edge size {}; expected {expected}", payload.len()).into());
        }
        let mut points = Vec::with_capacity(n);
        for i in 0..n {
            let v = read_scalars(payload, 6 + i*3*size, 3, p)?;
            points.push([v[0],v[1],v[2]]);
        }
        return Ok(ExactCurve::Polyline { flags: payload[1], points, trailing_bytes: 1 });
    }
    Ok(ExactCurve::Unknown { type_code: ty, flags: payload.get(1).copied(), raw_bytes: payload.len() })
}

// ---------- pcurves ----------

fn basis_functions(knots: &[f64], order: u32, param: f64) -> Result<(usize, Vec<f64>)> {
    let span = find_span(knots, order, param)?;
    let order = order as usize;
    let mut basis = vec![0.0; order];
    let mut left = vec![0.0; order];
    let mut right = vec![0.0; order];
    basis[0] = 1.0;
    for col in 1..order {
        left[col] = param - knots[span+1-col];
        right[col] = knots[span+col] - param;
        let mut saved: f64 = 0.0;
        for row in 0..col {
            let denom = right[row+1] + left[col-row];
            let factor = if denom == 0.0 { 0.0 } else { basis[row]/denom };
            basis[row] = saved + right[row+1]*factor;
            saved = left[col-row]*factor;
        }
        basis[col] = saved;
    }
    Ok((span, basis))
}
fn find_span(knots: &[f64], order: u32, param: f64) -> Result<usize> {
    let degree = order as usize - 1;
    let last_cp = knots.len() - order as usize - 1;
    if last_cp < degree { return Err("invalid B-spline knot/order combination".into()); }
    if param >= knots[last_cp+1] { return Ok(last_cp); }
    if param <= knots[degree] { return Ok(degree); }
    let (mut low, mut high) = (degree, last_cp+1);
    while high - low > 1 {
        let mid = (low+high)>>1;
        if param < knots[mid] { high = mid; } else { low = mid; }
    }
    Ok(low)
}
fn eval_rational_pcurve_2d(knots: &[f64], order: u32, poles: &[BsplinePole2], param: f64) -> Result<[f64;2]> {
    let (span, basis) = basis_functions(knots, order, param)?;
    let degree = order as usize - 1;
    let mut h = [0.0; 3];
    for li in 0..order as usize {
        let pi = span - degree + li;
        let pole = &poles[pi].homogeneous;
        for c in 0..3 { h[c] += basis[li]*pole[c]; }
    }
    if h[2].abs() < 1e-30 { Ok([h[0], h[1]]) } else { Ok([h[0]/h[2], h[1]/h[2]]) }
}
fn sample_rational_pcurve(knots: &[f64], order: u32, poles: &[BsplinePole2]) -> Result<Vec<[f64;2]>> {
    let degree = order as usize - 1;
    let lower = knots[degree];
    let upper = knots[knots.len()-order as usize];
    let mut params = vec![lower];
    let mut ki = degree;
    while ki < knots.len() - order as usize {
        let (a,b) = (knots[ki], knots[ki+1]);
        if b > a {
            for s in 1..=8 { params.push(a + (b-a)*(s as f64/8.0)); }
        }
        ki += 1;
    }
    if *params.last().unwrap() != upper { params.push(upper); }
    params.into_iter().map(|t| eval_rational_pcurve_2d(knots, order, poles, t)).collect()
}

pub fn parse_exact_pcurve(payload: &[u8], p: Precision, index: usize, offset: usize) -> Result<ExactPcurve> {
    if payload.len() < 3 { return Err("short exact pcurve record".into()); }
    let ty = payload[0];
    let size = p.scalar_size();
    let base = ExactPcurve { index, type_code: ty, type_name: String::new(), flags: payload[1], points: Vec::new(),
        origin: None, vector: None, center: None, cos_vector: None, sin_vector: None,
        parameter_range: None, order: None, knots: None, poles: None,
        terminator: Some(*payload.last().unwrap()), source_offset: offset };
    if ty == exact_curve_type::LINE {
        let expected = 2 + 4*size + 1;
        if payload.len() != expected {
            return Err(format!("unexpected 2D line pcurve size {}; expected {expected}", payload.len()).into());
        }
        let v = read_scalars(payload, 2, 4, p)?;
        let o = [v[0],v[1]]; let d = [v[2],v[3]];
        return Ok(ExactPcurve { type_name: "line".into(), origin: Some(o), vector: Some(d),
            points: vec![o, [o[0]+d[0], o[1]+d[1]]], ..base });
    }
    if ty == exact_curve_type::CIRCLE || ty == exact_curve_type::ELLIPSE {
        let expected = 2 + 8*size + 1;
        if payload.len() != expected {
            return Err(format!("unexpected 2D conic pcurve size {}; expected {expected}", payload.len()).into());
        }
        let v = read_scalars(payload, 2, 8, p)?;
        let (c, cv, sv, pr) = ([v[0],v[1]],[v[2],v[3]],[v[4],v[5]],[v[6],v[7]]);
        let span = (pr[1]-pr[0]).abs();
        let segs = (8.0f64).max((span/(std::f64::consts::PI/16.0)).ceil()) as usize;
        let mut points = Vec::with_capacity(segs+1);
        for i in 0..=segs {
            let t = pr[0] + (pr[1]-pr[0])*(i as f64/segs as f64);
            points.push([c[0]+cv[0]*t.cos()+sv[0]*t.sin(), c[1]+cv[1]*t.cos()+sv[1]*t.sin()]);
        }
        return Ok(ExactPcurve { type_name: if ty==1 {"circle".into()} else {"ellipse".into()},
            center: Some(c), cos_vector: Some(cv), sin_vector: Some(sv), parameter_range: Some(pr), points, ..base });
    }
    if ty == exact_curve_type::RATIONAL_BSPLINE {
        if payload.len() < 12 { return Err("short 2D B-spline pcurve".into()); }
        let order = read_u32le(payload, 3)?;
        let knot_count = read_u32le(payload, 7)? as usize;
        let mut off = 11usize;
        if off + knot_count*size + 4 > payload.len() { return Err("truncated 2D B-spline pcurve knots".into()); }
        let knots = read_scalars(payload, off, knot_count, p)?; off += knot_count*size;
        let pole_count = read_u32le(payload, off)? as usize; off += 4;
        if pole_count != knot_count - order as usize {
            return Err(format!("unexpected 2D B-spline pcurve pole count {pole_count}; knots/order imply {}", knot_count - order as usize).into());
        }
        if off + pole_count*3*size + 1 != payload.len() { return Err("unexpected 2D B-spline pcurve payload size".into()); }
        let mut poles = Vec::with_capacity(pole_count);
        for i in 0..pole_count {
            let h = read_scalars(payload, off + i*3*size, 3, p)?;
            let w = h[2];
            poles.push(BsplinePole2 {
                point: if w.abs()<1e-30 {[h[0],h[1]]} else {[h[0]/w,h[1]/w]},
                weight: w, homogeneous: [h[0],h[1],h[2]] });
        }
        let points = sample_rational_pcurve(&knots, order, &poles)?;
        return Ok(ExactPcurve { type_name: "rational_bspline".into(), order: Some(order),
            knots: Some(knots), poles: Some(poles), points, ..base });
    }
    if ty == exact_curve_type::POLYLINE {
        if payload.len() < 7 { return Err("short polyline pcurve".into()); }
        let n = read_u32le(payload, 2)? as usize;
        let expected = 7 + n*2*size;
        if payload.len() != expected {
            return Err(format!("unexpected pcurve size {}; expected {expected}", payload.len()).into());
        }
        let mut points = Vec::with_capacity(n);
        for i in 0..n {
            let v = read_scalars(payload, 6+i*2*size, 2, p)?;
            points.push([v[0],v[1]]);
        }
        return Ok(ExactPcurve { type_name: "polyline".into(), points, ..base });
    }
    Err(format!("unsupported exact pcurve type {ty}").into())
}

// ---------- complete stream parse ----------

pub fn parse_ol_exact_geometry(data: &[u8]) -> Result<Option<ExactModel>> {
    let found = find_ol_exact_stream(data)?;
    let Some(found) = found else { return Ok(None); };
    let tlvs = iter_ol_tlv(&found.stream_data)?;
    let mut faces: Vec<ExactFace> = Vec::new();
    let mut edges: Vec<ExactEdge> = Vec::new();
    let mut pcurves: Vec<ExactPcurve> = Vec::new();
    let mut unknown: BTreeMap<String, usize> = BTreeMap::new();
    let mut cur_face: Option<usize> = None;
    let mut cur_loop: Option<(usize, usize)> = None; // (face, loop)
    let mut stage = "faces";
    let mut saw_version = false;
    for rec in tlvs {
        let (tag, payload, offset) = (rec.tag, rec.payload, rec.offset);
        if tag == ol_tag::EXACT_VERSION && !saw_version {
            if payload.len() != 4 || read_u32le(payload,0)? != found.exact_version {
                return Err("inconsistent exact-geometry version record".into());
            }
            saw_version = true; continue;
        }
        if tag == ol_tag::EXACT_SURFACE && stage == "faces" {
            let idx = faces.len();
            faces.push(ExactFace { index: idx, surface: parse_exact_surface(payload, found.precision)?,
                loops: Vec::new(), singular_uses: Vec::new(), source_offset: offset });
            cur_face = Some(idx); cur_loop = None; continue;
        }
        if tag == ol_tag::EXACT_LOOP && stage == "faces" && cur_face.is_some() {
            if payload.len() != 4 { return Err(format!("unexpected face-loop marker size {}", payload.len()).into()); }
            let fi = cur_face.unwrap();
            let li = faces[fi].loops.len();
            faces[fi].loops.push(ExactLoop { index: li, marker: read_i32le(payload,0)?, coedges: Vec::new(), degenerate_marker: false });
            cur_loop = Some((fi, li)); continue;
        }
        if tag == ol_tag::EXACT_COEDGE && stage == "faces" && cur_face.is_some() && cur_loop.is_some() {
            if payload.len() != 13 { return Err(format!("unexpected coedge record size {}", payload.len()).into()); }
            let (fi, li) = cur_loop.unwrap();
            let edge_raw = read_u32le(payload,0)?;
            let pc_raw = read_u32le(payload,5)?;
            let npc_raw = read_u32le(payload,9)?;
            faces[fi].loops[li].coedges.push(Coedge {
                edge_index: if edge_raw == NULL_INDEX { None } else { Some(edge_raw as usize) },
                orientation: payload[4],
                pcurve_index: if pc_raw == NULL_INDEX { None } else { Some(pc_raw as usize) },
                normalized_pcurve_index: if npc_raw == NULL_INDEX { None } else { Some(npc_raw as usize) },
                collapsed_implicit: false, curve_sense: None,
            });
            continue;
        }
        if tag == ol_tag::EXACT_SINGULAR_USE && stage == "faces" && cur_face.is_some() {
            let size = found.precision.scalar_size();
            if payload.len() != 4 + 2*size {
                return Err(format!("unexpected singular-edge record size {}", payload.len()).into());
            }
            let fi = cur_face.unwrap();
            faces[fi].singular_uses.push(SingularUse {
                edge_index: read_u32le(payload,0)? as usize,
                parameter_range: [read_scalar(payload,4,found.precision)?, read_scalar(payload,4+size,found.precision)?],
            });
            continue;
        }
        if tag == ol_tag::TERMINATOR && stage == "faces" { cur_loop = None; continue; }
        if tag == ol_tag::EXACT_EDGE {
            stage = "edges"; cur_loop = None;
            let idx = edges.len();
            edges.push(ExactEdge { index: idx, curve: parse_exact_curve(payload, found.precision)?, source_offset: offset });
            continue;
        }
        if tag == ol_tag::EXACT_PCURVE {
            stage = "pcurves";
            let idx = pcurves.len();
            pcurves.push(parse_exact_pcurve(payload, found.precision, idx, offset)?);
            continue;
        }
        if tag != ol_tag::TERMINATOR {
            *unknown.entry(hex32(tag)).or_insert(0) += 1;
        }
    }
    if !saw_version { return Err("exact-geometry stream has no version marker".into()); }
    // index validation
    for f in &faces {
        for l in &f.loops {
            for c in &l.coedges {
                if let Some(ei) = c.edge_index {
                    if ei >= edges.len() { return Err(format!("coedge edge index {ei} out of range").into()); }
                }
                if let Some(pi) = c.pcurve_index {
                    if pi >= pcurves.len() { return Err(format!("coedge pcurve index {pi} out of range").into()); }
                }
                if let Some(ni) = c.normalized_pcurve_index {
                    if ni >= pcurves.len() { return Err(format!("coedge normalized pcurve index {ni} out of range").into()); }
                }
            }
        }
        for s in &f.singular_uses {
            if s.edge_index >= edges.len() { return Err(format!("singular edge index {} out of range", s.edge_index).into()); }
        }
    }
    let mut model = ExactModel {
        signature: found.signature.clone(), exact_version: found.exact_version,
        precision: if found.exact_version == 4 {"f64".into()} else {"f32".into()},
        precision_enum: found.precision, stream_type: found.stream_type,
        faces, edges, pcurves, unknown_tags: unknown,
        face_regions: Vec::new(), shells: Vec::new(),
    };
    model.face_regions = build_exact_face_regions(&mut model)?;
    model.shells = build_shell_candidates(&model);
    Ok(Some(model))
}

// ---------- evaluation ----------

fn dehom4(v: [f64;4]) -> V3 {
    if v[3].abs() < 1e-30 { [v[0],v[1],v[2]] } else { [v[0]/v[3],v[1]/v[3],v[2]/v[3]] }
}
fn eval_rational_curve(curve: &ExactCurve, param: f64) -> Option<V3> {
    if let ExactCurve::RationalBspline{order,knots,poles,..} = curve {
        let (span, basis) = basis_functions(knots, *order, param).ok()?;
        let degree = *order as usize - 1;
        let mut h = [0.0;4];
        for li in 0..*order as usize {
            let pole = &poles[span-degree+li].homogeneous;
            for c in 0..4 { h[c] += basis[li]*pole[c]; }
        }
        Some(dehom4(h))
    } else { None }
}
fn eval_rational_surface(s: &ExactSurface, u: f64, v: f64) -> Option<V3> {
    if let ExactSurface::RationalBspline{u_order,v_order,u_knots,v_knots,u_pole_count:_,v_pole_count,poles,..} = s {
        let (uspan, ub) = basis_functions(u_knots, *u_order, u).ok()?;
        let (vspan, vb) = basis_functions(v_knots, *v_order, v).ok()?;
        let (udeg, vdeg) = (*u_order as usize - 1, *v_order as usize - 1);
        let mut h = [0.0;4];
        for lu in 0..*u_order as usize {
            let ui = uspan - udeg + lu;
            for lv in 0..*v_order as usize {
                let vi = vspan - vdeg + lv;
                let pole = &poles[ui*v_pole_count+vi].homogeneous;
                let w = ub[lu]*vb[lv];
                for c in 0..4 { h[c] += w*pole[c]; }
            }
        }
        Some(dehom4(h))
    } else { None }
}

/// Evaluate a decoded supporting surface in OL native UV.
pub fn evaluate_exact_surface(surface: &ExactSurface, u: f64, v: f64) -> Option<V3> {
    match surface {
        ExactSurface::Plane{origin,u_vector,v_vector,..} =>
            Some(add3(*origin, combine3(&[(u_vector,u),(v_vector,v)]))),
        ExactSurface::Cylinder{origin,cos_vector,sin_vector,axis_step,..} =>
            Some(add3(*origin, combine3(&[(cos_vector,u.cos()),(sin_vector,u.sin()),(axis_step,v)]))),
        ExactSurface::Cone{origin,cos_growth,sin_growth,axis_step,radial_offset,radial_slope,..} => {
            let r = radial_offset + radial_slope*v;
            Some(add3(*origin, combine3(&[(cos_growth,u.cos()*r),(sin_growth,u.sin()*r),(axis_step,v)])))
        }
        ExactSurface::Sphere{center,cos_vector,sin_vector,axis_vector,..} => {
            let lat = v.cos();
            Some(add3(*center, combine3(&[(cos_vector,lat*u.cos()),(sin_vector,lat*u.sin()),(axis_vector,v.sin())])))
        }
        ExactSurface::Torus{origin,major_cos,major_sin,minor_cos,minor_axis,minor_sin_dir,..} => {
            let minor_r = norm3(*minor_cos);
            let major = combine3(&[(major_cos,u.cos()),(major_sin,u.sin())]);
            let minor = combine3(&[(minor_cos,u.cos()),(minor_sin_dir,-minor_r*u.sin())]);
            let m = combine3(&[(&minor,v.cos()),(minor_axis,v.sin())]);
            Some(add3(*origin, add3(major, m)))
        }
        ExactSurface::RationalBspline{..} => eval_rational_surface(surface, u, v),
        ExactSurface::Unknown{..} => None,
    }
}

/// Natural parameter interval of a 3D edge curve.
pub fn exact_curve_parameter_range(curve: &ExactCurve) -> Option<[f64;2]> {
    match curve {
        ExactCurve::Line{..} | ExactCurve::Polyline{..} => Some([0.0,1.0]),
        ExactCurve::Circle{parameter_range,..} => Some(*parameter_range),
        ExactCurve::Ellipse{parameter_range,..} => Some(*parameter_range),
        ExactCurve::DegeneratePoint{..} => Some([0.0,0.0]),
        ExactCurve::RationalBspline{order,knots,..} => {
            let d = *order as usize - 1;
            Some([knots[d], knots[knots.len()-*order as usize]])
        }
        ExactCurve::Unknown{..} => None,
    }
}

/// Evaluate one decoded 3D edge curve in native parameter space.
pub fn evaluate_exact_curve(curve: &ExactCurve, t: f64) -> Option<V3> {
    match curve {
        ExactCurve::Line{origin,vector,..} => Some(add3(*origin, combine3(&[(vector,t)]))),
        ExactCurve::Circle{center,cos_vector,sin_vector,..} |
        ExactCurve::Ellipse{center,cos_vector,sin_vector,..} =>
            Some(add3(*center, combine3(&[(cos_vector,t.cos()),(sin_vector,t.sin())]))),
        ExactCurve::DegeneratePoint{point,..} => Some(*point),
        ExactCurve::Polyline{points,..} => {
            if points.is_empty() { return None; }
            if points.len() == 1 { return Some(points[0]); }
            let scaled = t.max(0.0).min(1.0)*(points.len()-1) as f64;
            let idx = ((points.len()-2) as f64).min(scaled.floor()) as usize;
            let f = scaled - idx as f64;
            Some([0,1,2].map(|a| points[idx][a]+(points[idx+1][a]-points[idx][a])*f))
        }
        ExactCurve::RationalBspline{..} => eval_rational_curve(curve, t),
        ExactCurve::Unknown{..} => None,
    }
}

/// Natural start/end points of one decoded 3D edge curve.
pub fn exact_curve_endpoints(curve: &ExactCurve) -> Option<[V3;2]> {
    let r = exact_curve_parameter_range(curve)?;
    Some([evaluate_exact_curve(curve,r[0])?, evaluate_exact_curve(curve,r[1])?])
}

pub struct PcurveContext<'a> {
    pub index: usize,
    pub pcurve: &'a ExactPcurve,
    pub support: &'a ExactSurface,
    pub space: &'static str,
}

pub fn exact_coedge_pcurve_context<'a>(model: &'a ExactModel, face: &'a ExactFace, coedge: &Coedge) -> Option<PcurveContext<'a>> {
    if let Some(pi) = coedge.pcurve_index {
        return Some(PcurveContext { index: pi, pcurve: model.pcurves.get(pi)?, support: &face.surface, space: "native" });
    }
    if let Some(ni) = coedge.normalized_pcurve_index {
        let support = face.surface.normalized_surface()
            .or_else(|| if matches!(face.surface, ExactSurface::RationalBspline{..}) { Some(&face.surface) } else { None })?;
        return Some(PcurveContext { index: ni, pcurve: model.pcurves.get(ni)?, support, space: "normalized" });
    }
    None
}

#[derive(Debug, Clone)]
pub struct CoedgeClass { pub curve_sense: String, pub error: f64, pub forward_error: f64, pub reversed_error: f64 }

/// Geometric edge-sense classification (raw orientation byte NOT trusted).
pub fn classify_exact_coedge(model: &ExactModel, face: &ExactFace, coedge: &Coedge) -> Option<CoedgeClass> {
    let ei = coedge.edge_index?;
    let ctx = exact_coedge_pcurve_context(model, face, coedge)?;
    if ctx.pcurve.points.is_empty() { return None; }
    let edge = model.edges.get(ei)?;
    let [es, ee] = exact_curve_endpoints(&edge.curve)?;
    let first = ctx.pcurve.points[0];
    let last = *ctx.pcurve.points.last().unwrap();
    let ss = evaluate_exact_surface(ctx.support, first[0], first[1])?;
    let se = evaluate_exact_surface(ctx.support, last[0], last[1])?;
    if !ss.iter().all(|x| x.is_finite()) || !se.iter().all(|x| x.is_finite()) { return None; }
    let fwd = distance3(ss,es).max(distance3(se,ee));
    let rev = distance3(ss,ee).max(distance3(se,es));
    Some(CoedgeClass { curve_sense: if fwd <= rev {"forward".into()} else {"reversed".into()},
        error: fwd.min(rev), forward_error: fwd, reversed_error: rev })
}

#[derive(Debug, Clone)]
pub struct LoopClass { pub traversal: String, pub stored_error: f64, pub reversed_error: f64, pub pcurve_reversed_by_original_index: Vec<bool> }

pub fn classify_exact_loop(model: &ExactModel, face: &ExactFace, aloop: &ExactLoop) -> Option<LoopClass> {
    if aloop.coedges.is_empty() {
        return Some(LoopClass { traversal: "either".into(), stored_error: 0.0, reversed_error: 0.0, pcurve_reversed_by_original_index: vec![] });
    }
    let mut endpoints: Vec<[V3;2]> = Vec::new();
    for c in &aloop.coedges {
        let ctx = exact_coedge_pcurve_context(model, face, c)?;
        if ctx.pcurve.points.is_empty() { return None; }
        let first = ctx.pcurve.points[0];
        let last = *ctx.pcurve.points.last().unwrap();
        let s = evaluate_exact_surface(ctx.support, first[0], first[1])?;
        let e = evaluate_exact_surface(ctx.support, last[0], last[1])?;
        if !s.iter().all(|x| x.is_finite()) || !e.iter().all(|x| x.is_finite()) { return None; }
        endpoints.push([s,e]);
    }
    let normalized_only = aloop.coedges.iter().all(|c| c.pcurve_index.is_none() && c.normalized_pcurve_index.is_some());
    if normalized_only {
        let solve = |reversed_order: bool| {
            let n = endpoints.len();
            let order: Vec<usize> = if reversed_order { (0..n).rev().collect() } else { (0..n).collect() };
            let mut best: Option<(f64,f64,Vec<bool>)> = None;
            for first_rev in [false, true] {
                let mut rev_by: Vec<bool> = vec![false; n];
                let fi = order[0];
                rev_by[fi] = first_rev;
                let mut prev_end = if first_rev { endpoints[fi][0] } else { endpoints[fi][1] };
                let (mut max_e, mut sum_e): (f64, f64) = (0.0, 0.0);
                for oi in 1..order.len() {
                    let o = order[oi];
                    let (rs, re) = (endpoints[o][0], endpoints[o][1]);
                    let (gf, gr) = (distance3(prev_end,rs), distance3(prev_end,re));
                    let rev = gr < gf;
                    rev_by[o] = rev;
                    let gap = gf.min(gr);
                    max_e = max_e.max(gap); sum_e += gap;
                    prev_end = if rev { rs } else { re };
                }
                let fs = if first_rev { endpoints[fi][1] } else { endpoints[fi][0] };
                let closure = distance3(prev_end, fs);
                max_e = max_e.max(closure); sum_e += closure;
                let cand = (max_e, sum_e, rev_by);
                best = Some(match best {
                    None => cand,
                    Some(b) => if cand.0 < b.0 - 1e-14 || ((cand.0-b.0).abs() <= 1e-14 && cand.1 < b.1) { cand } else { b },
                });
            }
            best.unwrap()
        };
        let (smax, ssum, _) = solve(false);
        let (rmax, rsum, _) = solve(true);
        let choose_stored = smax < rmax - 1e-14 || ((smax-rmax).abs() <= 1e-14 && ssum <= rsum);
        let chosen = solve(!choose_stored);
        return Some(LoopClass { traversal: if choose_stored {"stored".into()} else {"reversed".into()},
            stored_error: smax, reversed_error: rmax, pcurve_reversed_by_original_index: chosen.2 });
    }
    let (mut stored_e, mut reversed_e): (f64, f64) = (0.0, 0.0);
    for i in 0..endpoints.len() {
        let n = endpoints.len();
        let (nx, pv) = ((i+1)%n, (i+n-1)%n);
        stored_e = stored_e.max(distance3(endpoints[i][1], endpoints[nx][0]));
        reversed_e = reversed_e.max(distance3(endpoints[i][1], endpoints[pv][0]));
    }
    let traversal = if (stored_e-reversed_e).abs() <= 1e-14 {"either"} else if stored_e < reversed_e {"stored"} else {"reversed"};
    Some(LoopClass { traversal: traversal.into(), stored_error: stored_e, reversed_error: reversed_e,
        pcurve_reversed_by_original_index: vec![false; aloop.coedges.len()] })
}

// ---------- validation ----------

#[derive(Debug, Clone, serde::Serialize)]
pub struct ExactValidation {
    pub ok: bool,
    pub tolerance: f64,
    pub tested: usize,
    pub skipped: usize,
    pub failures: usize,
    #[serde(rename = "maxError")]
    pub max_error: f64,
    #[serde(rename = "meanError")]
    pub mean_error: f64,
    #[serde(rename = "surfaceTypes")]
    pub surface_types: BTreeMap<String, usize>,
    #[serde(rename = "curveTypes")]
    pub curve_types: BTreeMap<String, usize>,
    pub errors: Vec<serde_json::Value>,
    #[serde(rename = "errorsTruncated")]
    pub errors_truncated: usize,
}

pub fn validate_exact_geometry(model: &ExactModel, tolerance: f64) -> ExactValidation {
    let mut errors = Vec::new();
    let mut surface_types: BTreeMap<String,usize> = BTreeMap::new();
    let mut curve_types: BTreeMap<String,usize> = BTreeMap::new();
    let (mut tested, mut skipped) = (0, 0);
    let (mut max_error, mut sum_error): (f64, f64) = (0.0, 0.0);
    for face in &model.faces {
        for aloop in &face.loops {
            for c in &aloop.coedges {
                let Some(ei) = c.edge_index else { skipped += 1; continue; };
                let Some(ctx) = exact_coedge_pcurve_context(model, face, c) else { skipped += 1; continue; };
                let Some(edge) = model.edges.get(ei) else { skipped += 1; continue; };
                let Some([es, ee]) = exact_curve_endpoints(&edge.curve) else { skipped += 1; continue; };
                if ctx.pcurve.points.is_empty() { skipped += 1; continue; }
                let first = ctx.pcurve.points[0];
                let last = *ctx.pcurve.points.last().unwrap();
                let (Some(ss), Some(se)) = (
                    evaluate_exact_surface(ctx.support, first[0], first[1]),
                    evaluate_exact_surface(ctx.support, last[0], last[1])) else { skipped += 1; continue; };
                if !ss.iter().all(|x| x.is_finite()) || !se.iter().all(|x| x.is_finite()) { skipped += 1; continue; }
                let fwd = distance3(ss,es).max(distance3(se,ee));
                let rev = distance3(ss,ee).max(distance3(se,es));
                let err = fwd.min(rev);
                tested += 1; sum_error += err; max_error = max_error.max(err);
                *surface_types.entry(face.surface.type_name()).or_insert(0) += 1;
                *curve_types.entry(edge.curve.type_name()).or_insert(0) += 1;
                if err > tolerance {
                    errors.push(serde_json::json!({
                        "faceIndex": face.index, "edgeIndex": edge.index,
                        "surfaceType": face.surface.type_name(), "curveType": edge.curve.type_name(),
                        "error": err }));
                }
            }
        }
    }
    let failures = errors.len();
    let truncated = errors.len().saturating_sub(100);
    errors.truncate(100);
    ExactValidation { ok: failures == 0, tolerance, tested, skipped, failures,
        max_error, mean_error: if tested>0 {sum_error/tested as f64} else {0.0},
        surface_types, curve_types, errors, errors_truncated: truncated }
}

// ---------- UV nesting / face regions ----------

fn exact_surface_periods(s: &ExactSurface) -> (Option<f64>, Option<f64>) {
    match s {
        ExactSurface::Cylinder{..} | ExactSurface::Cone{..} | ExactSurface::Sphere{..} =>
            (Some(std::f64::consts::PI*2.0), None),
        ExactSurface::Torus{..} => (Some(std::f64::consts::PI*2.0), Some(std::f64::consts::PI*2.0)),
        _ => (None, None),
    }
}

fn align_loop_pcurves(model: &ExactModel, face: &ExactFace, aloop: &ExactLoop) -> Vec<[f64;2]> {
    let class = classify_exact_loop(model, face, aloop);
    let mut order: Vec<usize> = (0..aloop.coedges.len()).collect();
    if class.as_ref().map(|c| c.traversal.as_str()) == Some("reversed") { order.reverse(); }
    // periods come from the face's own (normalized, if present) support surface
    let periods = exact_surface_periods(face.surface.normalized_surface().unwrap_or(&face.surface));
    let mut points: Vec<[f64;2]> = Vec::new();
    let mut prev_end: Option<[f64;2]> = None;
    for &oi in &order {
        let c = &aloop.coedges[oi];
        let ctx = match exact_coedge_pcurve_context(model, face, c) { Some(x) => x, None => continue };
        if ctx.pcurve.points.is_empty() { continue; }
        let mut raw: Vec<[f64;2]> = ctx.pcurve.points.clone();
        if class.as_ref().map(|cl| cl.pcurve_reversed_by_original_index.get(oi).copied().unwrap_or(false)).unwrap_or(false) {
            raw.reverse();
        }
        let (mut uo, mut vo): (f64, f64) = (0.0, 0.0);
        if let Some(pe) = prev_end {
            if let Some(p) = periods.0 { uo = ((pe[0]-raw[0][0])/p).round()*p; }
            if let Some(p) = periods.1 { vo = ((pe[1]-raw[0][1])/p).round()*p; }
        }
        let seg: Vec<[f64;2]> = raw.iter().map(|pt| [pt[0]+uo, pt[1]+vo]).collect();
        if points.is_empty() { points.extend(seg); }
        else {
            let prev = *points.last().unwrap();
            let dup = ((prev[0]-seg[0][0]).hypot(prev[1]-seg[0][1])) < 1e-10;
            points.extend(seg.into_iter().skip(if dup {1} else {0}));
        }
        prev_end = points.last().copied();
    }
    if points.len() > 2 {
        let (f, l) = (points[0], *points.last().unwrap());
        if (f[0]-l[0]).hypot(f[1]-l[1]) < 1e-10 { points.pop(); }
    }
    points
}

fn polygon_area(poly: &[[f64;2]]) -> f64 {
    let mut t: f64 = 0.0;
    for i in 0..poly.len() {
        let (a, b) = (poly[i], poly[(i+1)%poly.len()]);
        t += a[0]*b[1]-b[0]*a[1];
    }
    t/2.0
}
fn point_in_polygon(pt: [f64;2], poly: &[[f64;2]]) -> bool {
    let mut inside = false;
    let mut prev = poly.len()-1;
    for i in 0..poly.len() {
        let (c, p) = (poly[i], poly[prev]);
        if (c[1] > pt[1]) != (p[1] > pt[1]) {
            let iu = (p[0]-c[0])*(pt[1]-c[1])/(p[1]-c[1])+c[0];
            if pt[0] < iu { inside = !inside; }
        }
        prev = i;
    }
    inside
}
fn interior_sample(poly: &[[f64;2]]) -> [f64;2] {
    if poly.is_empty() { return [0.0,0.0]; }
    let area = polygon_area(poly);
    let (mut mnu, mut mnv) = (f64::INFINITY, f64::INFINITY);
    let (mut mxu, mut mxv) = (f64::NEG_INFINITY, f64::NEG_INFINITY);
    for p in poly { mnu=mnu.min(p[0]); mnv=mnv.min(p[1]); mxu=mxu.max(p[0]); mxv=mxv.max(p[1]); }
    let eps = (mxu-mnu).max(mxv-mnv).max(1.0)*1e-7;
    let orient = if area >= 0.0 {1.0} else {-1.0};
    let (mut best, mut best_len) = (None, -1.0);
    for i in 0..poly.len() {
        let (a, b) = (poly[i], poly[(i+1)%poly.len()]);
        let (dx, dy) = (b[0]-a[0], b[1]-a[1]);
        let len = dx.hypot(dy);
        if len < 1e-14 { continue; }
        let cand = [(a[0]+b[0])/2.0 + orient*(-dy/len)*eps, (a[1]+b[1])/2.0 + orient*(dx/len)*eps];
        if point_in_polygon(cand, poly) && len > best_len { best = Some(cand); best_len = len; }
    }
    best.unwrap_or([(mnu+mxu)/2.0_f64.max(1.0)*0.0 + poly.iter().map(|p| p[0]).sum::<f64>()/poly.len() as f64,
        poly.iter().map(|p| p[1]).sum::<f64>()/poly.len() as f64])
}
fn periodic_containment(sample: [f64;2], parent: &[[f64;2]], periods: (Option<f64>, Option<f64>)) -> bool {
    let us: Vec<i32> = if periods.0.is_some() { vec![-2,-1,0,1,2] } else { vec![0] };
    let vs: Vec<i32> = if periods.1.is_some() { vec![-2,-1,0,1,2] } else { vec![0] };
    for u in &us { for v in &vs {
        let s = [sample[0]+*u as f64*periods.0.unwrap_or(0.0), sample[1]+*v as f64*periods.1.unwrap_or(0.0)];
        if point_in_polygon(s, parent) { return true; }
    }}
    false
}

struct LoopInfo { pos: usize, area: f64, polygon: Vec<[f64;2]>, sample: [f64;2], parent: Option<usize>, depth: usize }

fn split_loop_group_by_nesting(model: &ExactModel, face: &ExactFace, loops: &[(usize, usize)]) -> Vec<(Vec<(usize,usize)>, Vec<(usize,usize)>, usize)> {
    // loops: list of (face_index, loop_index)
    if loops.len() <= 1 {
        return vec![(loops.to_vec(), vec![], 0)];
    }
    let periods = exact_surface_periods(face.surface.normalized_surface().unwrap_or(&face.surface));
    let get_loop = |r: &(usize, usize)| &model.faces[r.0].loops[r.1];
    let mut info: Vec<LoopInfo> = loops.iter().enumerate().map(|(pos, r)| {
        let l = get_loop(r);
        let poly = align_loop_pcurves(model, face, l);
        let area = if poly.len() >= 3 { polygon_area(&poly) } else { 0.0 };
        let sample = if poly.len() >= 3 { interior_sample(&poly) } else { [0.0,0.0] };
        LoopInfo { pos, area, polygon: poly, sample, parent: None, depth: 0 }
    }).collect();
    for ci in 0..info.len() {
        let mut parents: Vec<usize> = info.iter().enumerate()
            .filter(|(pi, p)| *pi != ci && p.area.abs() > info[ci].area.abs() + 1e-14
                && !p.polygon.is_empty() && !info[ci].polygon.is_empty()
                && periodic_containment(info[ci].sample, &p.polygon, periods))
            .map(|(pi,_)| pi).collect();
        parents.sort_by(|&a,&b| info[a].area.abs().partial_cmp(&info[b].area.abs()).unwrap());
        info[ci].parent = parents.first().copied();
    }
    // depths
    for i in 0..info.len() {
        let mut depth = 0;
        let mut p = info[i].parent;
        let mut seen = HashSet::new();
        while let Some(pi) = p {
            if !seen.insert(pi) { break; }
            depth += 1;
            p = info[pi].parent;
        }
        info[i].depth = depth;
    }
    info.iter().filter(|x| x.depth % 2 == 0).map(|outer| {
        let holes: Vec<(usize,usize)> = info.iter()
            .filter(|x| x.parent == Some(outer.pos) && x.depth == outer.depth + 1)
            .map(|x| loops[x.pos]).collect();
        (vec![loops[outer.pos]], holes, outer.depth)
    }).collect()
}

/// Convert supporting-surface records into topological face regions.
///
/// Marker semantics (validated): `-2` starts an outer-loop group,
/// `>= 0` is a backward reference to the global index of an earlier `-2`,
/// `-1` is degenerate/null (diagnostic only, no STEP face).
/// UV nesting depth is resolved separately: even depth = material, odd = hole.
pub fn build_exact_face_regions(model: &mut ExactModel) -> Result<Vec<FaceRegion>> {
    // Phase A: mark degenerate loops, snapshot markers.
    let mut markers: Vec<(usize, usize, i32)> = Vec::new();
    for face in model.faces.iter_mut() {
        for aloop in face.loops.iter_mut() {
            if aloop.marker == -1 {
                aloop.degenerate_marker = true;
            } else {
                markers.push((face.index, aloop.index, aloop.marker));
            }
        }
    }
    // Global loop indices cover every loop, including degenerate (-1) ones,
    // because backreference markers count loops in stored order. Degenerate
    // loops never enter a group, so their indices are never looked up.
    let mut global: HashMap<(usize, usize), usize> = HashMap::new();
    let mut next = 0usize;
    for f in model.faces.iter() {
        for l in &f.loops { global.insert((f.index, l.index), next); next += 1; }
    }
    struct Group { face_index: usize, outer_global: usize, loops: Vec<(usize, usize)> }
    let mut groups: Vec<Group> = Vec::new();
    let mut group_by_outer: HashMap<usize, usize> = HashMap::new();
    for (fi, li, marker) in markers {
        let gi = global[&(fi, li)];
        if marker == -2 {
            let idx = groups.len();
            groups.push(Group { face_index: fi, outer_global: gi, loops: vec![(fi, li)] });
            group_by_outer.insert(gi, idx);
        } else if marker >= 0 {
            let gi_out = marker as usize;
            let gidx = *group_by_outer.get(&gi_out).ok_or_else(|| openpvz_error(format!("loop marker {marker} does not reference an earlier outer loop")))?;
            if groups[gidx].face_index != fi {
                return Err(format!("loop marker {marker} crosses supporting-surface records").into());
            }
            groups[gidx].loops.push((fi, li));
        } else {
            return Err(format!("unsupported exact loop marker {marker}").into());
        }
    }
    // Phase B: nesting with full model access (edges/pcurves available).
    let mut regions: Vec<FaceRegion> = Vec::new();
    let mut local_count: HashMap<usize, usize> = HashMap::new();
    for g in &groups {
        let face: ExactFace = model.faces.iter().find(|f| f.index == g.face_index).cloned()
            .ok_or_else(|| openpvz_error(format!("face {} not found", g.face_index)))?;
        let nested = split_loop_group_by_nesting(model, &face, &g.loops);
        for (outer_loops, holes, depth) in nested {
            let local = *local_count.get(&face.index).unwrap_or(&0);
            local_count.insert(face.index, local + 1);
            let mut refs = outer_loops;
            refs.extend(holes);
            regions.push(FaceRegion { index: regions.len(), face_index: face.index,
                local_index: local, outer_loop_global_index: Some(g.outer_global),
                nesting_depth: depth, loop_refs: refs, singular_uses: vec![] });
        }
    }
    // Phase C: singular uses.
    let faces_snapshot = model.faces.clone();
    for f in &faces_snapshot {
        let mut local: Vec<usize> = regions.iter().filter(|r| r.face_index == f.index).map(|r| r.index).collect();
        if local.is_empty() && !f.singular_uses.is_empty() {
            let idx = regions.len();
            regions.push(FaceRegion { index: idx, face_index: f.index, local_index: 0,
                outer_loop_global_index: None, nesting_depth: 0, loop_refs: vec![], singular_uses: vec![] });
            local = vec![idx];
        }
        for s in &f.singular_uses {
            let mut target: Option<usize> = None;
            for &ri in &local {
                let mut uses = false;
                for (rfi, rli) in &regions[ri].loop_refs.clone() {
                    let fl = faces_snapshot.iter().find(|x| x.index == *rfi)
                        .and_then(|x| x.loops.iter().find(|l| l.index == *rli));
                    if let Some(fl) = fl {
                        if fl.coedges.iter().any(|c| c.edge_index == Some(s.edge_index)) { uses = true; break; }
                    }
                }
                if uses { target = Some(ri); break; }
            }
            let t = target.or_else(|| local.first().copied());
            if let Some(t) = t { regions[t].singular_uses.push(s.clone()); }
        }
    }
    Ok(regions)
}

// ---------- shells ----------

fn collect_singular_edge_indices(model: &ExactModel) -> HashSet<usize> {
    model.faces.iter().flat_map(|f| f.singular_uses.iter().map(|s| s.edge_index)).collect()
}
fn edge_closes_on_itself(model: &ExactModel, edge_index: usize) -> bool {
    match model.edges.get(edge_index).and_then(|e| exact_curve_endpoints(&e.curve)) {
        Some([a,b]) => distance3(a,b) <= 1e-9,
        None => false,
    }
}

pub fn build_shell_candidates(model: &ExactModel) -> Vec<ShellCandidate> {
    let regions: Vec<&FaceRegion> = model.face_regions.iter()
        .filter(|r| !r.loop_refs.is_empty() || !r.singular_uses.is_empty()).collect();
    let region_by_index: HashMap<usize, &FaceRegion> = model.face_regions.iter().map(|r| (r.index, r)).collect();
    let singular = collect_singular_edge_indices(model);
    // uses by edge
    let mut uses_by_edge: HashMap<usize, Vec<usize>> = HashMap::new(); // edge -> region indices
    let loop_of = |rfi: usize, rli: usize| -> Option<&ExactLoop> {
        model.faces.iter().find(|f| f.index == rfi)?.loops.iter().find(|l| l.index == rli)
    };
    for r in &regions {
        for (rfi, rli) in &r.loop_refs {
            if let Some(l) = loop_of(*rfi, *rli) {
                for c in &l.coedges {
                    if let Some(ei) = c.edge_index {
                        uses_by_edge.entry(ei).or_default().push(r.index);
                    }
                }
            }
        }
    }
    // adjacency (BTree for deterministic traversal order)
    let mut adj: BTreeMap<usize, BTreeSet<usize>> = regions.iter().map(|r| (r.index, BTreeSet::new())).collect();
    for uses in uses_by_edge.values() {
        let uniq: HashSet<usize> = uses.iter().copied().collect();
        let v: Vec<usize> = uniq.into_iter().collect();
        for &a in &v { for &b in &v {
            if a != b { adj.get_mut(&a).unwrap().insert(b); }
        }}
    }
    let mut seen = HashSet::new();
    let mut shells = Vec::new();
    for seed in &regions {
        if seen.contains(&seed.index) { continue; }
        let mut stack = vec![seed.index];
        let mut comp_regions = Vec::new();
        let mut comp_edges = HashSet::new();
        seen.insert(seed.index);
        while let Some(ri) = stack.pop() {
            comp_regions.push(ri);
            if let Some(r) = region_by_index.get(&ri) {
                for (rfi, rli) in &r.loop_refs {
                    if let Some(l) = loop_of(*rfi, *rli) {
                        for c in &l.coedges { if let Some(ei) = c.edge_index { comp_edges.insert(ei); } }
                    }
                }
            }
            if let Some(neigh) = adj.get(&ri).cloned() {
                for n in neigh { if seen.insert(n) { stack.push(n); } }
            }
        }
        let region_set: HashSet<usize> = comp_regions.iter().copied().collect();
        let (mut boundary, mut nonman, mut degen) = (0, 0, 0);
        for &ei in &comp_edges {
            let edge = model.edges.get(ei);
            let topo_singular = matches!(edge.map(|e| &e.curve), Some(ExactCurve::DegeneratePoint{..})) || singular.contains(&ei);
            if topo_singular { degen += 1; continue; }
            let count = uses_by_edge.get(&ei).map(|v| v.iter().filter(|r| region_set.contains(r)).count()).unwrap_or(0);
            if count == 1 { boundary += 1; } else if count != 2 { nonman += 1; }
        }
        let mut face_indices: Vec<usize> = comp_regions.iter().map(|i| region_by_index[i].face_index).collect::<HashSet<_>>().into_iter().collect();
        face_indices.sort_unstable();
        comp_regions.sort_unstable();
        shells.push(ShellCandidate { index: shells.len(), face_region_indices: comp_regions,
            face_indices, edge_count: comp_edges.len(), boundary_edges: boundary,
            non_manifold_edges: nonman, degenerate_edges: degen,
            closed_candidate: boundary == 0 && nonman == 0 });
    }
    shells
}

// ---------- summaries / vertices / occ ----------

#[derive(Debug, Clone, serde::Serialize)]
pub struct ExactSummary {
    pub signature: String,
    #[serde(rename = "exactVersion")] pub exact_version: u32,
    pub precision: String,
    #[serde(rename = "streamType")] pub stream_type: String,
    pub faces: usize,
    #[serde(rename = "topologicalFaces")] pub topological_faces: usize,
    pub loops: usize,
    pub coedges: usize,
    #[serde(rename = "singularUses")] pub singular_uses: usize,
    #[serde(rename = "degenerateMarkerLoops")] pub degenerate_marker_loops: usize,
    #[serde(rename = "faceRegions")] pub face_regions: usize,
    pub edges: usize,
    pub pcurves: usize,
    #[serde(rename = "surfaceTypes")] pub surface_types: BTreeMap<String, usize>,
    #[serde(rename = "curveTypes")] pub curve_types: BTreeMap<String, usize>,
    #[serde(rename = "edgeUseHistogram")] pub edge_use_histogram: BTreeMap<String, usize>,
    #[serde(rename = "shellCandidates")] pub shell_candidates: usize,
    #[serde(rename = "closedShellCandidates")] pub closed_shell_candidates: usize,
    #[serde(rename = "unknownTags")] pub unknown_tags: BTreeMap<String, usize>,
}

pub fn exact_geometry_summary(model: &ExactModel) -> ExactSummary {
    let mut surface_types: BTreeMap<String,usize> = BTreeMap::new();
    let mut curve_types: BTreeMap<String,usize> = BTreeMap::new();
    let mut edge_histogram: BTreeMap<String,usize> = BTreeMap::new();
    let mut edge_use_counts = vec![0usize; model.edges.len()];
    let (mut loops, mut coedges, mut singular, mut topo, mut degen) = (0,0,0,0,0);
    for f in &model.faces {
        if !f.loops.is_empty() || !f.singular_uses.is_empty() { topo += 1; }
        loops += f.loops.len();
        singular += f.singular_uses.len();
        *surface_types.entry(f.surface.type_name()).or_insert(0) += 1;
        for l in &f.loops {
            if l.marker == -1 { degen += 1; }
            for c in &l.coedges {
                coedges += 1;
                if let Some(ei) = c.edge_index { if ei < edge_use_counts.len() { edge_use_counts[ei] += 1; } }
            }
        }
    }
    for e in &model.edges { *curve_types.entry(e.curve.type_name()).or_insert(0) += 1; }
    for c in edge_use_counts { *edge_histogram.entry(c.to_string()).or_insert(0) += 1; }
    ExactSummary { signature: model.signature.clone(), exact_version: model.exact_version,
        precision: model.precision.clone(), stream_type: hex32(model.stream_type),
        faces: model.faces.len(), topological_faces: topo, loops, coedges, singular_uses: singular,
        degenerate_marker_loops: degen, face_regions: model.face_regions.len(),
        edges: model.edges.len(), pcurves: model.pcurves.len(), surface_types, curve_types,
        edge_use_histogram: edge_histogram,
        shell_candidates: model.shells.len(),
        closed_shell_candidates: model.shells.iter().filter(|s| s.closed_candidate).count(),
        unknown_tags: model.unknown_tags.clone() }
}

// union-find
struct UnionFind { parent: Vec<usize>, rank: Vec<u8> }
impl UnionFind {
    fn new(n: usize) -> Self { Self { parent: (0..n).collect(), rank: vec![0; n] } }
    fn find(&mut self, mut x: usize) -> usize {
        let mut root = x;
        while self.parent[root] != root { root = self.parent[root]; }
        while self.parent[x] != x { let p = self.parent[x]; self.parent[x] = root; x = p; }
        root
    }
    fn union(&mut self, a: usize, b: usize) {
        let (mut ra, mut rb) = (self.find(a), self.find(b));
        if ra == rb { return; }
        if self.rank[ra] < self.rank[rb] { std::mem::swap(&mut ra, &mut rb); }
        self.parent[rb] = ra;
        if self.rank[ra] == self.rank[rb] { self.rank[ra] += 1; }
    }
}

fn natural_endpoint_slot(sense: &str, is_start: bool) -> usize {
    let fwd = sense == "forward";
    if is_start { if fwd {0} else {1} } else { if fwd {1} else {0} }
}

#[derive(Debug, Clone, serde::Serialize)]
pub struct VertexInfo { pub index: usize, pub point: V3, pub spread: f64, #[serde(rename = "endpointSlots")] pub endpoint_slots: Vec<usize> }
#[derive(Debug, Clone, serde::Serialize)]
pub struct EdgeVertex { #[serde(rename = "edgeIndex")] pub edge_index: usize, #[serde(rename = "startVertex")] pub start_vertex: Option<usize>, #[serde(rename = "endVertex")] pub end_vertex: Option<usize> }
#[derive(Debug, Clone)]
pub struct VertexTopology { pub vertices: Vec<VertexInfo>, pub edges: Vec<EdgeVertex>, pub slot_to_vertex: Vec<Option<usize>>, pub max_spread: f64 }

pub fn build_exact_vertex_topology(model: &ExactModel) -> VertexTopology {
    let endpoint_count = model.edges.len()*2;
    let mut uf = UnionFind::new(endpoint_count.max(1));
    let singular = collect_singular_edge_indices(model);
    for e in &model.edges {
        if matches!(e.curve, ExactCurve::DegeneratePoint{..}) || singular.contains(&e.index) {
            uf.union(e.index*2, e.index*2+1);
        }
    }
    for face in &model.faces {
        for aloop in &face.loops {
            let class = classify_exact_loop(model, face, aloop);
            let mut ordered: Vec<usize> = (0..aloop.coedges.len()).collect();
            if class.as_ref().map(|c| c.traversal.as_str()) == Some("reversed") { ordered.reverse(); }
            if ordered.is_empty() { continue; }
            // classified senses with pcurve-reverse adjustment
            let mut classified: Vec<(usize, Option<String>)> = Vec::new();
            for &oi in &ordered {
                let c = &aloop.coedges[oi];
                let mut sense = classify_exact_coedge(model, face, c).map(|x| x.curve_sense);
                let rev = class.as_ref().and_then(|cl| cl.pcurve_reversed_by_original_index.get(oi).copied()).unwrap_or(false);
                if sense.is_some() && rev {
                    sense = Some(if sense.as_deref() == Some("forward") {"reversed".into()} else {"forward".into()});
                }
                classified.push((oi, sense));
            }
            let connectable: Vec<usize> = classified.iter().enumerate()
                .filter(|(_, (oi, s))| s.is_some() || !aloop.coedges[*oi].collapsed_implicit)
                .map(|(i,_)| i).collect();
            let ce: Vec<usize> = connectable.iter().copied().filter(|&i| classified[i].1.is_some()).collect();
            for k in 0..ce.len() {
                let (ci, ni) = (ce[k], ce[(k+1)%ce.len()]);
                let (oi_c, oi_n) = (classified[ci].0, classified[ni].0);
                if ci == ni && !edge_closes_on_itself(model, aloop.coedges[oi_c].edge_index.unwrap_or(usize::MAX)) { continue; }
                // adjacency in connectable order
                let pos_c = connectable.iter().position(|&x| x == ci).unwrap();
                let pos_n = connectable.iter().position(|&x| x == ni).unwrap();
                if (pos_c+1)%connectable.len() != pos_n { continue; }
                let (Some(ei_c), Some(ei_n)) = (aloop.coedges[oi_c].edge_index, aloop.coedges[oi_n].edge_index) else { continue };
                let (Some(s_c), Some(s_n)) = (classified[ci].1.as_deref(), classified[ni].1.as_deref()) else { continue };
                uf.union(ei_c*2+natural_endpoint_slot(s_c,false), ei_n*2+natural_endpoint_slot(s_n,true));
            }
        }
    }
    let mut slot_points: Vec<Option<V3>> = vec![None; endpoint_count];
    for e in &model.edges {
        if let Some([a,b]) = exact_curve_endpoints(&e.curve) {
            slot_points[e.index*2] = Some(a);
            slot_points[e.index*2+1] = Some(b);
        }
    }
    let mut by_root: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for s in 0..slot_points.len() { by_root.entry(uf.find(s)).or_default().push(s); }
    let mut vertices = Vec::new();
    let mut slot_to_vertex: Vec<Option<usize>> = vec![None; slot_points.len()];
    let mut max_spread: f64 = 0.0;
    for slots in by_root.values() {
        let pts: Vec<V3> = slots.iter().filter_map(|&s| slot_points[s]).filter(|p| p.iter().all(|x| x.is_finite())).collect();
        if pts.is_empty() { continue; }
        let n = pts.len() as f64;
        let avg = [pts.iter().map(|p| p[0]).sum::<f64>()/n, pts.iter().map(|p| p[1]).sum::<f64>()/n, pts.iter().map(|p| p[2]).sum::<f64>()/n];
        let spread = pts.iter().map(|p| distance3(*p, avg)).fold(0.0, f64::max);
        max_spread = max_spread.max(spread);
        let idx = vertices.len();
        vertices.push(VertexInfo { index: idx, point: avg, spread, endpoint_slots: slots.clone() });
        for &s in slots { slot_to_vertex[s] = Some(idx); }
    }
    let edges = model.edges.iter().map(|e| EdgeVertex { edge_index: e.index,
        start_vertex: slot_to_vertex.get(e.index*2).copied().flatten(),
        end_vertex: slot_to_vertex.get(e.index*2+1).copied().flatten() }).collect();
    VertexTopology { vertices, edges, slot_to_vertex, max_spread }
}

/// JSON-friendly diagnostic intermediate (schema creo-view-ol-exact/2).
pub fn to_occ_intermediate(model: &ExactModel, include_normalized_pcurves: bool) -> serde_json::Value {
    let mut primary = HashSet::new();
    let mut normalized = HashSet::new();
    for f in &model.faces { for l in &f.loops { for c in &l.coedges {
        if let Some(p) = c.pcurve_index { primary.insert(p); }
        if let Some(n) = c.normalized_pcurve_index { normalized.insert(n); }
    }}}
    let vt = build_exact_vertex_topology(model);
    let surfaces: Vec<serde_json::Value> = model.faces.iter().map(|f| {
        let mut v = serde_json::to_value(&SerSurface(&f.surface)).unwrap_or(serde_json::Value::Null);
        if let serde_json::Value::Object(ref mut m) = v { m.insert("faceIndex".into(), f.index.into()); }
        v
    }).collect();
    let curves: Vec<serde_json::Value> = model.edges.iter().map(|e| {
        let mut v = serde_json::to_value(&SerCurve(&e.curve)).unwrap_or(serde_json::Value::Null);
        if let serde_json::Value::Object(ref mut m) = v {
            m.insert("edgeIndex".into(), e.index.into());
            if let Some(ev) = vt.edges.get(e.index) {
                m.insert("startVertex".into(), ev.start_vertex.map(|x| x as u64).map(serde_json::Value::from).unwrap_or(serde_json::Value::Null));
                m.insert("endVertex".into(), ev.end_vertex.map(|x| x as u64).map(serde_json::Value::from).unwrap_or(serde_json::Value::Null));
            }
        }
        v
    }).collect();
    let pcurves: Vec<serde_json::Value> = model.pcurves.iter()
        .filter(|p| primary.contains(&p.index) || (include_normalized_pcurves && normalized.contains(&p.index)))
        .map(|p| serde_json::json!({ "index": p.index, "flags": p.flags, "points": p.points,
            "normalized": normalized.contains(&p.index) && !primary.contains(&p.index) })).collect();
    serde_json::json!({
        "schema": "creo-view-ol-exact/2",
        "source": { "signature": model.signature, "exactVersion": model.exact_version, "precision": model.precision },
        "vertices": vt.vertices,
        "vertexTopology": { "maxSpread": vt.max_spread },
        "surfaces": surfaces, "curves3d": curves, "pcurves2d": pcurves,
        "shells": serde_json::to_value(&model.shells).unwrap_or(serde_json::Value::Null),
    })
}

// serde helpers for surfaces/curves (structural dump)
use serde::ser::{Serialize, Serializer, SerializeMap};
struct SerSurface<'a>(&'a ExactSurface);
impl Serialize for SerSurface<'_> {
    fn serialize<S: Serializer>(&self, s: S) -> std::result::Result<S::Ok, S::Error> {
        let mut m = s.serialize_map(None)?;
        m.serialize_entry("type", &self.0.type_name())?;
        match self.0 {
            ExactSurface::Plane{origin,u_vector,v_vector,normal,u_range,v_range,..} => {
                m.serialize_entry("origin", origin)?; m.serialize_entry("uVector", u_vector)?;
                m.serialize_entry("vVector", v_vector)?; m.serialize_entry("normal", normal)?;
                m.serialize_entry("uRange", u_range)?; m.serialize_entry("vRange", v_range)?;
            }
            ExactSurface::Cylinder{origin,cos_vector,sin_vector,axis_step,u_range,v_range,..} => {
                m.serialize_entry("origin", origin)?; m.serialize_entry("cosVector", cos_vector)?;
                m.serialize_entry("sinVector", sin_vector)?; m.serialize_entry("axisStepVector", axis_step)?;
                m.serialize_entry("uRange", u_range)?; m.serialize_entry("vRange", v_range)?;
            }
            ExactSurface::Cone{origin,cos_growth,sin_growth,axis_step,radial_offset,radial_slope,u_range,v_range,..} => {
                m.serialize_entry("origin", origin)?; m.serialize_entry("cosGrowthVector", cos_growth)?;
                m.serialize_entry("sinGrowthVector", sin_growth)?; m.serialize_entry("axisStepVector", axis_step)?;
                m.serialize_entry("radialOffset", radial_offset)?; m.serialize_entry("radialSlope", radial_slope)?;
                m.serialize_entry("uRange", u_range)?; m.serialize_entry("vRange", v_range)?;
            }
            ExactSurface::Sphere{center,cos_vector,sin_vector,axis_vector,u_range,v_range,..} => {
                m.serialize_entry("center", center)?; m.serialize_entry("cosVector", cos_vector)?;
                m.serialize_entry("sinVector", sin_vector)?; m.serialize_entry("axisVector", axis_vector)?;
                m.serialize_entry("uRange", u_range)?; m.serialize_entry("vRange", v_range)?;
            }
            ExactSurface::Torus{origin,major_cos,major_sin,axis_unit,minor_cos,minor_axis,minor_sin_dir,u_range,v_range,..} => {
                m.serialize_entry("origin", origin)?; m.serialize_entry("majorCosVector", major_cos)?;
                m.serialize_entry("majorSinVector", major_sin)?; m.serialize_entry("axisUnit", axis_unit)?;
                m.serialize_entry("minorCosVector", minor_cos)?; m.serialize_entry("minorAxisVector", minor_axis)?;
                m.serialize_entry("minorSinDirection", minor_sin_dir)?;
                m.serialize_entry("uRange", u_range)?; m.serialize_entry("vRange", v_range)?;
            }
            ExactSurface::RationalBspline{u_order,v_order,u_knots,v_knots,u_pole_count,v_pole_count,poles,..} => {
                m.serialize_entry("uOrder", u_order)?; m.serialize_entry("vOrder", v_order)?;
                m.serialize_entry("uKnots", u_knots)?; m.serialize_entry("vKnots", v_knots)?;
                m.serialize_entry("uPoleCount", u_pole_count)?; m.serialize_entry("vPoleCount", v_pole_count)?;
                let pts: Vec<V3> = poles.iter().map(|p| p.point).collect();
                m.serialize_entry("poles", &pts)?;
            }
            ExactSurface::Unknown{type_code,raw_bytes,..} => {
                m.serialize_entry("typeCode", type_code)?; m.serialize_entry("rawBytes", raw_bytes)?;
            }
        }
        m.end()
    }
}
struct SerCurve<'a>(&'a ExactCurve);
impl Serialize for SerCurve<'_> {
    fn serialize<S: Serializer>(&self, s: S) -> std::result::Result<S::Ok, S::Error> {
        let mut m = s.serialize_map(None)?;
        m.serialize_entry("type", &self.0.type_name())?;
        match self.0 {
            ExactCurve::Line{origin,vector,..} => { m.serialize_entry("origin", origin)?; m.serialize_entry("vector", vector)?; }
            ExactCurve::Circle{center,cos_vector,sin_vector,parameter_range,..} => {
                m.serialize_entry("center", center)?; m.serialize_entry("cosVector", cos_vector)?;
                m.serialize_entry("sinVector", sin_vector)?; m.serialize_entry("parameterRange", parameter_range)?; }
            ExactCurve::Ellipse{center,cos_vector,sin_vector,parameter_range,..} => {
                m.serialize_entry("center", center)?; m.serialize_entry("cosVector", cos_vector)?;
                m.serialize_entry("sinVector", sin_vector)?; m.serialize_entry("parameterRange", parameter_range)?; }
            ExactCurve::DegeneratePoint{point,..} => { m.serialize_entry("point", point)?; }
            ExactCurve::RationalBspline{order,knots,poles,..} => {
                m.serialize_entry("order", order)?; m.serialize_entry("knots", knots)?;
                let pts: Vec<V3> = poles.iter().map(|p| p.point).collect();
                m.serialize_entry("poles", &pts)?; }
            ExactCurve::Polyline{points,..} => { m.serialize_entry("points", points)?; }
            ExactCurve::Unknown{type_code,raw_bytes,..} => {
                m.serialize_entry("typeCode", type_code)?; m.serialize_entry("rawBytes", raw_bytes)?; }
        }
        m.end()
    }
}

impl serde::Serialize for ShellCandidate {
    fn serialize<S: Serializer>(&self, s: S) -> std::result::Result<S::Ok, S::Error> {
        let mut m = s.serialize_map(None)?;
        m.serialize_entry("index", &self.index)?;
        m.serialize_entry("faceRegionIndices", &self.face_region_indices)?;
        m.serialize_entry("faceIndices", &self.face_indices)?;
        m.serialize_entry("edgeCount", &self.edge_count)?;
        m.serialize_entry("boundaryEdges", &self.boundary_edges)?;
        m.serialize_entry("nonManifoldEdges", &self.non_manifold_edges)?;
        m.serialize_entry("degenerateEdges", &self.degenerate_edges)?;
        m.serialize_entry("closedCandidate", &self.closed_candidate)?;
        m.end()
    }
}
