//! Creo View visual appearance decoding and exact-face alignment.
use crate::binary::clamp01;
use crate::ol::{Appearance, extract_ol_surface_meshes, SurfaceMesh};

/// Renderer-friendly normalized appearance (bounded values).
#[derive(Debug, Clone)]
pub struct NormalizedAppearance {
    pub name: String,
    pub base_index: u32,
    pub rgb: [f64; 3],
    pub diffuse_rgb: [f64; 3],
    pub specular_rgb: [f64; 3],
    pub shininess: f64,
    pub emissive_rgb: [f64; 3],
    pub opacity: f64,
    pub raw_values: [f64; 14],
}

pub fn normalize_appearance(app: &Appearance) -> NormalizedAppearance {
    let raw = app.raw_values;
    let clamp3 = |v: [f64; 3]| [clamp01(v[0]), clamp01(v[1]), clamp01(v[2])];
    NormalizedAppearance {
        name: app.name.clone(),
        base_index: app.base_index,
        rgb: clamp3(app.rgb),
        diffuse_rgb: clamp3(app.diffuse_rgb),
        specular_rgb: clamp3(app.specular_rgb),
        shininess: if app.shininess.is_finite() { app.shininess } else { raw[9] },
        emissive_rgb: clamp3(app.emissive_rgb),
        opacity: clamp01(app.opacity),
        raw_values: raw,
    }
}

pub fn normalize_opt(app: Option<&Appearance>) -> Option<NormalizedAppearance> {
    app.map(normalize_appearance)
}

/// Stable cache/grouping key.
pub fn appearance_key(app: Option<&Appearance>) -> String {
    match app {
        None => "<none>".to_string(),
        Some(a) => {
            let n = normalize_appearance(a);
            serde_json::to_string(&(
                n.name, n.base_index, n.rgb, n.diffuse_rgb, n.specular_rgb,
                n.shininess, n.emissive_rgb, n.opacity,
            )).unwrap_or_default()
        }
    }
}

pub fn appearance_key_normalized(n: Option<&NormalizedAppearance>) -> String {
    match n {
        None => "<none>".to_string(),
        Some(v) => serde_json::to_string(&(
            v.name.clone(), v.base_index, v.rgb, v.diffuse_rgb, v.specular_rgb,
            v.shininess, v.emissive_rgb, v.opacity,
        )).unwrap_or_default(),
    }
}

/// Legacy shininess exponent -> perceptual roughness approximation.
pub fn shininess_to_roughness(shininess: f64) -> f64 {
    let exp = if shininess.is_finite() { shininess.max(0.0) } else { 0.0 };
    (2.0 / (exp + 2.0)).sqrt().max(0.04).min(1.0)
}

// The geometric exact<->tessellation alignment used by STEP styling and GLB
// construction filtering operates on the exact model (see `step.rs` and
// `glb.rs`) to avoid a circular module dependency. This module keeps the
// pure-appearance helpers plus mesh-bounds utilities shared by both.

#[derive(Debug, Clone, Copy)]
pub struct Bounds3 { pub lo: [f64; 3], pub hi: [f64; 3] }

pub fn mesh_bounds(mesh: &SurfaceMesh) -> Option<Bounds3> {
    if mesh.vertices.is_empty() { return None; }
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for v in &mesh.vertices {
        for a in 0..3 { lo[a] = lo[a].min(v[a]); hi[a] = hi[a].max(v[a]); }
    }
    Some(Bounds3 { lo, hi })
}

pub fn bounds_match_cost(a: Option<Bounds3>, b: Option<Bounds3>) -> f64 {
    let (Some(a), Some(b)) = (a, b) else { return 5.0; };
    let mut scale: f64 = 0.0;
    let mut delta: f64 = 0.0;
    for axis in 0..3 {
        scale = scale.max(a.hi[axis] - a.lo[axis]).max(b.hi[axis] - b.lo[axis]);
        delta = delta.max((a.lo[axis]-b.lo[axis]).abs()).max((a.hi[axis]-b.hi[axis]).abs());
    }
    delta / scale.max(1e-9)
}

/// Extract meshes for appearance mapping (same as OL tessellation).
pub fn meshes_for_mapping(ol_data: &[u8]) -> crate::Result<Vec<SurfaceMesh>> {
    extract_ol_surface_meshes(ol_data)
}
