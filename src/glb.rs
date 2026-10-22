//! glTF 2.0 / GLB serializer for tessellated OL geometry and PVS hierarchy.
use crate::appearance::{normalize_appearance, shininess_to_roughness};
use crate::ol::{extract_ol_surface_meshes, merge_surface_meshes, merge_surface_meshes_by_appearance, Appearance, MergedMesh, SurfaceMesh};
use crate::ol_exact::parse_ol_exact_geometry;
use crate::{Result, openpvz_error};
use std::collections::HashMap;

const GLTF_ARRAY_BUFFER: u32 = 34962;
const GLTF_ELEMENT_ARRAY_BUFFER: u32 = 34963;
const GLTF_FLOAT: u32 = 5126;
const GLTF_UNSIGNED_INT: u32 = 5125;
const GLTF_TRIANGLES: u32 = 4;

struct GlbBuilder {
    bin_parts: Vec<Vec<u8>>,
    bin_length: usize,
    buffer_views: Vec<serde_json::Value>,
    accessors: Vec<serde_json::Value>,
    meshes: Vec<serde_json::Value>,
    materials: Vec<serde_json::Value>,
    nodes: Vec<serde_json::Value>,
    material_cache: HashMap<String, usize>,
    extensions_used: std::collections::BTreeSet<String>,
}

impl GlbBuilder {
    fn new() -> Self {
        Self { bin_parts: vec![], bin_length: 0, buffer_views: vec![], accessors: vec![],
            meshes: vec![], materials: vec![], nodes: vec![],
            material_cache: HashMap::new(), extensions_used: Default::default() }
    }

    fn append_bytes(&mut self, blob: Vec<u8>, target: Option<u32>) -> usize {
        let padding = (4 - (self.bin_length % 4)) % 4;
        if padding > 0 {
            self.bin_parts.push(vec![0u8; padding]);
            self.bin_length += padding;
        }
        let offset = self.bin_length;
        let len = blob.len();
        self.bin_parts.push(blob);
        self.bin_length += len;
        let mut view = serde_json::json!({ "buffer": 0, "byteOffset": offset, "byteLength": len });
        if let Some(t) = target { view["target"] = t.into(); }
        self.buffer_views.push(view);
        self.buffer_views.len() - 1
    }

    fn add_material(&mut self, app: Option<&Appearance>) -> Option<usize> {
        let app = app?;
        let n = normalize_appearance(app);
        let key = crate::appearance::appearance_key(Some(app));
        if let Some(hit) = self.material_cache.get(&key) { return Some(*hit); }
        let mut mat = serde_json::json!({
            "name": if n.name.is_empty() { format!("appearance_{}", n.base_index) } else { n.name.clone() },
            "pbrMetallicRoughness": {
                "baseColorFactor": [n.rgb[0], n.rgb[1], n.rgb[2], n.opacity],
                "metallicFactor": 0,
                "roughnessFactor": shininess_to_roughness(n.shininess),
            },
            "extras": { "creoViewAppearance": {
                "baseIndex": n.base_index, "sourceRgb": n.rgb,
                "diffuseRgb": n.diffuse_rgb, "specularRgb": n.specular_rgb,
                "shininess": n.shininess, "emissiveRgb": n.emissive_rgb,
                "opacity": n.opacity, "rawValues": n.raw_values,
            }},
        });
        if n.emissive_rgb.iter().any(|v| *v > 1e-6) {
            mat["emissiveFactor"] = serde_json::json!(n.emissive_rgb);
        }
        let spec_max = n.specular_rgb.iter().cloned().fold(0.0_f64, f64::max);
        if spec_max > 1e-6 {
            mat["extensions"] = serde_json::json!({ "KHR_materials_specular": {
                "specularFactor": spec_max,
                "specularColorFactor": n.specular_rgb.map(|v| v / spec_max),
            }});
            self.extensions_used.insert("KHR_materials_specular".to_string());
        }
        if n.opacity < 0.999 {
            mat["alphaMode"] = "BLEND".into();
        }
        self.materials.push(mat);
        let idx = self.materials.len() - 1;
        self.material_cache.insert(key, idx);
        Some(idx)
    }

    fn add_primitive(&mut self, mesh: &MergedMesh, app: Option<&Appearance>) -> Result<serde_json::Value> {
        if mesh.indices.is_empty() || mesh.positions.is_empty() {
            return Err("cannot add empty triangle mesh".into());
        }
        if mesh.normals.len() != mesh.positions.len() {
            return Err("GLB mesh position/normal count mismatch".into());
        }
        let mut pos_bytes = vec![0u8; mesh.positions.len() * 12];
        let mut nrm_bytes = vec![0u8; mesh.normals.len() * 12];
        let mut idx_bytes = vec![0u8; mesh.indices.len() * 4];
        let (mut mn, mut mx) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
        for (vi, (p, n)) in mesh.positions.iter().zip(mesh.normals.iter()).enumerate() {
            for axis in 0..3 {
                pos_bytes[vi*12+axis*4..vi*12+axis*4+4].copy_from_slice(&(p[axis] as f32).to_le_bytes());
                nrm_bytes[vi*12+axis*4..vi*12+axis*4+4].copy_from_slice(&(n[axis] as f32).to_le_bytes());
                mn[axis] = mn[axis].min(p[axis]);
                mx[axis] = mx[axis].max(p[axis]);
            }
        }
        for (i, ix) in mesh.indices.iter().enumerate() {
            idx_bytes[i*4..i*4+4].copy_from_slice(&ix.to_le_bytes());
        }
        let pv = self.append_bytes(pos_bytes, Some(GLTF_ARRAY_BUFFER));
        let nv = self.append_bytes(nrm_bytes, Some(GLTF_ARRAY_BUFFER));
        let iv = self.append_bytes(idx_bytes, Some(GLTF_ELEMENT_ARRAY_BUFFER));
        self.accessors.push(serde_json::json!({ "bufferView": pv, "componentType": GLTF_FLOAT,
            "count": mesh.positions.len(), "type": "VEC3", "min": mn, "max": mx }));
        let pa = self.accessors.len() - 1;
        self.accessors.push(serde_json::json!({ "bufferView": nv, "componentType": GLTF_FLOAT,
            "count": mesh.normals.len(), "type": "VEC3" }));
        let na = self.accessors.len() - 1;
        self.accessors.push(serde_json::json!({ "bufferView": iv, "componentType": GLTF_UNSIGNED_INT,
            "count": mesh.indices.len(), "type": "SCALAR" }));
        let ia = self.accessors.len() - 1;
        let mut prim = serde_json::json!({ "attributes": { "POSITION": pa, "NORMAL": na },
            "indices": ia, "mode": GLTF_TRIANGLES });
        if let Some(mi) = self.add_material(app) { prim["material"] = mi.into(); }
        Ok(prim)
    }

    fn add_mesh(&mut self, name: &str, groups: &[(Option<Appearance>, MergedMesh)]) -> Result<usize> {
        let mut prims = vec![];
        for (app, mesh) in groups {
            if mesh.indices.is_empty() { continue; }
            prims.push(self.add_primitive(mesh, app.as_ref())?);
        }
        if prims.is_empty() { return Err("cannot add mesh with no triangle primitives".into()); }
        self.meshes.push(serde_json::json!({ "name": name, "primitives": prims }));
        Ok(self.meshes.len() - 1)
    }

    fn add_node(&mut self, node: serde_json::Value) -> usize {
        self.nodes.push(node);
        self.nodes.len() - 1
    }

    fn write(&self, path: &str, root_node: usize) -> Result<()> {
        let mut bin: Vec<u8> = Vec::with_capacity(self.bin_length);
        for part in &self.bin_parts { bin.extend_from_slice(part); }
        let mut doc = serde_json::json!({
            "asset": { "version": "2.0", "generator": "openpvz-rs" },
            "scene": 0, "scenes": [{ "nodes": [root_node] }],
            "nodes": self.nodes, "meshes": self.meshes,
        });
        if self.bin_length > 0 {
            doc["buffers"] = serde_json::json!([{ "byteLength": bin.len() }]);
            doc["bufferViews"] = self.buffer_views.clone().into();
            doc["accessors"] = self.accessors.clone().into();
        }
        if !self.materials.is_empty() { doc["materials"] = self.materials.clone().into(); }
        if !self.extensions_used.is_empty() {
            let v: Vec<serde_json::Value> = self.extensions_used.iter().map(|s| serde_json::Value::String(s.clone())).collect();
            doc["extensionsUsed"] = v.into();
        }
        let mut json = serde_json::to_vec(&doc).map_err(|e| openpvz_error(format!("glTF JSON failed: {e}")))?;
        while json.len() % 4 != 0 { json.push(0x20); }
        while bin.len() % 4 != 0 { bin.push(0); }
        let has_bin = !bin.is_empty();
        let total = 12 + 8 + json.len() + if has_bin { 8 + bin.len() } else { 0 };
        let mut out: Vec<u8> = Vec::with_capacity(total);
        out.extend_from_slice(b"glTF");
        out.extend_from_slice(&2u32.to_le_bytes());
        out.extend_from_slice(&(total as u32).to_le_bytes());
        out.extend_from_slice(&(json.len() as u32).to_le_bytes());
        out.extend_from_slice(b"JSON");
        out.extend_from_slice(&json);
        if has_bin {
            out.extend_from_slice(&(bin.len() as u32).to_le_bytes());
            out.extend_from_slice(b"BIN\x00");
            out.extend_from_slice(&bin);
        }
        std::fs::write(path, &out).map_err(|e| openpvz_error(format!("cannot write {path}: {e}")))?;
        Ok(())
    }
}

/// Omit tessellated support/construction surfaces identifiable via exact topology.
fn select_renderable(
    ol_data: &[u8], surfaces: Vec<SurfaceMesh>, include_construction: bool,
) -> (Vec<SurfaceMesh>, usize, bool) {
    if include_construction { return (surfaces, 0, false); }
    let exact = match parse_ol_exact_geometry(ol_data) {
        Ok(Some(m)) => m,
        _ => return (surfaces, 0, true),
    };
    if exact.faces.is_empty() { return (surfaces, 0, false); }
    // mapping: ordered or geometric alignment on bounds
    let mapping: Vec<Option<usize>> = if exact.faces.len() == surfaces.len() {
        (0..surfaces.len()).map(Some).collect()
    } else {
        // reuse appearance-mapping alignment indirectly: map faces->meshes via bounds DP
        // (compact reimplementation to avoid module cycle)
        let fb: Vec<_> = (0..exact.faces.len()).map(|i| {
            let face = &exact.faces[i];
            let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
            let mut n = 0;
            for aloop in &face.loops {
                for c in &aloop.coedges {
                    let ctx = crate::ol_exact::exact_coedge_pcurve_context(&exact, face, c);
                    let Some(ctx) = ctx else { continue };
                    for uv in &ctx.pcurve.points {
                        let Some(p) = crate::ol_exact::evaluate_exact_surface(ctx.support, uv[0], uv[1]) else { continue };
                        if !p.iter().all(|x| x.is_finite()) { continue; }
                        for a in 0..3 { lo[a] = lo[a].min(p[a]); hi[a] = hi[a].max(p[a]); }
                        n += 1;
                    }
                }
            }
            if n == 0 { None } else { Some((lo, hi)) }
        }).collect();
        let tb: Vec<_> = surfaces.iter().map(|m| {
            if m.vertices.is_empty() { return None; }
            let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
            for v in &m.vertices { for a in 0..3 { lo[a] = lo[a].min(v[a]); hi[a] = hi[a].max(v[a]); } }
            Some((lo, hi))
        }).collect();
        // monotonic DP (same penalties as appearance mapping)
        let (ec, tc) = (exact.faces.len(), surfaces.len());
        let stride = tc + 1;
        if (ec + 1) * stride > 20_000_000 { return (surfaces, 0, true); }
        let cost = |a: Option<([f64;3],[f64;3])>, b: Option<([f64;3],[f64;3])>| -> f64 {
            let (Some((alo,ahi)), Some((blo,bhi))) = (a, b) else { return 5.0; };
            let (mut s, mut d): (f64, f64) = (0.0, 0.0);
            for ax in 0..3 {
                s = s.max(ahi[ax]-alo[ax]).max(bhi[ax]-blo[ax]);
                d = d.max((alo[ax]-blo[ax]).abs()).max((ahi[ax]-bhi[ax]).abs());
            }
            d / s.max(1e-9)
        };
        let mut c = vec![f64::INFINITY; (ec+1)*stride];
        let mut b = vec![0u8; (ec+1)*stride];
        c[0] = 0.0;
        for ei in 0..=ec { for mi in 0..=tc {
            let cur = ei*stride+mi;
            let base = c[cur];
            if !base.is_finite() { continue; }
            if ei < ec && mi < tc {
                let nx = (ei+1)*stride+mi+1;
                let v = base + cost(fb[ei], tb[mi]);
                if v < c[nx] { c[nx] = v; b[nx] = 1; }
            }
            if ei < ec {
                let nx = (ei+1)*stride+mi;
                let f = &exact.faces[ei];
                let pen = if !f.loops.is_empty() || !f.singular_uses.is_empty() { 0.32 } else { 0.03 };
                if base + pen < c[nx] { c[nx] = base + pen; b[nx] = 2; }
            }
            if mi < tc {
                let nx = ei*stride+mi+1;
                if base + 0.32 < c[nx] { c[nx] = base + 0.32; b[nx] = 3; }
            }
        }}
        let mut mapping: Vec<Option<usize>> = vec![None; ec];
        let (mut ei, mut mi) = (ec, tc);
        while ei > 0 || mi > 0 {
            match b[ei*stride+mi] {
                1 => { mapping[ei-1] = Some(mi-1); ei -= 1; mi -= 1; }
                2 => { ei -= 1; }
                3 => { mi -= 1; }
                _ => return (surfaces, 0, true),
            }
        }
        mapping
    };
    let mut omit = std::collections::HashSet::new();
    for (fi, face) in exact.faces.iter().enumerate() {
        if !face.loops.is_empty() || !face.singular_uses.is_empty() { continue; }
        if let Some(Some(mi)) = mapping.get(fi) { omit.insert(*mi); }
    }
    if omit.is_empty() { return (surfaces, 0, false); }
    let n = omit.len();
    let kept: Vec<SurfaceMesh> = surfaces.into_iter().enumerate()
        .filter(|(i, _)| !omit.contains(i)).map(|(_, m)| m).collect();
    (kept, n, false)
}

#[derive(Debug, serde::Serialize)]
pub struct GlbStats {
    pub definitions: usize,
    pub occurrences: usize,
    pub unique_triangle_meshes: usize,
    pub gltf_materials: usize,
    pub unique_vertices: usize,
    pub unique_triangles: usize,
    pub line_only_ol: Vec<String>,
    pub construction_only_ol_count: usize,
    pub construction_surfaces_omitted: usize,
    pub definitions_with_construction_surfaces: usize,
    pub construction_filter_exact_fallbacks: usize,
    pub construction_surfaces_included: bool,
}

/// Export the PVS occurrence tree and OL tessellation to one GLB scene.
pub fn export_glb(
    archive: &crate::pvz::PvzArchive,
    output: &str,
    include_construction: bool,
    progress: Option<&dyn Fn(usize, usize)>,
) -> Result<GlbStats> {
    let model = &archive.pvs;
    let mut builder = GlbBuilder::new();
    let mut definition_mesh: HashMap<usize, usize> = HashMap::new();
    let mut geometry_stats: HashMap<usize, (usize, usize)> = HashMap::new();
    let mut raw_triangles: HashMap<usize, usize> = HashMap::new();
    let geometry_definitions: Vec<_> = model.definitions.iter().filter(|d| d.ol_name.is_some()).collect();
    let mut omitted_construction = 0;
    let mut defs_with_construction = 0;
    let mut exact_fallbacks = 0;

    for (n, definition) in geometry_definitions.iter().enumerate() {
        let ol_data = archive.read_ol(definition.ol_name.as_ref().unwrap())?;
        let all = extract_ol_surface_meshes(&ol_data)?;
        raw_triangles.insert(definition.index, all.iter().map(|s| s.triangles.len()).sum());
        let (surfaces, omitted, fallback) = select_renderable(&ol_data, all, include_construction);
        if omitted > 0 { omitted_construction += omitted; defs_with_construction += 1; }
        if fallback { exact_fallbacks += 1; }
        let merged = merge_surface_meshes(&surfaces);
        let groups = merge_surface_meshes_by_appearance(&surfaces);
        let owned: Vec<(Option<Appearance>, MergedMesh)> = groups.into_iter().map(|g| (g.appearance, g.mesh)).collect();
        geometry_stats.insert(definition.index, (merged.positions.len(), merged.indices.len() / 3));
        if !merged.indices.is_empty() {
            definition_mesh.insert(definition.index, builder.add_mesh(&definition.source_name, &owned)?);
        }
        if let Some(cb) = progress { cb(n + 1, geometry_definitions.len()); }
    }

    // occurrence emission with cycle detection
    let mut occurrence_count = 0;
    fn emit(
        builder: &mut GlbBuilder,
        model: &crate::pvs::PvsModel,
        definition_mesh: &HashMap<usize, usize>,
        definition_index: usize,
        feature_id: Option<&str>,
        placement: Option<&crate::pvs::Placement>,
        stack: &mut Vec<usize>,
        occurrence_count: &mut usize,
    ) -> Result<usize> {
        if stack.contains(&definition_index) {
            return Err(format!("cycle in PVS assembly graph at definition {definition_index}").into());
        }
        *occurrence_count += 1;
        let definition = &model.definitions[definition_index];
        let mut node = serde_json::json!({
            "name": match feature_id {
                None => definition.source_name.clone(),
                Some(f) => format!("{} [{f}]", definition.source_name),
            },
            "extras": { "pvsDefinitionIndex": definition.index },
        });
        if let Some(f) = feature_id { node["extras"]["featureId"] = f.into(); }
        if let Some(p) = placement {
            if !p.is_identity() { node["matrix"] = p.gltf_matrix().into(); }
        }
        if let Some(mi) = definition_mesh.get(&definition_index) {
            node["mesh"] = (*mi).into();
        }
        let node_index = builder.add_node(node);
        stack.push(definition_index);
        let mut children = vec![];
        for child in &definition.children {
            children.push(emit(builder, model, definition_mesh, child.definition_index,
                Some(&child.feature_id), Some(&child.placement), stack, occurrence_count)?);
        }
        stack.pop();
        if !children.is_empty() {
            builder.nodes[node_index]["children"] = children.into();
        }
        Ok(node_index)
    }

    let mut stack = vec![];
    let root_node = emit(&mut builder, model, &definition_mesh, model.root_index, None, None, &mut stack, &mut occurrence_count)?;
    builder.write(output, root_node)?;

    let unique_vertices: usize = geometry_stats.values().map(|v| v.0).sum();
    let unique_triangles: usize = geometry_stats.values().map(|v| v.1).sum();
    let line_only_ol: Vec<String> = raw_triangles.iter()
        .filter(|(_, c)| **c == 0)
        .map(|(i, _)| model.definitions[*i].source_name.clone()).collect();
    let construction_only = geometry_stats.iter()
        .filter(|(i, s)| raw_triangles.get(i).copied().unwrap_or(0) > 0 && s.1 == 0).count();
    Ok(GlbStats {
        definitions: model.definitions.len(),
        occurrences: occurrence_count,
        unique_triangle_meshes: definition_mesh.len(),
        gltf_materials: builder.materials.len(),
        unique_vertices, unique_triangles, line_only_ol,
        construction_only_ol_count: construction_only,
        construction_surfaces_omitted: omitted_construction,
        definitions_with_construction_surfaces: defs_with_construction,
        construction_filter_exact_fallbacks: exact_fallbacks,
        construction_surfaces_included: include_construction,
    })
}
