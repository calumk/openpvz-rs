//! Low-level OL container, TLV, tessellation and appearance parser.
use crate::binary::hex32;
use crate::constants::{ol_stream, ol_tag, NULL_INDEX};
use crate::{Result, openpvz_error};
use flate2::read::GzDecoder;
use std::collections::BTreeMap;
use std::io::Read;

pub const OL_STREAM_ATTRS: u32 = ol_stream::ATTRIBUTES;
pub const OL_STREAM_TESSELLATION: u32 = ol_stream::TESSELLATION;
pub const OL_STREAM_EDGE_TESSELLATION: u32 = ol_stream::EDGE_TESSELLATION;
pub const OL_STREAM_EXACT_V3: u32 = ol_stream::EXACT_V3;
pub const OL_STREAM_EXACT_V4: u32 = ol_stream::EXACT_V4;
pub const OL_STREAM_AUX: u32 = ol_stream::EXACT_V3;
pub const OL_STREAM_EXACT: u32 = ol_stream::EXACT_V4;

fn read_u32le(b: &[u8], off: usize) -> Result<u32> {
    if off + 4 > b.len() { return Err("truncated u32".into()); }
    Ok(u32::from_le_bytes([b[off],b[off+1],b[off+2],b[off+3]]))
}
fn read_f32le(b: &[u8], off: usize) -> Result<f32> {
    if off + 4 > b.len() { return Err("truncated f32".into()); }
    Ok(f32::from_le_bytes([b[off],b[off+1],b[off+2],b[off+3]]))
}

#[derive(Debug, Clone)]
pub struct OlStream {
    pub stream_type: u32,
    pub data: Vec<u8>,
}

#[derive(Debug, Clone)]
pub struct OlParsed {
    pub signature: String,
    pub version: u32,
    pub streams: Vec<OlStream>,
}

/// Parse the OL stream directory and gunzip each contained stream.
pub fn parse_ol_streams(data: &[u8]) -> Result<OlParsed> {
    if data.len() < 22 || &data[0..6] != b"DIV-OL" {
        return Err("not a supported OL file".into());
    }
    let signature = String::from_utf8_lossy(&data[0..10]).into_owned();
    let version = read_u32le(data, 10)?;
    let directory_bytes = read_u32le(data, 14)? as usize;
    let entry_count = read_u32le(data, 18)? as usize;
    if directory_bytes != 4 + entry_count * 8 {
        return Err("unexpected OL directory size".into());
    }
    let mut pos = 22usize;
    let mut entries: Vec<(u32, usize)> = Vec::new();
    for _ in 0..entry_count {
        if pos + 8 > data.len() { return Err("truncated OL stream directory".into()); }
        entries.push((read_u32le(data, pos)?, read_u32le(data, pos+4)? as usize));
        pos += 8;
    }
    let end_entry = entries.last().ok_or_else(|| openpvz_error("OL stream directory has no end sentinel".to_string()))?;
    if end_entry.0 != ol_stream::END {
        return Err("OL stream directory has no end sentinel".into());
    }
    if end_entry.1 != data.len() {
        return Err("OL end sentinel does not point at EOF".into());
    }
    let mut streams = Vec::new();
    for i in 0..entries.len()-1 {
        let (stream_type, start) = entries[i];
        let end = entries[i+1].1;
        if !(start <= end && end <= data.len()) { return Err("invalid OL stream offsets".into()); }
        let mut dec = GzDecoder::new(&data[start..end]);
        let mut payload = Vec::new();
        dec.read_to_end(&mut payload).map_err(|e| openpvz_error(format!("failed to decompress OL stream {}: {e}", hex32(stream_type))))?;
        streams.push(OlStream { stream_type, data: payload });
    }
    Ok(OlParsed { signature, version, streams })
}

#[derive(Debug, Clone)]
pub struct OlTlv<'a> {
    pub tag: u32,
    pub payload: &'a [u8],
    pub offset: usize,
}

/// Iterate [u32 tag][u32 size][payload] records.
pub fn iter_ol_tlv(data: &[u8]) -> Result<Vec<OlTlv<'_>>> {
    let mut out = Vec::new();
    let mut pos = 0usize;
    while pos + 8 <= data.len() {
        let tag = read_u32le(data, pos)?;
        let size = read_u32le(data, pos+4)? as usize;
        let record_offset = pos;
        pos += 8;
        let end = pos + size;
        if end > data.len() {
            return Err(format!("OL TLV tag {} runs past stream end", hex32(tag)).into());
        }
        out.push(OlTlv { tag, payload: &data[pos..end], offset: record_offset });
        pos = end;
    }
    if pos != data.len() { return Err("OL TLV stream has trailing bytes".into()); }
    Ok(out)
}

#[derive(Debug, Clone)]
pub struct Appearance {
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

/// Decode legacy shaded-material table from OL attribute stream.
pub fn parse_ol_appearances(streams: &[OlStream]) -> Result<(Vec<Option<Appearance>>, usize)> {
    let attrs: Vec<&OlStream> = streams.iter().filter(|s| s.stream_type == ol_stream::ATTRIBUTES).collect();
    if attrs.len() != 1 {
        return Err(format!("expected one OL attribute stream, found {}", attrs.len()).into());
    }
    let mut pending: Option<[f64; 14]> = None;
    let mut unpaired = 0usize;
    let mut references: Vec<(String, u32)> = Vec::new();
    let mut by_base: BTreeMap<u32, Appearance> = BTreeMap::new();
    for rec in iter_ol_tlv(&attrs[0].data)? {
        if rec.tag == ol_tag::APPEARANCE_VALUES && rec.payload.len() == 56 {
            if pending.is_some() { unpaired += 1; }
            let mut v = [0.0f64; 14];
            for i in 0..14 { v[i] = read_f32le(rec.payload, i*4)? as f64; }
            pending = Some(v);
            continue;
        }
        if rec.tag != ol_tag::APPEARANCE_NAME || rec.payload.len() < 8 { continue; }
        let name_len = read_u32le(rec.payload, 0)? as usize;
        let base_off = 4 + name_len;
        if base_off + 4 > rec.payload.len() { return Err("truncated OL appearance-name record".into()); }
        let raw_name = &rec.payload[4..base_off];
        let term = raw_name.iter().position(|&b| b == 0).unwrap_or(raw_name.len());
        let name = String::from_utf8_lossy(&raw_name[..term]).into_owned();
        let base_index = read_u32le(rec.payload, base_off)?;
        references.push((name.clone(), base_index));
        if let Some(pv) = pending {
            by_base.insert(base_index, Appearance {
                name,
                base_index,
                rgb: [pv[0],pv[1],pv[2]],
                diffuse_rgb: [pv[3],pv[4],pv[5]],
                specular_rgb: [pv[6],pv[7],pv[8]],
                shininess: pv[9],
                emissive_rgb: [pv[10],pv[11],pv[12]],
                opacity: pv[13],
                raw_values: pv,
            });
            pending = None;
        }
    }
    let resolved: Vec<Option<Appearance>> = references.into_iter()
        .map(|(_, base)| by_base.get(&base).cloned()).collect();
    Ok((resolved, unpaired))
}

fn triangles_from_primitive(tag: u32, indices: &[u32]) -> Vec<[u32; 3]> {
    let mut out = Vec::new();
    if tag == ol_tag::TRIANGLE_LIST {
        let mut i = 0;
        while i + 2 < indices.len() { out.push([indices[i],indices[i+1],indices[i+2]]); i += 3; }
    } else if tag == ol_tag::TRIANGLE_STRIP {
        for i in 0..indices.len().saturating_sub(2) {
            if i & 1 == 1 { out.push([indices[i+1],indices[i],indices[i+2]]); }
            else { out.push([indices[i],indices[i+1],indices[i+2]]); }
        }
    } else if tag == ol_tag::TRIANGLE_FAN {
        for i in 1..indices.len().saturating_sub(1) {
            out.push([indices[0],indices[i],indices[i+1]]);
        }
    }
    out
}

#[derive(Debug, Clone)]
pub struct SurfaceMesh {
    pub vertices: Vec<[f64; 6]>,
    pub triangles: Vec<[u32; 3]>,
    pub appearance: Option<Appearance>,
}

/// Extract shaded tessellation as face-local vertex/index blocks.
pub fn extract_ol_surface_meshes(data: &[u8]) -> Result<Vec<SurfaceMesh>> {
    let parsed = parse_ol_streams(data)?;
    let tess: Vec<&OlStream> = parsed.streams.iter().filter(|s| s.stream_type == ol_stream::TESSELLATION).collect();
    if tess.len() != 1 {
        return Err(format!("expected one OL tessellation stream, found {}", tess.len()).into());
    }
    let (appearances, _) = parse_ol_appearances(&parsed.streams)?;
    let mut meshes = Vec::new();
    let mut current_mesh: Option<SurfaceMesh> = None;
    let mut current_appearance: Option<Appearance> = None;
    for rec in iter_ol_tlv(&tess[0].data)? {
        if rec.tag == ol_tag::TESSELLATION_SURFACE_HEADER && rec.payload.len() >= 13 {
            let app_ref = read_u32le(rec.payload, 0)?;
            let relation_count = read_u32le(rec.payload, 9)? as usize;
            if rec.payload.len() != 13 + relation_count * 4 {
                return Err("unexpected OL surface-header size".into());
            }
            if app_ref == NULL_INDEX { current_appearance = None; }
            else if (app_ref as usize) < appearances.len() { current_appearance = appearances[app_ref as usize].clone(); }
            else { return Err("OL surface appearance reference out of range".into()); }
            continue;
        }
        if rec.tag == ol_tag::TESSELLATION_VERTICES && rec.payload.len() >= 5 {
            let flags = rec.payload[0];
            let count = read_u32le(rec.payload, 1)? as usize;
            let body = &rec.payload[5..];
            if let Some(m) = current_mesh.take() { meshes.push(m); }
            if count == 0 || flags != 1 || body.len() != count * 24 { continue; }
            let mut vertices = Vec::with_capacity(count);
            for i in 0..count {
                let off = i * 24;
                vertices.push([
                    read_f32le(body, off)? as f64, read_f32le(body, off+4)? as f64,
                    read_f32le(body, off+8)? as f64, read_f32le(body, off+12)? as f64,
                    read_f32le(body, off+16)? as f64, read_f32le(body, off+20)? as f64,
                ]);
            }
            current_mesh = Some(SurfaceMesh { vertices, triangles: Vec::new(), appearance: current_appearance.clone() });
            continue;
        }
        if let Some(ref mut m) = current_mesh {
            if rec.tag == ol_tag::TRIANGLE_FAN || rec.tag == ol_tag::TRIANGLE_STRIP || rec.tag == ol_tag::TRIANGLE_LIST {
                if rec.payload.len() % 4 != 0 {
                    return Err(format!("unaligned OL index payload for tag {}", hex32(rec.tag)).into());
                }
                let mut indices = Vec::with_capacity(rec.payload.len()/4);
                for i in 0..rec.payload.len()/4 { indices.push(read_u32le(rec.payload, i*4)?); }
                for tri in triangles_from_primitive(rec.tag, &indices) {
                    if tri.iter().max().copied().unwrap_or(0) as usize >= m.vertices.len() {
                        return Err("OL triangle index out of range".into());
                    }
                    m.triangles.push(tri);
                }
            }
        }
    }
    if let Some(m) = current_mesh { meshes.push(m); }
    Ok(meshes)
}

#[derive(Debug, Clone, Default)]
pub struct MergedMesh {
    pub positions: Vec<[f64; 3]>,
    pub normals: Vec<[f64; 3]>,
    pub indices: Vec<u32>,
}

/// Merge face-local blocks into one indexed mesh, preserving source order.
pub fn merge_surface_meshes(meshes: &[SurfaceMesh]) -> MergedMesh {
    let mut out = MergedMesh::default();
    let mut base: u32 = 0;
    for m in meshes {
        for v in &m.vertices {
            out.positions.push([v[0],v[1],v[2]]);
            out.normals.push([v[3],v[4],v[5]]);
        }
        for t in &m.triangles {
            out.indices.push(base + t[0]);
            out.indices.push(base + t[1]);
            out.indices.push(base + t[2]);
        }
        base += m.vertices.len() as u32;
    }
    out
}

pub fn appearance_key_for_ol(app: Option<&Appearance>) -> String {
    match app {
        None => "<none>".to_string(),
        Some(a) => serde_json::to_string(&(a.name.clone(), a.base_index, a.raw_values)).unwrap_or_default(),
    }
}

pub struct AppearanceGroup {
    pub appearance: Option<Appearance>,
    pub mesh: MergedMesh,
}

/// Group tessellation blocks by appearance.
pub fn merge_surface_meshes_by_appearance(meshes: &[SurfaceMesh]) -> Vec<AppearanceGroup> {
    use std::collections::HashMap;
    let mut order: Vec<String> = Vec::new();
    let mut groups: HashMap<String, Vec<usize>> = HashMap::new();
    let mut app_by_key: HashMap<String, Option<Appearance>> = HashMap::new();
    for (i, m) in meshes.iter().enumerate() {
        if m.triangles.is_empty() { continue; }
        let key = appearance_key_for_ol(m.appearance.as_ref());
        if !groups.contains_key(&key) { order.push(key.clone()); app_by_key.insert(key.clone(), m.appearance.clone()); }
        groups.entry(key).or_default().push(i);
    }
    order.into_iter().map(|key| {
        let idxs = groups.remove(&key).unwrap();
        let subset: Vec<SurfaceMesh> = idxs.into_iter().map(|i| meshes[i].clone()).collect();
        AppearanceGroup { appearance: app_by_key.remove(&key).unwrap(), mesh: merge_surface_meshes(&subset) }
    }).collect()
}

/// Tessellation XYZ bounds without allocating complete mesh objects.
pub fn extract_ol_position_bounds(data: &[u8]) -> Result<Option<[f64; 6]>> {
    let parsed = parse_ol_streams(data)?;
    let tess: Vec<&OlStream> = parsed.streams.iter().filter(|s| s.stream_type == ol_stream::TESSELLATION).collect();
    if tess.is_empty() { return Ok(None); }
    if tess.len() != 1 {
        return Err(format!("expected at most one OL tessellation stream, found {}", tess.len()).into());
    }
    let mut lower = [f64::INFINITY; 3];
    let mut upper = [f64::NEG_INFINITY; 3];
    let mut count_points = 0usize;
    for rec in iter_ol_tlv(&tess[0].data)? {
        if rec.tag != ol_tag::TESSELLATION_VERTICES || rec.payload.len() < 5 { continue; }
        let flags = rec.payload[0];
        let count = read_u32le(rec.payload, 1)? as usize;
        let body = &rec.payload[5..];
        let stride = if flags == 1 && body.len() == count * 24 { 24 }
            else if flags == 0 && body.len() == count * 12 { 12 }
            else { continue };
        for i in 0..count {
            let off = i * stride;
            for axis in 0..3 {
                let v = read_f32le(body, off + axis*4)? as f64;
                lower[axis] = lower[axis].min(v);
                upper[axis] = upper[axis].max(v);
            }
            count_points += 1;
        }
    }
    if count_points == 0 { Ok(None) }
    else { Ok(Some([lower[0],lower[1],lower[2],upper[0],upper[1],upper[2]])) }
}

#[derive(Debug, Clone, Default, serde::Serialize)]
pub struct ExactCatalog {
    pub present: bool,
    pub stream_type: Option<String>,
    pub exact_version: Option<u32>,
    pub precision: Option<String>,
    pub tag_counts: BTreeMap<String, usize>,
    pub tag_0x13_prefixes: BTreeMap<String, usize>,
    pub tag_0x14_prefixes: BTreeMap<String, usize>,
}

/// Lightweight exact-stream tag catalogue.
pub fn exact_geometry_catalog(data: &[u8]) -> Result<ExactCatalog> {
    let parsed = parse_ol_streams(data)?;
    let candidates: Vec<&OlStream> = parsed.streams.iter()
        .filter(|s| s.stream_type == ol_stream::EXACT_V3 || s.stream_type == ol_stream::EXACT_V4).collect();
    let mut exact_stream: Option<&OlStream> = None;
    let mut exact_version: Option<u32> = None;
    for s in candidates {
        let tlvs = iter_ol_tlv(&s.data)?;
        if let Some(first) = tlvs.first() {
            if first.tag == ol_tag::EXACT_VERSION && first.payload.len() == 4 {
                let v = read_u32le(first.payload, 0)?;
                if v == 3 || v == 4 { exact_stream = Some(s); exact_version = Some(v); break; }
            }
        }
    }
    let Some(es) = exact_stream else {
        return Ok(ExactCatalog::default());
    };
    let mut tag_counts: BTreeMap<String, usize> = BTreeMap::new();
    let mut surf_pre: BTreeMap<String, usize> = BTreeMap::new();
    let mut edge_pre: BTreeMap<String, usize> = BTreeMap::new();
    for rec in iter_ol_tlv(&es.data)? {
        *tag_counts.entry(hex32(rec.tag)).or_insert(0) += 1;
        if rec.tag == ol_tag::EXACT_SURFACE {
            let n = rec.payload.len().min(3);
            let k: String = rec.payload[..n].iter().map(|b| format!("{b:02x}")).collect();
            *surf_pre.entry(k).or_insert(0) += 1;
        } else if rec.tag == ol_tag::EXACT_EDGE {
            let n = rec.payload.len().min(3);
            let k: String = rec.payload[..n].iter().map(|b| format!("{b:02x}")).collect();
            *edge_pre.entry(k).or_insert(0) += 1;
        }
    }
    let v = exact_version.unwrap();
    Ok(ExactCatalog {
        present: true,
        stream_type: Some(hex32(es.stream_type)),
        exact_version: Some(v),
        precision: Some(if v == 4 { "f64".to_string() } else { "f32".to_string() }),
        tag_counts,
        tag_0x13_prefixes: surf_pre,
        tag_0x14_prefixes: edge_pre,
    })
}
