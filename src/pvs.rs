//! PVS structure parser: component definitions, hierarchy, geometry
//! references and local-to-parent placements.
use crate::binary::{component_kind, read_pvs_atom, read_uleb128, PvsAtom};
use crate::constants::{pvs_name_marker, pvs_placement, pvs_record, pvs_stream, PVS_FILE_PREFIX};
use crate::{Result, openpvz_error};
use flate2::read::GzDecoder;
use std::collections::{BTreeMap, HashSet};
use std::io::Read;

const GEOMETRY_ATOM_ENCODING: u8 = 0x01;
const AUXILIARY_NAME_SUFFIX: u8 = 0x02;
const COMPONENT_NAME_OPTIONAL_PREFIX: u8 = 0x21;
const COMPONENT_NAME_TAILS: [&str; 3] = ["07070303", "07070404", "0d0d0808"];

fn parse_length_prefixed_utf8_to_eof(data: &[u8], start: usize) -> Option<Vec<String>> {
    let mut strings = Vec::new();
    let mut pos = start;
    while pos < data.len() {
        let (length, next) = read_uleb128(data, pos).ok()?;
        pos = next;
        let length = length as usize;
        let end = pos.checked_add(length)?;
        if end > data.len() { return None; }
        let value = std::str::from_utf8(&data[pos..end]).ok()?.to_string();
        if value.as_bytes().len() != length { return None; }
        strings.push(value);
        pos = end;
    }
    if pos == data.len() { Some(strings) } else { None }
}

/// Parse the PVS string pool.
pub fn parse_pvs_string_table(stream: &[u8]) -> Result<Vec<String>> {
    if stream.is_empty() || stream[0] != pvs_stream::STRING_TABLE {
        return Err("not a PVS string-table stream".into());
    }
    let mut candidates: Vec<(usize, usize, Vec<String>)> = Vec::new();
    let upper = 24.min(stream.len());
    for start in 2..upper {
        let strings = match parse_length_prefixed_utf8_to_eof(stream, start) {
            Some(s) => s,
            None => continue,
        };
        let mut pos = 1usize;
        let mut prefix_values: Vec<u64> = Vec::new();
        let mut ok = true;
        while pos < start {
            match read_uleb128(stream, pos) {
                Ok((v, next)) => { prefix_values.push(v); pos = next; }
                Err(_) => { ok = false; break; }
            }
        }
        if ok && pos == start && !prefix_values.is_empty()
            && *prefix_values.last().unwrap() as usize == strings.len() {
            candidates.push((strings.len(), start, strings));
        }
    }
    if candidates.is_empty() {
        return Err("could not locate PVS string-table payload".into());
    }
    candidates.sort_by(|a, b| b.0.cmp(&a.0));
    Ok(candidates.into_iter().next().unwrap().2)
}

/// Local-to-parent component placement. `rotation` stores three consecutive
/// local basis vectors X/Y/Z in parent coordinates (NOT row-major matrix).
#[derive(Debug, Clone)]
pub struct Placement {
    pub translation: [f64; 3],
    pub rotation: [f64; 9],
}

impl Default for Placement {
    fn default() -> Self {
        Self { translation: [0.0, 0.0, 0.0], rotation: [1.0,0.0,0.0, 0.0,1.0,0.0, 0.0,0.0,1.0] }
    }
}

impl Placement {
    pub fn is_identity(&self) -> bool {
        let eps = 1e-14;
        let id = [1.0,0.0,0.0, 0.0,1.0,0.0, 0.0,0.0,1.0];
        self.rotation.iter().zip(id.iter()).all(|(a,b)| (a-b).abs() < eps)
            && self.translation.iter().all(|v| v.abs() < eps)
    }
    /// glTF column-major 4x4 local-to-parent matrix.
    pub fn gltf_matrix(&self) -> [f64; 16] {
        let r = &self.rotation;
        let t = &self.translation;
        [r[0],r[1],r[2],0.0, r[3],r[4],r[5],0.0, r[6],r[7],r[8],0.0, t[0],t[1],t[2],1.0]
    }
}

#[derive(Debug, Clone)]
pub struct PvsStream {
    pub stream_type: u8,
    pub name: String,
    pub compressed: Vec<u8>,
    pub data: Vec<u8>,
}

#[derive(Debug, Clone)]
pub struct ChildOccurrence {
    pub definition_index: usize,
    pub feature_id: String,
    pub placement: Placement,
    pub placement_kind: u8,
    pub placement_encoding: u8,
    pub placement_flags: u8,
}

#[derive(Debug, Clone)]
pub struct Definition {
    pub index: usize,
    pub source_name: String,
    pub kind: String,
    pub ol_name: Option<String>,
    pub bbox: Option<[f64; 6]>,
    pub children: Vec<ChildOccurrence>,
    pub extra_blocks: Vec<Vec<u8>>,
}

#[derive(Debug, Clone)]
pub struct PvsModel {
    pub signature: String,
    pub definitions: Vec<Definition>,
    pub root_index: usize,
    pub streams: Vec<PvsStream>,
    pub strings: Option<Vec<String>>,
}

fn read_f32le(b: &[u8], off: usize) -> Result<f32> {
    if off + 4 > b.len() { return Err("truncated f32".into()); }
    Ok(f32::from_le_bytes([b[off],b[off+1],b[off+2],b[off+3]]))
}
fn read_f64le(b: &[u8], off: usize) -> Result<f64> {
    if off + 8 > b.len() { return Err("truncated f64".into()); }
    Ok(f64::from_le_bytes([b[off],b[off+1],b[off+2],b[off+3],b[off+4],b[off+5],b[off+6],b[off+7]]))
}
fn read_u16le(b: &[u8], off: usize) -> Result<u16> {
    if off + 2 > b.len() { return Err("truncated u16".into()); }
    Ok(u16::from_le_bytes([b[off],b[off+1]]))
}
fn read_u32le(b: &[u8], off: usize) -> Result<u32> {
    if off + 4 > b.len() { return Err("truncated u32".into()); }
    Ok(u32::from_le_bytes([b[off],b[off+1],b[off+2],b[off+3]]))
}

/// Parse the outer PVS stream directory and gunzip each stream.
pub fn parse_pvs_container(data: &[u8]) -> Result<(String, Vec<PvsStream>)> {
    if data.len() < 18 || &data[0..PVS_FILE_PREFIX.len()] != PVS_FILE_PREFIX.as_bytes() {
        return Err("not a supported PVS file".into());
    }
    let signature = String::from_utf8_lossy(&data[0..10]).into_owned();
    let first_stream_offset = read_u32le(data, 10)? as usize;
    let stream_count = read_u32le(data, 14)? as usize;
    let mut pos = 18usize;
    let mut dir: Vec<(u8, String, usize)> = Vec::new();
    for _ in 0..stream_count {
        if pos + 3 > data.len() { return Err("truncated PVS stream directory".into()); }
        let stream_type = data[pos];
        let codec = data[pos+1];
        let name_len = data[pos+2] as usize;
        pos += 3;
        if pos + name_len + 4 > data.len() { return Err("truncated PVS stream directory entry".into()); }
        let name = String::from_utf8_lossy(&data[pos..pos+name_len]).into_owned();
        pos += name_len;
        let compressed_size = read_u32le(data, pos)? as usize;
        pos += 4;
        if codec != 1 { return Err(format!("unsupported PVS stream codec/version {codec}").into()); }
        dir.push((stream_type, name, compressed_size));
    }
    if pos != first_stream_offset {
        return Err(format!("PVS directory ended at {pos}, expected first stream at {first_stream_offset}").into());
    }
    let mut streams = Vec::new();
    let mut offset = first_stream_offset;
    for (stream_type, name, compressed_size) in dir {
        let end = offset + compressed_size;
        if end > data.len() { return Err("PVS compressed stream runs past EOF".into()); }
        let compressed = data[offset..end].to_vec();
        let mut dec = GzDecoder::new(&compressed[..]);
        let mut payload = Vec::new();
        dec.read_to_end(&mut payload).map_err(|e| openpvz_error(format!("failed to decompress PVS stream {name:?}: {e}")))?;
        streams.push(PvsStream { stream_type, name, compressed, data: payload });
        offset = end;
    }
    if offset != data.len() {
        return Err(format!("PVS has {} unexplained trailing bytes", data.len() - offset).into());
    }
    Ok((signature, streams))
}

fn parse_geometry_block(block: &[u8], strings: Option<&[String]>) -> Result<(String, [f64; 6])> {
    if block.len() < 2 || block[0] != pvs_record::GEOMETRY_FAMILY || block[1] != pvs_record::LENGTH_U8_TYPED {
        return Err("not a PVS geometry block".into());
    }
    let mut pos = 3usize;
    if block.get(pos) != Some(&GEOMETRY_ATOM_ENCODING) { return Err("unknown PVS geometry-block encoding".into()); }
    pos += 1;
    let (atom, next) = read_pvs_atom(block, pos, strings)?;
    pos = next;
    let ol_name = match atom {
        PvsAtom::Text(s) => s,
        _ => return Err("OL filename unexpectedly dictionary-referenced".into()),
    };
    if pos + 24 != block.len() { return Err("unexpected PVS geometry-block payload size".into()); }
    let mut bbox = [0.0f64; 6];
    for i in 0..6 {
        bbox[i] = read_f32le(block, pos + i*4)? as f64;
    }
    Ok((ol_name, bbox))
}

fn read_doubles(block: &[u8], offset: usize, count: usize) -> Result<Vec<f64>> {
    let mut out = Vec::with_capacity(count);
    for i in 0..count { out.push(read_f64le(block, offset + i*8)?); }
    Ok(out)
}

fn parse_child_block(block: &[u8], strings: Option<&[String]>) -> Result<ChildOccurrence> {
    if block.len() < 2 || block[0] != pvs_record::CHILD_FAMILY || block[1] != pvs_record::LENGTH_U8_TYPED {
        return Err("not a PVS child block".into());
    }
    if block.len() < 6 { return Err("short PVS child block".into()); }
    let placement_kind = block[3];
    let placement_encoding = placement_kind & pvs_placement::ENCODING_MASK;
    let placement_flags = placement_kind & pvs_placement::FLAG_MASK;
    let mut pos = 4usize;
    let (def_idx, next) = read_uleb128(block, pos)?;
    pos = next;
    let placement = match placement_encoding {
        x if x == pvs_placement::IDENTITY => Placement::default(),
        x if x == pvs_placement::TRANSLATION => {
            if pos + 24 > block.len() { return Err("short translation placement".into()); }
            let v = read_doubles(block, pos, 3)?;
            pos += 24;
            Placement { translation: [v[0],v[1],v[2]], rotation: Placement::default().rotation }
        }
        x if x == pvs_placement::ROTATION => {
            if pos + 72 > block.len() { return Err("short rotation-only placement".into()); }
            let v = read_doubles(block, pos, 9)?;
            pos += 72;
            Placement { translation: [0.0,0.0,0.0], rotation: [v[0],v[1],v[2],v[3],v[4],v[5],v[6],v[7],v[8]] }
        }
        x if x == pvs_placement::FULL => {
            if pos + 96 > block.len() { return Err("short full placement".into()); }
            let v = read_doubles(block, pos, 12)?;
            pos += 96;
            Placement { translation: [v[0],v[1],v[2]], rotation: [v[3],v[4],v[5],v[6],v[7],v[8],v[9],v[10],v[11]] }
        }
        _ => return Err(format!("unknown PVS placement kind 0x{placement_kind:02x}").into()),
    };
    let (atom, next) = read_pvs_atom(block, pos, strings)?;
    pos = next;
    let feature_id = atom.into_text_lossy();
    if pos != block.len() { return Err("unexpected bytes after PVS child feature id".into()); }
    Ok(ChildOccurrence { definition_index: def_idx as usize, feature_id, placement, placement_kind, placement_encoding, placement_flags })
}

fn component_subrecord_length(stream: &[u8], pos: usize) -> Result<Option<usize>> {
    if pos + 1 >= stream.len() { return Err("truncated PVS record".into()); }
    let descriptor = stream[pos+1];
    match descriptor {
        x if x == pvs_record::LENGTH_U8 => {
            if pos + 3 > stream.len() { return Err("truncated PVS short record".into()); }
            Ok(Some(stream[pos+2] as usize + 3))
        }
        x if x == pvs_record::LENGTH_U8_TYPED => {
            if pos + 4 > stream.len() { return Err("truncated PVS typed short record".into()); }
            Ok(Some(stream[pos+2] as usize + 4))
        }
        x if x == pvs_record::LENGTH_U16 => {
            if pos + 4 > stream.len() { return Err("truncated PVS extended record".into()); }
            Ok(Some(read_u16le(stream, pos+2)? as usize + 4))
        }
        x if x == pvs_record::LENGTH_U16_TYPED => {
            if pos + 5 > stream.len() { return Err("truncated PVS typed extended record".into()); }
            Ok(Some(read_u16le(stream, pos+2)? as usize + 5))
        }
        _ => Ok(None),
    }
}

fn is_standard_geometry_or_child(family: u8, descriptor: u8) -> bool {
    descriptor == pvs_record::LENGTH_U8_TYPED
        && (family == pvs_record::GEOMETRY_FAMILY || family == pvs_record::CHILD_FAMILY)
}

fn validate_component_name_tail(stream: &[u8], mut pos: usize, source_name: &str) -> Result<usize> {
    if stream.get(pos) == Some(&COMPONENT_NAME_OPTIONAL_PREFIX) { pos += 1; }
    if pos + 4 > stream.len() {
        return Err(format!("unexpected bytes after PVS component name {source_name:?}").into());
    }
    let hex: String = stream[pos..pos+4].iter().map(|b| format!("{b:02x}")).collect();
    if !COMPONENT_NAME_TAILS.contains(&hex.as_str()) {
        let ctx: String = stream[pos..(pos+8).min(stream.len())].iter().map(|b| format!("{b:02x}")).collect();
        return Err(format!("unexpected bytes after PVS component name {source_name:?} at offset {pos}: {ctx}").into());
    }
    Ok(pos + 4)
}

/// Parse the component-definition stream.
pub fn parse_pvs_definitions(stream: &[u8], strings: Option<&[String]>) -> Result<Vec<Definition>> {
    let prefix = [pvs_record::DEFINITION_PREFIX_0, pvs_record::DEFINITION_PREFIX_1];
    let mut start_pos: Option<usize> = None;
    for i in 0..stream.len().saturating_sub(1) {
        if stream[i] == prefix[0] && stream[i+1] == prefix[1] { start_pos = Some(i); break; }
    }
    let mut pos = start_pos.ok_or_else(|| openpvz_error("PVS definition stream contains no component records".to_string()))?;
    let mut definitions: Vec<Definition> = Vec::new();
    while pos < stream.len() {
        let remaining = stream.len() - pos;
        if (remaining == 2 || remaining == 4) && stream[pos..].iter().all(|&b| b == 0) { break; }
        if stream[pos] != pvs_record::DEFINITION_PREFIX_0 || stream.get(pos+1) != Some(&pvs_record::DEFINITION_PREFIX_1) {
            return Err(format!("unexpected PVS definition marker at offset {pos}").into());
        }
        pos += 2;
        let (_, next) = read_uleb128(stream, pos)?;
        pos = next; // nameBlockLength consumed for alignment
        if pos >= stream.len() { return Err("truncated PVS name marker".into()); }
        let name_marker = stream[pos];
        if ![pvs_name_marker::AUXILIARY_INLINE, pvs_name_marker::COMPONENT_V1, pvs_name_marker::COMPONENT_V2].contains(&name_marker) {
            return Err(format!("unexpected PVS name marker 0x{name_marker:x} at offset {pos}").into());
        }
        pos += 1;
        let (atom, next) = read_pvs_atom(stream, pos, strings)?;
        pos = next;
        let source_name = atom.into_text_lossy();
        if name_marker == pvs_name_marker::AUXILIARY_INLINE {
            if stream.get(pos) != Some(&AUXILIARY_NAME_SUFFIX) {
                return Err(format!("unexpected bytes after PVS auxiliary name {source_name:?} at offset {pos}").into());
            }
            pos += 1;
        } else {
            pos = validate_component_name_tail(stream, pos, &source_name)?;
        }
        let mut def = Definition {
            index: definitions.len(),
            source_name: source_name.clone(),
            kind: component_kind(&source_name).to_string(),
            ol_name: None, bbox: None, children: Vec::new(), extra_blocks: Vec::new(),
        };
        while pos + 2 <= stream.len() && !(stream[pos] == 0 && stream[pos+1] == 0) {
            let family = stream[pos];
            let descriptor = stream[pos+1];
            let total = component_subrecord_length(stream, pos)?.unwrap_or(0);
            if total == 0 { break; }
            if pos + total > stream.len() { return Err("truncated PVS component subrecord".into()); }
            let block = &stream[pos..pos+total];
            if is_standard_geometry_or_child(family, descriptor) && family == pvs_record::GEOMETRY_FAMILY {
                if def.ol_name.is_some() {
                    return Err(format!("multiple geometry blocks on {source_name:?}").into());
                }
                let (ol, bbox) = parse_geometry_block(block, strings)?;
                def.ol_name = Some(ol);
                def.bbox = Some(bbox);
            } else if is_standard_geometry_or_child(family, descriptor) && family == pvs_record::CHILD_FAMILY {
                def.children.push(parse_child_block(block, strings)?);
            } else {
                def.extra_blocks.push(block.to_vec());
            }
            pos += total;
        }
        if stream.get(pos) != Some(&0) || stream.get(pos+1) != Some(&0) {
            let ctx: String = stream[pos..(pos+8).min(stream.len())].iter().map(|b| format!("{b:02x}")).collect();
            return Err(format!("component {source_name:?} did not terminate cleanly at PVS offset {pos}; next={ctx}").into());
        }
        pos += 2;
        definitions.push(def);
        if pos < stream.len() && stream[pos] == 0 && stream.get(pos+1) == Some(&0) {
            let another = stream.get(pos+2) == Some(&pvs_record::DEFINITION_PREFIX_0)
                && stream.get(pos+3) == Some(&pvs_record::DEFINITION_PREFIX_1);
            if !another { break; } // trailing terminator consumed implicitly; pos unused after loop
        }
    }
    let mut referenced = HashSet::new();
    for d in &definitions {
        for c in &d.children { referenced.insert(c.definition_index); }
    }
    let mut invalid: Vec<usize> = referenced.into_iter().filter(|i| *i >= definitions.len()).collect();
    invalid.sort_unstable();
    if !invalid.is_empty() {
        let preview: Vec<String> = invalid.iter().take(8).map(|i| i.to_string()).collect();
        return Err(format!("PVS child definition reference(s) out of range: {}", preview.join(", ")).into());
    }
    Ok(definitions)
}

/// Parse complete PVS and resolve single root.
pub fn parse_pvs(data: &[u8]) -> Result<PvsModel> {
    let (signature, streams) = parse_pvs_container(data)?;
    let string_streams: Vec<&PvsStream> = streams.iter().filter(|s| s.stream_type == pvs_stream::STRING_TABLE).collect();
    if string_streams.len() > 1 {
        return Err(format!("expected at most one PVS string stream, found {}", string_streams.len()).into());
    }
    let strings = if string_streams.len() == 1 {
        Some(parse_pvs_string_table(&string_streams[0].data)?)
    } else { None };
    let def_streams: Vec<&PvsStream> = streams.iter().filter(|s| s.stream_type == pvs_stream::DEFINITIONS).collect();
    if def_streams.len() != 1 {
        return Err(format!("expected one PVS definition stream, found {}", def_streams.len()).into());
    }
    let definitions = parse_pvs_definitions(&def_streams[0].data, strings.as_deref())?;
    let mut referenced = HashSet::new();
    for d in &definitions {
        for c in &d.children { referenced.insert(c.definition_index); }
    }
    let roots: Vec<usize> = definitions.iter().filter(|d| !referenced.contains(&d.index)).map(|d| d.index).collect();
    if roots.len() != 1 {
        return Err(format!("expected one PVS root definition, found [{}]", roots.iter().map(|i| i.to_string()).collect::<Vec<_>>().join(", ")).into());
    }
    Ok(PvsModel { signature, definitions, root_index: roots[0], streams, strings: strings.clone() })
}

/// Expose placement flag maps for diagnostics.
pub fn pvs_unknown_block_map(definitions: &[Definition]) -> BTreeMap<String, usize> {
    let mut map = BTreeMap::new();
    for d in definitions {
        for b in &d.extra_blocks {
            if b.len() >= 2 {
                let k = format!("{:02x}{:02x}", b[0], b[1]);
                *map.entry(k).or_insert(0) += 1;
            }
        }
    }
    map
}
