//! Small binary/collection helpers shared by the parsers.
use crate::Result;

/// Read one unsigned LEB128 value, returning (value, next_offset).
pub fn read_uleb128(data: &[u8], mut pos: usize) -> Result<(u64, usize)> {
    let mut value: u64 = 0;
    let mut shift: u32 = 0;
    loop {
        if pos >= data.len() {
            return Err("truncated ULEB128".into());
        }
        let byte = data[pos];
        pos += 1;
        value |= ((byte & 0x7f) as u64) << shift;
        if byte & 0x80 == 0 {
            if value > (usize::MAX as u64) {
                return Err("ULEB128 exceeds addressable range".into());
            }
            return Ok((value, pos));
        }
        shift += 7;
        if shift > 63 {
            return Err("ULEB128 too large".into());
        }
    }
}

/// PVS string atom: odd codes are string-table refs, even codes are inline
/// UTF-8 byte lengths. Returns (Some(text), pos) or (None, pos) with raw code.
pub fn read_pvs_atom(
    data: &[u8],
    pos: usize,
    strings: Option<&[String]>,
) -> Result<(PvsAtom, usize)> {
    let (code, mut pos) = read_uleb128(data, pos)?;
    if code & 1 == 1 {
        let index = ((code - 1) / 2) as usize;
        match strings {
            None => Ok((PvsAtom::RawRef(code), pos)),
            Some(table) => {
                if index >= table.len() {
                    return Err(format!("PVS string reference {index} out of range").into());
                }
                Ok((PvsAtom::Text(table[index].clone()), pos))
            }
        }
    } else {
        let byte_len = (code / 2) as usize;
        let end = pos + byte_len;
        if end > data.len() {
            return Err("truncated inline PVS atom".into());
        }
        let text = std::str::from_utf8(&data[pos..end])
            .map_err(|e| format!("invalid UTF-8 in PVS atom: {e}"))?
            .to_string();
        pos = end;
        Ok((PvsAtom::Text(text), pos))
    }
}

#[derive(Debug, Clone)]
pub enum PvsAtom {
    Text(String),
    RawRef(u64),
}

impl PvsAtom {
    pub fn into_text_lossy(self) -> String {
        match self {
            PvsAtom::Text(s) => s,
            PvsAtom::RawRef(code) => format!("@stringref:{code}"),
        }
    }
}

pub fn align4(value: usize) -> usize {
    (value + 3) & !3
}

pub fn hex32(value: u32) -> String {
    format!("0x{value:08x}")
}

pub fn component_kind(source_name: &str) -> &'static str {
    let lower = source_name.to_lowercase();
    if has_extension_token(&lower, ".prt") {
        return "part";
    }
    if has_extension_token(&lower, ".asm") {
        return "assembly";
    }
    "other"
}

/// True when `ext` occurs followed by a `,` or the end of the name
/// (e.g. `bracket.prt` or `bracket.prt,2`), but not `bracket.prtx`.
fn has_extension_token(name: &str, ext: &str) -> bool {
    name.match_indices(ext).any(|(i, _)| {
        matches!(name.as_bytes().get(i + ext.len()), None | Some(b','))
    })
}

pub fn clamp01(value: f64) -> f64 {
    value.max(0.0).min(1.0)
}
