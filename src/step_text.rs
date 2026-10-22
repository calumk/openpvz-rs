//! STEP Part-21 text primitives: number formatting, string encoding,
//! streaming entity writer, appearance styles.
use crate::{Result, openpvz_error};
use std::collections::HashMap;
use std::fs::File;
use std::io::{BufWriter, Write};

/// STEP REAL formatting: 15 significant digits, strip
/// trailing zeros, uppercase E, trailing '.' for integers.
pub fn fnum(x: f64) -> Result<String> {
    if !x.is_finite() {
        return Err(format!("non-finite STEP number {x}").into());
    }
    let mut x = x;
    if x.abs() < 1e-15 {
        x = 0.0;
    }
    Ok(format_15_sig(x))
}

fn format_15_sig(x: f64) -> String {
    if x == 0.0 {
        return "0.".to_string();
    }
    let neg = x < 0.0 || (x == 0.0 && 1.0 / x < 0.0);
    let ax = x.abs();
    // 15 significant digits in exponential form
    let e = format!("{ax:.14e}");
    let (mant, exp): (&str, i32) = {
        let mut it = e.split('e');
        (it.next().unwrap(), it.next().unwrap().parse().unwrap_or(0))
    };
    let digits: String = mant.replace('.', "").chars().collect();
    let stripped = digits.trim_end_matches('0');
    let stripped = if stripped.is_empty() { "0" } else { stripped };
    // exponential form when exp < -6 or exp >= 15 (same rule as 15-digit toPrecision)
    if exp < -6 || exp >= 15 {
        let m = if stripped.len() == 1 {
            stripped.to_string()
        } else {
            format!("{}.{}", &stripped[0..1], &stripped[1..])
        };
        return format!("{}{}E{exp}", if neg { "-" } else { "" }, m);
    }
    let point_pos = exp + 1;
    let mut out = if point_pos <= 0 {
        format!("0.{}{}", "0".repeat((-point_pos) as usize), stripped)
    } else if point_pos as usize >= stripped.len() {
        format!("{stripped}{}", "0".repeat(point_pos as usize - stripped.len()))
    } else {
        format!("{}.{}", &stripped[..point_pos as usize], &stripped[point_pos as usize..])
    };
    if out.contains('.') {
        out = out.trim_end_matches('0').trim_end_matches('.').to_string();
    }
    if !out.contains('.') && !out.contains('E') {
        out.push('.');
    }
    format!("{}{out}", if neg { "-" } else { "" })
}

pub fn tuple2(p: &[f64; 2]) -> Result<String> {
    Ok(format!("({},{})", fnum(p[0])?, fnum(p[1])?))
}
pub fn tuple3(p: &[f64; 3]) -> Result<String> {
    Ok(format!("({},{},{})", fnum(p[0])?, fnum(p[1])?, fnum(p[2])?))
}

pub fn refs(ids: &[u32]) -> String {
    format!("({})", ids.iter().map(|i| format!("#{i}")).collect::<Vec<_>>().join(","))
}

pub fn step_bool(v: bool) -> &'static str {
    if v { ".T." } else { ".F." }
}

pub fn encode_step_string(value: &str) -> String {
    let mut encoded = String::new();
    let mut run = String::new();
    let flush = |encoded: &mut String, run: &mut String| {
        if run.is_empty() {
            return;
        }
        let mut hex = String::new();
        for ch in run.chars() {
            let cp = ch as u32;
            if cp <= 0xffff {
                hex.push_str(&format!("{cp:04X}"));
            } else {
                let s = cp - 0x10000;
                hex.push_str(&format!("{:04X}", 0xd800 + (s >> 10)));
                hex.push_str(&format!("{:04X}", 0xdc00 + (s & 0x3ff)));
            }
        }
        encoded.push_str(&format!("\\X2\\{hex}\\X0\\"));
        run.clear();
    };
    for ch in value.chars() {
        let cp = ch as u32;
        if (0x20..=0x7e).contains(&cp) {
            flush(&mut encoded, &mut run);
            if ch == '\'' {
                encoded.push_str("''");
            } else if ch == '\\' {
                encoded.push_str("\\\\");
            } else {
                encoded.push(ch);
            }
        } else {
            run.push(ch);
        }
    }
    flush(&mut encoded, &mut run);
    format!("'{encoded}'")
}

fn utc_stamp() -> String {
    use std::time::{SystemTime, UNIX_EPOCH};
    let secs = SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0);
    let days = (secs / 86400) as i64;
    let sod = (secs % 86400) as i64;
    let z = days + 719468;
    let era = (if z >= 0 { z } else { z - 146096 }) / 146097;
    let doe = z - era * 146097;
    let yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = if m <= 2 { y + 1 } else { y };
    format!("{y:04}-{m:02}-{d:02}T{:02}:{:02}:{:02}", (sod / 3600) % 24, (sod / 60) % 60, sod % 60)
}

/// Streaming Part-21 entity writer with incrementing #ids.
pub struct StepWriter {
    out: BufWriter<File>,
    pub next_id: u32,
    pub entity_count: usize,
    finished: bool,
    pub appearance_style_cache: HashMap<String, (u32, crate::appearance::NormalizedAppearance)>,
}

impl StepWriter {
    pub fn new(path: &str, generator: &str, source_name: &str) -> Result<Self> {
        let file = File::create(path).map_err(|e| openpvz_error(format!("cannot write {path}: {e}")))?;
        let mut w = Self {
            out: BufWriter::with_capacity(4 * 1024 * 1024, file),
            next_id: 1,
            entity_count: 0,
            finished: false,
            appearance_style_cache: HashMap::new(),
        };
        w.raw("ISO-10303-21;\nHEADER;\n")?;
        w.raw("FILE_DESCRIPTION(('Creo View PVZ exact B-rep conversion'),'2;1');\n")?;
        w.raw(&format!(
            "FILE_NAME({},{},('openpvz-rs'),('openpvz-rs'),{},{},'');\n",
            encode_step_string(source_name),
            encode_step_string(&utc_stamp()),
            encode_step_string(generator),
            encode_step_string(generator)
        ))?;
        w.raw("FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));\nENDSEC;\nDATA;\n")?;
        Ok(w)
    }

    fn raw(&mut self, text: &str) -> Result<()> {
        self.out.write_all(text.as_bytes()).map_err(|e| openpvz_error(format!("STEP write failed: {e}")))?;
        Ok(())
    }

    pub fn ent(&mut self, body: &str) -> Result<u32> {
        let id = self.next_id;
        self.next_id += 1;
        self.raw(&format!("#{id} = {body};\n"))?;
        self.entity_count += 1;
        Ok(id)
    }

    pub fn finish(mut self) -> Result<()> {
        if !self.finished {
            self.raw("ENDSEC;\nEND-ISO-10303-21;\n")?;
            self.out.flush().map_err(|e| openpvz_error(format!("STEP flush failed: {e}")))?;
            self.finished = true;
        }
        Ok(())
    }
}

/// Attach AP214 presentation styles; returns (styled_items, unique, styled_faces, unstyled_faces, presentation).
pub fn step_appearance_style(
    writer: &mut StepWriter,
    app: &crate::ol::Appearance,
) -> Result<(u32, crate::appearance::NormalizedAppearance)> {
    let n = crate::appearance::normalize_appearance(app);
    let key = crate::appearance::appearance_key(Some(app));
    if let Some(hit) = writer.appearance_style_cache.get(&key) {
        return Ok(hit.clone());
    }
    let color = writer.ent(&format!(
        "COLOUR_RGB({},{},{},{})",
        encode_step_string(&n.name),
        fnum(n.rgb[0])?,
        fnum(n.rgb[1])?,
        fnum(n.rgb[2])?
    ))?;
    let fill_color = writer.ent(&format!("FILL_AREA_STYLE_COLOUR('',#{color})"))?;
    let fill_area = writer.ent(&format!("FILL_AREA_STYLE('',(#{fill_color}))"))?;
    let fill_style = writer.ent(&format!("SURFACE_STYLE_FILL_AREA(#{fill_area})"))?;
    let mut side_items = vec![fill_style];
    let transparency = (1.0 - n.opacity).max(0.0).min(1.0);
    if transparency > 1e-6 {
        let t = writer.ent(&format!("SURFACE_STYLE_TRANSPARENT({})", fnum(transparency)?))?;
        side_items.push(writer.ent(&format!(
            "SURFACE_STYLE_RENDERING_WITH_PROPERTIES(.NORMAL_SHADING.,#{color},(#{t}))"
        ))?);
    }
    let side_style = writer.ent(&format!(
        "SURFACE_SIDE_STYLE({},{})",
        encode_step_string(&n.name),
        refs(&side_items)
    ))?;
    let usage = writer.ent(&format!("SURFACE_STYLE_USAGE(.BOTH.,#{side_style})"))?;
    let assignment = writer.ent(&format!("PRESENTATION_STYLE_ASSIGNMENT((#{usage}))"))?;
    writer.appearance_style_cache.insert(key, (assignment, n.clone()));
    Ok((assignment, n))
}
