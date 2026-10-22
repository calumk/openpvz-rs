//! Minimal ZIP reader sufficient for unencrypted PVZ packages.
//! Supported: single-disk ZIP, stored/deflate members. Deliberately
//! unsupported: encryption and ZIP64.
use crate::{Result, openpvz_error};
use flate2::read::DeflateDecoder;
use std::collections::BTreeMap;
use std::io::Read;

const SIG_EOCD: u32 = 0x0605_4b50;
const SIG_CENTRAL: u32 = 0x0201_4b50;
const SIG_LOCAL: u32 = 0x0403_4b50;

const METHOD_STORED: u16 = 0;
const METHOD_DEFLATE: u16 = 8;

const FLAG_ENCRYPTED: u16 = 1 << 0;
const FLAG_UTF8: u16 = 1 << 11;

#[derive(Debug, Clone)]
pub struct ZipEntry {
    pub name: String,
    pub flags: u16,
    pub method: u16,
    pub crc32: u32,
    pub compressed_size: u32,
    pub uncompressed_size: u32,
    pub local_offset: u32,
}

pub struct ZipArchive {
    pub path: String,
    pub data: Vec<u8>,
    pub entries: BTreeMap<String, ZipEntry>,
}

impl ZipArchive {
    pub fn open(path: &str) -> Result<Self> {
        let data = std::fs::read(path).map_err(|e| openpvz_error(format!("cannot read {path}: {e}")))?;
        Self::from_bytes(path.to_string(), data)
    }

    pub fn from_bytes(path: String, data: Vec<u8>) -> Result<Self> {
        let mut archive = Self { path, data, entries: BTreeMap::new() };
        archive.parse_central_directory()?;
        Ok(archive)
    }

    fn read_u16(&self, pos: usize) -> Result<u16> {
        if pos + 2 > self.data.len() { return Err("truncated ZIP u16".into()); }
        Ok(u16::from_le_bytes([self.data[pos], self.data[pos + 1]]))
    }

    fn read_u32(&self, pos: usize) -> Result<u32> {
        if pos + 4 > self.data.len() { return Err("truncated ZIP u32".into()); }
        Ok(u32::from_le_bytes([self.data[pos], self.data[pos+1], self.data[pos+2], self.data[pos+3]]))
    }

    fn find_eocd(&self) -> Result<usize> {
        if self.data.len() < 22 { return Err("ZIP end-of-central-directory record not found".into()); }
        let minimum = self.data.len().saturating_sub(0xffff + 22);
        let mut pos = self.data.len() - 22;
        loop {
            if self.read_u32(pos)? == SIG_EOCD { return Ok(pos); }
            if pos == minimum { break; }
            pos -= 1;
        }
        Err("ZIP end-of-central-directory record not found".into())
    }

    fn parse_central_directory(&mut self) -> Result<()> {
        let eocd = self.find_eocd()?;
        let disk_number = self.read_u16(eocd + 4)?;
        let cd_disk = self.read_u16(eocd + 6)?;
        let entries_on_disk = self.read_u16(eocd + 8)?;
        let entry_count = self.read_u16(eocd + 10)?;
        let cd_size = self.read_u32(eocd + 12)?;
        let cd_offset = self.read_u32(eocd + 16)?;
        if disk_number != 0 || cd_disk != 0 || entries_on_disk != entry_count {
            return Err("multi-disk ZIP/PVZ is not supported".into());
        }
        if entry_count == 0xffff || cd_size == 0xffff_ffff || cd_offset == 0xffff_ffff {
            return Err("ZIP64 PVZ is not supported yet".into());
        }
        let mut pos = cd_offset as usize;
        let expected_end = cd_offset as usize + cd_size as usize;
        for index in 0..entry_count as usize {
            if pos + 46 > self.data.len() || self.read_u32(pos)? != SIG_CENTRAL {
                return Err(format!("invalid ZIP central directory at entry {index}").into());
            }
            let flags = self.read_u16(pos + 8)?;
            let method = self.read_u16(pos + 10)?;
            let crc32 = self.read_u32(pos + 16)?;
            let compressed_size = self.read_u32(pos + 20)?;
            let uncompressed_size = self.read_u32(pos + 24)?;
            let name_len = self.read_u16(pos + 28)? as usize;
            let extra_len = self.read_u16(pos + 30)? as usize;
            let comment_len = self.read_u16(pos + 32)? as usize;
            let local_offset = self.read_u32(pos + 42)?;
            let name_start = pos + 46;
            let name_end = name_start + name_len;
            if name_end > self.data.len() { return Err("truncated ZIP filename".into()); }
            let raw = &self.data[name_start..name_end];
            let name = if flags & FLAG_UTF8 != 0 {
                String::from_utf8_lossy(raw).into_owned()
            } else {
                raw.iter().map(|&b| b as char).collect()
            };
            self.entries.insert(name.clone(), ZipEntry {
                name, flags, method, crc32, compressed_size, uncompressed_size, local_offset,
            });
            pos = name_end + extra_len + comment_len;
        }
        if pos != expected_end {
            return Err(format!("ZIP central directory ended at {pos}, expected {expected_end}").into());
        }
        Ok(())
    }

    pub fn names(&self) -> Vec<String> {
        self.entries.keys().cloned().collect()
    }

    pub fn has(&self, name: &str) -> bool {
        self.entries.contains_key(name)
    }

    pub fn read(&self, name: &str) -> Result<Vec<u8>> {
        let entry = self.entries.get(name)
            .ok_or_else(|| openpvz_error(format!("ZIP member not found: {name}")))?;
        if entry.flags & FLAG_ENCRYPTED != 0 {
            return Err(format!("encrypted ZIP member is not supported: {name}").into());
        }
        let lo = entry.local_offset as usize;
        if lo + 30 > self.data.len() || self.read_u32(lo)? != SIG_LOCAL {
            return Err(format!("invalid ZIP local header for {name}").into());
        }
        let name_len = self.read_u16(lo + 26)? as usize;
        let extra_len = self.read_u16(lo + 28)? as usize;
        let start = lo + 30 + name_len + extra_len;
        let end = start + entry.compressed_size as usize;
        if end > self.data.len() {
            return Err(format!("compressed data runs past EOF for {name}").into());
        }
        let compressed = &self.data[start..end];
        let output = match entry.method {
            METHOD_STORED => compressed.to_vec(),
            METHOD_DEFLATE => {
                let mut dec = DeflateDecoder::new(compressed);
                let mut out = Vec::with_capacity(entry.uncompressed_size as usize);
                dec.read_to_end(&mut out).map_err(|e| openpvz_error(format!("deflate failed for {name}: {e}")))?;
                out
            }
            m => return Err(format!("unsupported ZIP compression method {m} for {name}").into()),
        };
        if output.len() != entry.uncompressed_size as usize {
            return Err(format!("size mismatch for {name}: got {}, expected {}", output.len(), entry.uncompressed_size).into());
        }
        Ok(output)
    }
}
