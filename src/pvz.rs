//! High-level PVZ archive facade joining ZIP storage with PVS structure.
use crate::pvs::{parse_pvs, PvsModel};
use crate::zip::ZipArchive;
use crate::Result;

pub struct PvzArchive {
    pub path: String,
    pub zip: ZipArchive,
    pub pvs_member: String,
    pub pvs: PvsModel,
    pub members: std::collections::HashSet<String>,
}

impl PvzArchive {
    pub fn open(path: &str) -> Result<Self> {
        let zip = ZipArchive::open(path)?;
        let mut pvs_members: Vec<String> = zip.names().into_iter()
            .filter(|n| n.to_lowercase().ends_with(".pvs")).collect();
        pvs_members.sort();
        if pvs_members.len() != 1 {
            return Err(format!("expected one PVS member in PVZ, found [{}]", pvs_members.join(", ")).into());
        }
        let pvs_member = pvs_members.into_iter().next().unwrap();
        let raw = zip.read(&pvs_member)?;
        let pvs = parse_pvs(&raw)?;
        let members: std::collections::HashSet<String> = zip.names().into_iter().collect();
        Ok(Self { path: path.to_string(), zip, pvs_member, pvs, members })
    }

    pub fn read_ol(&self, member: &str) -> Result<Vec<u8>> {
        self.zip.read(member)
    }

    /// PVS-referenced OL member names missing from the ZIP package.
    pub fn validate_members(&self) -> Vec<String> {
        let mut missing = Vec::new();
        for d in &self.pvs.definitions {
            if let Some(ol) = &d.ol_name {
                if !self.members.contains(ol) { missing.push(ol.clone()); }
            }
        }
        missing
    }
}
