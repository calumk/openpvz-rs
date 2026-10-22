//! Stable library surface for the PVZ converter.
//! Prefer this module so parser internals can keep evolving.
use std::fmt;

#[derive(Debug)]
pub struct Error(pub String);
impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result { write!(f, "{}", self.0) }
}
impl std::error::Error for Error {}
impl From<String> for Error {
    fn from(s: String) -> Self { Error(s) }
}
impl From<&str> for Error {
    fn from(s: &str) -> Self { Error(s.to_string()) }
}
pub type Result<T> = std::result::Result<T, Error>;
pub fn openpvz_error(s: impl Into<String>) -> Error { Error(s.into()) }

pub mod appearance;
pub mod binary;
pub mod constants;
pub mod diagnostics;
pub mod glb;
pub mod ol;
pub mod ol_exact;
pub mod pvs;
pub mod pvz;
pub mod step;
pub mod step_geom;
pub mod step_text;
pub mod tools;
pub mod version;
pub mod zip;

pub use version::VERSION;
pub use constants::{
    DEFAULT_IMPLICIT_EDGE_COLLAPSE_TOLERANCE, DEFAULT_MIN_SLIVER_WIDTH,
    DEFAULT_PCURVE_TOLERANCE, DEFAULT_STEP_UNCERTAINTY, NULL_INDEX,
};
pub use zip::ZipArchive;
pub use pvs::{parse_pvs, parse_pvs_container, parse_pvs_definitions, parse_pvs_string_table, Placement};
pub use ol::{
    exact_geometry_catalog, extract_ol_position_bounds, extract_ol_surface_meshes,
    iter_ol_tlv, merge_surface_meshes, merge_surface_meshes_by_appearance,
    parse_ol_appearances, parse_ol_streams,
};
pub use ol_exact::{
    build_exact_face_regions, build_exact_vertex_topology, build_shell_candidates,
    classify_exact_coedge, classify_exact_loop, evaluate_exact_curve,
    evaluate_exact_surface, exact_curve_endpoints, exact_curve_parameter_range,
    parse_exact_curve, parse_exact_pcurve, parse_exact_surface, parse_ol_exact_geometry,
    to_occ_intermediate, validate_exact_geometry,
};
pub use appearance::{
    appearance_key, normalize_appearance, shininess_to_roughness,
};
pub use pvz::PvzArchive;
pub use glb::export_glb;
pub use step::{emit_exact_part, export_step, export_step_exact_model, export_step_ol};
pub use step_text::StepWriter;
pub use step::{map_ol_appearances_to_exact_faces, AppearanceMapping};
pub use diagnostics::{model_stats, print_tree, validate_archive};
pub use tools::{
    dump_streams, exact_ol_info, export_occ_intermediate, export_ol_obj,
    inspect_containers, probe_exact_archive, probe_ol_archive, validate_exact_archive,
};
