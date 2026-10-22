/// Reverse-engineered format constants shared by the PVS and OL parsers.
/// Keep unexplained numeric values here rather than scattering literals.
pub const PVS_FILE_PREFIX: &str = "PV-PVS";

pub mod pvs_stream {
    pub const STRING_TABLE: u8 = 0x01;
    pub const DEFINITIONS: u8 = 0x02;
}

pub mod pvs_record {
    pub const DEFINITION_PREFIX_0: u8 = 0x20;
    pub const DEFINITION_PREFIX_1: u8 = 0x92;
    pub const CHILD_FAMILY: u8 = 0x21;
    pub const GEOMETRY_FAMILY: u8 = 0x23;
    pub const LENGTH_U8: u8 = 0x02;
    pub const LENGTH_U8_TYPED: u8 = 0x12;
    pub const LENGTH_U16: u8 = 0x04;
    pub const LENGTH_U16_TYPED: u8 = 0x14;
}

pub mod pvs_name_marker {
    pub const AUXILIARY_INLINE: u8 = 0x01;
    pub const COMPONENT_V1: u8 = 0x1e;
    pub const COMPONENT_V2: u8 = 0x1f;
}

pub mod pvs_placement {
    pub const ENCODING_MASK: u8 = 0x0f;
    pub const FLAG_MASK: u8 = 0xf0;
    pub const IDENTITY: u8 = 0x08;
    pub const TRANSLATION: u8 = 0x0a;
    pub const ROTATION: u8 = 0x0c;
    pub const FULL: u8 = 0x0e;
}

pub mod ol_stream {
    pub const ATTRIBUTES: u32 = 0x0100_0000;
    pub const TESSELLATION: u32 = 0x0100_0001;
    pub const EDGE_TESSELLATION: u32 = 0x0100_0002;
    pub const EXACT_V3: u32 = 0x0100_0003;
    pub const EXACT_V4: u32 = 0x0100_0004;
    pub const END: u32 = 0x00ff_ffff;
}

pub mod ol_tag {
    pub const APPEARANCE_VALUES: u32 = 0x05;
    pub const APPEARANCE_NAME: u32 = 0x07;
    pub const TRIANGLE_FAN: u32 = 0x19;
    pub const TRIANGLE_STRIP: u32 = 0x1a;
    pub const TRIANGLE_LIST: u32 = 0x1b;
    pub const TESSELLATION_SURFACE_HEADER: u32 = 0x0100_000d;
    pub const TESSELLATION_VERTICES: u32 = 0x0100_0016;
    pub const EXACT_VERSION: u32 = 0x0100_0002;
    pub const EXACT_COEDGE: u32 = 0x11;
    pub const EXACT_SURFACE: u32 = 0x13;
    pub const EXACT_EDGE: u32 = 0x14;
    pub const EXACT_PCURVE: u32 = 0x15;
    pub const EXACT_LOOP: u32 = 0x0100_0010;
    pub const EXACT_SINGULAR_USE: u32 = 0x0100_0012;
    pub const TERMINATOR: u32 = 0x00;
}

pub mod exact_surface_type {
    pub const CONE: u8 = 1;
    pub const CYLINDER: u8 = 2;
    pub const RATIONAL_BSPLINE: u8 = 4;
    pub const PLANE: u8 = 5;
    pub const SPHERE: u8 = 8;
    pub const TORUS: u8 = 9;
}

pub mod exact_curve_type {
    pub const CIRCLE: u8 = 1;
    pub const DEGENERATE_POINT: u8 = 3;
    pub const ELLIPSE: u8 = 4;
    pub const LINE: u8 = 5;
    pub const RATIONAL_BSPLINE: u8 = 6;
    pub const POLYLINE: u8 = 7;
}

pub const NULL_INDEX: u32 = 0xffff_ffff;

/// Cross-check tolerance used while validating independently encoded OL data.
pub const DEFAULT_PCURVE_TOLERANCE: f64 = 2e-5;
/// Collapse tolerance for edge-less OL08 trim uses.
pub const DEFAULT_IMPLICIT_EDGE_COLLAPSE_TOLERANCE: f64 = 1e-8;
/// Geometric uncertainty written to STEP.
pub const DEFAULT_STEP_UNCERTAINTY: f64 = 5e-6;
/// Narrow sanitizer for one observed sub-micron planar sliver.
pub const DEFAULT_MIN_SLIVER_WIDTH: f64 = 1e-6;
