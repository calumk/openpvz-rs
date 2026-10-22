//! Fixture-free unit tests for pure helpers.
use openpvz::binary::{component_kind, read_pvs_atom, read_uleb128};
use openpvz::pvs::Placement;
use openpvz::step_geom::{evaluate_step_local_surface, inverse_step_surface_uv, surface_frame};
use openpvz::step_text::{encode_step_string, fnum};

#[test]
fn uleb128_vectors() {
    assert_eq!(read_uleb128(&[0x00], 0).unwrap(), (0, 1));
    assert_eq!(read_uleb128(&[0x7f], 0).unwrap(), (127, 1));
    assert_eq!(read_uleb128(&[0x81, 0x00], 0).unwrap(), (1, 2));
    assert_eq!(read_uleb128(&[0xe5, 0x8e, 0x26], 0).unwrap(), (624485, 3));
    assert!(read_uleb128(&[], 0).is_err());
    assert!(read_uleb128(&[0x80], 0).is_err());
}

#[test]
fn pvs_atoms() {
    // inline "Hi": code = 2*2 = 4
    let (atom, pos) = read_pvs_atom(&[0x04, b'H', b'i'], 0, None).unwrap();
    assert_eq!(atom.into_text_lossy(), "Hi");
    assert_eq!(pos, 3);
    // string-table ref index 2: code = 2*2+1 = 5
    let table = vec!["a".to_string(), "b".to_string(), "c".to_string()];
    let (atom, _) = read_pvs_atom(&[0x05], 0, Some(&table)).unwrap();
    assert_eq!(atom.into_text_lossy(), "c");
}

#[test]
fn kinds() {
    assert_eq!(component_kind("bracket.PRT"), "part");
    assert_eq!(component_kind("bracket.prt,2"), "part");
    assert_eq!(component_kind("top.ASM"), "assembly");
    assert_eq!(component_kind("notes.prtx"), "other");
    assert_eq!(component_kind("datum"), "other");
}

#[test]
fn step_number_formatting() {
    assert_eq!(fnum(0.0).unwrap(), "0.");
    assert_eq!(fnum(-0.0).unwrap(), "0.");
    assert_eq!(fnum(1.0).unwrap(), "1.");
    assert_eq!(fnum(-1.0).unwrap(), "-1.");
    assert_eq!(fnum(1.5).unwrap(), "1.5");
    assert_eq!(fnum(0.1).unwrap(), "0.1");
    assert_eq!(fnum(100.0).unwrap(), "100.");
    assert_eq!(fnum(1e-6).unwrap(), "0.000001");
    assert_eq!(fnum(1e-7).unwrap(), "1E-7");
    assert_eq!(fnum(1e-16).unwrap(), "0.");
    assert_eq!(fnum(1.5e20).unwrap(), "1.5E20");
    assert!(fnum(f64::INFINITY).is_err());
    assert!(fnum(f64::NAN).is_err());
}

#[test]
fn step_strings() {
    assert_eq!(encode_step_string("plain"), "'plain'");
    assert_eq!(encode_step_string("a'b"), "'a''b'");
    assert_eq!(encode_step_string("A\\B"), "'A\\\\B'");
    assert_eq!(encode_step_string("caf\u{e9}"), "'caf\\X2\\00E9\\X0\\'");
}

#[test]
fn placement_identity() {
    let p = Placement::default();
    assert!(p.is_identity());
    assert_eq!(
        p.gltf_matrix(),
        [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    );
    // basis triplets are stored consecutively: glTF columns follow source order
    let q = Placement { translation: [1.0, 2.0, 3.0], rotation: [0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0] };
    assert!(!q.is_identity());
    let m = q.gltf_matrix();
    assert_eq!([m[0], m[1], m[2]], [0.0, 1.0, 0.0]);
    assert_eq!([m[12], m[13], m[14]], [1.0, 2.0, 3.0]);
}

#[test]
fn plane_uv_scales_through_step_frame() {
    // OL planes may use scaled basis vectors; STEP UV is in frame metres.
    let surface = openpvz::ol_exact::ExactSurface::Plane {
        flags: 0,
        marker: 0,
        origin: [0.0, 0.0, 0.0],
        u_vector: [2.0, 0.0, 0.0],
        v_vector: [0.0, 3.0, 0.0],
        normal: [0.0, 0.0, 1.0],
        u_range: [0.0, 1.0],
        v_range: [0.0, 1.0],
        normalized: None,
        trailing_bytes: 0,
    };
    let frame = surface_frame(&surface).unwrap().unwrap();
    assert_eq!(frame.x, [1.0, 0.0, 0.0]);
    let native = [0.5, 0.25];
    let p3 = openpvz::ol_exact::evaluate_exact_surface(&surface, native[0], native[1]).unwrap();
    assert_eq!(p3, [1.0, 0.75, 0.0]);
    let mapped = inverse_step_surface_uv(&surface, &p3, None).unwrap();
    assert!((mapped[0] - 1.0).abs() < 1e-12 && (mapped[1] - 0.75).abs() < 1e-12);
    let local = evaluate_step_local_surface(&surface, mapped).unwrap();
    assert!((local[0] - 1.0).abs() < 1e-12 && (local[2]).abs() < 1e-12);
}

#[test]
fn frames_normalize_large_vectors_without_overflow() {
    // A naive sqrt(x*x + y*y + z*z) overflows to infinity here.
    let surface = openpvz::ol_exact::ExactSurface::Plane {
        flags: 0,
        marker: 0,
        origin: [0.0, 0.0, 0.0],
        u_vector: [3e200, 4e200, 0.0],
        v_vector: [-4e200, 3e200, 0.0],
        normal: [0.0, 0.0, 2e200],
        u_range: [0.0, 1.0],
        v_range: [0.0, 1.0],
        normalized: None,
        trailing_bytes: 0,
    };
    let frame = surface_frame(&surface).unwrap().unwrap();
    assert!((frame.x[0] - 0.6).abs() < 1e-15 && (frame.x[1] - 0.8).abs() < 1e-15);
    assert_eq!(frame.z, [0.0, 0.0, 1.0]);
}
