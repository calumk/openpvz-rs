//! STEP geometry primitives: 3D curves, surfaces, canonical frames,
//! and native-OL-UV to canonical-STEP-UV mapping.
use crate::ol_exact::{evaluate_exact_curve, evaluate_exact_surface, exact_curve_parameter_range, ExactCurve, ExactSurface};
use crate::step_text::{fnum, refs, step_bool, StepWriter};
use crate::{Result, openpvz_error};

const TWO_PI: f64 = std::f64::consts::PI * 2.0;
const TORUS_SPHERE_EPS: f64 = 1e-8;

fn vnorm(v: &[f64; 3]) -> f64 { v[0].hypot(v[1]).hypot(v[2]) }
fn vdot(a: &[f64; 3], b: &[f64; 3]) -> f64 { a[0]*b[0]+a[1]*b[1]+a[2]*b[2] }
fn vsub(a: &[f64; 3], b: &[f64; 3]) -> [f64; 3] { [a[0]-b[0], a[1]-b[1], a[2]-b[2]] }
fn vcross(a: &[f64; 3], b: &[f64; 3]) -> [f64; 3] {
    [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]
}
fn vunit(v: &[f64; 3]) -> Result<[f64; 3]> {
    let l = vnorm(v);
    if !(l > 1e-30) { return Err("zero-length direction".into()); }
    Ok([v[0]/l, v[1]/l, v[2]/l])
}
fn vdist(a: &[f64; 3], b: &[f64; 3]) -> f64 { vnorm(&vsub(a, b)) }

fn point3(writer: &mut StepWriter, p: &[f64; 3]) -> Result<u32> {
    writer.ent(&format!("CARTESIAN_POINT('',({},{},{}))", fnum(p[0])?, fnum(p[1])?, fnum(p[2])?))
}
fn point2(writer: &mut StepWriter, p: &[f64; 2]) -> Result<u32> {
    writer.ent(&format!("CARTESIAN_POINT('',({},{}))", fnum(p[0])?, fnum(p[1])?))
}
fn direction3(writer: &mut StepWriter, v: &[f64; 3]) -> Result<u32> {
    let u = vunit(v)?;
    writer.ent(&format!("DIRECTION('',({},{},{}))", fnum(u[0])?, fnum(u[1])?, fnum(u[2])?))
}
fn axis2_placement(writer: &mut StepWriter, point: &[f64; 3], axis: &[f64; 3], refdir: &[f64; 3]) -> Result<u32> {
    let loc = point3(writer, point)?;
    let z = direction3(writer, axis)?;
    let x = direction3(writer, refdir)?;
    writer.ent(&format!("AXIS2_PLACEMENT_3D('',#{loc},#{z},#{x})"))
}

/// Convert a decoded PVS placement to STEP local-Z/local-X placement form.
/// PVS stores local X/Y/Z basis vectors consecutively in parent coordinates.
pub fn placement_axis(writer: &mut StepWriter, translation: &[f64; 3], rotation: &[f64; 9]) -> Result<u32> {
    let lx = [rotation[0], rotation[1], rotation[2]];
    let lz = [rotation[6], rotation[7], rotation[8]];
    axis2_placement(writer, translation, &lz, &lx)
}

fn compress_knots(full: &[f64]) -> (Vec<f64>, Vec<usize>) {
    let mut values: Vec<f64> = Vec::new();
    let mut mults: Vec<usize> = Vec::new();
    for &k in full {
        if let Some(last) = values.last() {
            if (k - *last).abs() < 1e-13 {
                *mults.last_mut().unwrap() += 1;
                continue;
            }
        }
        values.push(k);
        mults.push(1);
    }
    (values, mults)
}

/// Emit one decoded OL 3D curve as the nearest STEP curve primitive.
/// Returns None for topologically singular (degenerate-point) curves.
pub fn curve_entity_3d(writer: &mut StepWriter, curve: &ExactCurve) -> Result<Option<u32>> {
    match curve {
        ExactCurve::Line { origin, vector, .. } => {
            let p = point3(writer, origin)?;
            let d = direction3(writer, vector)?;
            let v = writer.ent(&format!("VECTOR('',#{d},1.)"))?;
            Ok(Some(writer.ent(&format!("LINE('',#{p},#{v})"))?))
        }
        ExactCurve::Circle { center, cos_vector, sin_vector, .. } => {
            let placement = axis2_placement(writer, center, &vcross(cos_vector, sin_vector), cos_vector)?;
            Ok(Some(writer.ent(&format!("CIRCLE('',#{placement},{})", fnum(vnorm(cos_vector))?))?))
        }
        ExactCurve::Ellipse { center, cos_vector, sin_vector, .. } => {
            let (mut major_r, mut minor_r) = (vnorm(cos_vector), vnorm(sin_vector));
            let mut refdir = *cos_vector;
            if minor_r > major_r {
                std::mem::swap(&mut major_r, &mut minor_r);
                refdir = *sin_vector;
            }
            let placement = axis2_placement(writer, center, &vcross(cos_vector, sin_vector), &refdir)?;
            Ok(Some(writer.ent(&format!("ELLIPSE('',#{placement},{},{})", fnum(major_r)?, fnum(minor_r)?))?))
        }
        ExactCurve::Polyline { points, .. } => {
            let mut ids = Vec::with_capacity(points.len());
            for p in points { ids.push(point3(writer, p)?); }
            Ok(Some(writer.ent(&format!("POLYLINE('',{}", refs(&ids)))?))
        }
        ExactCurve::RationalBspline { order, knots, poles, .. } => {
            let mut cps = Vec::with_capacity(poles.len());
            for pole in poles { cps.push(point3(writer, &pole.point)?); }
            let (values, mults) = compress_knots(knots);
            let degree = order - 1;
            let closed = vdist(&poles[0].point, &poles[poles.len()-1].point) < 1e-12;
            let weights: Vec<String> = poles.iter().map(|p| fnum(p.weight)).collect::<Result<_>>()?;
            let kvals: Vec<String> = values.iter().map(|v| fnum(*v)).collect::<Result<_>>()?;
            Ok(Some(writer.ent(&format!(
                "( BOUNDED_CURVE() B_SPLINE_CURVE({degree},{cps},.UNSPECIFIED.,{closed},{open}) B_SPLINE_CURVE_WITH_KNOTS(({m}),({k}),.UNSPECIFIED.) CURVE() GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_CURVE(({w})) REPRESENTATION_ITEM('') )",
                cps = refs(&cps),
                closed = step_bool(closed), open = step_bool(false),
                m = mults.iter().map(|m| m.to_string()).collect::<Vec<_>>().join(","),
                k = kvals.join(","),
                w = weights.join(",")
            ))?))
        }
        ExactCurve::DegeneratePoint { .. } => Ok(None),
        ExactCurve::Unknown { .. } => Err("unsupported exact curve type for STEP".into()),
    }
}

pub fn torus_is_sphere_like(s: &ExactSurface) -> bool {
    if let ExactSurface::Torus { major_cos, minor_cos, .. } = s {
        let (major, minor) = (vnorm(major_cos), vnorm(minor_cos));
        return minor > 0.0 && major / minor < TORUS_SPHERE_EPS;
    }
    false
}

#[derive(Debug, Clone)]
pub struct Frame { pub origin: [f64; 3], pub x: [f64; 3], pub y: [f64; 3], pub z: [f64; 3] }

/// Canonical orthonormal frame of the STEP surface emitted for an OL surface.
pub fn surface_frame(surface: &ExactSurface) -> Result<Option<Frame>> {
    match surface {
        ExactSurface::Plane { origin, u_vector, normal, .. } => {
            let z = vunit(normal)?;
            let x = vunit(u_vector)?;
            let y = vunit(&vcross(&z, &x))?;
            Ok(Some(Frame { origin: *origin, x, y, z }))
        }
        ExactSurface::Cylinder { origin, cos_vector, axis_step, .. } => {
            let z = vunit(axis_step)?;
            let x = vunit(cos_vector)?;
            let y = vunit(&vcross(&z, &x))?;
            Ok(Some(Frame { origin: *origin, x, y, z }))
        }
        ExactSurface::Cone { origin, cos_growth, axis_step, radial_offset, .. } => {
            let z = vunit(axis_step)?;
            let mut basis = *cos_growth;
            if *radial_offset < 0.0 { basis = [-basis[0], -basis[1], -basis[2]]; }
            let x = vunit(&basis)?;
            let y = vunit(&vcross(&z, &x))?;
            Ok(Some(Frame { origin: *origin, x, y, z }))
        }
        ExactSurface::Sphere { center, cos_vector, axis_vector, .. } => {
            let z = vunit(axis_vector)?;
            let x = vunit(cos_vector)?;
            let y = vunit(&vcross(&z, &x))?;
            Ok(Some(Frame { origin: *center, x, y, z }))
        }
        ExactSurface::Torus { origin, major_cos, axis_unit, minor_cos, minor_axis, .. } => {
            if torus_is_sphere_like(surface) {
                let z = vunit(minor_axis)?;
                let x = vunit(minor_cos)?;
                let y = vunit(&vcross(&z, &x))?;
                Ok(Some(Frame { origin: *origin, x, y, z }))
            } else {
                let z = vunit(axis_unit)?;
                let x = vunit(major_cos)?;
                let y = vunit(&vcross(&z, &x))?;
                Ok(Some(Frame { origin: *origin, x, y, z }))
            }
        }
        _ => Ok(None),
    }
}

/// Emit one decoded OL surface as a STEP exact surface.
pub fn surface_entity(writer: &mut StepWriter, surface: &ExactSurface) -> Result<u32> {
    match surface {
        ExactSurface::Plane { origin, u_vector, normal, .. } => {
            let p = axis2_placement(writer, origin, normal, u_vector)?;
            writer.ent(&format!("PLANE('',#{p})"))
        }
        ExactSurface::Cylinder { origin, cos_vector, axis_step, .. } => {
            let p = axis2_placement(writer, origin, axis_step, cos_vector)?;
            writer.ent(&format!("CYLINDRICAL_SURFACE('',#{p},{})", fnum(vnorm(cos_vector))?))
        }
        ExactSurface::Cone { origin, cos_growth, axis_step, radial_offset, radial_slope, .. } => {
            let radial_scale = vnorm(cos_growth);
            let axial_scale = vnorm(axis_step);
            let sign = if *radial_offset < 0.0 { -1.0 } else { 1.0 };
            let angle = (radial_scale * sign * radial_slope).atan2(axial_scale);
            let radius = (radial_scale * radial_offset).abs();
            let refdir = if sign < 0.0 { [-cos_growth[0], -cos_growth[1], -cos_growth[2]] } else { *cos_growth };
            let p = axis2_placement(writer, origin, axis_step, &refdir)?;
            writer.ent(&format!("CONICAL_SURFACE('',#{p},{},{})", fnum(radius)?, fnum(angle)?))
        }
        ExactSurface::Sphere { center, cos_vector, axis_vector, .. } => {
            let p = axis2_placement(writer, center, axis_vector, cos_vector)?;
            writer.ent(&format!("SPHERICAL_SURFACE('',#{p},{})", fnum(vnorm(cos_vector))?))
        }
        ExactSurface::Torus { origin, major_cos, axis_unit, minor_cos, minor_axis, .. } => {
            let (major_r, minor_r) = (vnorm(major_cos), vnorm(minor_cos));
            if torus_is_sphere_like(surface) {
                let p = axis2_placement(writer, origin, minor_axis, minor_cos)?;
                return writer.ent(&format!("SPHERICAL_SURFACE('',#{p},{})", fnum(minor_r)?));
            }
            let p = axis2_placement(writer, origin, axis_unit, major_cos)?;
            if major_r < minor_r {
                writer.ent(&format!("DEGENERATE_TOROIDAL_SURFACE('',#{p},{},{},.T.)", fnum(major_r)?, fnum(minor_r)?))
            } else {
                writer.ent(&format!("TOROIDAL_SURFACE('',#{p},{},{})", fnum(major_r)?, fnum(minor_r)?))
            }
        }
        ExactSurface::RationalBspline { u_order, v_order, u_knots, v_knots, u_pole_count, v_pole_count, poles, .. } => {
            let mut point_rows: Vec<String> = Vec::with_capacity(*u_pole_count);
            let mut weight_rows: Vec<String> = Vec::with_capacity(*u_pole_count);
            for ui in 0..*u_pole_count {
                let mut prow: Vec<String> = Vec::with_capacity(*v_pole_count);
                let mut wrow: Vec<String> = Vec::with_capacity(*v_pole_count);
                for vi in 0..*v_pole_count {
                    let pole = &poles[ui * *v_pole_count + vi];
                    prow.push(format!("#{}", point3(writer, &pole.point)?));
                    wrow.push(fnum(pole.weight)?);
                }
                point_rows.push(format!("({})", prow.join(",")));
                weight_rows.push(format!("({})", wrow.join(",")));
            }
            let (uk, um) = compress_knots(u_knots);
            let (vk, vm) = compress_knots(v_knots);
            let uks: Vec<String> = uk.iter().map(|v| fnum(*v)).collect::<Result<_>>()?;
            let vks: Vec<String> = vk.iter().map(|v| fnum(*v)).collect::<Result<_>>()?;
            writer.ent(&format!(
                "( BOUNDED_SURFACE() B_SPLINE_SURFACE({uo},{vo},({rows}),.UNSPECIFIED.,.F.,.F.,.F.) B_SPLINE_SURFACE_WITH_KNOTS(({u}),({v}),({kuu}),({kvv}),.UNSPECIFIED.) GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_SURFACE(({w})) REPRESENTATION_ITEM('') SURFACE() )",
                uo = u_order - 1, vo = v_order - 1,
                rows = point_rows.join(","),
                u = um.iter().map(|m| m.to_string()).collect::<Vec<_>>().join(","),
                v = vm.iter().map(|m| m.to_string()).collect::<Vec<_>>().join(","),
                kuu = uks.join(","),
                kvv = vks.join(","),
                w = weight_rows.join(",")
            ))
        }
        ExactSurface::Unknown { .. } => Err("unsupported exact surface type for STEP".into()),
    }
}

// ---------- UV mapping ----------

fn unwrap_angle(value: f64, previous: Option<f64>) -> f64 {
    let Some(mut prev) = previous else { return value; };
    let _ = &mut prev;
    let mut v = value;
    let p = previous.unwrap();
    while v - p > std::f64::consts::PI { v -= TWO_PI; }
    while v - p < -std::f64::consts::PI { v += TWO_PI; }
    v
}

/// Invert the canonical STEP surface emitted by surface_entity().
pub fn inverse_step_surface_uv(surface: &ExactSurface, point: &[f64; 3], previous: Option<[f64; 2]>) -> Result<[f64; 2]> {
    if matches!(surface, ExactSurface::RationalBspline { .. }) {
        return Err("B-spline inverse should use native UV".into());
    }
    let frame = surface_frame(surface)?.ok_or_else(|| openpvz_error("no frame for surface inverse"))?;
    let local = vsub(point, &frame.origin);
    let (x, y, z) = (vdot(&local, &frame.x), vdot(&local, &frame.y), vdot(&local, &frame.z));
    match surface {
        ExactSurface::Plane { .. } => Ok([x, y]),
        ExactSurface::Cylinder { .. } => Ok([unwrap_angle(y.atan2(x), previous.map(|p| p[0])), z]),
        ExactSurface::Cone { cos_growth, axis_step, radial_offset, radial_slope, .. } => {
            let radial_scale = vnorm(cos_growth);
            let axial_scale = vnorm(axis_step);
            let sign = if *radial_offset < 0.0 { -1.0 } else { 1.0 };
            let angle = (radial_scale * sign * radial_slope).atan2(axial_scale);
            let cosine = angle.cos();
            let v = if cosine.abs() > 1e-14 { z / cosine } else { 0.0 };
            let radius = (radial_scale * radial_offset).abs() + v * angle.sin();
            let mut u = y.atan2(x);
            if radius < 0.0 { u += std::f64::consts::PI; }
            Ok([unwrap_angle(u, previous.map(|p| p[0])), v])
        }
        ExactSurface::Sphere { .. } => Ok([
            unwrap_angle(y.atan2(x), previous.map(|p| p[0])),
            z.atan2(x.hypot(y)),
        ]),
        ExactSurface::Torus { major_cos, minor_cos, .. } => {
            if torus_is_sphere_like(surface) {
                return Ok([
                    unwrap_angle(y.atan2(x), previous.map(|p| p[0])),
                    z.atan2(x.hypot(y)),
                ]);
            }
            let major_r = vnorm(major_cos).max(1e-18);
            let minor_r = vnorm(minor_cos);
            let rho = x.hypot(y);
            let azimuth = y.atan2(x);
            let mut candidates: Vec<(f64, f64, f64, f64)> = Vec::new();
            for signed_rho in [rho, -rho] {
                let cos_v = (signed_rho - major_r) / minor_r;
                let sin_v = z / minor_r;
                if cos_v.abs() > 1.0001 || sin_v.abs() > 1.0001 { continue; }
                let mut u = azimuth + if signed_rho < 0.0 { std::f64::consts::PI } else { 0.0 };
                let mut v = sin_v.clamp(-1.0, 1.0).atan2(cos_v.clamp(-1.0, 1.0));
                u = unwrap_angle(u, previous.map(|p| p[0]));
                v = unwrap_angle(v, previous.map(|p| p[1]));
                let radius = major_r + minor_r * v.cos();
                let rec = [radius * u.cos(), radius * u.sin(), minor_r * v.sin()];
                let err = vnorm(&[x - rec[0], y - rec[1], z - rec[2]]);
                let cont = previous.map(|p| (u - p[0]).hypot(v - p[1])).unwrap_or(0.0);
                candidates.push((err, cont, u, v));
            }
            if candidates.is_empty() {
                return Ok([
                    unwrap_angle(azimuth, previous.map(|p| p[0])),
                    unwrap_angle(z.atan2(rho - major_r), previous.map(|p| p[1])),
                ]);
            }
            candidates.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap().then(a.1.partial_cmp(&b.1).unwrap()));
            Ok([candidates[0].2, candidates[0].3])
        }
        _ => Err("unsupported surface inverse".into()),
    }
}

/// Reconstruct a canonical STEP-local 3D point from a mapped UV pair (residual check only).
pub fn evaluate_step_local_surface(surface: &ExactSurface, uv: [f64; 2]) -> Option<[f64; 3]> {
    let (u, v) = (uv[0], uv[1]);
    match surface {
        ExactSurface::Plane { .. } => Some([u, v, 0.0]),
        ExactSurface::Cylinder { cos_vector, .. } => {
            let r = vnorm(cos_vector);
            Some([r * u.cos(), r * u.sin(), v])
        }
        ExactSurface::Cone { cos_growth, axis_step, radial_offset, radial_slope, .. } => {
            let radial_scale = vnorm(cos_growth);
            let axial_scale = vnorm(axis_step);
            let sign = if *radial_offset < 0.0 { -1.0 } else { 1.0 };
            let angle = (radial_scale * sign * radial_slope).atan2(axial_scale);
            let base = (radial_scale * radial_offset).abs();
            let radius = base + v * angle.sin();
            Some([radius * u.cos(), radius * u.sin(), v * angle.cos()])
        }
        ExactSurface::Sphere { cos_vector, .. } => {
            let r = vnorm(cos_vector);
            let cv = v.cos();
            Some([r * cv * u.cos(), r * cv * u.sin(), r * v.sin()])
        }
        ExactSurface::Torus { major_cos, minor_cos, .. } => {
            let minor_r = vnorm(minor_cos);
            if torus_is_sphere_like(surface) {
                let cv = v.cos();
                return Some([minor_r * cv * u.cos(), minor_r * cv * u.sin(), minor_r * v.sin()]);
            }
            let major_r = vnorm(major_cos).max(1e-18);
            let radius = major_r + minor_r * v.cos();
            Some([radius * u.cos(), radius * u.sin(), minor_r * v.sin()])
        }
        _ => None,
    }
}

/// Map sampled OL p-curve points into the emitted STEP surface parameter space.
pub fn map_native_pcurve_to_step(
    surface: &ExactSurface,
    points: &[[f64; 2]],
    source_surface: &ExactSurface,
) -> Result<(Vec<[f64; 2]>, f64)> {
    if std::ptr::eq(source_surface as *const _, surface as *const _)
        && matches!(surface, ExactSurface::RationalBspline { .. }) {
        return Ok((points.to_vec(), 0.0));
    }
    // NURBS identity case by structural equality fallback
    if matches!(surface, ExactSurface::RationalBspline { .. })
        && matches!(source_surface, ExactSurface::RationalBspline { .. }) {
        return Ok((points.to_vec(), 0.0));
    }
    let frame = surface_frame(surface)?.ok_or_else(|| openpvz_error("no frame for pcurve mapping"))?;
    let mut out = Vec::with_capacity(points.len());
    let mut previous: Option<[f64; 2]> = None;
    let mut max_residual: f64 = 0.0;
    for nuv in points {
        let p3 = evaluate_exact_surface(source_surface, nuv[0], nuv[1])
            .ok_or_else(|| openpvz_error(format!("failed evaluating {} pcurve", surface.type_name())))?;
        if !p3.iter().all(|x| x.is_finite()) {
            return Err(format!("failed evaluating {} pcurve", surface.type_name()).into());
        }
        let mapped = inverse_step_surface_uv(surface, &p3, previous)?;
        out.push(mapped);
        previous = Some(mapped);
        if let Some(rec) = evaluate_step_local_surface(surface, mapped) {
            let local = vsub(&p3, &frame.origin);
            let actual = [vdot(&local, &frame.x), vdot(&local, &frame.y), vdot(&local, &frame.z)];
            max_residual = max_residual.max(vdist(&actual, &rec));
        }
    }
    Ok((out, max_residual))
}

fn dist2(a: &[f64; 3], b: &[f64; 3]) -> f64 {
    let (dx, dy, dz) = (a[0]-b[0], a[1]-b[1], a[2]-b[2]);
    dx*dx + dy*dy + dz*dz
}

/// Nearest point on an exact 3D edge curve (48-sample coarse + 28 golden iterations).
pub fn closest_exact_curve_parameter(curve: &ExactCurve, target: &[f64; 3]) -> Option<f64> {
    let [start, end] = exact_curve_parameter_range(curve)?;
    if !(end > start) { return Some(start); }
    let n = 48;
    let (mut bi, mut bd): (usize, f64) = (0, f64::INFINITY);
    for i in 0..=n {
        let t = start + (end - start) * (i as f64 / n as f64);
        let p = evaluate_exact_curve(curve, t)?;
        let d = dist2(&p, target);
        if d < bd { bd = d; bi = i; }
    }
    let mut low = start + (end - start) * (bi.saturating_sub(1) as f64 / n as f64);
    let mut high = start + (end - start) * ((bi + 1).min(n) as f64 / n as f64);
    let golden = (5.0_f64.sqrt() - 1.0) / 2.0;
    let mut left = high - golden * (high - low);
    let mut right = low + golden * (high - low);
    let mut ld = evaluate_exact_curve(curve, left).map(|p| dist2(&p, target)).unwrap_or(f64::INFINITY);
    let mut rd = evaluate_exact_curve(curve, right).map(|p| dist2(&p, target)).unwrap_or(f64::INFINITY);
    for _ in 0..28 {
        if ld <= rd {
            high = right; right = left; rd = ld;
            left = high - golden * (high - low);
            ld = evaluate_exact_curve(curve, left).map(|p| dist2(&p, target)).unwrap_or(f64::INFINITY);
        } else {
            low = left; left = right; ld = rd;
            right = low + golden * (high - low);
            rd = evaluate_exact_curve(curve, right).map(|p| dist2(&p, target)).unwrap_or(f64::INFINITY);
        }
    }
    Some(if ld <= rd { left } else { right })
}

/// Damped Gauss-Newton UV refinement so the decoded surface passes through target.
pub fn refine_surface_uv_to_point(surface: &ExactSurface, target: &[f64; 3], initial: [f64; 2]) -> ([f64; 2], f64) {
    let (mut u, mut v) = (initial[0], initial[1]);
    for _ in 0..24 {
        let p = match evaluate_exact_surface(surface, u, v) { Some(p) if p.iter().all(|x| x.is_finite()) => p, _ => break };
        let r = vsub(&p, target);
        let old_err = vnorm(&r);
        if old_err < 1e-10 { break; }
        let hu = (1e-7_f64).max(u.abs() * 1e-8);
        let hv = (1e-7_f64).max(v.abs() * 1e-8);
        let (pu, pv) = match (evaluate_exact_surface(surface, u + hu, v), evaluate_exact_surface(surface, u, v + hv)) {
            (Some(a), Some(b)) if a.iter().all(|x| x.is_finite()) && b.iter().all(|x| x.is_finite()) => (a, b),
            _ => break,
        };
        let du = [(pu[0]-p[0])/hu, (pu[1]-p[1])/hu, (pu[2]-p[2])/hu];
        let dv = [(pv[0]-p[0])/hv, (pv[1]-p[1])/hv, (pv[2]-p[2])/hv];
        let (aa, ab, bb) = (vdot(&du,&du), vdot(&du,&dv), vdot(&dv,&dv));
        let (ar, br) = (-vdot(&du,&r), -vdot(&dv,&r));
        let det = aa*bb - ab*ab;
        if det.abs() < 1e-30 { break; }
        let (su, sv) = ((ar*bb - ab*br)/det, (aa*br - ab*ar)/det);
        let mut scale = 1.0;
        let mut accepted = false;
        for _ in 0..16 {
            let (cu, cv) = (u + su*scale, v + sv*scale);
            if let Some(c) = evaluate_exact_surface(surface, cu, cv) {
                if c.iter().all(|x| x.is_finite()) && vdist(&c, target) < old_err {
                    u = cu; v = cv; accepted = true; break;
                }
            }
            scale *= 0.5;
        }
        if !accepted || (su*scale).hypot(sv*scale) < 1e-12 { break; }
    }
    let residual = evaluate_exact_surface(surface, u, v)
        .filter(|p| p.iter().all(|x| x.is_finite()))
        .map(|p| vdist(&p, target)).unwrap_or(f64::INFINITY);
    ([u, v], residual)
}

/// Emit a sampled-polyline PCURVE + DEFINITIONAL_REPRESENTATION.
pub fn pcurve_entity(writer: &mut StepWriter, surface_ref: u32, points: &[[f64; 2]], parametric_context: u32) -> Result<u32> {
    if points.len() < 2 { return Err("pcurve requires at least two points".into()); }
    let mut ids = Vec::with_capacity(points.len());
    for p in points {
        ids.push(point2(writer, p)?);
    }
    let curve2d = writer.ent(&format!("POLYLINE('',{}", refs(&ids)))?;
    let rep = writer.ent(&format!("DEFINITIONAL_REPRESENTATION('',(#{curve2d}),#{parametric_context})"))?;
    writer.ent(&format!("PCURVE('',#{surface_ref},#{rep})"))
}
