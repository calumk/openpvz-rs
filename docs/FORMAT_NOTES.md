# Reverse-engineering notes

These notes describe the PTC Creo View / ProductView PVZ formats as observed in the sample files analysed for this project. All field names are names chosen during reverse engineering; none come from PTC documentation.

## PVZ

A `.pvz` is a ZIP package containing a `.pvs` structure file and `.ol` geometry members.

## PVS

Observed signatures:

- `PV-PVS0301`
- `PV-PVS0304`

The header describes named GZIP streams. In the analysed files:

- stream type 1: string pool
- stream type 2: component definitions/hierarchy
- other streams: metadata/parameters not required by the current importer

### String atoms

ULEB128 atom code:

- even: inline UTF-8 byte length = code / 2
- odd: string-pool index = (code - 1) / 2

### Component subrecord framing

Observed generic framing:

```text
xx 02 LL       <LL bytes>
xx 12 LL TT    <LL bytes>
xx 04 LL LL    <uint16_le length bytes>
xx 14 LL LL TT <uint16_le length bytes>
```

For example, `3c14` is the extended typed form of the `3c12` auxiliary-record family.

### Placement kinds

- `0x08`: identity
- `0x0a`: translation, 3 x float64
- `0x0c`: 3x3 orientation, 9 x float64
- `0x0e`: translation + 3x3 orientation, 12 x float64

The nine orientation values are **three serialized basis vectors**, in local X, Y, Z order. Each triplet is an axis expressed in the parent coordinate system. They must therefore become the **columns** of the active local-to-parent rotation matrix. Treating the nine values as a conventional row-major active matrix transposes/inverts the rotation and gives visibly wrong assembly orientations.

This convention is independently supported by PVS0304 assembly-level OL extents: among 129 assembly definitions whose baked OL bounds discriminate between the two conventions, the basis-vector/transpose interpretation matches better in 124 cases. Several assemblies match exactly only with the basis-vector interpretation.

## OL container

Observed signatures:

- `DIV-OL08BL`
- `DIV-OL09BL`

The OL header contains a stream directory. Each payload stream is GZIP compressed and, after decompression, consists of TLV records:

```text
uint32_le tag
uint32_le payload_size
byte[payload_size] payload
```

Operational stream identities:

- `0x01000000`: attributes/appearances
  - tag `0x05`, 56 bytes: 14 float32 legacy shaded-material values: source RGB, lit/diffuse RGB, specular RGB, shininess, emissive RGB, opacity
  - tag `0x07`: appearance name/base-index indirection
- `0x01000001`: surface tessellation
- `0x01000002`: edge tessellation
- `0x01000003`: OL09 exact geometry, internal version 3 / float32
- `0x01000004`: OL08 exact geometry, internal version 4 / float64
- `0x00ffffff`: directory end sentinel

The exact stream begins with TLV tag `0x01000002`, payload uint32 exact-version.

## Tessellation

Key records:

- `0x0100000d`: surface/appearance header
- `0x01000016`: vertex block
- `0x19`: triangle fan
- `0x1a`: triangle strip
- `0x1b`: triangle list

Observed normal-bearing vertex block:

```text
uint8 flags = 1
uint32 vertex_count
repeat:
  float32 x y z
  float32 nx ny nz
```

## Exact topology

Exact records are ordered as face data, then the shared edge table, then the pcurve table.

- `0x13`: supporting surface / face record
- `0x01000010`: begin face loop; payload is a signed 32-bit marker
- `0x11`: coedge / use of shared edge; payload length 13
- `0x00`: loop terminator
- `0x01000012`: singular/degenerate edge use
- `0x14`: shared 3D edge curve
- `0x15`: 2D pcurve sample table


### Loop marker semantics (`0x01000010`)

The payload is a signed 32-bit marker. Corpus-wide behavior is now deterministic:

- `-2`: starts a new outer-loop group on the current supporting surface.
- positive value: references the **global loop index** of an earlier `-2` outer loop on the same support surface. This associates later hole/island loops with their owning outer group even when they are not adjacent in the stream.
- `-1`: observed once, on a zero-area planar two-edge loop. It is retained diagnostically and excluded from emitted STEP face regions.

The positive-reference rule held for all 8,484 positive markers in the analysed files.

A marker group can contain nested loops. UV containment depth is therefore used after marker association: depth 0 loops form outer face regions, depth 1 loops are holes, depth 2 loops become islands/new faces, and so on. This is required for valid STEP topology on concentric/nested trims.

### Coedge payload (`0x11`)

```text
uint32 edge_index
uint8  orientation_byte
uint32 primary_pcurve_index
uint32 normalized_pcurve_index   // 0xffffffff when absent
```

The raw orientation byte is retained but should not be assumed to map directly to an edge orientation. The decoder derives actual 3D curve sense by comparing the pcurve-on-surface endpoints with the shared 3D edge endpoints.

### Singular edge use (`0x01000012`)

```text
uint32 edge_index
scalar parameter_0
scalar parameter_1
```

This record marks a **topologically singular edge use**. The referenced `0x14` curve is often a type-3 degenerate point, but it can also be a numerically tiny B-spline/polyline. In the analysed files those non-point singular curves collapse within model tolerance. STEP export unions their endpoint vertices and omits the collapsed coedge from the wire.

### Pcurve (`0x15`)

```text
uint8  type = 7
uint8  flags
uint32 count
repeat count:
  scalar u
  scalar v
uint8 terminator = 0
```

The primary pcurve is in the native supporting-surface parameter domain. The optional second pcurve is a normalized/domain-mapped form. Native pcurves are used for exact validation and topology/sense recovery.

## Exact surfaces (`0x13`)

Scalar width is determined by exact version: v4 float64, v3 float32.

### Type 5: plane

Validated parameterization:

```text
O[3], U[3], V[3], N[3], uMin,uMax,vMin,vMax
P(u,v) = O + U*u + V*v
```

### Type 2: cylinder

```text
O[3], X[3], Y[3], Zstep[3], uMin,uMax,vMin,vMax
P(u,v) = O + X*cos(u) + Y*sin(u) + Zstep*v
```

### Type 1: cone

```text
O[3], Xgrowth[3], Ygrowth[3], Zstep[3], A,B, uMin,uMax,vMin,vMax
P(u,v) = O + Zstep*v + (Xgrowth*cos(u)+Ygrowth*sin(u))*(A+B*v)
```

In the analysed files A=0 and B=1 in every sampled record.

### Type 8: sphere

```text
C[3], X[3], Y[3], Z[3], uMin,uMax,vMin,vMax
P(u,v) = C + cos(v)*(X*cos(u)+Y*sin(u)) + Z*sin(v)
```

### Type 9: torus

The minor/tube frame is encoded independently from the major-circle frame:

```text
O[3]
majorCos[3]
majorSin[3]
axisUnit[3]
minorCos[3]
minorAxis[3]
minorSinDirection[3]
uMin,uMax,vMin,vMax

r = |minorCos|
major = majorCos*cos(u) + majorSin*sin(u)
minorRadial = minorCos*cos(u) - minorSinDirection*r*sin(u)
P = O + major + minorRadial*cos(v) + minorAxis*sin(v)
```

This formulation also validates the near-zero-major-radius torus records present in OL09.

### Type 4: rational B-spline surface

```text
byte type=4
byte flags
byte marker
byte reserved
uint32 uOrder
uint32 uKnotCount
scalar uKnots[uKnotCount]
uint32 vOrder
uint32 vKnotCount
scalar vKnots[vKnotCount]
uint32 uPoleCount
uint32 vPoleCount
scalar homogeneousPoles[uPoleCount*vPoleCount][4] // x,y,z,w
...
```

Validated identities:

```text
uPoleCount = uKnotCount - uOrder
vPoleCount = vKnotCount - vOrder
```

Pole ordering is U-major:

```text
index = uIndex * vPoleCount + vIndex
```

The xyz values are homogeneous coordinates and are divided by w to obtain Euclidean poles; w is the rational weight.

## Exact 3D edge curves (`0x14`)

### Type 5: line

```text
O[3], D[3]
P(t) = O + D*t, t in [0,1]
```

### Type 1: circle / circular arc

Trimmed form:

```text
center[3], cosVector[3], sinVector[3], parameterStart,parameterEnd
```

A compact 9-scalar full-circle form also exists. All compact full-circle records in the analysed files are unreferenced construction geometry.

### Type 4: ellipse / elliptical arc

Same layout as trimmed circle, with unequal radius-vector magnitudes.

### Type 3: degenerate point

```text
point[3]
```

### Type 6: rational B-spline curve

```text
byte type=6
byte flags
byte reserved
uint32 order
uint32 knotCount
scalar knots[knotCount]
uint32 poleCount
scalar homogeneousPoles[poleCount][4]
byte terminator=0
```

Validated `poleCount = knotCount - order`.

### Type 7: polyline

```text
byte type=7
byte flags
uint32 pointCount
scalar xyz[pointCount][3]
byte terminator=0
```

## Validation result

The exact decoder is validated by comparing two independently encoded paths to the same geometry:

```text
primary pcurve UV -> decoded support surface -> 3D point
shared edge record -> decoded 3D curve -> 3D endpoint
```

At tolerance 20 micrometres:

- smaller sample set: 40,176 / 40,176 coedges pass
- larger sample set: 499,373 / 499,373 coedges pass

All 111,940 observed loops also close in 3D after selecting stored or reversed coedge-list traversal from the pcurve endpoints.

## STEP mapping used by the standalone writer

The writer maps the decoded exact model directly to AP214/ISO-10303-21 entities; no CAD kernel is used.

Core topology mapping:

```text
OL shared vertex       -> VERTEX_POINT
OL 0x14 curve          -> STEP 3D curve
shared edge            -> EDGE_CURVE
OL 0x15 + surface      -> PCURVE
3D curve + pcurve(s)   -> SURFACE_CURVE
OL 0x11 coedge         -> ORIENTED_EDGE
reconstructed loop     -> EDGE_LOOP
outer/inner trim       -> FACE_OUTER_BOUND / FACE_BOUND
face region            -> ADVANCED_FACE
closed component       -> CLOSED_SHELL + MANIFOLD_SOLID_BREP
open component         -> OPEN_SHELL + SHELL_BASED_SURFACE_MODEL
PVS definition/child   -> AP214 product + assembly occurrence placement
```

Creo View's native UV coordinates are not assumed to equal STEP's canonical UV coordinates. Pcurve samples are evaluated on the decoded OL surface and transformed into the STEP surface's native parameter domain, including periodic branch alignment. Near-zero-major-radius OL tori are canonicalized as spheres and their pcurves are remapped consistently.

The writer separates two tolerances:

- pcurve validation tolerance: default `2e-5 m` (20 µm), used to reject inconsistent decoded geometry;
- STEP declared uncertainty: default `5e-6 m` (5 µm), used in the STEP representation context.

A narrow optional sliver sanitizer removes planar, one-loop, two-edge regions with characteristic width below `1e-6 m` by default. In the analysed files this omits exactly one ~0.596 µm OL09 face that an independent OpenCascade reader reports as unorientable. Set `--min-sliver-width 0` to disable this behavior.

