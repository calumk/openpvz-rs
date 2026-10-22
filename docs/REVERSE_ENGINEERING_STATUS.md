# Reverse-engineering status

## High confidence / corpus validated

- PVZ ZIP packaging
- PVS0301 and PVS0304 stream containers
- PVS string pool / atoms
- component definitions, occurrence hierarchy and repeated instances
- identity, translation, rotation-only and full placements
- PVS `3c12` / `3c14` auxiliary-record framing
- OL08BL and OL09BL stream directories
- tessellated vertices/normals and fan/strip/list primitives
- appearance indirection and 14-float visual-material records (RGB/diffuse/specular/shininess/emissive/opacity)
- exact-face appearance alignment, topology-safe inference and STEP AP214 presentation styles
- glTF PBR/specular/transparency material mapping with lossless source appearance extras
- exact stream version and precision selection
- supporting surfaces, loops, coedges, shared 3D edges and pcurves
- explicit shared vertex reconstruction
- singular-edge topology
- plane / cylinder / cone / sphere / torus parameterisations
- rational B-spline surface orders, knots, U-major homogeneous control net and weights
- line / circle / ellipse / point / rational-B-spline / polyline 3D curves
- geometric coedge sense
- stored-vs-reversed loop traversal
- loop-marker back references and nested island/hole decomposition
- shell candidate connectivity
- AP214 exact B-rep output with pcurves
- AP214 product hierarchy and assembly placement output
- content-level OL geometry deduplication

## Particularly strong evidence

The parser cross-validates independently encoded UV/surface geometry against shared 3D edge geometry:

- smaller sample set: 40,176 / 40,176 coedges pass
- larger sample set: 499,373 / 499,373 coedges pass
- maximum errors are ~2.45 µm and ~13.27 µm respectively
- third sample set: 1,053,923 coedges tested, 82 fail the 20 µm check. All 82 are in 4 of 3,195 parts, with a maximum error of ~0.14 mm (mostly on cone faces). The cause has not been determined.
- all 111,940 observed loops close under the derived traversal rule

Positive loop-marker semantics were checked across all analysed files: all 8,484 positive markers point backward to the global loop index of an earlier `-2` outer-loop group on the same supporting surface.

## Special topology now handled

- `0x01000010 = -1`: one observed zero-area/null OL09 loop; retained diagnostically and excluded from STEP face regions.
- `0x01000012`: topological singular use. The referenced 3D curve is not always a type-3 point; some are tiny B-splines/polylines and are still singular at model tolerance.
- one OL09 planar two-edge face has ~0.596 µm characteristic width and independently becomes unorientable in OpenCascade. The STEP writer has a narrow, configurable sliver filter and reports each omission.
- one supporting-surface record can contain multiple disconnected face regions and nested hole/island loops; the converter splits these into separate `ADVANCED_FACE`s.

## Current limitations / uncertainty

- PVZ/PVS/OL versions not present in the analysed files may introduce new records or geometry classes.
- Several raw flag bytes and trailing supporting-surface payload fields remain unnamed; they have not been necessary for geometry reconstruction.
- STEP currently serializes RGB and transparency; legacy shininess/specular/emissive are represented more completely in GLB than in STEP.
- Textures/decals have not been decoded or exported.
- In the larger OL09 sample set, 7 of 37,159 unique emitted STEP faces have no unambiguous explicit/inferable source appearance and are intentionally left unstyled.
- Secondary/normalized pcurves are retained by diagnostics but the STEP writer uses the validated primary native pcurve.
- The STEP writer intentionally sanitizes singular topology rather than attempting to reproduce kernel-internal degenerate edges byte-for-byte.
- Full OpenCascade transfer/BRep checking of the largest sample assembly was not completed; its STEP output parses as one root and representative complex part exports validate independently.

## Runtime architecture

```text
PVZ -> PVS/OL decoder -> exact B-rep model -> AP214 STEP
                    \-> tessellation -> GLB
```

OpenCascade was used only as an independent reader to check STEP output during development. It is not a dependency.
