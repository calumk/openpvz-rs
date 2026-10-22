# openpvz-rs

Standalone converter for PTC Creo View / ProductView `.pvz` packages, written
in Rust.

```text
Input: .PVZ  -->  openpvz  -->  Output: .GLB / .STEP
```

Two output paths:

- **PVZ → GLB**: tessellated geometry, normals, decoded visual materials,
  hierarchy, transforms and repeated instances.
- **PVZ → STEP/STP**: AP214 exact B-rep writer with analytic/NURBS geometry,
  p-curves, shared-edge topology, open/closed shells, visual styles and
  assembly placements.

No CAD kernel, no Creo installation and no external converter is needed.

> **Provenance.** This is an independent implementation, reverse-engineered
> only from the bytes of lawfully obtained PVZ data files, for
> interoperability. No PTC software, source code, SDK, documentation, or
> confidential material was used, and no protection measure was circumvented.
> AI assistance was used; it received no PTC material. Not affiliated with or
> endorsed by PTC. Method and details: [docs/LEGAL.md](docs/LEGAL.md).

## Requirements

- Rust stable (2021 edition). `cargo build --release` is all you need.
- Dependencies (crates.io): `flate2`, `serde`/`serde_json`, `sha2`, `clap`.

## Build

```bash
cargo build --release
./target/release/pvz convert model.pvz model.step
./target/release/pvz convert model.pvz model.glb
```

## CLI

```text
pvz info <file.pvz>
pvz tree <file.pvz> [--depth N]
pvz validate <file.pvz> [--json report.json]

pvz convert <file.pvz> <output.glb|output.step|output.stp> [options]
pvz step-ol <file.pvz> <member.ol> <output.step> [STEP options]
pvz obj <file.pvz> <member.ol> <output.obj>

pvz exact <file.pvz> <member.ol> [--validate] [--tolerance M]
pvz validate-exact <file.pvz> [--tolerance M] [--json report.json]
pvz probe-exact <file.pvz>
pvz probe-ol <file.pvz>
pvz inspect <file.pvz> [--sample-ol N]
pvz dump <file.pvz> <dir> [--ol member.ol]
pvz occ-json <file.pvz> <member.ol> <output.json> [--include-normalized-pcurves]
```

STEP options (`convert` to STEP and `step-ol`):

```text
--pcurve-tolerance M   UV/3D validation tolerance; default 2e-5 m
--step-uncertainty M   uncertainty written into STEP; default 5e-6 m
--min-sliver-width M   omit pathological planar two-edge slivers below M;
                       default 1e-6 m, set 0 to preserve everything
--tolerance M          alias for --pcurve-tolerance
```

GLB option: `--include-construction` keeps datum/support surfaces that are
otherwise omitted when exact topology identifies them as non-faces.

## Library use

```rust
use openpvz::{PvzArchive, export_glb, export_step};

let archive = PvzArchive::open("model.pvz")?;
export_step(&archive, "model.step", 2e-5, 5e-6, 1e-6, None)?;
export_glb(&archive, "model.glb", false, None)?;
```

To consume the exact B-rep model directly instead of writing STEP:

```rust
use openpvz::{PvzArchive, parse_ol_exact_geometry};

let archive = PvzArchive::open("part.pvz")?;
let ol = archive.read_ol("part_1.ol")?;
let model = parse_ol_exact_geometry(&ol)?.expect("exact stream");
// model.faces / .edges / .pcurves / .face_regions / .shells  (metres, radians)
```

## Layout

| Module | Responsibility |
|---|---|
| `binary.rs` | ULEB128, PVS atoms, small helpers |
| `zip.rs` | Minimal stored/deflate ZIP reader for PVZ |
| `pvs.rs` | PVS container, definitions, hierarchy, placements |
| `ol.rs` | OL stream directory, TLV, appearances, tessellation |
| `ol_exact.rs` | Exact surfaces/curves/pcurves, B-rep topology, validation |
| `appearance.rs` | Appearance normalization, PBR mapping helpers |
| `pvz.rs` | ZIP + PVS archive facade |
| `glb.rs` | glTF/GLB serializer |
| `step_text.rs` | Part-21 writer, REAL formatting, string encoding, styles |
| `step_geom.rs` | STEP curves/surfaces/frames, OL-UV → STEP-UV mapping |
| `step.rs` | B-rep emission, orientation solving, assembly export |
| `diagnostics.rs` | Structure/tessellation checks, tree printing |
| `tools.rs` | Inspection/probing/extraction helpers |
| `constants.rs` | Reverse-engineered IDs and shared tolerances |

## Format documentation

- [docs/FORMAT_NOTES.md](docs/FORMAT_NOTES.md): observed PVZ/PVS/OL binary
  layout, record types and geometry parameterisations.
- [docs/REVERSE_ENGINEERING_STATUS.md](docs/REVERSE_ENGINEERING_STATUS.md):
  what is validated and what is still uncertain.
- [docs/LEGAL.md](docs/LEGAL.md): provenance and legal basis.

## Critical invariants (do not break)

1. **PVS rotations** are three local basis vectors, not a row-major matrix.
2. **Face regions**: loop marker `-2` starts an outer group; `>= 0` is a
   backward reference to an earlier `-2`; `-1` is degenerate/null.
3. **Nesting parity**: even depth = material, odd = hole; islands split out.
4. **Coedge sense** is derived geometrically, never from the raw byte.
5. **P-curves**: OL UV ≠ STEP UV; remap through 3D before emitting.
6. **Tolerances**: 20 µm validation vs 5 µm STEP uncertainty stay separate.
7. **Dedup**: STEP hashes OL payload bytes; product structure stays distinct.

## Behaviour notes

- **Deterministic output**: all map iteration that affects entity numbering
  uses `BTreeMap`/sorted order. Two runs produce byte-identical STEP `DATA`
  sections and GLB files.
- **Failed-part quarantine**: `export_step` records undecodable members in
  `failed_parts` and continues instead of aborting the whole assembly.
- **Construction filtering fallback**: when the exact model cannot be aligned
  to the tessellation, GLB export keeps all surfaces for that part and
  counts it in `construction_filter_exact_fallbacks`.

## Tests

```bash
cargo test                # unit tests (+ local corpus tests, if installed)
cargo test -- --ignored   # heavy local corpus entries
```

No sample `.pvz` files are distributed. Corpus regression tests in
`tests/corpus.rs` run against your own files: put them in `test_files/` with
a `test_files/corpus.json` manifest listing the expected values (copy
`tests/corpus.example.json`). Without a manifest the corpus tests skip.

## Licence

MIT. See [LICENSE](LICENSE).
