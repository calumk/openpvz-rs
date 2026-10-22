# Provenance and legal basis

## Summary

openpvz-rs is an independent implementation of the PTC Creo View /
ProductView PVZ file formats. It was developed for interoperability, so that
users can open their own Creo View files in open tools.

The format knowledge comes only from analysing the bytes of lawfully obtained
PVZ data files. The project had no access to PTC source code, binaries,
SDKs, toolkits, format documentation, or confidential material, and used none.
No PTC software was installed, run, or inspected at any stage. This includes
Creo View, Creo View Express, and Creo.

The project is clean-room in this sense: no PTC implementation or
documentation was ever available to it, so there was nothing to separate the
implementation from.

## Method

1. **Byte inspection.** Hex views of the ZIP members and decompressed GZIP
   streams, record framing, tag/length structure, counts, and sizes.
2. **Hypothesis testing.** A proposed meaning for a record or field was kept
   only when it held across every analysed file. A hypothesis that failed on
   any file was rejected or refined.
3. **Self-consistency checks.** The files encode the same geometry more than
   once, in independent forms. A decoded p-curve evaluated on its decoded
   surface must meet the separately stored 3D edge curve within 20 µm. Every
   decoded loop must close in 3D. Results per sample set, including the known
   failures, are in [REVERSE_ENGINEERING_STATUS.md](REVERSE_ENGINEERING_STATUS.md).
4. **Independent output check.** Generated STEP files were read with an
   independent open-source CAD reader (OpenCascade) during development. It is
   not a dependency.

No protection measure was circumvented. PVZ members are ordinary ZIP and GZIP
streams with no encryption or access control.

## AI assistance

An AI model (Anthropic Claude) assisted with byte analysis and with writing
the code and documentation. The model received only the data files, the
analysis results, and this project's own code. It received no PTC
documentation, help pages, forum posts, or other PTC material. Every format
claim in [FORMAT_NOTES.md](FORMAT_NOTES.md) was checked against the bytes of
the analysed files and is not taken on the model's word.

## Scope of the claims

- All record and field names are names chosen for this project. None are
  PTC's names.
- The notes describe the formats as observed in the analysed files. They can
  be incomplete or wrong for versions and features not present in those
  files. They come with no warranty.
- The implementation reproduces file-format behaviour and interoperability
  information. It does not copy PTC program code or protected expression.
- The sample files used for analysis are not distributed with this project.
  Their authors own them.

## Trademarks

PTC, Creo, Creo View, and ProductView are trademarks of their respective
owners. This project uses those names only to identify the files it reads. It
is independent and is not affiliated with, endorsed by, or sponsored by PTC.
