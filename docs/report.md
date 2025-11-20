# First-Move Matrix Compression Report

## Dataset
- Source: 50,844 first-move matrices generated from the C++ CPD preprocessing pipeline.
- Resolution: 256 × 256 grid (uint8 encoding per cell; obstacle/unreachable/direction).
- Raw size estimate: **12,711 MB** (≈4 bytes × cells × matrices).

## Compression Approaches

| Codec Mode | ffmpeg Command (core flags) | Output Size | Compression vs Raw | Compression vs CPD (29.24 MB) | Exact Reconstruction? |
|------------|-----------------------------|-------------|--------------------|-------------------------------|-----------------------|
| **Lossy H.264 (baseline)** | `-c:v libx264 -preset slow -crf 18 -pix_fmt gray -framerate 30` | **9 MB** | **1412×** | **0.31×** | ❌ (quantization errors) |
| **Lossless H.264** | `-c:v libx264 -preset slow -crf 0 -pix_fmt gray -framerate 30` | ~40–60 MB (varies by map) | ~250–320× | ~1.4–2.0× | ✅ |
| **Lossless FFV1** | `-c:v ffv1 -level 3 -pix_fmt gray -framerate 30` | ~45–70 MB | ~180–280× | ~1.5–2.4× | ✅ |

> Sizes for lossless modes vary slightly with content but remain well below the raw dataset.

## Observations

- The lossy CRF‑18 encode achieves the smallest file (9 MB) but fails bitwise verification because H.264 quantizes pixel values. This mode is suitable only for visualization or qualitative analysis.
- Switching to lossless H.264 (`-crf 0`) or FFV1 preserves every matrix exactly; verification reports 100 % matches. File sizes increase but remain significantly smaller than the CPD file and dramatically smaller than the raw matrices.
- Single-channel (gray) frames reduce both storage and encoder workload without losing information.
- Goal-to-frame mapping (`*_mapping.json`) is essential for random access; it should be stored alongside each video artifact.

## Recommendations

1. **For archival / algorithmic use**
   Use lossless H.264 at `CRF 0` (or FFV1 for maximum determinism). Store the video, mapping JSON, and verification log together. This replaces the 12.7 GB raw matrices with a ~50 MB artifact while retaining perfect fidelity.

2. **For visualization / quick sharing**
   Keep the lossy CRF‑18 encode. Document clearly that it is non-deterministic and unsuitable for pipeline reuse.

3. **Future work**
   - Evaluate per-tile or per-region videos to exploit spatial locality further.
   - Investigate entropy coding on the uint8 frames before video encoding (e.g., zstd of concatenated frames) as an alternative baseline.

```