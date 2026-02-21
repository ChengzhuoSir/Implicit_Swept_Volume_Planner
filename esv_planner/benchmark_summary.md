# Benchmark Summary

| scenario | robot | method | success_rate | runtime_ms | duration_s | length_m | min_svsdf |
|---|---|---:|---:|---:|---:|---:|---:|
| maze | L | ESV | 1.000 | 1256.49 | 11.24 | 11.26 | 0.000 |
| maze | L | RC_ESDF_like | 1.000 | 1340.11 | 10.83 | 13.45 | 0.000 |
| maze | L | SVSDF_like | 1.000 | 1288.39 | 10.83 | 10.97 | 0.000 |
| maze | T | ESV | 1.000 | 2758.84 | 12.11 | 11.10 | 0.000 |
| maze | T | RC_ESDF_like | 1.000 | 2776.32 | 10.83 | 17.31 | 0.000 |
| maze | T | SVSDF_like | 1.000 | 2594.28 | 10.83 | 10.96 | 0.000 |
| office | L | ESV | 1.000 | 863.34 | 10.72 | 10.71 | 0.000 |
| office | L | RC_ESDF_like | 1.000 | 907.54 | 10.36 | 14.70 | 0.000 |
| office | L | SVSDF_like | 1.000 | 874.10 | 10.36 | 10.32 | 0.000 |
| office | T | ESV | 1.000 | 1572.48 | 10.93 | 11.22 | 0.000 |
| office | T | RC_ESDF_like | 1.000 | 1457.88 | 10.36 | 15.49 | 0.000 |
| office | T | SVSDF_like | 1.000 | 1347.25 | 10.36 | 10.47 | 0.000 |

## ESV vs Baselines

| scenario | robot | compare_to | success_delta | runtime_speedup_x | length_reduction_m |
|---|---|---|---:|---:|---:|
| maze | L | SVSDF_like | 0.000 | 1.03 | -0.30 |
| maze | L | RC_ESDF_like | 0.000 | 1.07 | 2.19 |
| maze | T | SVSDF_like | 0.000 | 0.94 | -0.14 |
| maze | T | RC_ESDF_like | 0.000 | 1.01 | 6.21 |
| office | L | SVSDF_like | 0.000 | 1.01 | -0.39 |
| office | L | RC_ESDF_like | 0.000 | 1.05 | 3.99 |
| office | T | SVSDF_like | 0.000 | 0.86 | -0.75 |
| office | T | RC_ESDF_like | 0.000 | 0.93 | 4.27 |
