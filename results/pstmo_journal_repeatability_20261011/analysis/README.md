# PSTMO journal repeatability analysis

- Trials: 250 (122 Constrained successes; 121 PSTMO successes).
- Paired Raw hashes: 125/125.
- Independent statistical units: 25 route-planner blocks.
- Audit issues: 0.

| Metric | Constrained mean | PSTMO mean | Difference | 95% CI | Holm p |
| --- | ---: | ---: | ---: | ---: | ---: |
| Execution time (s) | 47.5602 | 44.6539 | -2.85727 | [-3.71649, -2.03914] | 1.19209e-06 |
| Tracking RMSE (m) | 0.0256832 | 0.0227861 | -0.00298481 | [-0.00448747, -0.00150584] | 0.000489473 |
| Executed curvature energy (1/m) | 3.27441 | 2.22057 | -1.03773 | [-1.33167, -0.76741] | 5.36442e-07 |

Differences are PSTMO minus Constrained. Repetitions quantify fixed-input 
execution repeatability; they are not independent random-seed or noise trials.
