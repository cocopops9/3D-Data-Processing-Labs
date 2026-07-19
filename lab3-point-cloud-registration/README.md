# Lab 3: Full Point Cloud Registration

Global and local registration of partial point cloud scans (bunny, dragon, vase) using feature-based initial alignment followed by ICP refinement, with Ceres-based pose optimization. Implemented in C++ with Open3D and Eigen.

Course: 3D Data Processing, University of Padova.
Graded 27/30 (remaining loss cosmetic, per official assessment).

## Status

Placeholder. Source code, report, and result figures for this lab have not yet been added to this repository.

## Planned content

- `src/`: registration pipeline (feature-based initial alignment, ICP refinement).
- `report/`: lab report.
- `results/`: registered point clouds for the bunny, dragon, and vase sequences.
- `assets/`: rendered before/after registration comparisons and pipeline diagram for this README.

## Known implementation notes

Two latent bugs in the official starter code were identified and fixed independently: an incorrect accumulator composition order, and a keyword argument binding issue in `ICPConvergenceCriteria`. Both were later confirmed by the official TA correction.
