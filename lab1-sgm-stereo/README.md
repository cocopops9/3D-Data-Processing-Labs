# Lab 1: Semi-Global Matching Stereo and Monocular Depth Refinement

Dense disparity estimation via Semi-Global Matching (SGM), combined with a monocular refinement stage. Implemented in C++.

Course: 3D Data Processing, University of Padova.

## Status

Placeholder. Source code, report, and result figures for this lab have not yet been added to this repository.

## Planned content

- `src/`: SGM cost aggregation, disparity refinement, C++ implementation.
- `report/`: lab report.
- `results/`: disparity maps for the Aloe, Cones, Plastic, and Rocks1 datasets.
- `assets/`: rendered disparity map comparisons and pipeline diagram for this README.

## Known implementation notes

Filtering zero-valued pixels before scale estimation improved results consistently across datasets, a preprocessing detail worth carrying into any subsequent depth fusion pipeline.
