# 3D Data Processing Labs

Lab work for the 3D Data Processing course, University of Padova. Four labs spanning the classical and learned 3D computer vision pipeline: stereo depth estimation, structure from motion, point cloud registration, and deep learning on sparse LiDAR data.

![C++](https://img.shields.io/badge/C%2B%2B-23-00599C?logo=cplusplus&logoColor=white)
![Python](https://img.shields.io/badge/Python-3-3776AB?logo=python&logoColor=white)
![OpenCV](https://img.shields.io/badge/OpenCV-used-5C3EE8?logo=opencv&logoColor=white)
![Open3D](https://img.shields.io/badge/Open3D-used-000000)
![Ceres Solver](https://img.shields.io/badge/Ceres%20Solver-used-orange)
![PyTorch](https://img.shields.io/badge/PyTorch-used-EE4C2C?logo=pytorch&logoColor=white)

## Labs

| # | Lab | Topic | Stack | Grade | Status |
|---|---|---|---|---|---|
| 1 | [SGM Stereo + Monocular Refinement](lab1-sgm-stereo/) | Dense stereo depth via Semi-Global Matching | C++ | Assessed | Placeholder |
| 2 | [Structure from Motion](lab2-structure-from-motion/) | Incremental SfM: matching, RANSAC pose estimation, triangulation, bundle adjustment | C++, OpenCV, Ceres | Assessed | Populated |
| 3 | [Point Cloud Registration](lab3-point-cloud-registration/) | Global and local registration of partial 3D scans | C++, Open3D, Ceres, Eigen | 27/30 | Placeholder |
| 4 | [Deep 3D Descriptors: MinkUNet Segmentation](lab4-minkunet-segmentation/) | Sparse convolutional semantic segmentation on LiDAR | Python, PyTorch, MinkowskiEngine | 30/30 | Placeholder |

Labs marked "Placeholder" have a README describing planned content but source and results are pending upload to this repository.

## Structure from Motion, in detail

Lab 2 is fully populated and demonstrates the classical geometric pipeline end to end: feature correspondence, RANSAC-based pose estimation, triangulation, and Ceres-based bundle adjustment. See [lab2-structure-from-motion/README.md](lab2-structure-from-motion/README.md) for the reconstructed point clouds and implementation notes.

![SfM pipeline](lab2-structure-from-motion/assets/sfm_pipeline.svg)

## Repository layout

```
3D-Data-Processing-Labs/
├── lab1-sgm-stereo/
├── lab2-structure-from-motion/
├── lab3-point-cloud-registration/
├── lab4-minkunet-segmentation/
└── README.md
```

Each lab folder is self-contained: `src/` for implementation, `report/` for the written report, `results/` for raw output data, `assets/` for figures used in that lab's README.

## Course context

3D Data Processing, University of Padova. Course material spans sensing (structured light, LiDAR, stereo rigs), rigid body transformations, epipolar geometry, stereo matching (classical and deep), structure from motion, visual SLAM, 3D data representations, local descriptors, shape registration, and neural implicit representations.
