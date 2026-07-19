# Lab 3, Full Cloud Registration

Author: Giuseppe D'Auria, ID 2163508

Two stage point cloud registration on the bunny, dragon, and vase datasets. A descriptor
based global alignment with FPFH features gives a coarse initial pose, and an ICP refinement
then converges to the final alignment. The code implements four methods in
`src/Registration.cpp` and compares two custom ICP solvers against the three Open3D solvers
under a grid of injected noise.

## Dependencies

- Open3D 0.19 (prebuilt binary, built against libstdc++)
- Ceres Solver with glog
- Eigen 3
- CMake 3.20 or newer
- A C++ compiler for the standard set in `CMakeLists.txt` (C++23, the template default)

## Build

```
mkdir build && cd build
cmake ..
make
```

On the course platform Open3D is installed system wide, so `cmake ..` finds it. If Open3D lives elsewhere, point cmake at it with
`cmake -DOpen3D_DIR=/path/to/open3d/lib/cmake/Open3D ..`.

## Run

Single mode aligns one dataset and writes the registered clouds. The last argument selects
the ICP variant (`svd`, `lm`, `o3d-p2point`, `o3d-p2plane`, or `o3d-gen`):

```
cd build
./registration ../data/bunny/source.ply  ../data/bunny/target.ply  svd
./registration ../data/dragon/source.ply ../data/dragon/target.ply svd
./registration ../data/vase/source.ply   ../data/vase/target.ply   svd
```

Each run writes three merged clouds into `build/`: `merged_initial.ply`,
`merged_after_descriptor.ply`, and `merged_registered_cloud.ply`. The source is painted
orange and the target blue so the overlap is easy to read in MeshLab. In this archive these
three stages are saved per dataset under `results/<dataset>/` as `<dataset>_01_initial.ply`,
`<dataset>_02_after_descriptor.ply`, and `<dataset>_03_after_icp.ply`. The
`<dataset>_03_after_icp.ply` file is the registered cloud deliverable for that dataset.

The `all` mode runs the full 4 by 4 noise grid for the five ICP variants and prints the
benchmark table of rotation noise, translation noise, method, iterations, time, and final
RMSE:

```
./registration ../data/bunny/source.ply  ../data/bunny/target.ply  all
./registration ../data/dragon/source.ply ../data/dragon/target.ply all
./registration ../data/vase/source.ply   ../data/vase/target.ply   all
```

## Layout

```
src/Registration.h          class declaration (provided, unchanged)
src/Registration.cpp        the four implemented methods, on the corrected starter
registration_trial.cpp      driver, with the three stage saves and the threshold tuning
CMakeLists.txt              build configuration
report.pdf                  the report
data/<dataset>/             provided input clouds: source.ply and target.ply
results/<dataset>/          per dataset outputs, for bunny, dragon, and vase:
    <dataset>_03_after_icp.ply           the registered cloud deliverable
    <dataset>_02_after_descriptor.ply    after the FPFH plus RANSAC global alignment
    <dataset>_01_initial.ply             source and target before alignment
    <dataset>_0{1,2,3}_*.png             MeshLab screenshots of the three stages
    <dataset>_table.txt                  the noise sweep benchmark table
    <dataset>_transformation.txt         the final transformation matrix
```

## Notes

This submission builds on the corrected starter the course released, so
`execute_icp_registration` is the official fixed version. The only changes to
`Registration.cpp` are the four method bodies and the colours added in `save_merged_cloud`.
`CMakeLists.txt` keeps the template's C++23 standard.

I developed this on Ubuntu 22.04 and verified that it builds and runs on the course's
Xubuntu 24.04 platform. In the VM the Open3D bundled GLFW viewer renders poorly for lack of
GPU acceleration, so the `draw_registration_result` calls do little on screen. The program
still finishes and writes every `.ply`, and I took the screenshots by opening the saved
clouds in MeshLab.
