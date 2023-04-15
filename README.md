# Robotics2-stuff

| Script | Tested | Description | Author |
| ------------- | ------------- | ------------- | ------------- |
| calibrationStep.m  | :heavy_check_mark: | Single step of calibration algorithm | [@DaniAffCH](https://www.github.com/DaniAffCH) |
| wpinv.m  | :heavy_check_mark: | Weighted pseudoinverse of Jacobian | [@DaniAffCH](https://www.github.com/DaniAffCH) |
| MovingFrameGeneric.m  | :heavy_check_mark: | Computes the Moving Frames coefficients (one step) | [@DaniAffCH](https://www.github.com/DaniAffCH) |
| PlanarNR_COMinZero.m.m  | :heavy_check_mark: | Computes the Center Of Mass position in frame 0. It works only for planar-fully revolute robots. It has a dependency with the submodule | [@DaniAffCH](https://www.github.com/DaniAffCH) |
| movingFrames2R.m  | :heavy_multiplication_x: | Apply moving frames (Dynamic model estimation) to 2R robot | [@luigi-ga](https://www.github.com/luigi-ga) |
| movingFrames3R.m  | :heavy_multiplication_x: | Apply moving frames (Dynamic model estimation) to 3R robot | [@luigi-ga](https://www.github.com/luigi-ga) |
| DHMatrix.m  | :heavy_check_mark: | Compute homogeneous transf. from DH table parameters | [@luigi-ga](https://www.github.com/luigi-ga) |
| factorization.m  | :heavy_check_mark: | Compute the Christoffel factorization $c(q, \dot{q}) = S\dot{q}$| [@francomano](https://www.github.com/francomano) |
| taskPriority.m | :heavy_check_mark: | Compute Task Priority algorithm up to 2 tasks | [@francomano](https://www.github.com/francomano) & [@DavideEspositoPelella](https://www.github.com/DavideEspositoPelella) |
| n_task_priority.m | :heavy_check_mark: | Compute Task Priority algorithm up to N tasks | [@ValerioSpagnoli](https://github.com/ValerioSpagnoli)|
| external dynamic submodule | :heavy_check_mark: | It isn't fully tested yet but most of the scripts are fine so far | [@EmanueleGiacomini](https://www.github.com/EmanueleGiacomini) |


### Installing

Pay attention to the `--recursive` flag. It is required to clone also the submodules.
```bash
git clone --recursive https://github.com/DaniAffCH/Robotics2-stuff.git
```
