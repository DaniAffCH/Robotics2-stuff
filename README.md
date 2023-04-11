# Robotics2-stuff

| Script | Tested | Description |
| ------------- | ------------- | ------------- |
| calibrationStep.m  | :heavy_check_mark: | Single step of calibration algorithm |
| wpinv.m  | :heavy_check_mark: | Weighted pseudoinverse of Jacobian |
| movingFrames2R.m  | :heavy_multiplication_x: | Apply moving frames (Dynamic model estimation) to 2R robot |
| movingFrames3R.m  | :heavy_multiplication_x: | Apply moving frames (Dynamic model estimation) to 3R robot |
| DHMatrix.m  | :heavy_check_mark: | Compute homogeneous transf. from DH table parameters |
| factorization.m  | :heavy_check_mark: | Compute the Christoffel factorization $c(q, \dot{q}) = S\dot{q}$|
| taskPriority.m | :heavy_check_mark: | Compute Task Priority algorithm up to 2 tasks |
| external dynamic submodule | :heavy_check_mark: | It isn't fully tested yet but most of the scripts are fine |


### Installing

Pay attention to the `--recursive` flag. It is required to clone also the submodules.
```bash
git clone --recursive https://github.com/DaniAffCH/Robotics2-stuff.git
```
