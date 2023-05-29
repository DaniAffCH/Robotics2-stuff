# Robotics2-stuff

This repository provides a set of matlab functions useful for robot dynamics computations. There is also an external module that refers to [r2_toolbox](https://github.com/EmanueleGiacomini/r2_toolbox) covering most of the topics disregarded in this repository. 

The functions have been developed for the course Robotics2@Sapienza, however they have been designed to be as generic as possible. 

# Functions available

| Script | Tested | Description | Author |
| ------------- | ------------- | ------------- | ------------- |
| calibrationStep.m  | :heavy_check_mark: | Single step of calibration algorithm | [@DaniAffCH](https://www.github.com/DaniAffCH) |
| wpinv.m  | :heavy_check_mark: | Weighted pseudoinverse of Jacobian | [@DaniAffCH](https://www.github.com/DaniAffCH) |
| MovingFrameGeneric.m  | :heavy_check_mark: | Computes the Moving Frames coefficients (one step) | [@DaniAffCH](https://www.github.com/DaniAffCH) |
| MovingFrameDH.m | :heavy_check_mark: | Computes the Moving Frames coefficients from the DH table (one step) | [@Pab99lis](https://github.com/Pab99lis)|
| MovingFrameDHLive.mlx | :heavy_check_mark: | LiveScript to call MovingFrameDH.m and visualize computations | [@Pab99lis](https://github.com/Pab99lis)|
| PlanarNR_COMinZero.m  | :heavy_check_mark: | Computes the Center Of Mass position in frame 0. It works only for planar-fully revolute robots. It has a dependency from the submodule | [@DaniAffCH](https://www.github.com/DaniAffCH) |
| DHMatrix.m  | :heavy_check_mark: | Compute homogeneous transf. from DH table parameters | [@luigi-ga](https://www.github.com/luigi-ga) |
| factorization.m  | :heavy_check_mark: | Compute the Christoffel factorization $c(q, \dot{q}) = S\dot{q}$. It provides both an $S_1$ such that $\dot{M} - 2S_1$ is skewsymmetric and $S_2$ that doesn't meet such condition| [@francomano](https://www.github.com/francomano) & [@Pab99lis](https://github.com/Pab99lis) |
| taskPriority.m | :heavy_check_mark: | Compute Task Priority algorithm up to 2 tasks | [@francomano](https://www.github.com/francomano) & [@DavideEspositoPelella](https://www.github.com/DavideEspositoPelella) |
| n_task_priority.m | :heavy_check_mark: | Compute Task Priority algorithm up to N tasks | [@ValerioSpagnoli](https://github.com/ValerioSpagnoli)|
| coordinateChange.m | :heavy_check_mark: | Dynamic model after linear transformation of generalized coordinates | [@Pab99lis](https://github.com/Pab99lis)|
| external dynamic submodule | :heavy_check_mark: | It isn't fully tested yet but most of the scripts are fine so far | [@EmanueleGiacomini](https://www.github.com/EmanueleGiacomini) |


### Installing

Pay attention to the `--recursive` flag. It is required to clone also the submodules.
```bash
git clone --recursive https://github.com/DaniAffCH/Robotics2-stuff.git
```

### Known issues
Sometimes the trigonometric functions returns weird results if the argument contains the `pi`. This happens because of the float (or double) precision. In order to fix that use
`sym(pi)` instead of `pi`.

E.g.
```
>> cos(pi/2)

ans =

   6.1232e-17
```

```
>> cos(sym(pi)/2)
 
ans =
 
  0
```
