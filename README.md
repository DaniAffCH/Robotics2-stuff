# Robotics2-stuff

Facciamo un check di quello che siamo sicuri al 100% che funziona e la roba da testare.

| Script | Tested | Description |
| ------------- | ------------- | ------------- |
| calibrationStep.m  | :heavy_multiplication_x: | Single step of calibration algorithm |
| wpinv.m  | :heavy_check_mark: | Weighted pseudoinverse of Jacobian |
| movingFrames2R.m  | :heavy_multiplication_x: | Apply moving frames (Dynamic model estimation) to 2R robot |
| movingFrames3R.m  | :heavy_multiplication_x: | Apply moving frames (Dynamic model estimation) to 3R robot |
| DHMatrix.m  | :heavy_check_mark: | Compute homogeneous transf. from DH table parameters |


### Roba che potrebbe essere utile implementare

- Fattorizzazione da $c(q, \dot{q}) = S\dot{q}$
- Task priority algorithm
