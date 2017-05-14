The software is free to be used or modified for research work. For commercial purpose, please contact Jingwei Song (jingweisong@yahoo.com).


The purpose of the handeye calibration is to estimate the tranformation between Electromagnetic Sensor to Camera in following steps:

1. Creat checkboard. Run main.m in "createcheckerboard". Note that the height and length should not be the same.

2. Sample each corner of checkerboard corners from EM sensor.

3. Sample photos of checkerboard

3. Run "getGlobalPosCheckerboard.m" to filter. Note that the path to be set beforehand.

4. Run "Main.m" or "Main_improved.m" to estimate the transformation.


To test the system, just run "Main.m" or "Main_improved.m"