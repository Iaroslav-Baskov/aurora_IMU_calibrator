# aurora_IMU_calibrator
scripts for calibration of gyroscope and magnitometer
first it collects data from sensors, when temperature is changing (because of inside heating film). it is important for calibration of sensor temperature addiction. 
when i analyse this data with python script to get sensor offsets by temperature.
also it collects data from test with fixed temperature, but much random rotations of the probe to calculate offsets of sensors at one temperature.
when python script combines this data to get IMU offsets and temperature addiction
