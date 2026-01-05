Rocket Simulation (Python) - Used to test PID and understand how TVC worked

Working_pointing_code_TEST (Arduino IDE, C++) - used to test all features on the thrust vectoring portion, served as the testing ground before transfering to flight code. Includes output used
for calibration, and solutions to different problems that may arise, whether due to power loss or calibration.

IMU_Calibration (Arduino IDE, C++) - Used to calibrate IMU and find offsets

launch_code (Arduino IDE, C++) - Used on actual flight of rocket. Contains state machine and control code. 

project used a Teensy 4.1 for flight computer and SparkFunLSM6DS3 for IMU. 3 SG90 servos used.
