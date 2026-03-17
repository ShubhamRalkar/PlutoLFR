# PlutoLFR
Linefollower based on PrimusX2 board

In this program, we are implementing a simple line following robot using PlutoPilot. The robot uses an ADC sensor to detect the line and adjusts the motor speeds accordingly to follow the line. The PID controller is used to calculate the necessary adjustments to the motor speeds based on the error between the desired line position and the actual sensor reading. The motors are controlled using bidirectional commands, allowing for both forward and reverse motion.
Motor input are between 1000 and 2000, where 1000 means stop and 2000 is full speed in one direction, and 0 is full speed in the opposite direction. The error is calculated as the difference between the sensor reading and a target value (1925 in this case), which represents the desired line position. The PID controller then computes the necessary adjustments to the motor speeds to minimize this error, allowing the robot to follow the line effectively.

