// Do not remove the include below
#include "PlutoPilot.h"
#include "API/API-Utils.h"

/**
 * Configures Pluto's receiver to use PPM or default ESP mode; activate the line matching your setup.
 * AUX channel configurations is only for PPM recievers if no custom configureMode function is called this are the default setup
 * ARM mode : Rx_AUX2, range 1300 to 2100
 * ANGLE mode : Rx_AUX2, range 900 to 2100
 * BARO mode : Rx_AUX3, range 1300 to 2100
 * MAG mode : Rx_AUX1, range 900 to 1300
 * HEADFREE mode : Rx_AUX1, range 1300 to 1700
 * DEV mode : Rx_AUX4, range 1500 to 2100
 */

#define M1_FWD ANTICLOCK_WISE
#define M1_REV CLOCK_WISE
#define M2_FWD CLOCK_WISE
#define M2_REV ANTICLOCK_WISE

#define midVal 2000

#define BASE_cmd  1250
#define MAX_cmd 1900

#define KP 0.38 
#define KI 0
#define KD 0.16  

float lastError = 0;
float integral  = 0;

void lfr();

void plutoRxConfig ( void ) {
  // Receiver mode: Uncomment one line for ESP or CAM or PPM setup.
  Receiver_Mode ( Rx_ESP );    // Onboard ESP
  // Receiver_Mode ( Rx_CAM );    // WiFi CAMERA
  // Receiver_Mode ( Rx_PPM );    // PPM based
}

// The setup function is called once at Pluto's hardware startup
void plutoInit() {
  //setUserLoopFrequency(1);
  Peripheral_Init(ADC_1);
  Motor_Init(M1);
  Motor_Init(M2);
}

// The function is called once before plutoLoop when you activate Developer Mode
void onLoopStart ( void ) {
  // do your one time stuffs here
}

// The loop function is called in an endless loop
void plutoLoop () {
  //Monitor_Println("Sensor_Value: " , Peripheral_Read(ADC_1));
  lfr();
}

// The function is called once after plutoLoop when you deactivate Developer Mode
void onLoopFinish ( void ) {
  // do your cleanup stuffs here
}

void setMotor(bidirectional_motor_e motor, uint16_t cmd ){
  cmd = constrain(cmd, 0, MAX_cmd);
  if (motor == M1){
    if ( cmd >= 1000 ) {
    Motor_SetDir ( M1, M1_FWD );
    Motor_Set ( M1, cmd );
  } else {
    Motor_SetDir ( M1, M1_REV );
    Motor_Set ( M1, (2000 - cmd) );
  }
  } else if(motor == M2){
    if ( cmd >= 1000 ) {
    Motor_SetDir ( M2, M2_FWD );
    Motor_Set ( M2, cmd );
  } else {
    Motor_SetDir ( M2, M2_REV );
    Motor_Set ( M2, (2000 - cmd));
  }
  }
}

void lfr(){
  int16_t error = - (Peripheral_Read(ADC_1) - 1925);
  Monitor_Println("Error Value: ", error);
  
  if(error >= 1700 || error <= -1700){
    setMotor(M1, 1000); //motor stop if error is too high, to avoid damage to the motors
    setMotor(M2, 1000); //motor stop if error is too high, to avoid damage to the motors
  }
  else{
    //Monitor_Println("Error Value: ", error);
  integral += error;
  integral = constrain (integral, -100, 100);

  float derivative = error - lastError;
  float pid = (KP * error) + (KI * integral) + (KD * derivative);

  lastError = error;

  int leftSpeed  = BASE_cmd - pid;
  int rightSpeed = BASE_cmd + pid;

  setMotor(M1, leftSpeed);
  setMotor(M2, rightSpeed);
  }


}
/*
In this program, we are implementing a simple line following robot using PlutoPilot. The robot uses an ADC sensor to detect the line and adjusts the motor speeds accordingly to follow the line. The PID controller is used to calculate the necessary adjustments to the motor speeds based on the error between the desired line position and the actual sensor reading. The motors are controlled using bidirectional commands, allowing for both forward and reverse motion.
Motor input are between 1000 and 2000, where 1000 means stop and 2000 is full speed in one direction, and 0 is full speed in the opposite direction. The error is calculated as the difference between the sensor reading and a target value (1925 in this case), which represents the desired line position. The PID controller then computes the necessary adjustments to the motor speeds to minimize this error, allowing the robot to follow the line effectively.
*/
