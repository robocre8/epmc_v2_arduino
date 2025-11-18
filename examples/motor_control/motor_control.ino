/*
 * Basic example code on how to control via I2C your geared DC motor with quadrature
 * encoder which is already connected to the Easy PID Motor Controller module and have already
 * succesfully set up their velocity PID control using the epmc_setup_application
 *
 * The code basically sends a low target velocity (in rad/s), waits for some time and then
 * sends a high target velocity (in rad/s). it also prints out the motors' angular positions
 * (in rad) and angular velocities (in rad/s).
 *
 * you can copy the code and use it in your project as you will.
 */

// Easy PID Motor Control i2c communication library
#include <epmcV2.h>

int i2c_address = 0x55; // set this address to the same address you have during setup via the GUI app
EPMC_V2 epmcV2(i2c_address);

///////// my sepcial delay function ///////////////
void delayMs(int ms)
{
  for (int i = 0; i < ms; i += 1)
  {
    delayMicroseconds(1000);
  }
}
//////////////////////////////////////////////////

float pos0, pos1, pos2, pos3; // (in rad)
float v0, v1, v2, v3; // (in rad/sec)

float lowTargetVel = 0.00;  // rad/sec
float highTargetVel = 5.00; // rad/sec
bool sendHigh = true;

long prevTime;
long sampleTime = 15; // millisec

long ctrlPrevTime;
long ctrlSampleTime = 2500; // millisec

void setup()
{
  // start i2c communication
  Wire.begin();

  // setup serial communication to print result on serial minitor
  Serial.begin(115200);

  delay(3000);

  epmcV2.clearDataBuffer();
  epmcV2.writeSpeed(0.0, 0.0, 0.0, 0.0);

  int cmd_vel_timeout = 10000; // 0 to deactivate.
  epmcV2.setCmdTimeout(cmd_vel_timeout); // set motor command velocity timeout
  cmd_vel_timeout = epmcV2.getCmdTimeout(); // get the stored command velocity timeout
  Serial.print("motor command vel timeout in ms: ");
  Serial.println(cmd_vel_timeout);

  sendHigh = true;

  prevTime = millis();
  ctrlPrevTime = millis();
}

void loop()
{
  if ((millis() - ctrlPrevTime) >= ctrlSampleTime)
  {
    if (sendHigh)
    {
      epmcV2.writeSpeed(highTargetVel, highTargetVel, highTargetVel, highTargetVel);
      highTargetVel *= -1;
      sendHigh = false;
    }
    else
    {
      epmcV2.writeSpeed(lowTargetVel, lowTargetVel, lowTargetVel, lowTargetVel);
      sendHigh = true;
    }
    ctrlPrevTime = millis();
  }

  if ((millis() - prevTime) >= sampleTime)
  {
    /* CODE SHOULD GO IN HERE*/
    epmcV2.readMotorData(pos0, pos1, pos2, pos3, v0, v1, v2, v3);

    // Print results
    Serial.println("-----------------------------------");
    Serial.print("Motor 0: ");
    Serial.print(pos0, 2); Serial.print("\t"); Serial.println(v0, 4);
    Serial.print("Motor 1: ");
    Serial.print(pos1, 2); Serial.print("\t"); Serial.println(v1, 4);
    Serial.print("Motor 2: ");
    Serial.print(pos2, 2); Serial.print("\t"); Serial.println(v2, 4);
    Serial.print("Motor 3: ");
    Serial.print(pos3, 2); Serial.print("\t"); Serial.println(v3, 4);
    Serial.println("------------------------------------");
    
    // Serial.println();

    prevTime = millis();
  }
}