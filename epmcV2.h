#ifndef EPMC_V2_H
#define EPMC_V2_H

#include <Arduino.h>
#include <Wire.h>

class EPMC_V2
{
public:
  EPMC_V2(int);

  int writeSpeed(float v0, float v1, float v2, float v3);
  int writePWM(int pwm0, int pwm1, int pwm2, int pwm3);
  void readPos(float &pos0, float &pos1, float &pos2, float &pos3);
  void readVel(float &v0, float &v1, float &v2, float &v3);
  void readUVel(float &v0, float &v1, float &v2, float &v3);
  int setCmdTimeout(int timeout_ms);
  int getCmdTimeout();
  int setPidMode(int motor_no, int mode);
  int getPidMode(int motor_no);
  void readMotorData(float &pos0, float &pos1, float &pos2, float &pos3, float &v0, float &v1, float &v2, float &v3);


private:
  int slaveAddr;
  void send_packet_without_payload(uint8_t cmd);
  void write_data1(uint8_t cmd, uint8_t pos, float val);
  void write_data3(uint8_t cmd, float val0, float val1, float val2);
  void write_data4(uint8_t cmd, float val0, float val1, float val2, float val3);
  float read_data1();
  void read_data3(float &val0, float &val1, float &val2);
  void read_data4(float &val0, float &val1, float &val2, float &val3);
  void read_data8(float &val0, float &val1, float &val2, float &val3, float &val4, float &val5, float &val6, float &val7);

  //  Protocol Command IDs -------------
  const uint8_t START_BYTE = 0xAA;
  const uint8_t WRITE_VEL = 0x01;
  const uint8_t WRITE_PWM = 0x02;
  const uint8_t READ_POS = 0x03;
  const uint8_t READ_VEL = 0x04;
  const uint8_t READ_UVEL = 0x05;
  const uint8_t SET_PID_MODE = 0x15;
  const uint8_t GET_PID_MODE = 0x16;
  const uint8_t SET_CMD_TIMEOUT = 0x17;
  const uint8_t GET_CMD_TIMEOUT = 0x18;
  const uint8_t READ_MOTOR_DATA = 0x2A;
  //---------------------------------------------
};

#endif