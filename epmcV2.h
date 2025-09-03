#ifndef EPMC_V2_H
#define EPMC_V2_H

#include <Arduino.h>
#include <Wire.h>

class EPMC_V2
{
public:
  EPMC_V2(int);

  bool writeSpeed(int, float);

  bool writePWM(int, int);

  float readPos(int);

  float readVel(int);

  float readUVel(int);

  bool setCmdTimeout(int);

  int getCmdTimeout();

  bool setPidMode(int, int);

  int getPidMode(int);


private:
  int slaveAddr;

  float get(String, int);

  bool send(String, int, float);

  void masterSendData(String);

  String masterReceiveData();
};

#endif