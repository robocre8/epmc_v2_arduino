#include "epmcV2.h"

EPMC_V2::EPMC_V2(int slave_addr)
{
  slaveAddr = slave_addr;
}

bool EPMC_V2::writeSpeed(int motor_no, float speed)
{
  return send("/vel", motor_no, speed);
}

bool EPMC_V2::writePWM(int motor_no, int pwm)
{
  return send("/pwm", motor_no, pwm);
}

float EPMC_V2::readPos(int motor_no)
{
  float angPos = get("/pos", motor_no);
  return angPos;
}

float EPMC_V2::readVel(int motor_no)
{
  float filteredAngVel = get("/vel", motor_no);
  return filteredAngVel;
}

float EPMC_V2::readUVel(int motor_no)
{
  float unfilteredAngVel = get("/u-vel", motor_no);
  return unfilteredAngVel;
}

bool EPMC_V2::setCmdTimeout(int timeout_ms = 0)
{
  return send("/timeout", -1, timeout_ms);
}

int EPMC_V2::getCmdTimeout()
{
  float timeout_ms = get("/timeout", -1);
  return (int)timeout_ms;
}

bool EPMC_V2::setPidMode(int motor_no, int mode)
{
  return send("/mode", motor_no, mode);
}

int EPMC_V2::getPidMode(int motor_no)
{
  float mode = get("/mode", motor_no);
  return (int)mode;
}

float EPMC_V2::get(String cmd_route, int motor_no)
{
  String msg_buffer = cmd_route;
  msg_buffer += ",";
  msg_buffer += String(motor_no);

  masterSendData(msg_buffer);
  masterReceiveData();
  String dataMsg = masterReceiveData();

  float val = dataMsg.toFloat();
  return val;
}

bool EPMC_V2::send(String cmd_route, int motor_no, float val)
{
  String msg_buffer = cmd_route;
  msg_buffer += ",";
  msg_buffer += String(motor_no);
  msg_buffer += ",";
  msg_buffer += String(val, 3);

  masterSendData(msg_buffer);
  masterReceiveData();
  String data = masterReceiveData();
  if (data == "1")
    return true;
  else
    return false;
}

void EPMC_V2::masterSendData(String i2c_msg)
{
  Wire.beginTransmission(slaveAddr);
  Wire.print(i2c_msg);
  Wire.endTransmission(true);
}

String EPMC_V2::masterReceiveData()
{
  String i2c_msg = "";
  Wire.flush();
  uint8_t dataSizeInBytes = Wire.requestFrom(slaveAddr, 15);
  for (int i = 0; i < dataSizeInBytes; i += 1)
  {
    char c = Wire.read();
    i2c_msg += c;
  }
  int indexPos = i2c_msg.indexOf((char)255);
  if (indexPos != -1)
  {
    return i2c_msg.substring(0, indexPos);
  }
  i2c_msg.trim();
  return i2c_msg;
}