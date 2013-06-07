/**
 * @file ActroidBase.h
 * @brief Actroid Controller Base Class
 * @copyright Ogata Laboratory 2013
 * @license GPL for commercial. LGPL for non-commercial.
 * @date 2013/06/06
 * @author Yuki Suga (ysuga@ysuga.net)
 */

#include "SerialPort.h"
#include "ActroidBase.h"

#ifdef WIN32
#define _USE_MATH_DEFINES
#endif
#include <math.h>

using namespace ogata_lab;
using namespace net::ysuga;


static const uint8_t _start = 0xfe;
static const uint8_t _stop  = 0x01;
static const uint8_t _get = 0x77;
static const uint8_t _set = 0x74;
static const uint8_t _set2 = 0x18;
static const uint8_t _online = 0x55;
static const uint8_t _offline = 0xdf;

static const uint8_t _ack = 0x06;
static const uint8_t _nack = 0x15;

static const uint8_t _num_joint = 24;

static const uint8_t online_command[] = {_start, _online, _stop};
static const uint8_t offline_command[] = {_start, _online, _stop};
static const uint8_t joint_read_command[] = {_start, _get, 0, _num_joint, _stop};

static const double _MaxAngle[NUM_JOINT] = {
  M_PI, M_PI, M_PI, M_PI, M_PI,
  M_PI, M_PI, M_PI, M_PI, M_PI,

  M_PI, M_PI, M_PI, M_PI, M_PI,
  M_PI, M_PI, M_PI, M_PI, M_PI,

  M_PI, M_PI, M_PI, M_PI,
};

static const double _MinAngle[NUM_JOINT] = {
  -M_PI, -M_PI, -M_PI, -M_PI, -M_PI,
  -M_PI, -M_PI, -M_PI, -M_PI, -M_PI,

  -M_PI, -M_PI, -M_PI, -M_PI, -M_PI,
  -M_PI, -M_PI, -M_PI, -M_PI, -M_PI,

  -M_PI, -M_PI, -M_PI, -M_PI
};

static const uint8_t _DefaultRawAngle[NUM_JOINT] = {
  DEFAULT_RAW_ANGLE, DEFAULT_RAW_ANGLE, DEFAULT_RAW_ANGLE, DEFAULT_RAW_ANGLE, DEFAULT_RAW_ANGLE,
  DEFAULT_RAW_ANGLE, DEFAULT_RAW_ANGLE, DEFAULT_RAW_ANGLE, DEFAULT_RAW_ANGLE, DEFAULT_RAW_ANGLE,

  DEFAULT_RAW_ANGLE, DEFAULT_RAW_ANGLE, DEFAULT_RAW_ANGLE, DEFAULT_RAW_ANGLE, DEFAULT_RAW_ANGLE,
  DEFAULT_RAW_ANGLE, DEFAULT_RAW_ANGLE, DEFAULT_RAW_ANGLE, DEFAULT_RAW_ANGLE, DEFAULT_RAW_ANGLE,

  DEFAULT_RAW_ANGLE, DEFAULT_RAW_ANGLE, DEFAULT_RAW_ANGLE, DEFAULT_RAW_ANGLE
};

ActroidBase::ActroidBase(const char* portName) throw(ActroidException)
{
  m_pSerialPort = new SerialPort(portName, BAUDRATE);
  _writePacket(online_command, 3);

  m_CurrentRawAngle[0] = 0;
  for (int i = 0;i < NUM_JOINT;i++) {
    m_CurrentRawAngle[i + 1] = _DefaultRawAngle[i];
    m_TargetRawAngle[i] = _DefaultRawAngle[i];
  }

}


ActroidBase::~ActroidBase() throw(ActroidException)
{
  _writePacket(offline_command, 3);
  delete m_pSerialPort;
}

void ActroidBase::_writePacket(const uint8_t* packet, const int len) throw(ActroidException)
{
  if (m_pSerialPort->write(packet, len) != len) {
    throw ActroidException("Packet Write Error");
  }

  uint8_t ack;
  while(m_pSerialPort->getSizeInRxBuffer() < 1) {
	  Sleep(1);
  }

  m_pSerialPort->read(&ack, 1);
  if (ack != _ack) {
    throw ActroidException("Nack received.");
  }
}

void ActroidBase::_readRawAngle() throw(ActroidException)
{
  _writePacket(joint_read_command, 5);
  while(m_pSerialPort->getSizeInRxBuffer() < NUM_JOINT+1) {
	  Sleep(10);
  }
  m_pSerialPort->read(m_CurrentRawAngle, NUM_JOINT+1);
  if(m_CurrentRawAngle[0] != 24) {
    throw ActroidException("Invalid Joint Angle Packet Received.");
  }
}

void ActroidBase::_writeRawAngle() throw(ActroidException)
{
  uint8_t command[NUM_JOINT + 5];
  command[0] = _start;
  command[1] = _set;
  command[2] = _set2;
  
  uint8_t sum = 24;
  for (int i = 0;i < NUM_JOINT;i++) {
    sum += getTargetRawAngle(i);
    command[3 + i] = getTargetRawAngle(i);
  }
  command[3 + NUM_JOINT] = ~sum + 1;
  command[4 + NUM_JOINT] = _stop;

  _writePacket(command, NUM_JOINT+5);
}

void ActroidBase::setTargetAngle(const int index, const double angle)
{
  m_TargetRawAngle[index] = (angle)/(_MaxAngle[index]-_MinAngle[index]) * 255.0 + _DefaultRawAngle[index];
}

double ActroidBase::getCurrentAngle(const int index)
{
  return (m_CurrentRawAngle[index]-_DefaultRawAngle[index])/255.0 
  * (_MaxAngle[index]-_MinAngle[index]) + _MinAngle[index];
}
