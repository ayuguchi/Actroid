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
#include <iostream>

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


#define RADIANS(x) ((x)/180.0*M_PI)

/*
static const double _MaxAngle[NUM_JOINT] = {
  RADIANS(255), RADIANS(255), RADIANS(255), RADIANS(255), RADIANS(255), RADIANS(255), RADIANS(255), RADIANS(255),
  RADIANS(120.0), RADIANS(58), RADIANS(45), RADIANS(112.312+14.497), RADIANS(90.0), RADIANS(25.833), RADIANS(28.423),
  RADIANS(120.0), RADIANS(58), RADIANS(45), RADIANS(112.312+14.497), RADIANS(90.0), RADIANS(25.833), RADIANS(28.423),
  RADIANS(255), RADIANS(255)
};
*/

static const double _MaxAngle[NUM_JOINT] = {
  RADIANS(255), RADIANS(255), RADIANS(255), RADIANS(255), RADIANS(255), RADIANS(255), RADIANS(255), RADIANS(255),
  RADIANS(120.0), RADIANS(58), RADIANS(45), RADIANS(140), RADIANS(90.0), RADIANS(25.833), RADIANS(28.423),
  RADIANS(120.0), RADIANS(58), RADIANS(45), RADIANS(140), RADIANS(90.0), RADIANS(25.833), RADIANS(28.423),
  RADIANS(255), RADIANS(255)
};

/*
static const double _MinAngle[NUM_JOINT] = {
  RADIANS(0), RADIANS(0), RADIANS(0), RADIANS(0), RADIANS(0), RADIANS(0), RADIANS(0), RADIANS(0),
  RADIANS(-14.0), RADIANS(-15.0), RADIANS(-45.0), RADIANS(14.497), RADIANS(-65.0), RADIANS(-15.458), RADIANS(-39.876),
  RADIANS(-14.0), RADIANS(-15.0), RADIANS(-45.0), RADIANS(14.497), RADIANS(-65.0), RADIANS(-15.458), RADIANS(-39.876),
  RADIANS(0), RADIANS(0)
};
*/


static const double _MinAngle[NUM_JOINT] = {
  RADIANS(0), RADIANS(0), RADIANS(0), RADIANS(0), RADIANS(0), RADIANS(0), RADIANS(0), RADIANS(0),
  RADIANS(-18.0), RADIANS(-15.0), RADIANS(-45.0), RADIANS(20), RADIANS(-65.0), RADIANS(-15.458), RADIANS(-39.876),
  RADIANS(-18.0), RADIANS(-15.0), RADIANS(-45.0), RADIANS(20), RADIANS(-65.0), RADIANS(-15.458), RADIANS(-39.876),
  RADIANS(0), RADIANS(0)
};

static const double _DefaultAngle[NUM_JOINT] = {
  RADIANS(128), RADIANS(128), RADIANS(128), RADIANS(128), 
  RADIANS(0), RADIANS(128), RADIANS(128), RADIANS(128),
  RADIANS(0), RADIANS(0), RADIANS(0), RADIANS(90),  RADIANS(0), RADIANS(0), RADIANS(0),
  RADIANS(0), RADIANS(0), RADIANS(0), RADIANS(90),  RADIANS(0), RADIANS(0), RADIANS(0),
  RADIANS(0), RADIANS(128)
};


static const uint8_t _DefaultRawAngle[NUM_JOINT] = {
  DEFAULT_RAW_ANGLE, DEFAULT_RAW_ANGLE, DEFAULT_RAW_ANGLE, DEFAULT_RAW_ANGLE, DEFAULT_RAW_ANGLE,
  DEFAULT_RAW_ANGLE, DEFAULT_RAW_ANGLE, DEFAULT_RAW_ANGLE, 143, 86,

  128, 218, 128, 128, 128,
  113, 210, 0, 0, 0,

  DEFAULT_RAW_ANGLE, DEFAULT_RAW_ANGLE, DEFAULT_RAW_ANGLE, DEFAULT_RAW_ANGLE
};

static const double _AngleMargin[NUM_JOINT] = {
	0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 
	0.040, 0.040, 0.001, 0.1, 0.001, 0.001, 0.001,
	0.001, 0.001, 0.001, 0.05, 0.001, 0.001, 0.001,
	0.001, 0.001

};

ActroidBase::ActroidBase(const char* portName) throw(ActroidException)
{
  m_pSerialPort = new SerialPort(portName, BAUDRATE);
  _writePacket(online_command, 3);

  m_CurrentRawAngle[0] = 0;
  for (int i = 0;i < NUM_JOINT;i++) {
    m_CurrentRawAngle[i+1] = _DefaultRawAngle[i];
    //m_TargetRawAngle[i] = _DefaultRawAngle[i];
	setTargetAngle(i, _DefaultAngle[i]);
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

void ActroidBase::setTargetAngle(const int index, double angle)
{
  //m_TargetRawAngle[index] = (angle)/(_MaxAngle[index]-_MinAngle[index]) * 255.0 + _DefaultRawAngle[index];
	if(angle >= (_MaxAngle[index]-_AngleMargin[index])) {
		 angle = _MaxAngle[index] - _AngleMargin[index];
	} else if(angle <= (_MinAngle[index] + _AngleMargin[index])) {
		 angle = _MinAngle[index] + _AngleMargin[index];
	}

  m_TargetRawAngle[index] = (angle) * 255.0/(_MaxAngle[index]-_MinAngle[index]) - (_MinAngle[index] * 255.0 / (_MaxAngle[index]-_MinAngle[index]));
//  if (index == 15)
//   {
//    std::cout << "TargetRawAngle is  " << static_cast<int>(m_TargetRawAngle[15]) << std::endl;
//	  std::cout << "_MaxAngle[15] is  " << static_cast<int>(_MaxAngle[15]) << std::endl;
//	  std::cout << "_MinAngle[15] is  " << static_cast<int>(_MinAngle[15]) << std::endl;
//	  std::cout << "_DefaultRawAngle[15] is  " << static_cast<int>(_DefaultRawAngle[15]) << std::endl;
//	  std::cout << "angle is  " << angle << std::endl;
//   }
}

double ActroidBase::getCurrentAngle(const int index)
{
  //return (m_CurrentRawAngle[index]-_DefaultRawAngle[index])/255.0 * (_MaxAngle[index]-_MinAngle[index]) + _MinAngle[index];
//  if (index == 15)
//   {
//    std::cout << "m_CurrentRawAngle[15] is  " << static_cast<int>(m_CurrentRawAngle[15]) << std::endl;
// 	std::cout << "_MaxAngle[15] is  " << static_cast<int>(_MaxAngle[15]) << std::endl;
//	std::cout << "_MinAngle[15] is  " << static_cast<int>(_MinAngle[15]) << std::endl;
//   }
  return (m_CurrentRawAngle[index+1] * (_MaxAngle[index]-_MinAngle[index]))/255.0 + _MinAngle[index];

}
