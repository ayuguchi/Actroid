/**
 * @file ActroidBase.h
 * @brief Actroid Controller Base Class
 * @copyright Ogata Laboratory 2013
 * @license GPL for commercial. LGPL for non-commercial.
 * @date 2013/06/06
 * @author Yuki Suga (ysuga@ysuga.net)
 */


#pragma once

#include <stdint.h>
#include <string>
#include <exception>

namespace net {
  namespace ysuga { 
    class SerialPort;
  };
};

namespace ogata_lab {
  class ActroidException : public std::exception {
  private:
    std::string msg;
  public:
    ActroidException(const char* msg) {
      this->msg = msg;
    }

    ~ActroidException() throw() {
    }

    const char* what() const throw() { 
      return this->msg.c_str();
    }

  };
  
#define BAUDRATE 115200
#define NUM_JOINT 24
#define DEFAULT_RAW_ANGLE (255/2)

  /**
  [CH1]眉上下,173,128,0,255
    [CH2]瞼開閉,0,0,0,255
    [CH3]眼左右,181,128,0,255
    [CH4]眼上下,128,128,0,255
    [CH5]口開閉,0,0,0,255
    [CH6]首左伸,128,128,0,255
    [CH7]首右伸,128,128,0,255
    [CH8]首旋回,128,128,0,255
    [CH9]左腕上,128,128,0,255
    [CH10]左腕開,127,128,0,255
    [CH11]左上腕,128,128,0,255
    [CH12]左肘,128,128,0,255
    [CH13]左前腕,128,128,0,255
    [CH14]左手縦,128,128,0,255
    [CH15]左手横,128,128,0,255
    [CH16]右腕上,128,128,0,255
    [CH17]右腕開,128,128,0,255
    [CH18]右上腕,128,128,0,255
    [CH19]右肘,128,128,0,255
    [CH20]右前腕,128,128,0,255
    [CH21]右手縦,128,128,0,255
    [CH22]右手横,128,128,0,255
    [CH23]胴前後,0,0,0,255
    [CH24]胴旋回,128,128,0,255
  **/

  class ActroidBase {
  private:
    net::ysuga::SerialPort* m_pSerialPort;
    uint8_t m_CurrentRawAngle[NUM_JOINT+1];
    uint8_t m_TargetRawAngle[NUM_JOINT];
  private:
    void _writePacket(const uint8_t* packet, const int len) throw(ActroidException);
    void _readRawAngle() throw(ActroidException);
    void _writeRawAngle() throw(ActroidException);

  public:
    /**
     *
     */
    ActroidBase(const char* portName) throw(ActroidException);

    /**
     *
     */
    ~ActroidBase() throw (ActroidException);

    /**
     *
     */
    uint8_t getCurrentRawAngle(const int index) {return m_CurrentRawAngle[1+index];}

    /**
     *
     */
    uint8_t getTargetRawAngle(const int index) {
		return m_TargetRawAngle[index];
	}

    /**
     *
     */
    void setTargetAngle(const int index, const double angle);

    double getCurrentAngle(const int index);

    void updateTargetAngles() {
      this->_writeRawAngle();
    }

    void updateCurrentAngles() {
      this->_readRawAngle();
    }
    
  };

};
