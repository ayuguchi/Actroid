// -*- C++ -*-
/*!
 * @file  Actroid.cpp
 * @brief Actroid RTC
 * @date $Date$
 *
 * $Id$
 */

#include "Actroid.h"

// Module specification
// <rtc-template block="module_spec">
static const char* actroid_spec[] =
  {
    "implementation_id", "Actroid",
    "type_name",         "Actroid",
    "description",       "Actroid RTC",
    "version",           "1.0.0",
    "vendor",            "Ogata Lab",
    "category",          "Experimenta",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.debug", "1",
    "conf.default.port", "COM1",
    // Widget
    "conf.__widget__.debug", "text",
    "conf.__widget__.port", "text",
    // Constraints
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
Actroid::Actroid(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_targetJointIn("targetJoint", m_targetJoint),
    m_currentJointOut("currentJoint", m_currentJoint)

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
Actroid::~Actroid()
{
}



RTC::ReturnCode_t Actroid::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("targetJoint", m_targetJointIn);
  
  // Set OutPort buffer
  addOutPort("currentJoint", m_currentJointOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debug", m_debug, "1");
  bindParameter("port", m_port, "COM1");
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t Actroid::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Actroid::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Actroid::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t Actroid::onActivated(RTC::UniqueId ec_id)
{
  // Here for Actroid, open COM port and initialize each joints.
  return RTC::RTC_OK;
}


RTC::ReturnCode_t Actroid::onDeactivated(RTC::UniqueId ec_id)
{
  // Here, finalize (cleanup) Actroid.
  return RTC::RTC_OK;
}


RTC::ReturnCode_t Actroid::onExecute(RTC::UniqueId ec_id)
{
  // Here, periodically called method is placed.

  if (m_targetJointIn.isNew()) {
    m_targetJointIn.read();
    
    if(m_debug) {
      // Print out target command.
      std::cout << "Target is " << std::endl;
      for (int i = 0;i < m_targetJoint.data.length();i++) {
	std::cout << m_targetJoint.data[i] << ", ";
      }
      std::cout << std::endl;


      
    }

  }


  // int JOINT_NUM = 25????
  // m_currentJoint.data.lenght(JOINT_NUM);
  // for (int i = 0;i < JOINT_NUM;i++) {
  //    m_currentJoint.data[i] = hoge;
  // }
  // m_currentJointOut.write();
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t Actroid::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Actroid::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t Actroid::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t Actroid::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Actroid::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void ActroidInit(RTC::Manager* manager)
  {
    coil::Properties profile(actroid_spec);
    manager->registerFactory(profile,
                             RTC::Create<Actroid>,
                             RTC::Delete<Actroid>);
  }
  
};


