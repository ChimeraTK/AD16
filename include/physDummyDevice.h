/*
 * physDummyDevice.h - base class for MTCA.4u dummy devices that fit into the physics simulation framework
 *
 *  Created on: Aug 27, 2015
 *      Author: martin.hierholzer@desy.de
 */

#ifndef PHYSDUMMYDEVICE_H
#define PHYSDUMMYDEVICE_H

#include <string>

#include <MtcaMappedDevice/DummyDevice.h>
#include <boost/bind.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/front/state_machine_def.hpp>
#include <boost/msm/front/euml/euml.hpp>

namespace msm = boost::msm;
namespace mpl = boost::mpl;

//
// Declare an event. Use instead of BOOST_MSM_EUML_EVENT, as we must not create instances for the events as well.
//
#define DECLARE_EVENT(name)                                                                                     \
  class name : public msm::front::euml::euml_event< name > {};

//
// Declare a plain state, without any entry or exit functions
//
#define DECLARE_PLAIN_STATE(name)                                                                               \
  class name : public msm::front::state<> , public msm::front::euml::euml_state<name> {};

//
// Declare a state. The second arg decl can be used to define onEntry and onExit conditions with the macros
// STATE_ON_ENTRY and STATE_ON_EXIT.
//
#define DECLARE_STATE(name,decl)                                                                                \
  class name : public msm::front::state<> , public msm::front::euml::euml_state<name> {                         \
    public:                                                                                                     \
      decl                                                                                                      \
  };

// for use inside DECLARE_STATE(), see documentation there. Put the code to be executed on entry in braces after the
// macro invocation.
#define STATE_ON_ENTRY                                                                                          \
   template <class Event,class FSM>                                                                             \
   void on_entry(Event const&,FSM&)

// for use inside DECLARE_STATE(), see documentation there Put the code to be executed on exit in braces after the
// macro invocation.
#define STATE_ON_EXIT                                                                                           \
   template <class Event,class FSM>                                                                             \
   void on_exit(Event const&,FSM&)

//
// Declare a logging state. Entry and exit of this state will be looged to std::cout.
//
#define DECLARE_LOGGING_STATE(name)                                                                             \
  class name : public msm::front::state<> , public msm::front::euml::euml_state<name> {                         \
    public:                                                                                                     \
      template <class Event,class FSM>                                                                          \
      void on_entry(Event const&,FSM&) { std::cout << "Entering state: " << #name << std::endl; }               \
      template <class Event,class FSM>                                                                          \
      void on_exit(Event const&,FSM&) { std::cout << "Leaving state: " << #name << std::endl; }                 \
  };

//
// Provide a "table" of events and register names using the CONNECT_REGISTER_EVENT macro for write events
//
#define WRITE_EVENT_TABLE(table)                                                                                \
  void regWriteEvents(uint32_t regOffset, int32_t const *data, size_t size, uint8_t bar) {                      \
    (void)regOffset; (void)data; (void)size; (void)bar;                                                         \
    table                                                                                                       \
  }

//
// Provide a "table" of events and register names using the CONNECT_REGISTER_EVENT macro for read events
//
#define READ_EVENT_TABLE(table)                                                                                 \
  void regReadEvents(uint32_t regOffset, int32_t const *data, size_t size, uint8_t bar) {                       \
    (void)regOffset; (void)data; (void)size; (void)bar;                                                         \
    table                                                                                                       \
  }

//
// Connect events with register names. The first argument eventName is the name of an event previously
// declared using BOOST_MSM_EUML_EVENT. Use this macro in the of the WRITE_EVENT_TABLE and READ_EVENT_TABLE macros.
// Do not separate multiple calls to CONNECT_REGISTER_EVENT inside the same WRITE_EVENT_TABLE/READ_EVENT_TABLE macro by
// any separator (i.e. no comma in between - starting a new line is allowed, though!).
//
#define CONNECT_REGISTER_EVENT(eventName, registerModule, registerName)                                         \
  {                                                                                                             \
    mapFile::mapElem elem;                                                                                      \
    _registerMapping->getRegisterInfo(registerName, elem, registerModule);                                      \
    if(bar == elem.reg_bar && regOffset >= elem.reg_address && regOffset < elem.reg_address+elem.reg_size) {    \
      theStateMachine->process_event(eventName ());                                                             \
    }                                                                                                           \
  }

//
// Declare a guard condition on the value of a register. The resulting guard condition can be used on an event
// defined with the CONNECT_REGISTER_EVENT macro. The first argument guardName is the name of the resulting guard class
// and the second argument will be parsed as the guard condition (standard C++ syntax). Inside the condition, the
// variable "value" can be used which contains the value of the register written in the event (type: int32_t, always the
// first word written).
//
#define DECLARE_REGISTER_GUARD(guardName, condition)                                                            \
    class guardName :  msm::front::euml::euml_action<guardName>                                                 \
    {                                                                                                           \
      public:                                                                                                   \
        guardName() {}                                                                                          \
        typedef guardName action_name;                                                                          \
        template <class Fsm,class Evt,class SourceState,class TargetState>                                      \
        bool operator()(Evt const& ,Fsm& fsm,SourceState&,TargetState& )                                        \
        {                                                                                                       \
            myDummyDeviceType *dev = fsm.dev;                                                                   \
            assert( dev->lastWrittenData != NULL );                                                             \
            int32_t value = *(dev->lastWrittenData);                                                            \
            (void)value;                                                                                        \
            return ( condition );                                                                               \
        }                                                                                                       \
    };

//
// declare an action with arbitrary code. The second argument contains the code.
//
#define DECLARE_ACTION(actionName, code)                                                                        \
    class actionName :  msm::front::euml::euml_action<actionName>                                               \
    {                                                                                                           \
      public:                                                                                                   \
      actionName() {}                                                                                           \
        typedef actionName action_name;                                                                         \
        template <class Fsm,class Evt,class SourceState,class TargetState>                                      \
        void operator()(Evt const& ,Fsm& fsm,SourceState&,TargetState& )                                        \
        {                                                                                                       \
            myDummyDeviceType *dev = fsm.dev;                                                                   \
            (void)dev;                                                                                          \
            code                                                                                                \
        }                                                                                                       \
    };

namespace mtca4u {


  /** The dummy device opens a mapping file instead of a device, and
   *  implements all registers defined in the mapping file in memory.
   *  Like this it mimics the real PCIe device.
   *
   *  This class helps implementing dummy devices using a state machine and allows an easy connection with the physics
   *  simulation framework.
   */
  class physDummyDevice : public DummyDevice
  {
    public:

      physDummyDevice() :
        lastWrittenData(NULL),
        lastWrittenSize(0),
        isOpened(false)
      {}
      virtual ~physDummyDevice() {}

      /// on device open: fire the device-open event
      virtual void openDev(const std::string &mappingFileName, int perm=O_RDWR, devConfigBase *pConfig=NULL) {
        if(isOpened) throw DummyDeviceException("Device is already opened.", DummyDeviceException::ALREADY_OPEN);
        isOpened = true;
        DummyDevice::openDev(mappingFileName, perm, pConfig);
      }

      /// on device close: fire the device-close event
      virtual void closeDev() {
        if(!isOpened) throw DummyDeviceException("Device is already closed.", DummyDeviceException::ALREADY_CLOSED);
        isOpened = false;
        DummyDevice::closeDev();
      }

      /// redirect writeReg to writeArea, so the events get triggered here, too
      virtual void writeReg(uint32_t regOffset, int32_t data, uint8_t bar) {
        writeArea(regOffset, &data, 4, bar);
      }

      /// redirect readReg to writeArea, so the events get triggered here, too
      virtual void readReg(uint32_t regOffset, int32_t *data, uint8_t bar) {
        readArea(regOffset, data, 4, bar);
      }

      /// override writeArea to fire the events
      virtual void writeArea(uint32_t regOffset, int32_t const *data, size_t size, uint8_t bar) {
        // save as last written data, for use inside guards of the events we may trigger now
        lastWrittenData = data;
        lastWrittenSize = size;

        // perform the actual write
        DummyDevice::writeArea(regOffset, data, size, bar);

        // trigger events
        regWriteEvents(regOffset,data,size,bar);
      }

      /// override readArea to fire the events
      virtual void readArea(uint32_t regOffset, int32_t *data, size_t size, uint8_t bar) {
        // perform the actual write
        DummyDevice::readArea(regOffset, data, size, bar);

        // trigger events
        regReadEvents(regOffset,data,size,bar);
      }

    protected:

      /// trigger register-write events. Will be implemented using WRITE_EVENT_TABLE in the device implementation
      virtual void regWriteEvents(uint32_t regOffset, int32_t const *data, size_t size, uint8_t bar) = 0;

      /// trigger register-read events. Will be implemented using READ_EVENT_TABLE in the device implementation
      virtual void regReadEvents(uint32_t regOffset, int32_t const *data, size_t size, uint8_t bar) = 0;

      /// timer class
      template<class timerAction, class dummyDeviceType>
      class timer {
        public:
          timer(dummyDeviceType *_dev) : dev(_dev),request(-1),current(0) {}
          dummyDeviceType *dev;

          /// set the timer to fire in tval milliseconds
          void set(double tval) {
            request = tval+current;
          }

          /// advance the timer's current time by tval milliseconds. Returns true if the timer was fired
          bool advance(double tval) {
            current += tval;
            if(request > 0 && current >= request) {
              request = -1;
              dev->theStateMachine->process_event( timerAction() );
              return true;
            }
            return false;
          }

          /// advance the timer to the next requested time. Returns true if the timer was fired. Otherwise the timer
          /// was not set and thus the current time remains unchanged.
          bool advance() {
            if(request > 0) {
              current = request;
              request = -1;
              dev->theStateMachine->process_event( timerAction() );
              return true;
            }
            return false;
          }

          /// obtain remaining time until the timer fires. Will be negative if the timer is not set.
          double getRemaining() {
            if(request > 0) {
              return request-current;
            }
            else {
              return -1;
            }
          }

        protected:

          /// requested time
          double request;

          /// current time
          double current;
      };

      /// master timer interface class (TODO find better name?)
      /// The implementation needs to be specific for all actual timer types
      class masterTimer {
        public:
          masterTimer() : current(0) {}
          virtual ~masterTimer() {}

          /// advance the timer's current time by tval milliseconds. Returns true if any timer was fired
          /// TODO must not advance more than getRemaining() at a time!
          virtual bool advanceBy(double tval) {
            current += tval;
            return false;
          }

          /// get remaining time until the next timer fires.
          virtual double getRemaining() = 0;

          /// advance the timer to the next requested time. Returns true if any timer was fired. Otherwise none of the
          /// timers was set and thus the current time remains unchanged
          bool advanceAll() {
            return advanceBy(getRemaining());
          }

          /// advance the timer to the next requested time of the given sub-timer. Returns true if a timer was fired.
          template<class T>
          bool advanceSub(T timer) {
              double tval = timer.getRemaining();
              if(tval < 0) return false;
              return advanceBy(tval);
          }

          /// get current time (in milliseconds)
          double getCurrent()
          {
            return current;
          }

        protected:

          /// current time
          double current;

      };

      /// register accessors (should go into DummyDevice class later?)
      template<typename T, class dummyDeviceType>
      class dummyRegister {
        public:

          void open(dummyDeviceType *dev, std::string module, std::string name)
          {
            _dev = dev;
            dev->_registerMapping->getRegisterInfo(name, elem, module);
          }

          T get(int index=0)
          {
            T value;
            _dev->readReg(elem.reg_address + sizeof(int32_t)*index, reinterpret_cast<int32_t*>(&value), elem.reg_bar);
            return value;
          }
          void set(T value, int index=0)
          {
            int32_t *v = reinterpret_cast<int32_t*>(&value);
            _dev->writeRegisterWithoutCallback(elem.reg_address + sizeof(int32_t)*index, *v, elem.reg_bar);
          }
          T operator[](int index)
          {
            return get(index);
          }
          T& operator=(T &rhs)
          {
            set(rhs);
            return rhs;
          }

        protected:
          mapFile::mapElem elem;
          dummyDeviceType *_dev;
      };

      /// last written data (into any register) and its size. Will be used in guard conditions.
      int32_t const *lastWrittenData;
      size_t lastWrittenSize;

      /// flag if device currenty opened
      bool isOpened;

  };

}//namespace mtca4u

#endif /* PHYSDUMMYDEVICE_H */
