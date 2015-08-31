#ifndef AD16_DUMMY_DEVICE_H
#define AD16_DUMMY_DEVICE_H

#include <string>
#include <float.h>
#include <math.h>

#include <physDummyDevice.h>
#include <boost/random/mersenne_twister.hpp>

namespace msm = boost::msm;
namespace mpl = boost::mpl;


namespace mtca4u {


  /** The dummy device opens a mapping file instead of a device, and
   *  implements all registers defined in the mapping file in memory.
   *  Like this it mimics the real PCIe device.
   */
  class ad16DummyDevice : public physDummyDevice
  {
    public:

      ad16DummyDevice() :
        timer(this),
        theStateMachine(this),
        uniform(0, (1<<18) - 1),    // 18 bit random number
        currentBuffer(0),
        currentOffset(0),
        clockFrequency(50000000),
        spiFrequency(25000000),
        testValue(999),
        triggerCounter(0)
      {
        theStateMachine.get_state< theDaq* >()->setDummyDevice(this);
        theStateMachine.start();
      }

      virtual ~ad16DummyDevice() {}

      /// on device open: fire the device-open event
      virtual void openDev(const std::string &mappingFileName, int perm=O_RDWR, devConfigBase *pConfig=NULL) {

        // open the underlying dummy device
        physDummyDevice::openDev(mappingFileName, perm, pConfig);

        // send onDeviceOpen event
        theStateMachine.process_event(onDeviceOpen());

        // setup registers
        regTrigSel.open(this,"APP0","WORD_TIMING_TRG_SEL");
        regTrigFreq.open(this,"APP0","WORD_TIMING_FREQ");
        regSamplingFreqA.open(this,"AD160","WORD_ADC_A_TIMING_DIV");
        regBufferA.open(this,"APP0","AREA_MULTIPLEXED_SEQUENCE_DAQ0_ADCA");
        regBufferB.open(this,"APP0","AREA_MULTIPLEXED_SEQUENCE_DAQ0_ADCB");
        regCurBuffer.open(this,"APP0","WORD_DAQ_CURR_BUF");
        regBaseClockFreq.open(this,"AD160","WORD_CLK_FREQ");

        // set default values (to match the fresh registers, they will be initialised with 0 on open)
        currentBuffer = 0;
        currentOffset = 0;

        // set clock frequency register
        regBaseClockFreq.set(clockFrequency,0);
        regBaseClockFreq.set(spiFrequency,1);
      }

      /// on device close: fire the device-close event
      virtual void closeDev() {
        physDummyDevice::closeDev();
        theStateMachine.process_event(onDeviceClose());
      }

      //typedef ad16DummyDevice myDummyDeviceType;

      /// handy name for the int32_t register accessor
      typedef dummyRegister<int32_t,ad16DummyDevice> intRegister;

      /// events for device opening and closing
      DECLARE_EVENT(onDeviceOpen)
      DECLARE_EVENT(onDeviceClose)

      /// event fired on a trigger (-> swap buffers)
      DECLARE_EVENT(onTrigger)

      /// event fired on strobe (-> fill a single sample for all channels)
      DECLARE_EVENT(onStrobe)

      /// master timer
      class ad16MasterTimer : public masterTimer {
        public:
          ad16MasterTimer(ad16DummyDevice *dev) :
            masterTimer(),
            strobe(dev),
            trigger(dev)
          {}

          /// advance the timer's current time by tval milliseconds. Returns true if any timer was fired
          virtual bool advanceBy(double tval) {
            bool r = false;
            r = strobe.advance(tval) || r;
            r = trigger.advance(tval) || r;
            masterTimer::advanceBy(tval);
            return r;
          }

          /// get remaining time until the next timer fires.
          virtual double getRemaining() {
            double r = DBL_MAX;
            double rr;
            rr= strobe.getRemaining();
            if(rr > 0) r = fmin(rr, r);
            rr = trigger.getRemaining();
            if(rr > 0) r = fmin(rr, r);
            if(r >= DBL_MAX) r = -1;
            return r;
          }

          /// strobe timer
          timer<onStrobe, ad16DummyDevice> strobe;

          /// trigger timer
          timer<onTrigger, ad16DummyDevice> trigger;

      };
      ad16MasterTimer timer;

      /// register read/write events
      DECLARE_EVENT(onWriteDaqEnable)
      DECLARE_EVENT(onWriteUserTrigger)
      DECLARE_EVENT(onWriteTrigSel)
      DECLARE_EVENT(onWriteTrigFreq)

      /// register accessors
      intRegister regTrigSel;           // APP0.WORD_TIMING_TRG_SEL
      intRegister regTrigFreq;          // APP0.WORD_TIMING_FREQ
      intRegister regSamplingFreqA;     // AD160.WORD_ADC_A_TIMING_DIV
      intRegister regBufferA;           // APP0.AREA_MULTIPLEXED_SEQUENCE_DAQ0_ADCA
      intRegister regBufferB;           // APP0.AREA_MULTIPLEXED_SEQUENCE_DAQ0_ADCB
      intRegister regCurBuffer;         // APP0.WORD_DAQ_CURR_BUF
      intRegister regBaseClockFreq;     // AD160.WORD_CLK_FREQ

      /// connect on-write events with register names
      WRITE_EVENT_TABLE(
        CONNECT_REGISTER_EVENT(onWriteDaqEnable, "APP0","WORD_DAQ_ENABLE")
        CONNECT_REGISTER_EVENT(onWriteTrigSel, "APP0","WORD_TIMING_TRG_SEL")
        CONNECT_REGISTER_EVENT(onWriteUserTrigger, "APP0","WORD_TIMING_USER_TRG")
        CONNECT_REGISTER_EVENT(onWriteTrigFreq, "APP0","WORD_TIMING_FREQ")
      )

      /// connect on-read events with register names
      READ_EVENT_TABLE()

      /// Guards for register values
      DECLARE_REGISTER_GUARD( regIsTrue, value != 0 )            // for use with any boolean register
      DECLARE_REGISTER_GUARD( regIsFalse, value == 0 )           // for use with any boolean register

      DECLARE_REGISTER_GUARD( userTriggerSelected, dev->regTrigSel.get() == 8 )
      DECLARE_REGISTER_GUARD( internalTriggerSelected, dev->regTrigSel.get() == 0 )

      /// states
      DECLARE_STATE(DevClosed)
      DECLARE_STATE(DaqStopped)
      DECLARE_STATE(DaqSetup)
      DECLARE_STATE(DaqRunning)
      DECLARE_STATE(TriggerSetup)
      DECLARE_STATE(TriggerUser)
      DECLARE_STATE(TriggerInternal)

      /// action: set the timer for the internal trigger
      DECLARE_ACTION(setTriggerTimer,
        int trig = dev->regTrigSel.get();
        int fdiv = dev->regTrigFreq.get(trig);
        dev->timer.trigger.set( 1.e3 * (fdiv+1.) / dev->clockFrequency );
      )

      /// action: set the strobe timer
      DECLARE_ACTION(setStrobeTimer,
        int fdiv = dev->regSamplingFreqA.get();
        dev->timer.strobe.set( 1.e3 * (fdiv+1.) / dev->clockFrequency );
      )

      /// action: fill a single sample per channel into the buffer
      DECLARE_ACTION(fillBuffer,
        // do nothing if buffer is already full
        if(dev->currentOffset >= numberOfSamples) return;
        // determine buffer to write to
        intRegister *acc;
        if(dev->currentBuffer == 0) {
          acc = &dev->regBufferA;
        }
        else {
          acc = &dev->regBufferB;
        }
        // fill the buffer
        for(int ic=0; ic<numberOfChannels; ic++) {
          int32_t ival;
          if(ic == 0) {
            ival = 1000.*sin(2.*acos(-1) * 1000. * (float)dev->currentOffset/(float)numberOfSamples);
          }
          else if(ic == 1) {
            ival = dev->currentOffset;
          }
          else if(ic == 2) {
            ival = dev->testValue;
          }
          else if(ic == 3) {
            ival = dev->testValue;
          }
          else {
            ival = dev->uniform(dev->rng);
          }
          int ioffset = dev->currentOffset*numberOfChannels + ic;
          acc->set(ival, ioffset);
        }
        // increment the offset
        dev->currentOffset++;
      )

      /// action: execute trigger
      DECLARE_ACTION(executeTrigger,
        // change current buffer
        dev->currentBuffer = ( dev->currentBuffer == 0 ? 1 : 0 );
        dev->regCurBuffer = dev->currentBuffer;
        // reset offset
        dev->currentOffset = 0;
        // increment trigger counter
        dev->triggerCounter++;
      )

      /// events for enabling and disabling the trigger
      DECLARE_EVENT(enableTrigger)
      DECLARE_EVENT(disableTrigger)

      /// actions to send events to disable and enable the trigger
      DECLARE_ACTION(sendEnableTrigger,
        fsm.process_event(enableTrigger());
      )
      DECLARE_ACTION(sendDisableTrigger,
        fsm.process_event(disableTrigger());
      )

      /// define the state machine structure
      DECLARE_STATE_MACHINE(ad16DummyDevice, theDaq, DaqSetup() << TriggerSetup(), (
        // =======================================================================================================
        // DAQ region
        // setup the DAQ by starting the strobe timer
        DaqSetup() / setStrobeTimer() == DaqRunning(),

        // receive strobe: fill the buffer and restart the timer
        DaqRunning() + onStrobe() / ( fillBuffer(), setStrobeTimer() ),

        // =======================================================================================================
        // trigger region
        // setup the trigger
        TriggerSetup() [ userTriggerSelected() ] == TriggerUser(),
        TriggerSetup() [ internalTriggerSelected() ] / setTriggerTimer() == TriggerInternal(),

        // change selected trigger
        TriggerInternal() + onWriteTrigSel() == TriggerSetup(),
        TriggerUser() + onWriteTrigSel() == TriggerSetup(),

        // update the trigger frequency
        TriggerInternal() + onWriteTrigFreq() / setTriggerTimer(),

        // user trigger mode
        TriggerUser() + onWriteUserTrigger() / executeTrigger(),

        // internal trigger mode
        TriggerInternal() + onTrigger() / ( setTriggerTimer(), executeTrigger() ),

        // =======================================================================================================
        // ignore some events in certain states (might occur after state change and should not throw an exception)
        TriggerUser() + onWriteTrigFreq(),
        TriggerInternal() + onWriteUserTrigger(),
        TriggerUser() + onTrigger(),
        DaqSetup() + onDeviceOpen()
      ))

      /// define the state machine structure
      DECLARE_STATE_MACHINE(ad16DummyDevice, mainStateMachine, DevClosed(), (
        // =======================================================================================================
        // open and close the device
        DevClosed() + onDeviceOpen() == DaqStopped(),
        DaqStopped() + onDeviceClose() == DevClosed(),
        theDaq() + onDeviceClose() == DevClosed(),

        // start and stop the DAQ
        DaqStopped() + onWriteDaqEnable() [ regIsTrue() ] == theDaq(),
        theDaq() + onWriteDaqEnable() [ regIsFalse() ] == DaqStopped(),

        // =======================================================================================================
        // ignore some events in certain states (might occur after state change and should not throw an exception)
        DaqStopped() + onStrobe(),
        DevClosed() + onStrobe()
      ))

      mainStateMachine theStateMachine;

      /// random number generator to fill channels with white noise
      boost::mt11213b rng;
      boost::uniform_int<> uniform;    // 18 bit random number

      /// current buffer to write to
      int32_t currentBuffer;

      /// current offset into the buffer (i.e. sample number)
      int32_t currentOffset;

      /// number of channels. Must be a factor of 2 (since we have 2 chips) and is essentially fixed at 16. Don't expect
      /// things to work out-of-the-box if this number is changed.
      const static int32_t numberOfChannels = 16;

      /// number of samples per buffer and channel
      const static int32_t numberOfSamples = 65536;

      /// ADC clock frequency in Hz (-> WORD_CLK_FREQ[0])
      /// Defaults to 50 MHz but can be changed by the tests to make sure the library works with any frequency. It must
      /// be changed before the call to openDev() happens to be written to the register correctly!
      int32_t clockFrequency;

      /// SPI clock frequency in Hz (-> WORD_CLK_FREQ[1])
      /// Defaults to 50 MHz but can be changed by the tests to make sure the library works with any frequency. It must
      /// be changed before the call to openDev() happens to be written to the register correctly!
      int32_t spiFrequency;

      /// some test value to be written to the 3rd channel
      int32_t testValue;

      /// trigger counter, will be written into the 4th channel
      int32_t triggerCounter;

  };

}//namespace mtca4u

#endif // AD16_DUMMY_DEVICE_H
