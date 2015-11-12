#ifndef AD16_DUMMY_H
#define AD16_DUMMY_H

#include <vector>
#include <string>
#include <algorithm>
//#include <float.h>
//#include <math.h>

#include <mtca4uVirtualLab/VirtualLabBackend.h>
#include <mtca4uVirtualLab/SignalSink.h>
#include <boost/make_shared.hpp>

namespace msm = boost::msm;
namespace mpl = boost::mpl;

using namespace mtca4u::VirtualLab;

namespace mtca4u {


  /** The dummy device opens a mapping file instead of a device, and
   *  implements all registers defined in the mapping file in memory.
   *  Like this it mimics the real PCIe device.
   */
  class ad16dummy : public VirtualLabBackend<ad16dummy>
  {
    public:

      CONSTRUCTOR(ad16dummy,
          currentOffset(0),
          conversionFactor(0.),
          clockFrequency(50000000),
          spiFrequency(25000000),
          strobe(this),
          trigger(this),
          timers(this),
          regReset(this,"BOARD0","WORD_RESET_N"),
          regEnableDaq(this,"APP0","WORD_DAQ_ENABLE"),
          regTrigSel(this,"APP0","WORD_TIMING_TRG_SEL"),
          regTrigFreq(this,"APP0","WORD_TIMING_FREQ"),
          regTrigIntEna(this, "APP0","WORD_TIMING_INT_ENA"),
          regTrigUser(this,"APP0","WORD_TIMING_USER_TRG"),
          regSamplingFreqA(this,"AD160","WORD_ADC_A_TIMING_DIV"),
          regBufferA(this,"APP0","AREA_MULTIPLEXED_SEQUENCE_DAQ0_ADCA"),
          regBufferB(this,"APP0","AREA_MULTIPLEXED_SEQUENCE_DAQ0_ADCB"),
          regCurBuffer(this,"APP0","WORD_DAQ_CURR_BUF"),
          regBaseClockFreq(this,"AD160","WORD_CLK_FREQ"),
          regVoltageRangeA(this,"AD160","WORD_ADC_A_VOL_RANGE")
      )
        // initialise the signal sinks
        for(int i=0; i<numberOfChannels; i++) {
          sinks.push_back( boost::make_shared<SignalSink>(0) );
        }

        // initialise DAQ state machine
        INIT_SUB_STATE_MACHINE(theDaq)

        // set default values (to match the fresh registers, they will be initialised with 0 on open)
        regCurBuffer = 0;
        currentOffset = 0;

        // set clock frequency register
        regBaseClockFreq[0] = clockFrequency;
        regBaseClockFreq[1] = spiFrequency;
      END_CONSTRUCTOR

      /// current offset into the buffer (i.e. sample number)
      int32_t currentOffset;

      /// number of channels. Must be a factor of 2 (since we have 2 chips) and is essentially fixed at 16. Don't expect
      /// things to work out-of-the-box if this number is changed.
      const static int32_t numberOfChannels = 16;

      /// number of samples per buffer and channel
      const static int32_t numberOfSamples = 65536;

      /// Possible voltage ranges
      enum _voltageRange { RANGE_5Vpp=0, RANGE_10Vpp=1 };
      typedef enum _voltageRange voltageRange;

      /// conversion factor from Volts into digits
      double conversionFactor;

      /// ADC clock frequency in Hz (-> WORD_CLK_FREQ[0])
      /// Defaults to 50 MHz but can be changed by the tests to make sure the library works with any frequency. It must
      /// be changed before the call to openDev() happens to be written to the register correctly!
      int32_t clockFrequency;

      /// SPI clock frequency in Hz (-> WORD_CLK_FREQ[1])
      /// Defaults to 50 MHz but can be changed by the tests to make sure the library works with any frequency. It must
      /// be changed before the call to openDev() happens to be written to the register correctly!
      int32_t spiFrequency;

      /// event fired on a trigger (-> swap buffers)
      DECLARE_EVENT(onTrigger)

      /// event fired on strobe (-> fill a single sample for all channels)
      DECLARE_EVENT(onStrobe)

      /// timer group
      DECLARE_TIMER(strobe, onStrobe)
      DECLARE_TIMER(trigger, onTrigger)
      DECLARE_TIMER_GROUP(timers, strobe, trigger)

      /// register read/write events
      DECLARE_EVENT(onWriteReset)
      DECLARE_EVENT(onWriteDaqEnable)
      DECLARE_EVENT(onWriteUserTrigger)
      DECLARE_EVENT(onWriteTrigSel)
      DECLARE_EVENT(onWriteTrigFreq)
      DECLARE_EVENT(onWriteVoltageRange)

      /// register accessors
      DECLARE_REGISTER(int, regReset)             // BOARD0.WORD_RESET_N
      DECLARE_REGISTER(int, regEnableDaq)         // APP0.WORD_DAQ_ENABLE
      DECLARE_REGISTER(int, regTrigSel)           // APP0.WORD_TIMING_TRG_SEL
      DECLARE_REGISTER(int, regTrigFreq)          // APP0.WORD_TIMING_FREQ
      DECLARE_REGISTER(int, regTrigIntEna)        // APP0.WORD_TIMING_INT_ENA -> only used in testLibrary
      DECLARE_REGISTER(int, regTrigUser)          // APP0.WORD_TIMING_USER_TRG
      DECLARE_REGISTER(int, regSamplingFreqA)     // AD160.WORD_ADC_A_TIMING_DIV
      DECLARE_REGISTER(int, regBufferA)           // APP0.AREA_MULTIPLEXED_SEQUENCE_DAQ0_ADCA
      DECLARE_REGISTER(int, regBufferB)           // APP0.AREA_MULTIPLEXED_SEQUENCE_DAQ0_ADCB
      DECLARE_REGISTER(int, regCurBuffer)         // APP0.WORD_DAQ_CURR_BUF
      DECLARE_REGISTER(int, regBaseClockFreq)     // AD160.WORD_CLK_FREQ
      DECLARE_REGISTER(int, regVoltageRangeA)     // AD160.WORD_ADC_A_VOL_RANGE

      /// connect on-write events with register names
      WRITEEVENT_TABLE
        CONNECT_REGISTER_EVENT(onWriteReset, regReset)
        CONNECT_REGISTER_EVENT(onWriteDaqEnable, regEnableDaq)
        CONNECT_REGISTER_EVENT(onWriteTrigSel, regTrigSel)
        CONNECT_REGISTER_EVENT(onWriteUserTrigger, regTrigUser)
        CONNECT_REGISTER_EVENT(onWriteTrigFreq, regTrigFreq)
        CONNECT_REGISTER_EVENT(onWriteVoltageRange, regVoltageRangeA)
      END_WRITEEVENT_TABLE

      /// Guards for register values
      DECLARE_REGISTER_GUARD( regIsTrue, value != 0 )            // for use with any boolean register
      DECLARE_REGISTER_GUARD( regIsFalse, value == 0 )           // for use with any boolean register

      DECLARE_REGISTER_GUARD( userTriggerSelected, regTrigSel == 8 )
      DECLARE_REGISTER_GUARD( internalTriggerSelected, regTrigSel == 0 )

      /// states
      DECLARE_STATE(DaqStopped)
      DECLARE_STATE(DaqSetup)
      DECLARE_STATE(DaqRunning)
      DECLARE_STATE(TriggerSetup)
      DECLARE_STATE(TriggerUser)
      DECLARE_STATE(TriggerInternal)

      /// action: set the timer for the internal trigger
      DECLARE_ACTION(setTriggerTimer)
        int fdiv = regTrigFreq[regTrigSel];
        trigger.set( (fdiv+1.) / clockFrequency * seconds );
      END_DECLARE_ACTION

      /// action: set the strobe timer
      DECLARE_ACTION(setStrobeTimer)
        int fdiv = regSamplingFreqA;
        strobe.set( (fdiv+1.) / clockFrequency * seconds );
      END_DECLARE_ACTION

      /// signal sinks: the ad16 has 16 inputs
      std::vector< boost::shared_ptr<SignalSink> > sinks;

      /// action: fill a single sample per channel into the buffer
      DECLARE_ACTION(fillBuffer)
        // do nothing if buffer is already full
        if(currentOffset >= numberOfSamples) return;
        // fill the buffer
        for(int ic=0; ic<numberOfChannels; ic++) {
          int32_t ival = std::round( sinks[ic]->getValue(trigger.getCurrent()) * conversionFactor );
          int ioffset = currentOffset*numberOfChannels + ic;
          // write to the right buffer
          if(regCurBuffer == 0) {
            regBufferA[ioffset] = ival;
          }
          else {
            regBufferB[ioffset] = ival;
          }
        }
        // increment the offset
        currentOffset++;
      END_DECLARE_ACTION

      /// action: execute trigger
      DECLARE_ACTION(executeTrigger)
        // change current buffer
        regCurBuffer = ( regCurBuffer == 0 ? 1 : 0 );
        // reset offset
        currentOffset = 0;
      END_DECLARE_ACTION

      DECLARE_ACTION(configureInputRange)
        // compute conversion factor
        if(regVoltageRangeA == voltageRange::RANGE_5Vpp) {
          conversionFactor = pow(2., 17) / 5.;  // 18 bits signed
        }
        else {
          conversionFactor = pow(2., 17) / 10.;
        }
      END_DECLARE_ACTION

      DECLARE_ACTION(resetDevice)
        regCurBuffer = 0;
        currentOffset = 0;
        strobe.clear();
        trigger.clear();
      END_DECLARE_ACTION

      /// define the state machine structure
      DECLARE_STATE_MACHINE(theDaq, DaqSetup() << TriggerSetup(), (
        // =======================================================================================================
        // DAQ region
        // setup the DAQ by starting the strobe timer and configure voltage range
        DaqSetup() / ( setStrobeTimer(), configureInputRange() ) == DaqRunning(),

        // receive strobe: fill the buffer and restart the timer
        DaqRunning() + onStrobe() / fillBuffer() == DaqSetup(),

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
        TriggerInternal() + onTrigger() / ( executeTrigger(), setTriggerTimer() )
      ))

      /// define the state machine structure
      DECLARE_MAIN_STATE_MACHINE(DaqStopped(), (
        // handle reset
        DaqStopped() + onWriteReset() / resetDevice() == DaqStopped(),
        theDaq() + onWriteReset() / resetDevice() == DaqStopped(),

        // start and stop the DAQ
        DaqStopped() + onWriteDaqEnable() [ regIsTrue() ] == theDaq(),
        theDaq() + onWriteDaqEnable() [ regIsFalse() ] == DaqStopped(),

        // update input range selector
        theDaq() + onWriteVoltageRange() / configureInputRange()
      ))

  };


}//namespace mtca4u

#endif // AD16_DUMMY_H
