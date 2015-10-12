#ifndef AD16_DUMMY_DEVICE_H
#define AD16_DUMMY_DEVICE_H

#include <string>
#include <float.h>
#include <math.h>

#include <mtca4uVirtualLab/VirtualLabBackend.h>
#include <boost/random/uniform_int.hpp>
#include <boost/random/mersenne_twister.hpp>

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
          uniform(0, (1<<18) - 1),    // 18 bit random number
          currentOffset(0),
          clockFrequency(50000000),
          spiFrequency(25000000),
          testValue(999),
          triggerCounter(0),
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
          regBaseClockFreq(this,"AD160","WORD_CLK_FREQ")
      )
        INIT_SUB_STATE_MACHINE(theDaq)

        // set default values (to match the fresh registers, they will be initialised with 0 on open)
        regCurBuffer = 0;
        currentOffset = 0;

        // set clock frequency register
        regBaseClockFreq[0] = clockFrequency;
        regBaseClockFreq[1] = spiFrequency;
      END_CONSTRUCTOR

      /// random number generator to fill channels with white noise
      boost::mt11213b rng;
      boost::uniform_int<> uniform;    // 18 bit random number

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

      /// connect on-write events with register names
      WRITEEVENT_TABLE
        CONNECT_REGISTER_EVENT(onWriteReset, regReset)
        CONNECT_REGISTER_EVENT(onWriteDaqEnable, regEnableDaq)
        CONNECT_REGISTER_EVENT(onWriteTrigSel, regTrigSel)
        CONNECT_REGISTER_EVENT(onWriteUserTrigger, regTrigUser)
        CONNECT_REGISTER_EVENT(onWriteTrigFreq, regTrigFreq)
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
      DECLARE_STATE(TriggerInternal2)

      /// action: set the timer for the internal trigger
      DECLARE_ACTION(setTriggerTimer)
        int fdiv = regTrigFreq[regTrigSel];
        trigger.set( 1.e3 * (fdiv+1.) / clockFrequency );
      END_DECLARE_ACTION

      /// action: set the strobe timer
      DECLARE_ACTION(setStrobeTimer)
        int fdiv = regSamplingFreqA;
        strobe.set( 1.e3 * (fdiv+1.) / clockFrequency );
        END_DECLARE_ACTION

      /// action: fill a single sample per channel into the buffer
      DECLARE_ACTION(fillBuffer)
        // do nothing if buffer is already full
        if(currentOffset >= numberOfSamples) return;
        // fill the buffer
        for(int ic=0; ic<numberOfChannels; ic++) {
          int32_t ival;
          if(ic == 0) {
            ival = 1000.*sin(2.*acos(-1) * 1000. * (float)currentOffset/(float)numberOfSamples);
          }
          else if(ic == 1) {
            ival = currentOffset;
          }
          else if(ic == 2) {
            ival = testValue;
          }
          else if(ic == 3) {
            ival = testValue;
          }
          else {
            ival = uniform(rng);
          }
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
        // increment trigger counter
        triggerCounter++;
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
        // setup the DAQ by starting the strobe timer
        DaqSetup() / setStrobeTimer() == DaqRunning(),

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
        theDaq() + onWriteDaqEnable() [ regIsFalse() ] == DaqStopped()
      ))

  };


}//namespace mtca4u

#endif // AD16_DUMMY_DEVICE_H
