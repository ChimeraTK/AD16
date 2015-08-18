#ifndef AD16_DUMMY_DEVICE_H
#define AD16_DUMMY_DEVICE_H

#include <MtcaMappedDevice/DummyDevice.h>
#include <boost/random/mersenne_twister.hpp>
#include <boost/thread.hpp>

namespace mtca4u{

  /** The dummy device opens a mapping file instead of a device, and
   *  implements all registers defined in the mapping file in memory.
   *  Like this it mimics the real PCIe device.
   * 
   *  This implementation is very preliminary, as the firmware of the
   *  AD16 device is not yet ready.
   */
  class ad16DummyDevice : public DummyDevice
  {
    public:

      ad16DummyDevice();
      virtual ~ad16DummyDevice();

      virtual void openDev(const std::string &mappingFileName, int perm=O_RDWR, devConfigBase *pConfig=NULL);

      virtual void closeDev();

    protected:

      /// flag if the device is currently open or not
      bool isOpened;

      /// callback writing to WORD_DAQ_ENABLE
      void callbackDaqEnable();

      /// setup callback for configuration retoster
      void setupCallback(std::string registerName, boost::function<void()> cb);

      /// thread function to simulate the DAQ and its timing
      void threadDaq();

      /// boost thread object for threadDaq()
      boost::thread theThread;

      /// random number generator to fill channels with white noise
      boost::mt11213b rng;

      /// flag if a DAQ is currently running (to prevent starting multiple DAQ threads)
      volatile bool isDaqRunning;

      /// possible DAQ strobe sources
      enum mode { DAQ_STROBE_ADCA=0, DAQ_STROBE_ADCB=1, DAQ_STROBE_TRIGGER6=2 };

      /// current write position in buffer
      int32_t currentPosition;

      /// current buffer to write to
      int32_t currentBuffer;

      /// current ADC sample buffer (before DAQ)
      std::vector<int32_t> ADCvalA, ADCvalB;

      /// number of channels. Must be a factor of 2 (since we have 2 chips) and is essentially fixed at 16. Don't expect
      /// things to work out-of-the-box if this number is changed.
      const static int32_t numberOfChannels;

      /// number of samples per buffer and channel
      const static int32_t numberOfSamples;

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
