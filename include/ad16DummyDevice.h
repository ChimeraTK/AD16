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

      ad16DummyDevice() : isConversionRunning(false) {}
      virtual ~ad16DummyDevice() {}

      virtual void openDev (const std::string &mappingFileName, int perm=O_RDWR, devConfigBase *pConfig=NULL);

    protected:

      /// callback writing to the START_CONVERSION register
      void callbackStartConversion();

      /// thread function to simulate the time needed for the conversion
      void threadConversion();

      /// random number generator to fill channels with white noise
      boost::mt11213b rng;

      /// thread used to simulate conversion timing
      boost::thread theThread;

      /// flag if a conversion is currently running (to prevent starting multiple conversions)
      volatile bool isConversionRunning;

  };

}//namespace mtca4u

#endif // AD16_DUMMY_DEVICE_H
