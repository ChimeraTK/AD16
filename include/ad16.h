#ifndef MTCA4U_AD16_H
#define MTCA4U_AD16_H

#include <string>
#include <vector>
#include "ad16DummyDevice.h"
#include "ad16Exception.h"
#include <MtcaMappedDevice/devMap.h>

namespace mtca4u{

  /**
   *  Interface class to use the AD16 card.
   *
   *  This interface is very preliminary, as the firmware of the
   *  AD16 device is not yet ready.
   */
  class ad16 {
    public:

      ad16() {};
      ~ad16() {};

      /// Open AD16 device. Currently only a dummy device can be opened. The dummy device is selected by setting both arguments to the mapping file name.
      void open(const std::string &deviceFileName, const std::string &mappingFileName);

      /// Close AD16 device.
      void close();

      /// possible sampling rates
      enum rate { RATE_10000Hz=1, RATE_5000Hz=2, RATE_3333Hz=3, RATE_2500Hz=4, RATE_2000Hz=5, RATE_1000Hz=10, RATE_100Hz=100 };

      /// set sampling rate rivisor (see enum type above)
      void setSamplingRate(int divisor);

      /// set number of samples per conversion block and channel
      void setSamplesPerBlock(int samples);

      /// Start AD conversion (i.e. send software trigger)
      void startConversion();

      /// Tests if conversion is complete (returns true) or currently running (returns false)
      bool conversionComplete();

      /// Read all channels from AD16 buffer into software buffer
      void read();

      /// Get data for single channel after a previous read()
      std::vector<int> getChannelData(unsigned int channel);

    private:

      /// our mapped device
      devMap<devBase> _mappedDevice;

      /// pointer to dummy device (if used)
      boost::shared_ptr<ad16DummyDevice> _dummyDevice;

      /// accessor for multiplexed data
      boost::shared_ptr< mtca4u::MultiplexedDataAccessor<int32_t> > dataDemuxed;


  };

}

#endif /* INCLUDE_AD16_H_ */
