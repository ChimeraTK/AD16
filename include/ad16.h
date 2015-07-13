#ifndef MTCA4U_AD16_H
#define MTCA4U_AD16_H

#include <string>
#include <vector>
#include "ad16DummyDevice.h"
#include "ad16Exception.h"
#include <MtcaMappedDevice/devMap.h>

namespace mtca4u{

  typedef std::vector<int> DataVector;

  /**
   *  Interface class to use the AD16 card.
   *
   *  This interface is very preliminary, as the firmware of the
   *  AD16 device is not yet ready.
   */
  class ad16{
    public:

      ad16() {};
      ~ad16() {};

      /// Open AD16 device. Currently only a dummy device can be opened. The dummy device is selected by setting both arguments to the mapping file name.
      void open(const std::string &deviceFileName, const std::string &mappingFileName);

      /// Close AD16 device.
      void close();

      /// Start AD conversion (i.e. send software trigger)
      void startConversion();

      /// Read data from AD16 buffer into software buffer
      void read();

      /// Get data for single channel after a previous read()
      DataVector getChannelData(unsigned int channel);

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
