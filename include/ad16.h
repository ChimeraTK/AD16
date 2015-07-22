#ifndef MTCA4U_AD16_H
#define MTCA4U_AD16_H

#include <string>
#include <vector>
#include "ad16DummyDevice.h"
#include "ad16Exception.h"
#include <MtcaMappedDevice/devMap.h>
#include <MtcaMappedDevice/devPCIE.h>
#include <boost/python.hpp>

namespace mtca4u{

  /**
   *  Interface class to use the AD16 card.
   *
   *  This interface is very preliminary, as the firmware of the
   *  AD16 device is not yet ready.
   */
  class ad16 {
    public:

      ad16() : _samplesPerBlock(65536), _mode(0), _lastBuffer(-1), max_elem_nr(0) {};
      ~ad16() {};

      /// Open AD16 device. Currently only a dummy device can be opened. The dummy device is selected by setting both arguments to the mapping file name.
      void open(const std::string &deviceFileName, const std::string &mappingFileName);

      /// Close AD16 device.
      void close();

      /// possible operation modes
      enum mode { SOFTWARE_TRIGGER=8, AUTO_TRIGGER=0 };

      /// set mode of operation
      void setMode(int mode);

      /// possible sampling rates (base sample rate is assumed to be 100 kHz)
      enum rate { RATE_100000Hz=1, RATE_50000Hz=2, RATE_25000Hz=4, RATE_20000Hz=5, RATE_10000Hz=10, RATE_5000Hz=20, RATE_1000Hz=100 };

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

      /// Get data for single channel after a previous read(). Version for Python with numpy array
      void getChannelDataNumpy(unsigned int channel, boost::python::numeric::array &numpyArray);

      /// number of channels (constant)
      const static int32_t numberOfChannels;

    private:

      /// register map
      boost::shared_ptr<mapFile> _map;

      /// our mapped device
      boost::shared_ptr< devMap<devBase> > _mappedDevice;

      /// pointer to dummy device (if used)
      boost::shared_ptr<ad16DummyDevice> _dummyDevice;

      /// pointer to real device (if used)
      boost::shared_ptr<devPCIE> _realDevice;

      /// accessor for multiplexed data
      boost::shared_ptr< mtca4u::MultiplexedDataAccessor<int32_t> > _dataDemuxed;

      /// number of samples per conversion block and channel
      unsigned int _samplesPerBlock;

      /// mode of operation
      int _mode;

      /// last buffer read (used to detect completed conversion in case of double buffering)
      int _lastBuffer;

      /// maximum number of elements in buffers (as defined in the map file)
      /// this needs to be stored, as the map will be modified at run time (in-memory)
      uint32_t max_elem_nr;

      /// start time of last conversion
      boost::posix_time::ptime t0;


  };

}

#endif /* INCLUDE_AD16_H_ */
