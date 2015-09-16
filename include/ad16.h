#ifndef MTCA4U_AD16_H
#define MTCA4U_AD16_H

#include <string>
#include <vector>
#include <boost/any.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <MtcaMappedDevice/Device.h>
#include <MtcaMappedDevice/PcieBackend.h>

#include "ad16DummyDevice.h"
#include "ad16Exception.h"

#ifdef ENABLE_PYTHON_BINDINGS
#include <boost/python.hpp>
#endif

namespace mtca4u {

  /**
   *  Interface class to use the AD16 card.
   */
  class ad16 {
    public:

      ad16() : _isOpen(false), _mode(0), _currentBuffer(-1), _samplingFrequency(-1) {};
      ~ad16() {};

      /// Open AD16 device through a DMAP file
      /// deviceAlias is the alias name of the AD16 to open inside the dmap file.
      void openDmap(const std::string &dmapFileName, const std::string &deviceAlias);

      /// Open AD16 device by specifying the device and mapping file names
      /// If the deviceFileName and the mappingFileName both contain the mappingFileName, a dummy device based on that
      /// mapping file will be created.
      void open(const std::string &deviceFileName, const std::string &mappingFileName);

      /// Close AD16 device.
      void close();

      /// Possible oversampling ratios
      enum _oversampling { NO_OVERSAMPLING=0, RATIO_2=1, RATIO_4=2, RATIO_8=3, RATIO_16=4, RATIO_32=5, RATIO_64=6 };
      typedef enum _oversampling oversampling;

      /// Set sampling rate in Hz and oversampling ratio for both ADC chips on the board.
      /// The sampling rate might be rounded to the next possible value. The actually used sampling rate can be
      /// obtained using getSamplingRate().
      /// It will throw an ad16Exception::IMPOSSIBLE_TIMING_CONFIGURATION, if the sampling rate is out of range (for the
      /// selected oversampling ratio).
      void setSamplingRate(float rate, oversampling oversampling = NO_OVERSAMPLING);

      /// Get the actual sampling rate after rounding to the next possible value
      float getSamplingRate();

      /// Possible voltage ranges
      enum _voltageRange { RANGE_5Vpp=0, RANGE_10Vpp=1 };
      typedef enum _voltageRange voltageRange;

      /// Set voltage range
      void setVoltageRange(voltageRange voltageRange);

      /// Possible trigger types: PERIODIC is a periodic trigger based on an internal clock. USER will trigger on
      /// calling sendSoftwareTrigger(). EXTERNAL will trigger on external electrical signals.
      enum _trigger { PERIODIC, USER, EXTERNAL };
      typedef enum _trigger trigger;

      /// Select trigger mode. The meanung of the second argument depends on the selected trigger type. The PERIODIC
      /// trigger takes a float argument with the trigger frequency in Hz. The EXTERNAL trigger takes a int argument
      /// with the trigger input channel between 0 and 7. USER has no additional argument.
      /// Design note: We are using boost::any instead of classic overloads, since overloading int and float arguments
      /// will result in an ambiguous call.
      void setTriggerMode(trigger trigger, boost::any arg=boost::any());

      /// These functions are for the python bindings only (boost::any does not easily work from Python)
      void setTriggerModePy1(trigger trigger)
      {
        setTriggerMode(trigger);
      }
      void setTriggerModePy2(trigger trigger, int channel)
      {
        setTriggerMode(trigger, channel);
      }
      void setTriggerModePy3(trigger trigger, float frequency)
      {
        setTriggerMode(trigger, frequency);
      }

      /// Enable (or disable) the DAQ. When the device is freshly opened, the DAQ is disabled. While the DAQ has been
      /// enabled, data will be read from the ADC chips and written to the buffers. Even in user trigger mode, there is
      /// no need to send a trigger right after the DAQ has been enabled, as data taking will start immediately.
      void enableDaq(bool enable=true);

      /// Send a user trigger (if USER trigger type was selected). This function will block until the trigger was
      /// acknowledged by the hardware, with a timeout of 10ms, which will throw an ad16Exception::TIMEOUT
      /// Note: the effect of the trigger will be to swap the buffers and start filling the new buffer. Since read()
      /// will always read the currently inactive buffer, one must send a trigger right before reading the data to get
      /// the data taken after the previous trigger.
      void sendUserTrigger();

      /// Tests if conversion is complete (returns true) or currently running (returns false)
      bool conversionComplete();

      /// Read all channels from AD16 buffer into software buffer
      /// Note: since the AD16 uses double buffering and the buffer will swap on trigger, this function will read the
      /// data acquired until the last received trigger. Thus, in case of the USER trigger, it is important to send
      /// another trigger before reading the data, even if no new data needs to be taken.
      void read();

      /// Get data for single channel after a previous read()
      std::vector<int32_t> getChannelData(unsigned int channel);

#ifdef ENABLE_PYTHON_BINDINGS

      /// Get data for single channel after a previous read(). Version for Python with numpy array
      void getChannelDataNumpy(unsigned int channel, boost::python::numeric::array &numpyArray);

#endif

    protected:

      /// flag if already opened
      bool _isOpen;

      /// register map
      boost::shared_ptr<RegisterInfoMap> _map;

      /// our mapped device
      boost::shared_ptr<Device> _mappedDevice;

      /// pointer to dummy device (if used)
      boost::shared_ptr<ad16DummyDevice> _dummyDevice;

      /// pointer to real device (if used)
      boost::shared_ptr<PcieBackend> _realDevice;

      /// accessor for multiplexed data
      boost::shared_ptr< mtca4u::MultiplexedDataAccessor<int32_t> > _dataDemuxed;

      /// number of samples per conversion block and channel
      const static int32_t _samplesPerBlock;

      /// Number of channels
      const static int32_t _numberOfChannels;

      /// Number of trigger channels (8 inputs + 1 software trigger)
      const static int32_t _numberOfTriggers;

      /// mode of operation
      int _mode;

      /// last buffer read (used to detect completed conversion in case of double buffering)
      int _currentBuffer;

      /// actual sampling frequency
      float _samplingFrequency;

      /// start time of last conversion
      boost::posix_time::ptime t0;

      /// AD16 application clock frequency WORD_CLK_FREQ[]
      ///  [0] is the main clock, [1] is the SPI clock frequency in Hz
      int clock_frequency[2];

      /// Table of conversion times, needed to select the right read mode. The conversion time depends on the selected
      /// oversampling ratio, the index of this table it the value of the enum oversamplingRatio. The times are in
      /// microseconds.
      const static float _conversionTimes[8];


  };

}

#endif /* INCLUDE_AD16_H_ */
