#include "ad16.h"
#include <boost/utility/binary.hpp>
#include <MtcaMappedDevice/DummyDevice.h>
#include <MtcaMappedDevice/mapFileParser.h>
#include <MtcaMappedDevice/mapFile.h>
#include <mtca4upy/HelperFunctions.h>
#include <cstring>

namespace mtca4u{

  // initialise constants
  const int32_t ad16::_numberOfChannels = 16;
  const int32_t ad16::_samplesPerBlock = 65536;
  const int32_t ad16::_numberOfTriggers = 9;
  const float ad16::_conversionTimes[8] = { 4.15, 9.1, 18.8, 39, 78, 158, 315 }; // in usecs

  /*************************************************************************************************/
  void ad16::open(const std::string &deviceFileName, const std::string &mappingFileName) {

    // check if already opened
    if(_isOpen) throw ad16Exception("Device already opened.",ad16Exception::ALREADY_OPENED);

    // open register map
    _map = mapFileParser().parse(mappingFileName);

    // open dummy device
    if(deviceFileName == mappingFileName){

      // create the dummy device driver
      _dummyDevice = boost::shared_ptr<ad16DummyDevice>( new ad16DummyDevice );
      _dummyDevice->openDev(mappingFileName);

      // create mapped device
      _mappedDevice = boost::shared_ptr< devMap<devBase> >( new devMap<devBase> );
      _mappedDevice->openDev( _dummyDevice, _map);
    }
    // open real device
    else {

      // create the PCIe device driver
      _realDevice = boost::shared_ptr<devPCIE>( new devPCIE );
      _realDevice->openDev(deviceFileName);

      // create mapped device
      _mappedDevice = boost::shared_ptr< devMap<devBase> >( new devMap<devBase> );
      _mappedDevice->openDev( _realDevice, _map);
    }

    // set open flag. Needs to be done before initialisation, as we are using some functions testing this flag in the
    // initialisation sequence
    _isOpen = true;

    // Initialise device. Procedure taken from Matlab code "ad16_init.m" by Lukasz Butkowski

    // send reset
    _mappedDevice->getRegisterAccessor("WORD_RESET_N","BOARD0")->write(1);
    _mappedDevice->getRegisterAccessor("WORD_AD16_ENA","AD160")->write(0);

    // read clock frequency
    _mappedDevice->getRegisterAccessor("WORD_CLK_FREQ","AD160")->read(clock_frequency, 2);

    // set default sampling frequency to 100kHz and oversampling ratio of 2
    setSamplingRate(100000, RATIO_2);

    // set default voltage range
    setVoltageRange(RANGE_10Vpp);

    // enable the module
    _mappedDevice->getRegisterAccessor("WORD_AD16_ENA","AD160")->write(1);

    // initialise frequency dividers for triggers with default values
    int32_t timing_freq[_numberOfTriggers] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    _mappedDevice->getRegisterAccessor("WORD_TIMING_FREQ","APP0")->write(timing_freq,_numberOfTriggers);

    // disable all internal triggers
    _mappedDevice->getRegisterAccessor("WORD_TIMING_INT_ENA","APP0")->write(BOOST_BINARY(0));

    // select trigger for starting a new block (trigger 8, which is the user trigger)
    _mappedDevice->getRegisterAccessor("WORD_TIMING_TRG_SEL","APP0")->write(8);

    // disable the DAQ (probably unnecessary, as it should be disabled after reset)
    //_mappedDevice->getRegisterAccessor("WORD_DAQ_ENABLE","APP0")->write(0);

    // select source of strobe (= trigger of a single sample) in DAQ
    // note that the DAQ might run asynchronously with the ADCs, depending on this setting.
    // 0-ADCA done, 1-ADC2 done , 2-trigger[6](counting from 0)
    _mappedDevice->getRegisterAccessor("WORD_DAQ_STR_SEL","APP0")->write(0);

    // reset all FSM after configuration
    _mappedDevice->getRegisterAccessor("WORD_RESET_N","BOARD0")->write(0);
    _mappedDevice->getRegisterAccessor("WORD_RESET_N","BOARD0")->write(1);

    // read current buffer to have the local variable in sync with the hardware
    _mappedDevice->getRegisterAccessor("WORD_DAQ_CURR_BUF","APP0")->read(&_currentBuffer);
  }

  /*************************************************************************************************/
  void ad16::close() {

    // check if opened
    if(!_isOpen) throw ad16Exception("Device not opened.",ad16Exception::NOT_OPENED);

    // close device
    _mappedDevice->closeDev();
    _isOpen = false;
  }

  /*************************************************************************************************/
  void ad16::setSamplingRate(float rate, oversampling oversampling) {

    // check if opened
    if(!_isOpen) throw ad16Exception("Device not opened.",ad16Exception::NOT_OPENED);

    // check if rate exceeds specification
    if(rate > 100000.) throw ad16Exception("Sampling rate set too high.",ad16Exception::IMPOSSIBLE_TIMING_CONFIGURATION);

    // compute timing dividers and actual adc frequencies
    int tdiv = (clock_frequency[0]/rate)-1.;
    _samplingFrequency = clock_frequency[0]/(tdiv+1);

    // select correct read mode
    int32_t readMode = 1;
    double tcycle = 1./_samplingFrequency * 1.e6;
    double tread = 74./clock_frequency[1] * 1.e6;
    double tconv = _conversionTimes[oversampling];
    double tdelay = 0;          // TODO
    if(tcycle <= tdelay+tconv || tconv <= tread) {
      readMode = 0;
      if(tcycle <= tdelay+tconv+tread) {
        throw ad16Exception("Impossible setting of sampling rate and/or oversampling ratio.",ad16Exception::IMPOSSIBLE_TIMING_CONFIGURATION);
      }
    }

    // set ADCA rate
    _mappedDevice->getRegisterAccessor("WORD_ADC_A_READ_MODE","AD160")->write(readMode);
    _mappedDevice->getRegisterAccessor("WORD_ADC_A_TIMING_DIV","AD160")->write(tdiv);
    _mappedDevice->getRegisterAccessor("WORD_ADC_A_OVERSAMPLING","AD160")->write((int)oversampling);

    // set ADCB rate
    _mappedDevice->getRegisterAccessor("WORD_ADC_B_READ_MODE","AD160")->write(readMode);
    _mappedDevice->getRegisterAccessor("WORD_ADC_B_TIMING_DIV","AD160")->write(tdiv);
    _mappedDevice->getRegisterAccessor("WORD_ADC_B_OVERSAMPLING","AD160")->write((int)oversampling);


  }

  /*************************************************************************************************/
  float ad16::getSamplingRate() {
    return _samplingFrequency;
  }

  /*************************************************************************************************/
  void ad16::setVoltageRange(voltageRange voltageRange) {

    // check if opened
    if(!_isOpen) throw ad16Exception("Device not opened.",ad16Exception::NOT_OPENED);

    // set ADCA range
    int32_t val = voltageRange;
    _mappedDevice->getRegisterAccessor("WORD_ADC_A_VOL_RANGE","AD160")->write(val);
    _mappedDevice->getRegisterAccessor("WORD_ADC_B_VOL_RANGE","AD160")->write(val);
  }

  /*************************************************************************************************/
  void ad16::setTriggerMode(trigger trigger, boost::any arg) {
    if(trigger == USER) {

      // disable internal trigger 8 (counting from 0) without changing the others
      int trigs;
      _mappedDevice->getRegisterAccessor("WORD_TIMING_INT_ENA","APP0")->read(&trigs);
      trigs &= 0xFF;
      _mappedDevice->getRegisterAccessor("WORD_TIMING_INT_ENA","APP0")->write(trigs);

      // select trigger for starting a new block (trigger 8, which is the user trigger)
      _mappedDevice->getRegisterAccessor("WORD_TIMING_TRG_SEL","APP0")->write(8);

    }
    else if(trigger == EXTERNAL) {

      // get channel number from 2nd argument
      int channel;
      try {
        channel = boost::any_cast<int>(arg);
      }
      catch(boost::bad_any_cast &e) {
        throw ad16Exception("ad16::setTriggerType(): second argument missing or of wrong type for EXTERNAL trigger mode.",ad16Exception::ILLEGAL_PARAMETER);
      }

      // check if channel in range
      if(channel < 0 || channel > 7) {
        throw ad16Exception("Trigger channel out of range",ad16Exception::INCORRECT_TRIGGER_SETTING);
      }

      // disable internal trigger on selected channel number (and on channel 8)
      int trigs;
      _mappedDevice->getRegisterAccessor("WORD_TIMING_INT_ENA","APP0")->read(&trigs);
      trigs &= 0xFF ^ (1 << channel);
      _mappedDevice->getRegisterAccessor("WORD_TIMING_INT_ENA","APP0")->write(trigs);

      // select trigger for starting a new block (trigger 8, which is the user trigger)
      _mappedDevice->getRegisterAccessor("WORD_TIMING_TRG_SEL","APP0")->write(channel);

    }
    else if(trigger == PERIODIC) {

      // get frequency from 2nd argument.
      // We want to have some type tolerance, so we try to case to double, float and int.
      float freq;
      try {
        freq = boost::any_cast<double>(arg);
      }
      catch(boost::bad_any_cast &e) {
        try {
          freq = boost::any_cast<float>(arg);
        }
        catch(boost::bad_any_cast &e) {
          try {
            freq = boost::any_cast<int>(arg);
          }
          catch(boost::bad_any_cast &e) {
            throw ad16Exception("ad16::setTriggerType(): second argument missing or of wrong type for PERIODIC trigger mode.",ad16Exception::ILLEGAL_PARAMETER);
          }
        }
      }

      // compute frequency divider
      int div = round(clock_frequency[0]/freq) - 1;

      // update frequency divider for trigger 0
      int32_t timing_freq[_numberOfTriggers];
      _mappedDevice->getRegisterAccessor("WORD_TIMING_FREQ","APP0")->read(timing_freq,_numberOfTriggers);
      timing_freq[0] = div;
      _mappedDevice->getRegisterAccessor("WORD_TIMING_FREQ","APP0")->write(timing_freq,_numberOfTriggers);

      // enable internal trigger 0
      int trigs;
      _mappedDevice->getRegisterAccessor("WORD_TIMING_INT_ENA","APP0")->read(&trigs);
      trigs |= 1;
      _mappedDevice->getRegisterAccessor("WORD_TIMING_INT_ENA","APP0")->write(trigs);

      // select trigger 0
      _mappedDevice->getRegisterAccessor("WORD_TIMING_TRG_SEL","APP0")->write(0);

    }

  }

  /*************************************************************************************************/
  void ad16::enableDaq(bool enable) {
    int32_t val = 1;
    if(!enable) val = 0;
    _mappedDevice->getRegisterAccessor("WORD_DAQ_ENABLE","APP0")->write(val);
  }

  /*************************************************************************************************/
  void ad16::sendUserTrigger() {

    // check if opened
    if(!_isOpen) throw ad16Exception("Device not opened.",ad16Exception::NOT_OPENED);

    // read current buffer
    _mappedDevice->getRegisterAccessor("WORD_DAQ_CURR_BUF","APP0")->read(&_currentBuffer);

    // save trigger time
    t0 = boost::posix_time::microsec_clock::local_time();

    // write to trigger register
    _mappedDevice->getRegisterAccessor("WORD_TIMING_USER_TRG","APP0")->write(1);

    // wait until current buffer is updated by hardware
    int32_t currentBuffer;
    do {
      _mappedDevice->getRegisterAccessor("WORD_DAQ_CURR_BUF","APP0")->read(&currentBuffer);
      if(boost::posix_time::microsec_clock::local_time()-t0 > boost::posix_time::milliseconds(10)) {
        throw ad16Exception("Timeout sending user trigger.",ad16Exception::TIMEOUT);
      }
    } while (currentBuffer == _currentBuffer);
    _currentBuffer = currentBuffer;
}

  /*************************************************************************************************/
  bool ad16::conversionComplete() {

    // check if opened
    if(!_isOpen) throw ad16Exception("Device not opened.",ad16Exception::NOT_OPENED);

    // check if current buffer was changed
    int32_t currentBuffer;
    _mappedDevice->getRegisterAccessor("WORD_DAQ_CURR_BUF","APP0")->read(&currentBuffer);
    return (currentBuffer != _currentBuffer);

    // instead check timing (assume a conversion takes 1 second at most)
    boost::posix_time::ptime t1(boost::posix_time::microsec_clock::local_time());
    boost::posix_time::time_duration dt = t1-t0;
    if(dt.total_milliseconds() < 1000) return false;
    return true;
  }

  /*************************************************************************************************/
  void ad16::read() {

    // check if opened
    if(!_isOpen) throw ad16Exception("Device not opened.",ad16Exception::NOT_OPENED);

    // determine buffer to read from (which is not the "current" buffer the AD16 currently writes to)
    _mappedDevice->getRegisterAccessor("WORD_DAQ_CURR_BUF","APP0")->read(&_currentBuffer);
    std::string bufferName;
    if(_currentBuffer == 1) {           // hardware currently writing to buffer B
      bufferName = "DAQ0_ADCA";
    }
    else {                              // hardware currently writing to buffer A
      bufferName = "DAQ0_ADCB";
    }

    // modify register map so the DMA area has the right length
    for(mapFile::iterator i = _map->begin(); i != _map->end(); ++i) {
      if(i->reg_module == "APP0" && i->reg_name == "AREA_MULTIPLEXED_SEQUENCE_"+bufferName) {
        i->reg_elem_nr = _numberOfChannels*_samplesPerBlock;
        i->reg_size = sizeof(int32_t)*i->reg_elem_nr;
      }
    }

    // create custom accessor
    _dataDemuxed = _mappedDevice->getCustomAccessor< MultiplexedDataAccessor<int32_t> >(bufferName, "APP0");

    // read data
    _dataDemuxed->read();
  }

  /*************************************************************************************************/
  std::vector<int32_t> ad16::getChannelData(unsigned int channel) {

    // check if opened
    if(!_isOpen) throw ad16Exception("Device not opened.",ad16Exception::NOT_OPENED);

    // check if read() has been called already
    if(!_dataDemuxed) throw ad16Exception("No data has been read so far. Call read() first!",ad16Exception::ILLEGAL_PARAMETER);

    // check if channel is in range
    if(channel >= _dataDemuxed->getNumberOfDataSequences()) {
      throw ad16Exception("Channel number out of range",ad16Exception::CHANNEL_OUT_OF_RANGE);
    }

    // return demultiplexed data
    return (*_dataDemuxed)[channel];

  }

  /*************************************************************************************************/
  void ad16::getChannelDataNumpy(unsigned int channel, boost::python::numeric::array &numpyArray) {

    // check if opened
    if(!_isOpen) throw ad16Exception("Device not opened.",ad16Exception::NOT_OPENED);

    // check for correct data type
    if(mtca4upy::extractDataType(numpyArray) == mtca4upy::INT32) {
      // copy data to python array
      int* dataLocation = reinterpret_cast<int*>(mtca4upy::extractDataPointer(numpyArray));
      memcpy( dataLocation, (*_dataDemuxed)[channel].data(), (*_dataDemuxed)[channel].size() * sizeof(int));

    }
    else {
      throw mtca4upy::ArrayElementTypeNotSupported("getChannelData(): Incorrect data type found in numpy array.");
    }

  }

} // namespace mtca4u
