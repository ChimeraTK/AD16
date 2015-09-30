#include <cstring>
#include <boost/utility/binary.hpp>

#include <mtca4u/PcieBackendException.h>

#include "ad16.h"

#ifdef ENABLE_PYTHON_BINDINGS
#include <mtca4upy/HelperFunctions.h>
#endif

namespace mtca4u {

  // initialise constants
  const int32_t ad16::_numberOfChannels = 16;
  const int32_t ad16::_samplesPerBlock = 65536;
  const int32_t ad16::_numberOfTriggers = 9;
  const float ad16::_conversionTimes[8] = { 4.15, 9.1, 18.8, 39, 78, 158, 315 }; // in usecs

  /*************************************************************************************************/
  void ad16::open(const std::string &deviceAlias) {

    // check if already opened
    if(_isOpen) throw ad16Exception("Device already opened.",ad16Exception::ALREADY_OPENED);

    // open the device
    _device = boost::shared_ptr<Device>(new Device);
    _device->open(deviceAlias);

    // set open flag. Needs to be done before initialisation, as we are using some functions testing this flag in the
    // initialisation sequence
    _isOpen = true;

    // open register accessors
    regReset = _device->getBufferingRegisterAccessor<int>("BOARD0","WORD_RESET_N");
    regAd16Enable = _device->getBufferingRegisterAccessor<int>("AD160","WORD_AD16_ENA");
    regClockFrequency = _device->getBufferingRegisterAccessor<int>("AD160","WORD_CLK_FREQ");
    regTimingFrequency = _device->getBufferingRegisterAccessor<int>("APP0","WORD_TIMING_FREQ");
    regTimingInternalEnable = _device->getBufferingRegisterAccessor<int>("APP0","WORD_TIMING_INT_ENA");
    regTimingTriggerSelect = _device->getBufferingRegisterAccessor<int>("APP0","WORD_TIMING_TRG_SEL");
    regDaqEnable = _device->getBufferingRegisterAccessor<int>("APP0","WORD_DAQ_ENABLE");
    regStrobeSelect = _device->getBufferingRegisterAccessor<int>("APP0","WORD_DAQ_STR_SEL");
    regCurrentBuffer = _device->getBufferingRegisterAccessor<int>("APP0","WORD_DAQ_CURR_BUF");
    regReadModeA = _device->getBufferingRegisterAccessor<int>("AD160","WORD_ADC_A_READ_MODE");
    regReadModeB = _device->getBufferingRegisterAccessor<int>("AD160","WORD_ADC_B_READ_MODE");
    regTimingDivA = _device->getBufferingRegisterAccessor<int>("AD160","WORD_ADC_A_TIMING_DIV");
    regTimingDivB = _device->getBufferingRegisterAccessor<int>("AD160","WORD_ADC_B_TIMING_DIV");
    regOversamplingA = _device->getBufferingRegisterAccessor<int>("AD160","WORD_ADC_A_OVERSAMPLING");
    regOversamplingB = _device->getBufferingRegisterAccessor<int>("AD160","WORD_ADC_B_OVERSAMPLING");
    regVoltageRangeA = _device->getBufferingRegisterAccessor<int>("AD160","WORD_ADC_A_VOL_RANGE");
    regVoltageRangeB = _device->getBufferingRegisterAccessor<int>("AD160","WORD_ADC_B_VOL_RANGE");
    regTimingUserTrigger = _device->getBufferingRegisterAccessor<int>("APP0","WORD_TIMING_USER_TRG");


    // Initialise device. Procedure taken from Matlab code "ad16_init.m" by Lukasz Butkowski
    // send reset
    try {
      regReset = 1; regReset.write();
      regAd16Enable = 0; regAd16Enable.write();
    }
    catch(std::exception &e) {
      throw ad16Exception(e.what() ,ad16Exception::CANNOT_OPEN);
    }

    // read clock frequency
    regClockFrequency.read();

    // safety-check: clock frequency must be at a reasonable value.
    if(regClockFrequency[0] < 1000 || regClockFrequency[1] < 1000) {
      throw ad16Exception("Device reports unreasonable base clock frequencies.", ad16Exception::CANNOT_OPEN);
    }

    // set default sampling frequency to 100kHz and oversampling ratio of 2
    setSamplingRate(100000, RATIO_2);

    // set default voltage range
    setVoltageRange(RANGE_10Vpp);

    // enable the module
    regAd16Enable = 1; regAd16Enable.write();

    // initialise frequency dividers for triggers with default values
    for(int i=0; i<_numberOfTriggers; i++) regTimingFrequency[i] = 0;
    regTimingFrequency.write();

    // disable all internal triggers
    regTimingInternalEnable = 0; regTimingInternalEnable.write();

    // select trigger for starting a new block (trigger 8, which is the user trigger)
    regTimingTriggerSelect = 8; regTimingTriggerSelect.write();

    // disable the DAQ (probably unnecessary, as it should be disabled after reset)
    regDaqEnable = 0; regDaqEnable.write();

    // select source of strobe (= trigger of a single sample) in DAQ
    // note that the DAQ might run asynchronously with the ADCs, depending on this setting.
    // 0-ADCA done, 1-ADC2 done , 2-trigger[6](counting from 0)
    regStrobeSelect = 0; regStrobeSelect.write();

    // reset all FSM after configuration
    regReset = 0; regReset.write();
    regReset = 1; regReset.write();

    // read current buffer to have the local variable in sync with the hardware
    regCurrentBuffer.read();
    _currentBuffer = regCurrentBuffer;
  }

  /*************************************************************************************************/
  void ad16::close() {

    // check if opened
    if(!_isOpen) throw ad16Exception("Device not opened.",ad16Exception::NOT_OPENED);

    // close device
    _device->close();
    _isOpen = false;
  }

  /*************************************************************************************************/
  void ad16::setSamplingRate(float rate, oversampling oversampling) {

    // check if opened
    if(!_isOpen) throw ad16Exception("Device not opened.",ad16Exception::NOT_OPENED);

    // check if rate exceeds specification
    if(rate > 100000.) throw ad16Exception("Sampling rate set too high.",ad16Exception::IMPOSSIBLE_TIMING_CONFIGURATION);

    // compute timing dividers and actual adc frequencies
    int tdiv = (regClockFrequency[0]/rate)-1.;
    _samplingFrequency = regClockFrequency[0]/(tdiv+1);

    // select correct read mode
    int32_t readMode = 1;
    double tcycle = 1./_samplingFrequency * 1.e6;
    double tread = 74./regClockFrequency[1] * 1.e6;
    double tconv = _conversionTimes[oversampling];
    double tdelay = 0;
    if(tcycle <= tdelay+tconv || tconv <= tread) {
      readMode = 0;
      if(tcycle <= tdelay+tconv+tread) {
        throw ad16Exception("Impossible setting of sampling rate and/or oversampling ratio.",ad16Exception::IMPOSSIBLE_TIMING_CONFIGURATION);
      }
    }

    // set ADCA rate
    regReadModeA = readMode; regReadModeA.write();
    regTimingDivA = tdiv; regTimingDivA.write();
    regOversamplingA = static_cast<int>(oversampling); regOversamplingA.write();

    // set ADCB rate
    regReadModeB = readMode; regReadModeB.write();
    regTimingDivB = tdiv; regTimingDivB.write();
    regOversamplingB = static_cast<int>(oversampling); regOversamplingB.write();


  }

  /*************************************************************************************************/
  float ad16::getSamplingRate() {
    return _samplingFrequency;
  }

  /*************************************************************************************************/
  void ad16::setVoltageRange(voltageRange voltageRange) {

    // check if opened
    if(!_isOpen) throw ad16Exception("Device not opened.",ad16Exception::NOT_OPENED);

    // set ADCA and ADCB range
    regVoltageRangeA = static_cast<int>(voltageRange);
    regVoltageRangeB = static_cast<int>(voltageRange);
  }

  /*************************************************************************************************/
  void ad16::setTriggerMode(trigger trigger, boost::any arg) {

    // check if opened
    if(!_isOpen) throw ad16Exception("Device not opened.",ad16Exception::NOT_OPENED);

    if(trigger == USER) {

      // disable internal trigger 8 (counting from 0) without changing the others
      regTimingInternalEnable.read();
      regTimingInternalEnable = regTimingInternalEnable & 0xFF;
      regTimingInternalEnable.write();

      // select trigger for starting a new block (trigger 8, which is the user trigger)
      regTimingTriggerSelect = 8; regTimingTriggerSelect.write();

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
      regTimingInternalEnable.read();
      regTimingInternalEnable = regTimingInternalEnable & ( 0xFF ^ (1 << channel) );
      regTimingInternalEnable.write();

      // select trigger for starting a new block (trigger 8, which is the user trigger)
      regTimingTriggerSelect = channel; regTimingTriggerSelect.write();

    }
    else if(trigger == PERIODIC) {

      // get frequency from 2nd argument.
      // We want to have some type tolerance, so we try to cast to double, float and int.
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
      int div = round(regClockFrequency[0]/freq) - 1;

      // update frequency divider for trigger 0
      regTimingFrequency.read();
      regTimingFrequency[0] = div;
      regTimingFrequency.write();

      // enable internal trigger 0
      regTimingInternalEnable.read();
      regTimingInternalEnable = regTimingInternalEnable | 1;
      regTimingInternalEnable.write();

      // select trigger 0
      regTimingTriggerSelect = 0; regTimingTriggerSelect.write();

    }

  }

  /*************************************************************************************************/
  void ad16::enableDaq(bool enable) {

    // check if opened
    if(!_isOpen) throw ad16Exception("Device not opened.",ad16Exception::NOT_OPENED);

    // enable or disable DAQ
    regDaqEnable = (enable ? 1 : 0); regDaqEnable.write();
  }

  /*************************************************************************************************/
  void ad16::sendUserTrigger() {

    // check if opened
    if(!_isOpen) throw ad16Exception("Device not opened.",ad16Exception::NOT_OPENED);

    // read current buffer
    regCurrentBuffer.read();
    _currentBuffer = regCurrentBuffer;

    // save trigger time
    t0 = boost::posix_time::microsec_clock::local_time();

    // write to trigger register
    regTimingUserTrigger = 1;
    regTimingUserTrigger.write();

    // wait until current buffer is updated by hardware
    do {
      regCurrentBuffer.read();
      if(boost::posix_time::microsec_clock::local_time()-t0 > boost::posix_time::milliseconds(10)) {
        throw ad16Exception("Timeout sending user trigger.",ad16Exception::TIMEOUT);
      }
    }
    while(regCurrentBuffer == _currentBuffer);
    _currentBuffer = regCurrentBuffer;
}

  /*************************************************************************************************/
  bool ad16::conversionComplete() {

    // check if opened
    if(!_isOpen) throw ad16Exception("Device not opened.",ad16Exception::NOT_OPENED);

    // check if current buffer was changed
    regCurrentBuffer.read();
    return (regCurrentBuffer[0] != _currentBuffer);
  }

  /*************************************************************************************************/
  void ad16::read() {

    // check if opened
    if(!_isOpen) throw ad16Exception("Device not opened.",ad16Exception::NOT_OPENED);

    // determine buffer to read from (which is not the "current" buffer the AD16 currently writes to)
    regCurrentBuffer.read();
    _currentBuffer = regCurrentBuffer;
    std::string bufferName;
    if(_currentBuffer == 1) {           // hardware currently writing to buffer B
      bufferName = "DAQ0_ADCA";
    }
    else {                              // hardware currently writing to buffer A
      bufferName = "DAQ0_ADCB";
    }

    // create custom accessor
    _dataDemuxed = _device->getCustomAccessor< MultiplexedDataAccessor<int32_t> >(bufferName, "APP0");

    // read data
    _dataDemuxed->read();
  }

  /*************************************************************************************************/
  std::vector<int32_t> ad16::getChannelData(unsigned int channel) {

    // check if opened
    if(!_isOpen) throw ad16Exception("Device not opened.",ad16Exception::NOT_OPENED);

    // check if read() has been called already
    if(!_dataDemuxed) throw ad16Exception("No data has been read so far. Call read() first!",ad16Exception::NO_DATA_AVAILABLE);

    // check if channel is in range
    if(channel >= _dataDemuxed->getNumberOfDataSequences()) {
      throw ad16Exception("Channel number out of range",ad16Exception::CHANNEL_OUT_OF_RANGE);
    }

    // return demultiplexed data
    return (*_dataDemuxed)[channel];

  }

#ifdef ENABLE_PYTHON_BINDINGS
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
#endif

} // namespace mtca4u
