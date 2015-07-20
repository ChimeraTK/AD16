#include "ad16.h"
#include <MtcaMappedDevice/DummyDevice.h>
#include <MtcaMappedDevice/mapFileParser.h>
#include <MtcaMappedDevice/mapFile.h>

namespace mtca4u{
  const int32_t ad16::numberOfChannels = 16;

  /*************************************************************************************************/
  void ad16::open(const std::string &deviceFileName, const std::string &mappingFileName) {

    // currently only dummy device allowed
    if(deviceFileName != mappingFileName){
      throw ad16Exception("Real devices currently not supported",ad16Exception::ILLEGAL_PARAMETER);
    }

    // open dummy device
    _dummyDevice = boost::shared_ptr<ad16DummyDevice>( new ad16DummyDevice );
    _dummyDevice->openDev(mappingFileName);

    // open map and store maximum number of elements
    _map = mapFileParser().parse(mappingFileName);
    mapFile::mapElem areaInfo;
    _map->getRegisterInfo("AREA_MULTIPLEXED_SEQUENCE_BUFFER",areaInfo,"BOARD0");
    max_elem_nr = areaInfo.reg_elem_nr;

    /* no double buffering supported by the current firmware...
    /// check if length of buffer B is the same
    _map->getRegisterInfo("AREA_MULTIPLEXED_SEQUENCE_BUFFER_B",areaInfo,"AD16");
    if(max_elem_nr != areaInfo.reg_elem_nr) {
      throw ad16Exception("Number of elements in BUFFER_A and BUFFER_B are not consistent in map file.",ad16Exception::ILLEGAL_PARAMETER);
    }
    */

    // create mapped device
    _mappedDevice.openDev( _dummyDevice, _map);

    // Initialise device. Procedure taken from Matlab code "ad16_init.m" by Lukasz Butkowski
    int32_t val;

    // send reset
    val = 0;
    _mappedDevice.writeReg("WORD_ADC_ENA","AD160",&val);
    val = 0;
    _mappedDevice.writeReg("WORD_RESET_N","BOARD0",&val);
    usleep(100000); // 0.1 seconds

    // set trigger to user trigger
    val = 8;
    _mappedDevice.writeReg("WORD_TIMING_TRG_SEL","APP0",&val);

    // enable data taking
    val = 1;
    _mappedDevice.writeReg("WORD_DAQ_ENABLE","APP0",&val);

    // complete initialisation
    val = 1;
    _mappedDevice.writeReg("WORD_RESET_N","BOARD0",&val);
    usleep(100000); // 0.1 seconds

    val = 1;
    _mappedDevice.writeReg("WORD_ADC_RESET","AD160",&val);
    usleep(1000000); // 1 seconds

    val = 0;
    _mappedDevice.writeReg("WORD_ADC_RESET","AD160",&val);
    usleep(1000000); // 1 seconds

    val = 1;
    _mappedDevice.writeReg("WORD_ADC_ENA","AD160",&val);

  }

  /*************************************************************************************************/
  void ad16::close() {
    _mappedDevice.closeDev();
    _dummyDevice->closeDev();
  }

  /*************************************************************************************************/
  void ad16::setMode(int mode) {
    throw ad16Exception("Trigger mode cannot be changed currently",ad16Exception::NOT_IMPLEMENTED);
    //_mode = mode;
    //_mappedDevice.writeReg("MODE", "AD16", &mode);
  }

  /*************************************************************************************************/
  void ad16::setSamplingRate(int divisor) {
    throw ad16Exception("Sample rate cannot be changed in current firmware",ad16Exception::NOT_IMPLEMENTED);
    //_mappedDevice.writeReg("SAMPLING_RATE_DIV", "AD16", &divisor);
  }

  /*************************************************************************************************/
  void ad16::setSamplesPerBlock(int samples) {
    throw ad16Exception("Samples per block cannot be changed in current firmware",ad16Exception::NOT_IMPLEMENTED);
    //_samplesPerBlock = samples;
    //_mappedDevice.writeReg("SAMPLES_PER_BLOCK", "AD16", &samples);
  }

  /*************************************************************************************************/
  void ad16::startConversion() {

    // check if conversion is already running
    if(!conversionComplete()) {
      throw ad16Exception("Conversion already running",ad16Exception::CONVERSION_RUNNING);
    }

    // save starting time
    t0 = boost::posix_time::microsec_clock::local_time();

    // write to trigger register
    int32_t val = 1;
    _mappedDevice.writeReg("WORD_TIMING_USER_TRG", "APP0", &val);
  }

  /*************************************************************************************************/
  bool ad16::conversionComplete() {

    /* currently not implemented in firmware
    if(_mode == 0 || _mode == 1) {
      // in case of single buffering, simply check the register
      int32_t val;
      _mappedDevice.readReg("CONVERSION_RUNNING", "AD16", &val);
      return (val == 0);
    }
    else {
      int32_t lastBuffer;
      _mappedDevice.readReg("LAST_BUFFER","AD16", &lastBuffer);
      return (lastBuffer != _lastBuffer);
    }
    */

    // instead check timing (assume a conversion takes 1 second at most)
    boost::posix_time::ptime t1(boost::posix_time::microsec_clock::local_time());
    boost::posix_time::time_duration dt = t1-t0;
    if(dt.total_milliseconds() < 1000) return false;
    return true;
  }

  /*************************************************************************************************/
  void ad16::read() {

    // check if conversion is still running
    if(!conversionComplete()) {
      throw ad16Exception("Conversion still running",ad16Exception::CONVERSION_RUNNING);
    }

    // determine last written buffer to read from
    /* no double buffering in current firmware
    _mappedDevice.readReg("LAST_BUFFER","AD16", &_lastBuffer);
    std::string bufferName;
    if(_lastBuffer == 0) {
      bufferName = "BUFFER_A";
    }
    else {
      bufferName = "BUFFER_B";
    }
    */
    std::string bufferName = "BUFFER";

    // modify register map so the DMA area has the right length
    for(mapFile::iterator i = _map->begin(); i != _map->end(); ++i) {
      if(i->reg_module == "BOARD0" && i->reg_name == "AREA_MULTIPLEXED_SEQUENCE_"+bufferName) {
        i->reg_elem_nr = numberOfChannels*_samplesPerBlock;
        i->reg_size = sizeof(int32_t)*i->reg_elem_nr;
      }
    }

    // create custom accessor
    _dataDemuxed = _mappedDevice.getCustomAccessor< MultiplexedDataAccessor<int32_t> >(bufferName, "BOARD0");

    // read data
    _dataDemuxed->read();
  }

  /*************************************************************************************************/
  std::vector<int> ad16::getChannelData(unsigned int channel) {

    // check if channel is in range
    if(channel > _dataDemuxed->getNumberOfDataSequences()) {
      throw ad16Exception("Channel number out of range",ad16Exception::CHANNEL_OUT_OF_RANGE);
    }

    // return demultiplexed data
    return (*_dataDemuxed)[channel];

  }

} // namespace mtca4u
