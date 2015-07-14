#include "ad16.h"
#include <MtcaMappedDevice/DummyDevice.h>
#include <MtcaMappedDevice/mapFileParser.h>

namespace mtca4u{

  /*************************************************************************************************/
  void ad16::open(const std::string &deviceFileName, const std::string &mappingFileName) {

    // currently only dummy device allowed
    if(deviceFileName != mappingFileName){
      throw ad16Exception("Real devices currently not supported",ad16Exception::ILLEGAL_PARAMETER);
    }

    // open dummy device and put into mapped device
    _dummyDevice = boost::shared_ptr<ad16DummyDevice>( new ad16DummyDevice );
    _dummyDevice->openDev(mappingFileName);
    _mappedDevice.openDev( _dummyDevice, mapFileParser().parse(mappingFileName));
  }

  /*************************************************************************************************/
  void ad16::close() {
    _mappedDevice.closeDev();
    _dummyDevice->closeDev();
  }

  /*************************************************************************************************/
  void ad16::setSamplingRate(int divisor) {
    _mappedDevice.writeReg("SAMPLING_RATE_DIV", "AD16", &divisor);
  }

  /*************************************************************************************************/
  void ad16::setSamplesPerBlock(int samples) {
    _mappedDevice.writeReg("SAMPLES_PER_BLOCK", "AD16", &samples);
  }

  /*************************************************************************************************/
  void ad16::startConversion() {

    // check if conversion is already running
    if(!conversionComplete()) {
      throw ad16Exception("Conversion already running",ad16Exception::CONVERSION_RUNNING);
    }

    // write to trigger register
    int32_t val = 1;
    _mappedDevice.writeReg("START_CONVERSION", "AD16", &val);
  }

  /*************************************************************************************************/
  bool ad16::conversionComplete() {
    // check if conversion is currently running
    int32_t val;
    _mappedDevice.readReg("START_CONVERSION", "AD16", &val);
    return (val == 0);
  }

  /*************************************************************************************************/
  void ad16::read() {

    // check if conversion is still running
    if(!conversionComplete()) {
      throw ad16Exception("Conversion still running",ad16Exception::CONVERSION_RUNNING);
    }

    // if not yet done, create accessor for multiplexed data
    if(!dataDemuxed) dataDemuxed = _mappedDevice.getCustomAccessor< mtca4u::MultiplexedDataAccessor<int32_t> >("DMA", "AD16");

    // read data
    dataDemuxed->read();
  }

  /*************************************************************************************************/
  std::vector<int> ad16::getChannelData(unsigned int channel) {

    // check if channel is in range
    if(channel > dataDemuxed->getNumberOfDataSequences()){
      throw ad16Exception("Channel number out of range",ad16Exception::CHANNEL_OUT_OF_RANGE);
    }

    // return demultiplexed data
    return (*dataDemuxed)[channel];

  }

} // namespace mtca4u
