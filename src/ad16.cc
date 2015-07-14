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
    samplesPerBlock = samples;
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

    // Create accessor for multiplexed data
    // This contains currently an ugly work-around to reduce the amount of read data to what is needed. The number of
    // samples set by the user is used to reduce the length of the area read via DMA. This is currently not possible
    // with getCustomAccessor(), so we need to create the accessor manually.
    // TODO: It currently contains hardcoded information which should be obtained from the map file!
    SequenceInfo areaInfo;
    _mappedDevice.getRegisterMap()->getRegisterInfo("AREA_MULTIPLEXED_SEQUENCE_DMA",areaInfo,"AD16");
    std::vector< FixedPointConverter > converters;
    for(int i=0; i<16; i++) converters.push_back(FixedPointConverter(18,0,true));
    if(areaInfo.reg_elem_nr > 16*samplesPerBlock) areaInfo.reg_elem_nr = 16*samplesPerBlock;
    areaInfo.reg_size = 4*areaInfo.reg_elem_nr;
    dataDemuxed = boost::shared_ptr< FixedTypeMuxedDataAccessor< int32_t, int32_t > > (
        new FixedTypeMuxedDataAccessor< int32_t, int32_t >(_dummyDevice,areaInfo,converters) );

    // this was the original command which does not allow the flexibility to read only needed data
    // dataDemuxed = _mappedDevice.getCustomAccessor< MultiplexedDataAccessor<int32_t> >("DMA", "AD16");

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
