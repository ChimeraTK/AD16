#include "ad16.h"
#include <MtcaMappedDevice/DummyDevice.h>
#include <MtcaMappedDevice/mapFileParser.h>

namespace mtca4u{

  /*************************************************************************************************/
  void ad16::open(const std::string &deviceFileName, const std::string &mappingFileName){

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
  void ad16::close(){
    _mappedDevice.closeDev();
    _dummyDevice->closeDev();
  }

  /*************************************************************************************************/
  void ad16::startConversion(){
    int32_t val = 1;
    _mappedDevice.writeReg("START_CONVERSION", "AD16", &val);
  }

  /*************************************************************************************************/
  void ad16::read(){
    // if not yet done, create accessor for multiplexed data
    std::cout << "ad16::read()" << std::endl;
    if(!dataDemuxed) dataDemuxed = _mappedDevice.getCustomAccessor< mtca4u::MultiplexedDataAccessor<int32_t> >("DMA", "AD16");

    // read data
    dataDemuxed->read();
  }

  /*************************************************************************************************/
  std::vector<int> ad16::getChannelData(unsigned int channel){

    // check if channel is in range
    if(channel > dataDemuxed->getNumberOfDataSequences()){
      throw ad16Exception("Real devices currently not supported",ad16Exception::ILLEGAL_PARAMETER);
    }

    // return demultiplexed data
    return (*dataDemuxed)[channel];

  }

} // namespace mtca4u
