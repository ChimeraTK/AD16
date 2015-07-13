#include "ad16DummyDevice.h"
#include <boost/bind.hpp>
#include <boost/random/uniform_int.hpp>

namespace mtca4u{

  /// Nothing to set up
  ad16DummyDevice::ad16DummyDevice(){
  }

  // Nothing to clean up, all objects clean up for themselves when
  // they go out of scope.
  ad16DummyDevice::~ad16DummyDevice(){
  }

  // on device open: register callback functions
  void ad16DummyDevice::openDev (const std::string &mappingFileName, int perm, devConfigBase *pConfig) {

    // call parent class open device first
    DummyDevice::openDev (mappingFileName, perm, pConfig);

    // register callback function for control register write
    mapFile::mapElem    elem;
    _registerMapping->getRegisterInfo("START_CONVERSION", elem, "AD16");
    setWriteCallbackFunction( AddressRange(elem.reg_address, elem.reg_size, elem.reg_bar),
        boost::bind( &ad16DummyDevice::callbackControlRegister, this ) );
  }

  // callback for control register writes
  void ad16DummyDevice::callbackControlRegister(){

    // write white noise to ADC data block
    mapFile::mapElem    elem;
    _registerMapping->getRegisterInfo("AREA_MULTIPLEXED_SEQUENCE_DMA", elem, "AD16");

    boost::uniform_int<> uniform(0, (1<<18) - 1);    // 18 bit random number

    int ic=0;
    for(unsigned int i=0; i<elem.reg_elem_nr; i++){
      int32_t data;
      if(i % 16 == 0) {
        data = ic++;
      }
      else {
        data = uniform(rng);
      }
      writeRegisterWithoutCallback (elem.reg_address + i*sizeof(int32_t), data, elem.reg_bar);
    }
  }


}
