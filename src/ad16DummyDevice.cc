#include "ad16DummyDevice.h"
#include <boost/bind.hpp>
#include <boost/random/uniform_int.hpp>

namespace mtca4u{

  // on device open: register callback functions
  void ad16DummyDevice::openDev(const std::string &mappingFileName, int perm, devConfigBase *pConfig) {

    // call parent class open device first
    DummyDevice::openDev (mappingFileName, perm, pConfig);

    // register callback function for control register write
    mapFile::mapElem elem;
    _registerMapping->getRegisterInfo("START_CONVERSION", elem, "AD16");
    setWriteCallbackFunction( AddressRange(elem.reg_address, elem.reg_size, elem.reg_bar),
        boost::bind( &ad16DummyDevice::callbackStartConversion, this ) );

    // set sensible default values
    _registerMapping->getRegisterInfo("SAMPLING_RATE_DIV", elem, "AD16");
    writeReg(elem.reg_address, 1, elem.reg_bar);

    _registerMapping->getRegisterInfo("SAMPLES_PER_BLOCK", elem, "AD16");
    writeReg(elem.reg_address, 1024, elem.reg_bar);
}

  // callback for control register writes
  void ad16DummyDevice::callbackStartConversion(){
    // check if conversion thread is already running
    if(isConversionRunning) {
      std::cout << "ad16DummyDevice::callbackStartConversion(): Conversion already running!" << std::endl;
      return;
    }
    isConversionRunning = true;

    // start new conversion
    theThread =  boost::thread( boost::bind(&ad16DummyDevice::threadConversion, this) );
}
  void ad16DummyDevice::threadConversion(){
    mapFile::mapElem elem;

    // wait time the real device would need for the conversion
    int32_t samplingRateDiv, samplesPerBlock;

    _registerMapping->getRegisterInfo("SAMPLING_RATE_DIV", elem, "AD16");
    readReg(elem.reg_address, &samplingRateDiv, elem.reg_bar);

    _registerMapping->getRegisterInfo("SAMPLES_PER_BLOCK", elem, "AD16");
    readReg(elem.reg_address, &samplesPerBlock, elem.reg_bar);

    usleep(10*samplingRateDiv*samplesPerBlock);

    // write white noise to ADC data block
    _registerMapping->getRegisterInfo("AREA_MULTIPLEXED_SEQUENCE_DMA", elem, "AD16");

    boost::uniform_int<> uniform(0, (1<<18) - 1);    // 18 bit random number

    int ic = 0;
    if(samplesPerBlock > elem.reg_elem_nr/16) samplesPerBlock = elem.reg_elem_nr/16;  // limit written samples to actual block size
    for(unsigned int i=0; i<16*samplesPerBlock; i++) {
      int32_t data;
      if(i % 16 == 0) {
        data = ic++;
      }
      else {
        data = uniform(rng);
      }
      writeRegisterWithoutCallback(elem.reg_address + i*sizeof(int32_t), data, elem.reg_bar);
    }

    // change START_CONERSION register back to 0
    _registerMapping->getRegisterInfo("START_CONVERSION", elem, "AD16");
    writeRegisterWithoutCallback(elem.reg_address, 0, elem.reg_bar);

    isConversionRunning = false;
  }


}
