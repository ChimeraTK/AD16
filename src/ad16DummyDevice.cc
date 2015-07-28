#include "ad16DummyDevice.h"
#include <boost/bind.hpp>
#include <boost/random/uniform_int.hpp>

namespace mtca4u {
  const int32_t ad16DummyDevice::numberOfChannels = 16;

  // on device open: register callback functions
  void ad16DummyDevice::openDev(const std::string &mappingFileName, int perm, devConfigBase *pConfig) {

    // call parent class open device first
    DummyDevice::openDev (mappingFileName, perm, pConfig);

    // register callback function for control register write
    mapFile::mapElem elem;
    _registerMapping->getRegisterInfo("WORD_TIMING_USER_TRG", elem, "APP0");
    setWriteCallbackFunction( AddressRange(elem.reg_address, elem.reg_size, elem.reg_bar),
        boost::bind( &ad16DummyDevice::callbackStartConversion, this ) );

    /*    // set sensible default values
    _registerMapping->getRegisterInfo("SAMPLING_RATE_DIV", elem, "AD16");
    writeReg(elem.reg_address, 1, elem.reg_bar);

    _registerMapping->getRegisterInfo("SAMPLES_PER_BLOCK", elem, "AD16");
    writeReg(elem.reg_address, 1024, elem.reg_bar);*/

    isOpened = true;
  }

  // close device: clean up
  void ad16DummyDevice::closeDev() {

    // interrupt conversion thread
    if(isOpened) {
      theThread.interrupt();
      theThread.join();
    }

    // call parent class close device to clean up
    DummyDevice::closeDev();

    isOpened = false;
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

  // thread to simulate the conversion
  void ad16DummyDevice::threadConversion() {

    // measure start time (for auto trigger frequency)
    boost::posix_time::ptime t0(boost::posix_time::microsec_clock::local_time());

    // The conversion is done in this loop to handle multiple triggers. The loop is stopped after the first iteration
    // in case of the software/user trigger.
    while(true) {
      mapFile::mapElem elem;

      // obtain selected trigger
      int32_t mode;
      _registerMapping->getRegisterInfo("WORD_TIMING_TRG_SEL", elem, "APP0");
      readReg(elem.reg_address, &mode, elem.reg_bar);

      /* // currently we have no double-buffering, so use fixed buffer name
    // determine buffer to write to
    int32_t lastBuffer;
    _registerMapping->getRegisterInfo("LAST_BUFFER", elem, "AD16");
    readReg(elem.reg_address, &lastBuffer, elem.reg_bar);
    int32_t activeBuffer = 0;
    if(mode == AUTO_TRIGGER && lastBuffer == 0) activeBuffer = 1;
    std::string buffer_name;
    if(activeBuffer == 0) {
      buffer_name = "BUFFER_A";
    }
    else if(activeBuffer == 1) {
      buffer_name = "BUFFER_B";
    }
       */
      std::string buffer_name = "DAQ0_ADCA";

      // Obtain register description for buffer
      _registerMapping->getRegisterInfo("AREA_MULTIPLEXED_SEQUENCE_"+buffer_name, elem, "APP0");

      // total size of buffer to fill
      //int32_t blockSize = elem.reg_size;

      // compute number of samples
      int32_t nSamples = elem.reg_elem_nr/numberOfChannels;

      /* sampling rate and number of samples is currently fixed
    // obtain sample rate and number of samples per block
    int32_t samplingRateDiv, samplesPerBlock;

    _registerMapping->getRegisterInfo("SAMPLING_RATE_DIV", elem, "AD16");
    readReg(elem.reg_address, &samplingRateDiv, elem.reg_bar);

    _registerMapping->getRegisterInfo("SAMPLES_PER_BLOCK", elem, "AD16");
    readReg(elem.reg_address, &samplesPerBlock, elem.reg_bar);

    ... some code missing to compute uSecsPerSample and blockSize...
       */

      // random number generator for 18 bit number
      boost::uniform_int<> uniform(0, (1<<18) - 1);    // 18 bit random number

      // reset write position
      currentPosition = 0;

      // fill buffer
      for(int32_t iSample=0; iSample<nSamples; iSample++) {

        // write data to buffer
        for(int32_t i=0; i<numberOfChannels; i++) {
          int32_t data;
          if(i == 0) {
            data = iSample;
          }
          else if(i == 1) {
            data = 1000.*sin(2.*acos(-1) * (float)iSample/(float)nSamples*1000) +
                1000.*sin(2.*acos(-1) * (float)iSample/(float)nSamples* 100) +
                100.*sin(2.*acos(-1) * (float)iSample/(float)nSamples* 500);
          }
          else {
            data = uniform(rng);
          }
          writeRegisterWithoutCallback(elem.reg_address + currentPosition, data, elem.reg_bar);

          // increment position in buffer
          currentPosition += sizeof(int32_t);
        }
      }

      // wait some time to simulate conversion timing
      int32_t uSecsTotal = nSamples * (1. / 79100. * 1e6);         // sample rate fixed at ~79.1kHz
      boost::posix_time::ptime t1(boost::posix_time::microsec_clock::local_time());
      boost::posix_time::time_duration dt = boost::posix_time::microseconds(uSecsTotal) - (t1-t0);
      boost::this_thread::sleep(dt);

      isConversionRunning = false;

      /* we don't have these registers yet in the firmware...

    // update LAST_BUFFER regiser
    _registerMapping->getRegisterInfo("LAST_BUFFER", elem, "AD16");
    writeRegisterWithoutCallback(elem.reg_address, activeBuffer, elem.reg_bar);

    // change START_CONERSION register back to 0 and terminate thread (except in auto trigger mode)
    if(mode != AUTO_TRIGGER) {
      _registerMapping->getRegisterInfo("CONVERSION_RUNNING", elem, "AD16");
      writeRegisterWithoutCallback(elem.reg_address, 0, elem.reg_bar);
      isConversionRunning = false;
      return;
    }

       */

      // if software trigger, stop the conversion thread
      if(mode == SOFTWARE_TRIGGER) return;

      /* auto trigger stuff is currently disabled
      // if auto trigger, wait until next trigger occurs:
      // first measure time already used for the conversion
      boost::posix_time::ptime t1(boost::posix_time::microsec_clock::local_time());
      boost::posix_time::time_duration dt = t1-t0;
      int us_taken = dt.total_microseconds();

      // next compute the remaining time to wait
      _registerMapping->getRegisterInfo("WORD_TIMING_FREQ", elem, "APP0");
      int divider;
      readReg(elem.reg_address, &divider, elem.reg_bar);
      int us_needed = divider/50;

      // sleep
      std::cout << "Wait for next autotrigger..." << std::endl;
      usleep(us_needed - us_taken);
       */
      return;

    }


  }


}
