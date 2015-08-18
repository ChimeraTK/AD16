#include "ad16DummyDevice.h"
#include <boost/bind.hpp>
#include <boost/random/uniform_int.hpp>

namespace mtca4u {
  const int32_t ad16DummyDevice::numberOfChannels = 16;
  const int32_t ad16DummyDevice::numberOfSamples = 65536;

  /*************************************************************************************************/
  ad16DummyDevice::ad16DummyDevice()
  : isOpened(false),
    isDaqRunning(false),
    currentPosition(0),
    currentBuffer(0),
    clockFrequency(50000000),
    spiFrequency(25000000),
    testValue(999),
    triggerCounter(0)
  {
  }

  /*************************************************************************************************/
  ad16DummyDevice::~ad16DummyDevice()
  {
  }

  /*************************************************************************************************/
  void ad16DummyDevice::openDev(const std::string &mappingFileName, int perm, devConfigBase *pConfig) {
    mapFile::mapElem elem;

    // call parent class open device first
    DummyDevice::openDev (mappingFileName, perm, pConfig);

    // register callback function for enabling the DAQ
    setupCallback("WORD_DAQ_ENABLE", boost::bind( &ad16DummyDevice::callbackDaqEnable, this) );

    // set clock frequency register
    _registerMapping->getRegisterInfo("WORD_CLK_FREQ", elem, "AD160");
    writeRegisterWithoutCallback(elem.reg_address, clockFrequency, elem.reg_bar);
    writeRegisterWithoutCallback(elem.reg_address + sizeof(int32_t), spiFrequency, elem.reg_bar);
    setReadOnly(elem.reg_address, elem.reg_bar, elem.reg_elem_nr);

    // reset DAQ_ENABLE register
    _registerMapping->getRegisterInfo("WORD_DAQ_ENABLE", elem, "APP0");
    writeRegisterWithoutCallback(elem.reg_address, 0, elem.reg_bar);
    isDaqRunning = false;

    // reset WORD_DAQ_CURR_BUF register
    currentBuffer = 0;
    _registerMapping->getRegisterInfo("WORD_DAQ_CURR_BUF", elem, "APP0");
    writeRegisterWithoutCallback(elem.reg_address, currentBuffer, elem.reg_bar);

    // reset current position and trigger counter
    currentPosition = 0;
    triggerCounter = 0;

    isOpened = true;
  }

  /*************************************************************************************************/
  void ad16DummyDevice::setupCallback(std::string registerName,boost::function<void()> cb) {
    mapFile::mapElem elem;
    _registerMapping->getRegisterInfo(registerName, elem, "APP0");
    setWriteCallbackFunction( AddressRange(elem.reg_address, elem.reg_size, elem.reg_bar), cb );
  }

  /*************************************************************************************************/
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

  /*************************************************************************************************/
  void ad16DummyDevice::callbackDaqEnable() {
    // get content of DAQ enable register
    mapFile::mapElem elem;
    _registerMapping->getRegisterInfo("WORD_DAQ_ENABLE", elem, "APP0");
    int daq_enable;
    readReg(elem.reg_address, &daq_enable, elem.reg_bar);

    // enable or disable the DAQ
    if(daq_enable) {
      // check if conversion thread is already running
      if(isDaqRunning) return;
      // start DAQ thread
      isDaqRunning = true;
      theThread =  boost::thread( boost::bind(&ad16DummyDevice::threadDaq, this) );
    }
    else{
      if(!isDaqRunning) return;
      // shut down the DAQ
      theThread.interrupt();
      theThread.join();
      isDaqRunning = false;
    }
  }

  /*************************************************************************************************/
  void ad16DummyDevice::threadDaq() {
    mapFile::mapElem elem;

    // measure start time
    // we have three independent time counters for the DAQ, the ADC A and the ADC B
    boost::posix_time::ptime tnow(boost::posix_time::microsec_clock::local_time());
    boost::posix_time::ptime t0_DaqBuffer = tnow;     // start time for DAQ trigger (per-buffer)
    boost::posix_time::ptime t0_DaqStrobe = tnow;     // start time for DAQ strobe (i.e. per-sample trigger)
    boost::posix_time::ptime t0_a = tnow;             // start time for ADC A
    boost::posix_time::ptime t0_b = tnow;             // start time for ADC B

    // set sizes of ADC value buffers
    ADCvalA.resize(numberOfChannels/2);
    ADCvalB.resize(numberOfChannels/2);

    // random number generator for 18 bit number (actually only the converter...)
    boost::uniform_int<> uniform(0, (1<<18) - 1);    // 18 bit random number

    // write current buffer to register
    _registerMapping->getRegisterInfo("WORD_DAQ_CURR_BUF", elem, "APP0");
    writeRegisterWithoutCallback(elem.reg_address, currentBuffer, elem.reg_bar);

    // the DAQ is kept running as long as it is enabled. It will be shut down from callbackDaqEnable()
    while(true) {

      // yield to other threads and allow interruption
      boost::this_thread::yield();
      boost::this_thread::interruption_point();

      // get new timestamp
      tnow = boost::posix_time::microsec_clock::local_time();

      // obtain strobe select for DAQ
      int32_t DAQstrobeSelect;
      _registerMapping->getRegisterInfo("WORD_DAQ_STR_SEL", elem, "APP0");
      readReg(elem.reg_address, &DAQstrobeSelect, elem.reg_bar);

      // obtain ADC timing
      int32_t cdiv_a, cdiv_b;
      _registerMapping->getRegisterInfo("WORD_ADC_A_TIMING_DIV", elem, "AD160");
      readReg(elem.reg_address, &cdiv_a, elem.reg_bar);
      _registerMapping->getRegisterInfo("WORD_ADC_B_TIMING_DIV", elem, "AD160");
      readReg(elem.reg_address, &cdiv_b, elem.reg_bar);
      boost::posix_time::time_duration dt_a = boost::posix_time::microseconds(1000000.*cdiv_a/clockFrequency);
      boost::posix_time::time_duration dt_b = boost::posix_time::microseconds(1000000.*cdiv_b/clockFrequency);

      // check if ADC conversion complete
      bool conversion_complete_A = ( (tnow-t0_a) >= dt_a );
      bool conversion_complete_B = ( (tnow-t0_b) >= dt_b );

      // read timing frequencies
      int32_t timingFreqs[9];
      _registerMapping->getRegisterInfo("WORD_TIMING_FREQ", elem, "APP0");
      if(elem.reg_elem_nr != 9) {
        std::cerr << "APP0.WORD_TIMING_FREQ has other than 9 elements." << std::endl;
        exit(1);
      }
      readArea(elem.reg_address, timingFreqs, 9*sizeof(int32_t), elem.reg_bar);

      // get selected trigger
      int32_t timingTriggerSelected;
      _registerMapping->getRegisterInfo("WORD_TIMING_TRG_SEL", elem, "APP0");
      readReg(elem.reg_address, &timingTriggerSelected, elem.reg_bar);

      // check if DAQ is ready (depending on selected strobe signal)
      bool DAQready = false;
      if(DAQstrobeSelect == DAQ_STROBE_ADCA) {
        if(conversion_complete_A) DAQready = true;
      }
      else if(DAQstrobeSelect == DAQ_STROBE_ADCB) {
        if(conversion_complete_B) DAQready = true;
      }
      else if(DAQstrobeSelect == DAQ_STROBE_TRIGGER6) {
        boost::posix_time::time_duration dt_trig6 = boost::posix_time::microseconds(1000000.*(timingFreqs[6]+1)/clockFrequency);
        DAQready = ( (tnow-t0_DaqStrobe) >= dt_trig6 );
      }
      else {
        std::cerr << "Incorrect DAQ strobe selection setting." << std::endl;
        exit(1);
      }

      // if DAQ not ready, wait longer
      if(!DAQready) continue;
      t0_DaqStrobe = tnow;

      // generate ADC samples
      if(conversion_complete_A) {
        ADCvalA[0] = 1000.*sin(2.*acos(-1) * 1000. * (float)currentPosition/(float)numberOfSamples);    // should be based on time?
        ADCvalA[1] = currentPosition;
        ADCvalA[2] = testValue;
        ADCvalA[3] = triggerCounter;
        for(int32_t i=4; i<numberOfChannels/2; i++) ADCvalA[i] = uniform(rng);
        t0_a = tnow;
      }
      if(conversion_complete_B) {
        for(int32_t i=0; i<numberOfChannels/2; i++) ADCvalB[i] = uniform(rng);
        t0_b = tnow;
      }

      // Obtain register description for buffer
      if(currentBuffer == 0) {
        //std::cout << "writing to buffer A" << std::endl;
        _registerMapping->getRegisterInfo("AREA_MULTIPLEXED_SEQUENCE_DAQ0_ADCA", elem, "APP0");
      }
      else {
        //std::cout << "writing to buffer B" << std::endl;
        _registerMapping->getRegisterInfo("AREA_MULTIPLEXED_SEQUENCE_DAQ0_ADCB", elem, "APP0");
      }

      // write samples to buffer (if current position is not outside the buffer)
      if(currentPosition < numberOfSamples) {
        for(int32_t i=0; i<numberOfChannels; i++) {
          int32_t data;
          if(i < numberOfChannels/2) {
            data = ADCvalA[i];
          }
          else {
            data = ADCvalB[i-numberOfChannels/2];
          }
          int32_t addr = elem.reg_address + (currentPosition*numberOfChannels + i)*sizeof(int32_t);
          writeRegisterWithoutCallback(addr, data, elem.reg_bar);
        }
      }

      // increment pointer in buffer (with overrun protection)
      if(currentPosition <= numberOfSamples) currentPosition++;

      // check for (per-buffer) DAQ trigger
      bool isTriggered = false;
      if(timingTriggerSelected == 8) {          // user trigger via register
        // read user trigger register
        int32_t userTrigger;
        _registerMapping->getRegisterInfo("WORD_TIMING_USER_TRG", elem, "APP0");
        readReg(elem.reg_address, &userTrigger, elem.reg_bar);

        // if content is non-zero: trigger and reset register
        if(userTrigger != 0) {
          isTriggered = true;
          writeReg(elem.reg_address, 0, elem.reg_bar);
        }
      }
      else {
        boost::posix_time::time_duration dt = boost::posix_time::microseconds(1000000.*(timingFreqs[timingTriggerSelected]+1)/clockFrequency);
        if(tnow-t0_DaqBuffer >= dt) isTriggered = true;
      }

      // if triggered, reset current position and swap buffers
      if(isTriggered) {

        // reset pointer and time stamp
        currentPosition = 0;
        t0_DaqBuffer = tnow;

        // update current buffer
        if(currentBuffer == 0) {
          currentBuffer = 1;
        }
        else {
          currentBuffer = 0;
        }
        _registerMapping->getRegisterInfo("WORD_DAQ_CURR_BUF", elem, "APP0");
        writeRegisterWithoutCallback(elem.reg_address, currentBuffer, elem.reg_bar);

        // increase trigger counter
        triggerCounter++;
      }
    }   // main loop
  }

} // namespace mtca4u
