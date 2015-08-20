#include <boost/test/included/unit_test.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"

#include "ad16DummyDevice.h"
#include <MtcaMappedDevice/devMap.h>

using namespace boost::unit_test_framework;
using namespace mtca4u;

#define TEST_MAPPING_FILE "ad16_scope_fmc25_r1224.mapp"

/**********************************************************************************************************************/
// forward declaration so we can declare it friend
// of TestableDummyDevice.
class DummyDeviceTest;

/**********************************************************************************************************************/
/** The TestableDummyDevice is derived from 
 *  DummyDevice to get access to the protected members.
 *  This is done by declaring DummyDeviceTest as a friend.
 */
class TestableDummyDevice : public ad16DummyDevice
{
    friend class DummyDeviceTest;
};

/**********************************************************************************************************************/
class DummyDeviceTest {
  public:
    DummyDeviceTest() {
      _dummyDevice = boost::shared_ptr<TestableDummyDevice>( new TestableDummyDevice() );
    }

    void testExceptions();
    void testSoftwareTriggeredMode();
    void testAutoTriggerMode();
    void testTriggerRates();

  private:
    //TestableDummyDevice _dummyDevice;
    boost::shared_ptr<TestableDummyDevice> _dummyDevice;
    devMap<devBase> _dummyMapped;
    void openDevice();
    friend class DummyDeviceTestSuite;

    // helper function for testTriggerRates() to test with a single trigger rate
    // the rateDiv argiment is the trigger rate divider with the usual AD16 definition
    void testSingleTriggerRate(int rateDiv);

    /*    // helper routine for testTriggerRate(), returns milliseconds the conversion took
    // nSamples and fDiv are the number of samples and the sample rate divisor
    // msOffset is the calibration offset to be subtracted from the measured time. -1 will disable comparing the time
    // to the expectation (used to measure the offset)
    int measureTriggerTiming(int nSamples, int fDiv, int msOffset=-1); */

};

/**********************************************************************************************************************/
class  DummyDeviceTestSuite : public test_suite {
  public:
    DummyDeviceTestSuite() : test_suite("DummyDevice test suite") {
      boost::shared_ptr<DummyDeviceTest> dummyDeviceTest( new DummyDeviceTest );

      add( BOOST_CLASS_TEST_CASE( &DummyDeviceTest::testExceptions, dummyDeviceTest ) );
      add( BOOST_CLASS_TEST_CASE( &DummyDeviceTest::testSoftwareTriggeredMode, dummyDeviceTest ) );
      add( BOOST_CLASS_TEST_CASE( &DummyDeviceTest::testAutoTriggerMode, dummyDeviceTest ) );
      add( BOOST_CLASS_TEST_CASE( &DummyDeviceTest::testTriggerRates, dummyDeviceTest ) );
    }};

/**********************************************************************************************************************/
test_suite* init_unit_test_suite( int /*argc*/, char* /*argv*/ [] )
{
  framework::master_test_suite().p_name.value = "AD16 DummyDevice test suite";
  framework::master_test_suite().add(new DummyDeviceTestSuite);

  return NULL;
}

/**********************************************************************************************************************/
void DummyDeviceTest::openDevice() {
  _dummyDevice->openDev(TEST_MAPPING_FILE);
  _dummyMapped.openDev( _dummyDevice, _dummyDevice->_registerMapping);
}

/**********************************************************************************************************************/
void DummyDeviceTest::testExceptions() {
  std::cout << "testExceptions" << std::endl;

  // close the not opened device
  BOOST_CHECK_THROW( _dummyDevice->closeDev(), DummyDeviceException);
  try {
    _dummyDevice->closeDev();
  }
  catch(DummyDeviceException &a) {
    BOOST_CHECK( a.getID() == DummyDeviceException::ALREADY_CLOSED);
  }

  // open twice
  _dummyDevice->openDev(TEST_MAPPING_FILE);
  BOOST_CHECK_THROW( _dummyDevice->openDev(TEST_MAPPING_FILE), DummyDeviceException);
  try {
    _dummyDevice->openDev(TEST_MAPPING_FILE);
  }
  catch(DummyDeviceException &a) {
    BOOST_CHECK( a.getID() == DummyDeviceException::ALREADY_OPEN);
  }

  // close again
  _dummyDevice->closeDev();
}

/**********************************************************************************************************************/
void DummyDeviceTest::testSoftwareTriggeredMode() {
  std::cout << "testSoftwareTriggeredMode" << std::endl;
  openDevice();

  // set test value of dummy device (to have something changing between the tests). Will be the content of the 3rd channel
  _dummyDevice->testValue = 1;

  // select ADCA ready as DAQ strobe
  _dummyMapped.getRegisterAccessor("WORD_DAQ_STR_SEL","APP0")->write(0);        // DAQ_STROBE_ADCA

  // enable software trigger
  _dummyMapped.getRegisterAccessor("WORD_TIMING_TRG_SEL","APP0")->write(8);

  // enable DAQ
  _dummyMapped.getRegisterAccessor("WORD_DAQ_ENABLE","APP0")->write(1);

  // obtain last buffer
  int lastBuffer;
  _dummyMapped.getRegisterAccessor("WORD_DAQ_CURR_BUF","APP0")->read(&lastBuffer);

  // send user trigger
  _dummyMapped.getRegisterAccessor("WORD_TIMING_USER_TRG","APP0")->write(1);

  // wait for 1 second
  usleep(1000000);

  // create accessor for multiplexed data
  boost::shared_ptr< mtca4u::MultiplexedDataAccessor<int32_t> > dataDemuxed;
  if(lastBuffer == 1) {
    dataDemuxed = _dummyMapped.getCustomAccessor< mtca4u::MultiplexedDataAccessor<int32_t> >("DAQ0_ADCA", "APP0");
  }
  else {
    dataDemuxed = _dummyMapped.getCustomAccessor< mtca4u::MultiplexedDataAccessor<int32_t> >("DAQ0_ADCB", "APP0");
  }
  dataDemuxed->read();

  // Return the number of sequences found: should be 16
  uint numberOfDataSequences = dataDemuxed->getNumberOfDataSequences();
  BOOST_CHECK( numberOfDataSequences == 16 );

  // The accessor expects that all sequences are of the same length, and that a
  // described region should have at least one sequence.
  int lengthOfaSequence = (*dataDemuxed)[0].size();
  BOOST_CHECK( lengthOfaSequence == 65536 );

  // check data
  for(int iSample = 0; iSample < lengthOfaSequence; ++iSample) {
    BOOST_CHECK( (*dataDemuxed)[1][iSample] == iSample );
    BOOST_CHECK( (*dataDemuxed)[2][iSample] == 1 );             // this is our test value set above
  }

  _dummyMapped.closeDev();
}
/**********************************************************************************************************************/
void DummyDeviceTest::testAutoTriggerMode() {
  std::cout << "testAutoTriggerMode" << std::endl;
  openDevice();

  // set test value of dummy device (to have something changing between the tests). Will be the content of the 3rd channel
  _dummyDevice->testValue = 2;

  // select ADCA ready as DAQ strobe
  _dummyMapped.getRegisterAccessor("WORD_DAQ_STR_SEL","APP0")->write(0);        // DAQ_STROBE_ADCA

  // enable trigger 0 and set it to 1 Hz
  _dummyMapped.getRegisterAccessor("WORD_TIMING_TRG_SEL","APP0")->write(0);
  _dummyMapped.getRegisterAccessor("WORD_TIMING_FREQ","APP0")->write(49999999);

  // obtain current buffer
  int lastBuffer;
  _dummyMapped.getRegisterAccessor("WORD_DAQ_CURR_BUF","APP0")->read(&lastBuffer);

  // enable DAQ
  _dummyMapped.getRegisterAccessor("WORD_DAQ_ENABLE","APP0")->write(1);

  // wait until conversion is complete
  int currentBuffer;
  do {
    _dummyMapped.getRegisterAccessor("WORD_DAQ_CURR_BUF","APP0")->read(&currentBuffer);
    usleep(1);
  } while(currentBuffer == lastBuffer);

  // create accessor for multiplexed data
  boost::shared_ptr< mtca4u::MultiplexedDataAccessor<int32_t> > dataDemuxed;
  if(currentBuffer == 1) {
    dataDemuxed = _dummyMapped.getCustomAccessor< mtca4u::MultiplexedDataAccessor<int32_t> >("DAQ0_ADCA", "APP0");
  }
  else {
    dataDemuxed = _dummyMapped.getCustomAccessor< mtca4u::MultiplexedDataAccessor<int32_t> >("DAQ0_ADCB", "APP0");
  }
  dataDemuxed->read();

  // The accessor expects that all sequences are of the same length, and that a
  // described region should have at least one sequence.
  int lengthOfaSequence = (*dataDemuxed)[0].size();

  // check data:
  for(int iSample = 0; iSample < lengthOfaSequence; ++iSample) {
    BOOST_CHECK( (*dataDemuxed)[1][iSample] == iSample );
    BOOST_CHECK( (*dataDemuxed)[2][iSample] == 2 );             // this is our test value set above
  }

  _dummyMapped.closeDev();
}

/**********************************************************************************************************************/
void DummyDeviceTest::testTriggerRates() {
  testSingleTriggerRate(49999999);      // 1 sec
  testSingleTriggerRate(74999999);      // 1.5 sec
}

/**********************************************************************************************************************/
void DummyDeviceTest::testSingleTriggerRate(int rateDiv) {
  std::cout << "testSingleTriggerRate(" << rateDiv << ")" << std::endl;
  openDevice();

  // set test value of dummy device (to have something changing between the tests). Will be the content of the 3rd channel
  _dummyDevice->testValue = 3;

  // select ADCA ready as DAQ strobe
  _dummyMapped.getRegisterAccessor("WORD_DAQ_STR_SEL","APP0")->write(0);        // DAQ_STROBE_ADCA

  // enable trigger 0 and set it to seleted frequency
  _dummyMapped.getRegisterAccessor("WORD_TIMING_TRG_SEL","APP0")->write(0);
  int rateDivs[9] = {rateDiv, 0,0,0,0,0,0,0,0};
  _dummyMapped.getRegisterAccessor("WORD_TIMING_FREQ","APP0")->write(rateDivs,9);

  // set ADC sample rate
  _dummyMapped.getRegisterAccessor("WORD_ADC_A_TIMING_DIV","AD160")->write(499);
  _dummyMapped.getRegisterAccessor("WORD_ADC_B_TIMING_DIV","AD160")->write(499);

  // obtain current buffer
  int lastBuffer;
  _dummyMapped.getRegisterAccessor("WORD_DAQ_CURR_BUF","APP0")->read(&lastBuffer);

  // enable DAQ
  _dummyMapped.getRegisterAccessor("WORD_DAQ_ENABLE","APP0")->write(1);

  // measure time between each trigger (for 3 triggers, to have some statistics)
  // for each trigger, the recorded data will be checked if correct
  boost::posix_time::ptime t0(boost::posix_time::microsec_clock::local_time());         // start time
  std::vector<boost::posix_time::time_duration> triggerTimes;

  std::cout << "Triggers: " << std::flush;
  for(int i=0; i<3; i++) {
    // wait until conversion is complete
    int currentBuffer;
    do {
      _dummyMapped.getRegisterAccessor("WORD_DAQ_CURR_BUF","APP0")->read(&currentBuffer);
      usleep(1);
    } while(currentBuffer == lastBuffer);
    lastBuffer = currentBuffer;

    std::cout << "." << std::flush;

    // measure time
    boost::posix_time::ptime t1(boost::posix_time::microsec_clock::local_time());
    triggerTimes.push_back(t1-t0);
    t0 = t1;

    // create accessor for multiplexed data
    boost::shared_ptr< mtca4u::MultiplexedDataAccessor<int32_t> > dataDemuxed;
    if(currentBuffer == 1) {
      dataDemuxed = _dummyMapped.getCustomAccessor< mtca4u::MultiplexedDataAccessor<int32_t> >("DAQ0_ADCA", "APP0");
    }
    else {
      dataDemuxed = _dummyMapped.getCustomAccessor< mtca4u::MultiplexedDataAccessor<int32_t> >("DAQ0_ADCB", "APP0");
    }
    dataDemuxed->read();

    // check data
    int lengthOfaSequence = (*dataDemuxed)[0].size();
    int error_count = 0;
    for(int iSample = 0; iSample < lengthOfaSequence; ++iSample) {
      BOOST_CHECK( (*dataDemuxed)[1][iSample] == iSample );
      BOOST_CHECK( (*dataDemuxed)[2][iSample] == 3 );             // this is our test value set above
      BOOST_CHECK( (*dataDemuxed)[3][iSample] == i );             // this is the trigger counter
      if((*dataDemuxed)[1][iSample] != iSample || (*dataDemuxed)[2][iSample] != 3 || (*dataDemuxed)[3][iSample] != i ) error_count++;
      if(error_count > 3) {
        std::cerr << "*** Too many errors in this test, aborting current test at iSample = " << iSample << " for i = " << i << std::endl;
        break;
      }
    }

  }
  std::cout << std::endl;

  // compute average and rms of trigger times
  double mean = 0;      // in miliseconds
  double rms = 0;
  for(unsigned int i=0; i<triggerTimes.size(); i++) mean += triggerTimes[i].total_microseconds()/1000.;
  mean /= triggerTimes.size();
  for(unsigned int i=0; i<triggerTimes.size(); i++) rms += pow(triggerTimes[i].total_microseconds()/1000. - mean,2);
  rms = sqrt(rms)/triggerTimes.size();

  std::cout << "Mean = " << mean << "   RMS = " << rms << std::endl;

  double expectedMean = (rateDiv+1) / 50000.;
  BOOST_CHECK( mean > expectedMean/1.001 && mean < expectedMean*1.001 );
  BOOST_CHECK( rms < 1. );

  _dummyMapped.closeDev();
}

/**********************************************************************************************************************/
/*
int DummyDeviceTest::measureTriggerTiming(int nSamples, int fDiv, int msOffset) {
  // set number of samples per block and channel
  _dummyMapped.writeReg("SAMPLES_PER_BLOCK", "AD16", &nSamples);

  // set sample rate
  _dummyMapped.writeReg("SAMPLING_RATE_DIV", "AD16", &fDiv);

  // start conversion
  int32_t val = 1;
  _dummyMapped.writeReg("CONVERSION_RUNNING", "AD16", &val);

  // measure time until conversion is complete
  boost::posix_time::ptime t1(boost::posix_time::microsec_clock::local_time());
  while(val == 1) {
    _dummyMapped.readReg("CONVERSION_RUNNING", "AD16", &val);
    usleep(1);
  }

  // determine milliseconds milliseconds
  boost::posix_time::ptime t2(boost::posix_time::microsec_clock::local_time());
  boost::posix_time::time_duration dt = t2-t1;
  int ms = dt.total_milliseconds();

  // if msOffset is given, compare with expectation
  if(msOffset >= 0) {
    double tSample = 1./100.*fDiv;      // 100 kHz = 1/100 ms
    double tExpt = tSample * nSamples + msOffset;
    double deviation = fabs( (tExpt - ms)/tExpt );
    std::cout << nSamples << " samples @ " << (100./fDiv) << "kHz took " << ms << " ms ( Expectation: " << tExpt << " ms, deviation: " << deviation << ")" << std::endl;
    BOOST_CHECK(deviation < 0.1);       // allow 10% deviation
  }
  else {
    std::cout << nSamples << " samples @ " << (100./fDiv) << "kHz took " << ms << " ms" << std::endl;
  }

  // return measured time
  return ms;

}
 */
