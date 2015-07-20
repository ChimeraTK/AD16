#include <boost/test/included/unit_test.hpp>
using namespace boost::unit_test_framework;
#include <boost/lambda/lambda.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"

#include "ad16DummyDevice.h"
#include <MtcaMappedDevice/devMap.h>
using namespace mtca4u;

#define TEST_MAPPING_FILE "ad16dummy.map"

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

    void testSoftwareTriggeredMode();
    void testSampleRate();
    void testAutoTriggerMode();

  private:
    //TestableDummyDevice _dummyDevice;
    boost::shared_ptr<TestableDummyDevice> _dummyDevice;
    devMap<devBase> _dummyMapped;
    void freshlyOpenDevice();
    friend class DummyDeviceTestSuite;

    // helper routine for testSampleRate(), returns milliseconds the conversion took
    // nSamples and fDiv are the number of samples and the sample rate divisor
    // msOffset is the calibration offset to be subtracted from the measured time. -1 will disable comparing the time
    // to the expectation (used to measure the offset)
    int measureConversionTime(int nSamples, int fDiv, int msOffset=-1);

};

/**********************************************************************************************************************/
class  DummyDeviceTestSuite : public test_suite {
  public:
    DummyDeviceTestSuite() : test_suite("DummyDevice test suite") {
      boost::shared_ptr<DummyDeviceTest> dummyDeviceTest( new DummyDeviceTest );

      add( BOOST_CLASS_TEST_CASE( &DummyDeviceTest::testSoftwareTriggeredMode, dummyDeviceTest ) );
      //add( BOOST_CLASS_TEST_CASE( &DummyDeviceTest::testSampleRate, dummyDeviceTest ) );
    }};

/**********************************************************************************************************************/
test_suite* init_unit_test_suite( int /*argc*/, char* /*argv*/ [] )
{
  framework::master_test_suite().p_name.value = "AD16 DummyDevice test suite";
  framework::master_test_suite().add(new DummyDeviceTestSuite);

  return NULL;
}

/**********************************************************************************************************************/
void DummyDeviceTest::freshlyOpenDevice() {
  try{
    _dummyDevice->openDev(TEST_MAPPING_FILE);
    _dummyMapped.openDev( _dummyDevice, _dummyDevice->_registerMapping);
  }
  catch(DummyDeviceException &) {
    // make sure the device was freshly opened, so
    // registers are set to 0.
    _dummyMapped.closeDev();
    //_dummyDevice->closeDev();
    _dummyDevice->openDev(TEST_MAPPING_FILE);
    _dummyMapped.openDev( _dummyDevice, _dummyDevice->_registerMapping);
  }
}

/**********************************************************************************************************************/
void DummyDeviceTest::testSoftwareTriggeredMode() {
  int32_t val;
  freshlyOpenDevice();

  // set trigger to user trigger
  val = 8;
  _dummyMapped.writeReg("WORD_TIMING_TRG_SEL", "APP0", &val);

  // start conversion
  val = 1;
  _dummyMapped.writeReg("WORD_TIMING_USER_TRG", "APP0", &val);

  // number of samples is currently fixed
  int32_t nSamples = 65536;

  // wait until conversion is complete
  /* no synchronisation possible with current firmware...
  while(val == 1) {
    _dummyMapped.readReg("CONVERSION_RUNNING", "AD16", &val);
    usleep(1);
  }
  */
  usleep(1000000);

  // create accessor for multiplexed data
  boost::shared_ptr< mtca4u::MultiplexedDataAccessor<int32_t> > dataDemuxed =
      _dummyMapped.getCustomAccessor< mtca4u::MultiplexedDataAccessor<int32_t> >("BUFFER", "BOARD0");
  dataDemuxed->read();

  // Return the number of sequences found: should be 16
  uint numberOfDataSequences = dataDemuxed->getNumberOfDataSequences();
  std::cout << "Number Of dataSequences extracted: " << numberOfDataSequences << std::endl;

  // The accessor expects that all sequences are of the same length, and that a
  // described region should have at least one sequence.
  uint lengthOfaSequence = (*dataDemuxed)[0].size();
  std::cout << "Length of each sequence: " << lengthOfaSequence << std::endl;

  // check data: first sequence should contain sample number as values
  for (int columnCount = 0; columnCount < nSamples; ++columnCount) {
    //std::cout << "row 0 column " << columnCount << " " << (*dataDemuxed)[0][columnCount] << std::endl;
    BOOST_CHECK( (*dataDemuxed)[0][columnCount] == columnCount );
  }
  // This check would only make sense if the buffer were larger than the written data. This is currently not the case
  // as the number of samples is fixed.
  //BOOST_CHECK( (*dataDemuxed)[0][nSamples] == 0 );
}

/**********************************************************************************************************************/
void DummyDeviceTest::testSampleRate() {
  int32_t val;
  freshlyOpenDevice();

  // set mode
  val = 0;
  _dummyMapped.writeReg("MODE", "AD16", &val);

  // measure offset (1000 samples at 100kHz should take 10ms)
  int msOffset = measureConversionTime(1000,1) - 10.;

  // test various sample rates
  measureConversionTime(1000,2,msOffset);
  measureConversionTime(1000,10,msOffset);
  measureConversionTime(1000,5,msOffset);
  measureConversionTime(1000,100,msOffset);
}

/**********************************************************************************************************************/
int DummyDeviceTest::measureConversionTime(int nSamples, int fDiv, int msOffset) {

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

/**********************************************************************************************************************/
void DummyDeviceTest::testAutoTriggerMode() {
  int32_t val;
  freshlyOpenDevice();

  // set mode
  val = 0;
  _dummyMapped.writeReg("WORD_TIMING_TRG_SEL", "APP0", &val);

  // set number of samples per block
  int nSamples = 8;
  _dummyMapped.writeReg("SAMPLES_PER_BLOCK", "AD16", &nSamples);

  // read the current "last buffer"
  int lastBuffer;
  _dummyMapped.readReg("LAST_BUFFER", "AD16", &lastBuffer);

  // start conversion
  val = 1;
  _dummyMapped.writeReg("CONVERSION_RUNNING", "AD16", &val);

  // wait until conversion is complete
  while(val != lastBuffer) {
    _dummyMapped.readReg("LAST_BUFFER", "AD16", &val);
    usleep(1);
  }

  // create accessor for multiplexed data
  boost::shared_ptr< mtca4u::MultiplexedDataAccessor<int32_t> > dataDemuxed =
      _dummyMapped.getCustomAccessor< mtca4u::MultiplexedDataAccessor<int32_t> >("BUFFER_A", "AD16");
  dataDemuxed->read();

  // Return the number of sequences found: should be 16
  uint numberOfDataSequences = dataDemuxed->getNumberOfDataSequences();
  std::cout << "Number Of dataSequences extracted: " << numberOfDataSequences << std::endl;

  // The accessor expects that all sequences are of the same length, and that a
  // described region should have at least one sequence.
  uint lengthOfaSequence = (*dataDemuxed)[0].size();
  std::cout << "Length of each sequence: " << lengthOfaSequence << std::endl;
  // check data: first sequence should contain sample number as values
  for (int columnCount = 0; columnCount < nSamples; ++columnCount) {
//    std::cout << "row 0 column " << columnCount << " " << (*dataDemuxed)[0][columnCount] << std::endl;
    BOOST_CHECK( (*dataDemuxed)[0][columnCount] == columnCount );
  }
  BOOST_CHECK( (*dataDemuxed)[0][nSamples] == 0 );
}
