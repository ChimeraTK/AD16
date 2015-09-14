#include <math.h>
#include <boost/test/included/unit_test.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <MtcaMappedDevice/MappedDevice.h>

#include "ad16DummyDevice.h"

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
    TestableDummyDevice(std::string host, std::string instance, std::list< std::string > parameters)
    : ad16DummyDevice(host, instance, parameters)
    {}
    friend class DummyDeviceTest;
};

/**********************************************************************************************************************/
class DummyDeviceTest {
  public:
    DummyDeviceTest() {
      _dummyDevice = boost::shared_ptr<TestableDummyDevice>(
          new TestableDummyDevice(".",TEST_MAPPING_FILE,std::list<std::string>()));
    }

    void testExceptions();
    void testSoftwareTriggeredMode();
    void testAutoTriggerMode();

  private:
    //TestableDummyDevice _dummyDevice;
    boost::shared_ptr<TestableDummyDevice> _dummyDevice;
    MappedDevice _dummyMapped;
    ptrmapFile _registerMapping;
    void openDevice();
    friend class DummyDeviceTestSuite;


};

/**********************************************************************************************************************/
class  DummyDeviceTestSuite : public test_suite {
  public:
    DummyDeviceTestSuite() : test_suite("DummyDevice test suite") {
      boost::shared_ptr<DummyDeviceTest> dummyDeviceTest( new DummyDeviceTest );

      add( BOOST_CLASS_TEST_CASE( &DummyDeviceTest::testExceptions, dummyDeviceTest ) );
      add( BOOST_CLASS_TEST_CASE( &DummyDeviceTest::testSoftwareTriggeredMode, dummyDeviceTest ) );
      add( BOOST_CLASS_TEST_CASE( &DummyDeviceTest::testAutoTriggerMode, dummyDeviceTest ) );
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
  //_dummyDevice->open();
  _registerMapping = mapFileParser().parse(TEST_MAPPING_FILE);
  _dummyMapped.open( _dummyDevice, _registerMapping);
}

/**********************************************************************************************************************/
void DummyDeviceTest::testExceptions() {
  std::cout << "testExceptions" << std::endl;

  // close the not opened device
  BOOST_CHECK_THROW( _dummyDevice->close(), DummyDeviceException);
  try {
    _dummyDevice->close();
  }
  catch(DummyDeviceException &a) {
    BOOST_CHECK( a.getID() == DummyDeviceException::ALREADY_CLOSED);
  }

  // open twice
  _dummyDevice->open(TEST_MAPPING_FILE);
  BOOST_CHECK_THROW( _dummyDevice->open(TEST_MAPPING_FILE), DummyDeviceException);
  try {
    _dummyDevice->open(TEST_MAPPING_FILE);
  }
  catch(DummyDeviceException &a) {
    BOOST_CHECK( a.getID() == DummyDeviceException::ALREADY_OPEN);
  }

  // close again
  _dummyDevice->close();

  // open it now, should not throw an exception
  _dummyDevice->open(TEST_MAPPING_FILE);

  // close again
  _dummyDevice->close();

  // close the not opened device again
  BOOST_CHECK_THROW( _dummyDevice->close(), DummyDeviceException);
  try {
    _dummyDevice->close();
  }
  catch(DummyDeviceException &a) {
    BOOST_CHECK( a.getID() == DummyDeviceException::ALREADY_CLOSED);
  }
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

  // set ADC sample rate
  _dummyMapped.getRegisterAccessor("WORD_ADC_A_TIMING_DIV","AD160")->write(999);
  _dummyMapped.getRegisterAccessor("WORD_ADC_B_TIMING_DIV","AD160")->write(999);

  // enable DAQ
  _dummyMapped.getRegisterAccessor("WORD_DAQ_ENABLE","APP0")->write(1);

  // send user trigger
  _dummyMapped.getRegisterAccessor("WORD_TIMING_USER_TRG","APP0")->write(1);

  // advance dummy strobe timing until buffer is full
  bool ERROR_FOUND = false;
  for(int i=0; i<65536; i++) {
    if( round(_dummyDevice->timers.getRemaining()*1000000.) != 20000 ) {
      if(!ERROR_FOUND) std::cerr << "Wrong strobe timing: " << _dummyDevice->timers.getRemaining()*1000000. << std::endl;
      ERROR_FOUND = true;
    }
    _dummyDevice->timers.advance("strobe");
  }
  BOOST_CHECK( !ERROR_FOUND );

  // obtain current buffer
  int currentBuffer;
  _dummyMapped.getRegisterAccessor("WORD_DAQ_CURR_BUF","APP0")->read(&currentBuffer);

  // create accessor for multiplexed data
  boost::shared_ptr< mtca4u::MultiplexedDataAccessor<int32_t> > dataDemuxed;
  std::cout << "currentBuffer = " << currentBuffer << std::endl;
  if(currentBuffer == 0) {
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


  // check data:
  ERROR_FOUND = false;
  for(int iSample = 0; iSample < lengthOfaSequence; ++iSample) {
    if( ( (*dataDemuxed)[1][iSample] != iSample ) || ( (*dataDemuxed)[2][iSample] != 1 ) ) {
      if(!ERROR_FOUND) std::cerr << "Wrong sample values for sample " << iSample << ": " << (*dataDemuxed)[1][iSample] << " " << (*dataDemuxed)[2][iSample] << std::endl;
      ERROR_FOUND = true;
    }
  }
  BOOST_CHECK( !ERROR_FOUND );

  _dummyMapped.close();
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
  _dummyMapped.getRegisterAccessor("WORD_TIMING_FREQ","APP0")->write(49999999);
  _dummyMapped.getRegisterAccessor("WORD_TIMING_TRG_SEL","APP0")->write(0);

  // set ADC sample rate
  _dummyMapped.getRegisterAccessor("WORD_ADC_A_TIMING_DIV","AD160")->write(499);
  _dummyMapped.getRegisterAccessor("WORD_ADC_B_TIMING_DIV","AD160")->write(499);

  // obtain current buffer
  int lastBuffer;
  _dummyMapped.getRegisterAccessor("WORD_DAQ_CURR_BUF","APP0")->read(&lastBuffer);
  std::cout << "lastBuffer = " << lastBuffer << std::endl;

  // enable DAQ
  _dummyMapped.getRegisterAccessor("WORD_DAQ_ENABLE","APP0")->write(1);

  // check trigger timing
  BOOST_CHECK( round(_dummyDevice->trigger.getRemaining()*1000000.) == 1000000000. );

  // wait until conversion is complete
  int currentBuffer;
  bool ERROR_FOUND = false;
  do {
    _dummyMapped.getRegisterAccessor("WORD_DAQ_CURR_BUF","APP0")->read(&currentBuffer);
    if( round(_dummyDevice->timers.getRemaining()*1000000.) != 10000 ) {
      if(!ERROR_FOUND) std::cerr << "Wrong strobe timing: " << _dummyDevice->timers.getRemaining()*1000000. << std::endl;
      ERROR_FOUND = true;
    }
    _dummyDevice->timers.advance("strobe");
  } while(currentBuffer == lastBuffer);
  BOOST_CHECK( !ERROR_FOUND );

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

  lengthOfaSequence = 6;

  // check data:
  ERROR_FOUND = false;
  for(int iSample = 0; iSample < lengthOfaSequence; ++iSample) {
    if( ( (*dataDemuxed)[1][iSample] != iSample ) || ( (*dataDemuxed)[2][iSample] != 2 ) ) {
      if(!ERROR_FOUND) std::cerr << "Wrong sample values for sample " << iSample << ": " << (*dataDemuxed)[1][iSample] << " " << (*dataDemuxed)[2][iSample] << std::endl;
      ERROR_FOUND = true;
    }
  }
  BOOST_CHECK( !ERROR_FOUND );

  _dummyMapped.close();
}
