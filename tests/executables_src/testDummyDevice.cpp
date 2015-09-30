#include <math.h>
#include <boost/test/included/unit_test.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <mtca4u/Device.h>

#include "ad16dummy.h"

using namespace boost::unit_test_framework;
using namespace mtca4u;

#define DUMMY_ALIAS "DUMMY"

/**********************************************************************************************************************/
class DummyBackendTest {
  public:
    DummyBackendTest() {
      ad16dummy::backendRegisterer.dummy = 1;   // force linking the library...
      _dummyBackend = boost::static_pointer_cast<ad16dummy>( BackendFactory::getInstance().createBackend(DUMMY_ALIAS) );
    }

    void testExceptions();
    void testSoftwareTriggeredMode();
    void testAutoTriggerMode();

  private:
    boost::shared_ptr<ad16dummy> _dummyBackend;
    Device _dummy;
    void openDevice();
    friend class DummyDeviceTestSuite;
};

/**********************************************************************************************************************/
class  DummyDeviceTestSuite : public test_suite {
  public:
    DummyDeviceTestSuite() : test_suite("DummyDevice test suite") {
      boost::shared_ptr<DummyBackendTest> dummyDeviceTest( new DummyBackendTest );

      add( BOOST_CLASS_TEST_CASE( &DummyBackendTest::testExceptions, dummyDeviceTest ) );
      add( BOOST_CLASS_TEST_CASE( &DummyBackendTest::testSoftwareTriggeredMode, dummyDeviceTest ) );
      add( BOOST_CLASS_TEST_CASE( &DummyBackendTest::testAutoTriggerMode, dummyDeviceTest ) );
    }};

/**********************************************************************************************************************/
test_suite* init_unit_test_suite( int /*argc*/, char* /*argv*/ [] )
{
  framework::master_test_suite().p_name.value = "AD16 DummyDevice test suite";
  framework::master_test_suite().add(new DummyDeviceTestSuite);

  return NULL;
}

/**********************************************************************************************************************/
void DummyBackendTest::openDevice() {
  _dummy.open(DUMMY_ALIAS);
}

/**********************************************************************************************************************/
void DummyBackendTest::testExceptions() {
  std::cout << "testExceptions" << std::endl;

  // close the not opened device
  BOOST_CHECK_THROW( _dummyBackend->close(), DummyBackendException);
  try {
    _dummyBackend->close();
  }
  catch(DummyBackendException &a) {
    BOOST_CHECK( a.getID() == DummyBackendException::ALREADY_CLOSED);
  }

  // open twice
  _dummyBackend->open();
  BOOST_CHECK_THROW( _dummyBackend->open(), DummyBackendException);
  try {
    _dummyBackend->open();
  }
  catch(DummyBackendException &a) {
    BOOST_CHECK( a.getID() == DummyBackendException::ALREADY_OPEN);
  }

  // close again
  _dummyBackend->close();

  // open it now, should not throw an exception
  _dummyBackend->open();

  // close again
  _dummyBackend->close();

  // close the not opened device again
  BOOST_CHECK_THROW( _dummyBackend->close(), DummyBackendException);
  try {
    _dummyBackend->close();
  }
  catch(DummyBackendException &a) {
    BOOST_CHECK( a.getID() == DummyBackendException::ALREADY_CLOSED);
  }
}

/**********************************************************************************************************************/
void DummyBackendTest::testSoftwareTriggeredMode() {
  std::cout << "testSoftwareTriggeredMode" << std::endl;
  openDevice();

  // set test value of dummy device (to have something changing between the tests). Will be the content of the 3rd channel
  _dummyBackend->testValue = 1;

  // select ADCA ready as DAQ strobe
  _dummy.getRegisterAccessor("WORD_DAQ_STR_SEL","APP0")->write(0);        // DAQ_STROBE_ADCA

  // enable software trigger
  _dummy.getRegisterAccessor("WORD_TIMING_TRG_SEL","APP0")->write(8);

  // set ADC sample rate
  _dummy.getRegisterAccessor("WORD_ADC_A_TIMING_DIV","AD160")->write(999);
  _dummy.getRegisterAccessor("WORD_ADC_B_TIMING_DIV","AD160")->write(999);

  // enable DAQ
  _dummy.getRegisterAccessor("WORD_DAQ_ENABLE","APP0")->write(1);

  // send user trigger
  _dummy.getRegisterAccessor("WORD_TIMING_USER_TRG","APP0")->write(1);

  // advance dummy strobe timing until buffer is full
  bool ERROR_FOUND = false;
  for(int i=0; i<65536; i++) {
    if( round(_dummyBackend->timers.getRemaining()*1000000.) != 20000 ) {
      if(!ERROR_FOUND) std::cerr << "Wrong strobe timing: " << _dummyBackend->timers.getRemaining()*1000000. << std::endl;
      ERROR_FOUND = true;
    }
    _dummyBackend->timers.advance("strobe");
  }
  BOOST_CHECK( !ERROR_FOUND );

  // obtain current buffer
  int currentBuffer;
  _dummy.getRegisterAccessor("WORD_DAQ_CURR_BUF","APP0")->read(&currentBuffer);

  // create accessor for multiplexed data
  boost::shared_ptr< mtca4u::MultiplexedDataAccessor<int32_t> > dataDemuxed;
  std::cout << "currentBuffer = " << currentBuffer << std::endl;
  if(currentBuffer == 0) {
    dataDemuxed = _dummy.getCustomAccessor< mtca4u::MultiplexedDataAccessor<int32_t> >("DAQ0_ADCA", "APP0");
  }
  else {
    dataDemuxed = _dummy.getCustomAccessor< mtca4u::MultiplexedDataAccessor<int32_t> >("DAQ0_ADCB", "APP0");
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

  _dummy.close();
}

/**********************************************************************************************************************/
void DummyBackendTest::testAutoTriggerMode() {
  std::cout << "testAutoTriggerMode" << std::endl;
  openDevice();
  _dummy.getRegisterAccessor("WORD_RESET_N","BOARD0")->write(0);        // reset

  // set test value of dummy device (to have something changing between the tests). Will be the content of the 3rd channel
  _dummyBackend->testValue = 2;

  // select ADCA ready as DAQ strobe
  _dummy.getRegisterAccessor("WORD_DAQ_STR_SEL","APP0")->write(0);        // DAQ_STROBE_ADCA

  // enable trigger 0 and set it to 1 Hz
  _dummy.getRegisterAccessor("WORD_TIMING_FREQ","APP0")->write(49999999);
  _dummy.getRegisterAccessor("WORD_TIMING_TRG_SEL","APP0")->write(0);

  // set ADC sample rate
  _dummy.getRegisterAccessor("WORD_ADC_A_TIMING_DIV","AD160")->write(499);
  _dummy.getRegisterAccessor("WORD_ADC_B_TIMING_DIV","AD160")->write(499);

  // obtain current buffer
  int lastBuffer;
  _dummy.getRegisterAccessor("WORD_DAQ_CURR_BUF","APP0")->read(&lastBuffer);
  std::cout << "lastBuffer = " << lastBuffer << std::endl; std::cerr << std::flush;

  // enable DAQ
  _dummy.getRegisterAccessor("WORD_DAQ_ENABLE","APP0")->write(1);

  // check trigger timing
  BOOST_CHECK( round(_dummyBackend->trigger.getRemaining()*1000000.) == 1000000000. );

  // wait until conversion is complete
  int currentBuffer;
  bool ERROR_FOUND = false;
  do {
    _dummy.getRegisterAccessor("WORD_DAQ_CURR_BUF","APP0")->read(&currentBuffer);
    if( round(_dummyBackend->timers.getRemaining()*1000000.) != 10000 ) {
      if(!ERROR_FOUND) std::cerr << "Wrong strobe timing: " << _dummyBackend->timers.getRemaining()*1000000. << std::endl;
      ERROR_FOUND = true;
    }
    _dummyBackend->timers.advance("strobe");
  } while(currentBuffer == lastBuffer);
  BOOST_CHECK( !ERROR_FOUND );

  // create accessor for multiplexed data
  boost::shared_ptr< mtca4u::MultiplexedDataAccessor<int32_t> > dataDemuxed;
  if(currentBuffer == 1) {
    dataDemuxed = _dummy.getCustomAccessor< mtca4u::MultiplexedDataAccessor<int32_t> >("DAQ0_ADCA", "APP0");
  }
  else {
    dataDemuxed = _dummy.getCustomAccessor< mtca4u::MultiplexedDataAccessor<int32_t> >("DAQ0_ADCB", "APP0");
  }
  dataDemuxed->read();

  // The accessor expects that all sequences are of the same length, and that a
  // described region should have at least one sequence.
  int lengthOfaSequence = (*dataDemuxed)[0].size();

  // check data:
  ERROR_FOUND = false;
  for(int iSample = 0; iSample < lengthOfaSequence; ++iSample) {
    if( ( (*dataDemuxed)[1][iSample] != iSample ) || ( (*dataDemuxed)[2][iSample] != 2 ) ) {
      std::cout << std::flush;
      if(!ERROR_FOUND) std::cerr << "Wrong sample values for sample " << iSample << ": " << (*dataDemuxed)[1][iSample] << " " << (*dataDemuxed)[2][iSample] << std::endl;
      ERROR_FOUND = true;
    }
  }
  BOOST_CHECK( !ERROR_FOUND );

  // wait until another conversion is complete (this time via the trigger timer)
  _dummy.getRegisterAccessor("WORD_DAQ_CURR_BUF","APP0")->read(&lastBuffer);
  _dummyBackend->timers.advance("trigger");
  _dummy.getRegisterAccessor("WORD_DAQ_CURR_BUF","APP0")->read(&currentBuffer);
  BOOST_CHECK( lastBuffer != currentBuffer );

  _dummy.close();
}
