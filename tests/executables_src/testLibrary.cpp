#include <boost/test/included/unit_test.hpp>
using namespace boost::unit_test_framework;
#include <boost/lambda/lambda.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>

#include "ad16.h"
using namespace mtca4u;

#define TEST_MAPPING_FILE "ad16_scope_fmc25_r1224.mapp"

// forward declaration so we can declare it friend
// of TestableDummyDevice.
class Ad16Test;

/**********************************************************************************************************************/
/** The TestableAd16 is derived from ad16 DummyDevice to get access to the protected members.
 *  This is done by declaring DummyLibTest as a friend.
 */
class TestableAd16 : public ad16 {
    friend class Ad16Test;
};

/**********************************************************************************************************************/
class Ad16Test {
  public:
    Ad16Test() {
    }

    void testExceptions();
    void testTriggerModes();
    void testSoftwareTriggeredMode();
    void testPeriodicTriggeredMode();

  private:
    TestableAd16 ad;
    friend class Ad16TestSuite;

};

/**********************************************************************************************************************/
class Ad16TestSuite : public test_suite {
  public:
    Ad16TestSuite() : test_suite("DummyDevice test suite"){
      boost::shared_ptr<Ad16Test> ad16Test( new Ad16Test );
      add( BOOST_CLASS_TEST_CASE( &Ad16Test::testExceptions, ad16Test ) );
      add( BOOST_CLASS_TEST_CASE( &Ad16Test::testTriggerModes, ad16Test ) );
      add( BOOST_CLASS_TEST_CASE( &Ad16Test::testSoftwareTriggeredMode, ad16Test ) );
      add( BOOST_CLASS_TEST_CASE( &Ad16Test::testPeriodicTriggeredMode, ad16Test ) );
    }
};

/**********************************************************************************************************************/
test_suite* init_unit_test_suite( int /*argc*/, char* /*argv*/ [] ) {
  framework::master_test_suite().p_name.value = "AD16 DummyDevice test suite";
  framework::master_test_suite().add(new Ad16TestSuite);

  return NULL;
}

/**********************************************************************************************************************/
void Ad16Test::testExceptions() {
  std::cout << "testExceptions" << std::endl;

  // tests before opening the device: should all throw exceptions
  // close device
  BOOST_CHECK_THROW( ad.close() , ad16Exception);
  try {
    ad.close();
  }
  catch(ad16Exception &a) {
    BOOST_CHECK( a.getID() == ad16Exception::NOT_OPENED);
  }

  // send user trigger
  BOOST_CHECK_THROW( ad.sendUserTrigger() , ad16Exception);
  try {
    ad.sendUserTrigger();
  }
  catch(ad16Exception &a) {
    BOOST_CHECK( a.getID() == ad16Exception::NOT_OPENED);
  }

  // open /dev/null as device
  BOOST_CHECK_THROW( ad.open("/dev/null",TEST_MAPPING_FILE) , ad16Exception);
  try {
    ad.open("/dev/null",TEST_MAPPING_FILE);
  }
  catch(ad16Exception &a) {
    BOOST_CHECK( a.getID() == ad16Exception::CANNOT_OPEN);
  }

  // open non-existing dmap file
  BOOST_CHECK_THROW( ad.openDmap("this_file_does_not_exist","AD16") , ad16Exception);
  try {
    ad.openDmap("this_file_does_not_exist","AD16");
  }
  catch(ad16Exception &a) {
    BOOST_CHECK( a.getID() == ad16Exception::CANNOT_OPEN);
  }

  // open dummy-PCIe device (wrong type of PCIe device)
  BOOST_CHECK_THROW( ad.open("/dev/mtcadummys0",TEST_MAPPING_FILE) , ad16Exception);
  ad.close();
  try {
    ad.open("/dev/mtcadummys0",TEST_MAPPING_FILE);
  }
  catch(ad16Exception &a) {
    BOOST_CHECK( a.getID() == ad16Exception::CANNOT_OPEN);
  }
  ad.close();

  // open device
  ad.openDmap("ad16.dmap","DUMMY");

  // open a second time: should throw exception
  BOOST_CHECK_THROW( ad.open(TEST_MAPPING_FILE,TEST_MAPPING_FILE) , ad16Exception);
  try {
    ad.open(TEST_MAPPING_FILE,TEST_MAPPING_FILE);
  }
  catch(ad16Exception &a) {
    BOOST_CHECK( a.getID() == ad16Exception::ALREADY_OPENED);
  }

  // set illegal sampling rate
  BOOST_CHECK_THROW( ad.setSamplingRate(100000.1) , ad16Exception);
  try {
    ad.setSamplingRate(100000.1);
  }
  catch(ad16Exception &a) {
    BOOST_CHECK( a.getID() == ad16Exception::IMPOSSIBLE_TIMING_CONFIGURATION);
  }

  // set illegal combination of sampling rate and oversampling ratio
  BOOST_CHECK_THROW( ad.setSamplingRate(100000., ad16::RATIO_4) , ad16Exception);
  try {
    ad.setSamplingRate(100000., ad16::RATIO_4);
  }
  catch(ad16Exception &a) {
    BOOST_CHECK( a.getID() == ad16Exception::IMPOSSIBLE_TIMING_CONFIGURATION);
  }

  // trigger-type argument for PERIODIC missing
  BOOST_CHECK_THROW( ad.setTriggerMode(ad16::PERIODIC) , ad16Exception);
  try {
    ad.setTriggerMode(ad16::PERIODIC);
  }
  catch(ad16Exception &a) {
    BOOST_CHECK( a.getID() == ad16Exception::ILLEGAL_PARAMETER);
  }

  // trigger-type argument for EXTERNAL of wrong type
  BOOST_CHECK_THROW( ad.setTriggerMode(ad16::EXTERNAL,std::string("test")) , ad16Exception);
  try {
    ad.setTriggerMode(ad16::EXTERNAL,std::string("test"));
  }
  catch(ad16Exception &a) {
    BOOST_CHECK( a.getID() == ad16Exception::ILLEGAL_PARAMETER);
  }

  // trigger-type argument for EXTERNAL out of range
  BOOST_CHECK_THROW( ad.setTriggerMode(ad16::EXTERNAL,-1) , ad16Exception);
  try {
    ad.setTriggerMode(ad16::EXTERNAL,-1);
  }
  catch(ad16Exception &a) {
    BOOST_CHECK( a.getID() == ad16Exception::INCORRECT_TRIGGER_SETTING);
  }

  // timeout when sending a user trigger without enabled DAQ
  BOOST_CHECK_THROW( ad.sendUserTrigger() , ad16Exception);
  try {
    ad.sendUserTrigger();
  }
  catch(ad16Exception &a) {
    BOOST_CHECK( a.getID() == ad16Exception::TIMEOUT);
  }

  // read data (otherwise getChannelData() does not work)
  ad.read();

  // get channel out of range: should throw exception
  BOOST_CHECK_THROW( ad.getChannelData(16) , ad16Exception);
  try {
    ad.getChannelData(16);
  }
  catch(ad16Exception &a) {
    BOOST_CHECK(a.getID() == ad16Exception::CHANNEL_OUT_OF_RANGE);
  }

  // close device
  ad.close();

}

/**********************************************************************************************************************/
void Ad16Test::testTriggerModes() {
  std::cout << "testTriggerModes" << std::endl;
  mapFile::mapElem elem;
  int32_t val;

  // open device
  ad.open(TEST_MAPPING_FILE,TEST_MAPPING_FILE);

  // test periodic trigger mode
  ad.setTriggerMode(ad16::PERIODIC,1.);
  ad._map->getRegisterInfo("WORD_TIMING_TRG_SEL", elem, "APP0");
  ad._dummyDevice->readReg(elem.reg_address,&val,elem.reg_bar);
  BOOST_CHECK( val == 0 );
  ad._map->getRegisterInfo("WORD_TIMING_INT_ENA", elem, "APP0");
  ad._dummyDevice->readReg(elem.reg_address,&val,elem.reg_bar);
  BOOST_CHECK( (val & (1<<0)) != 0 );
  ad._map->getRegisterInfo("WORD_TIMING_FREQ", elem, "APP0");
  ad._dummyDevice->readReg(elem.reg_address,&val,elem.reg_bar);
  BOOST_CHECK( val == 49999999 );               // 1 Hz

  // test user trigger mode
  ad.setTriggerMode(ad16::USER);
  ad._map->getRegisterInfo("WORD_TIMING_TRG_SEL", elem, "APP0");
  ad._dummyDevice->readReg(elem.reg_address,&val,elem.reg_bar);
  BOOST_CHECK( val == 8 );
  ad._map->getRegisterInfo("WORD_TIMING_INT_ENA", elem, "APP0");
  ad._dummyDevice->readReg(elem.reg_address,&val,elem.reg_bar);
  BOOST_CHECK( (val & (1<<8)) == 0 );

  // test external trigger mode on channel 3
  ad.setTriggerMode(ad16::EXTERNAL,3);
  ad._map->getRegisterInfo("WORD_TIMING_TRG_SEL", elem, "APP0");
  ad._dummyDevice->readReg(elem.reg_address,&val,elem.reg_bar);
  BOOST_CHECK( val == 3 );
  ad._map->getRegisterInfo("WORD_TIMING_INT_ENA", elem, "APP0");
  ad._dummyDevice->readReg(elem.reg_address,&val,elem.reg_bar);
  BOOST_CHECK( (val & (1<<3)) == 0 );

  // test external trigger mode on channel 0 after the PERIODIC trigger was enabled (they are on the same channel)
  ad.setTriggerMode(ad16::PERIODIC,1.);
  ad.setTriggerMode(ad16::EXTERNAL,0);
  ad._map->getRegisterInfo("WORD_TIMING_TRG_SEL", elem, "APP0");
  ad._dummyDevice->readReg(elem.reg_address,&val,elem.reg_bar);
  BOOST_CHECK( val == 0 );
  ad._map->getRegisterInfo("WORD_TIMING_INT_ENA", elem, "APP0");
  ad._dummyDevice->readReg(elem.reg_address,&val,elem.reg_bar);
  BOOST_CHECK( (val & (1<<0)) == 0 );

  // close device
  ad.close();

}

/**********************************************************************************************************************/
void Ad16Test::testSoftwareTriggeredMode() {
  std::cout << "testSoftwareTriggeredMode" << std::endl;

  // open device
  ad.open(TEST_MAPPING_FILE,TEST_MAPPING_FILE);

  // setup software trigger
  ad.setTriggerMode(ad16::USER);

  // enable the DAQ
  ad.enableDaq();

  // wait until conversion is complete
  for(int i=0; i<65536; i++) ad._dummyDevice->timers.advanceAll();

  // send another trigger to swap the buffers
  ad.sendUserTrigger();

  // read data
  ad.read();

  // compare first channel data
  std::vector<int> data = ad.getChannelData(1);
  bool FOUND_ERROR = false;
  for (uint columnCount = 0; columnCount < data.size(); ++columnCount) {
    if( data[columnCount] != (int)columnCount ) FOUND_ERROR = true;
  }
  BOOST_CHECK( !FOUND_ERROR );

  // close device
  ad.close();

}

/**********************************************************************************************************************/
void Ad16Test::testPeriodicTriggeredMode() {
  std::cout << "testPeriodicTriggeredMode" << std::endl;

  // open device
  ad.open(TEST_MAPPING_FILE,TEST_MAPPING_FILE);

  // setup software trigger
  ad.setTriggerMode(ad16::PERIODIC,1.);    // 1 Hz

  // enable the DAQ
  ad.enableDaq();

  // wait until conversion is complete
  while(!ad.conversionComplete()) ad._dummyDevice->timers.advanceAll();

  // read data
  ad.read();

  // compare first channel data
  std::vector<int> data = ad.getChannelData(1);
  bool FOUND_ERROR = false;
  for (uint columnCount = 0; columnCount < data.size(); ++columnCount) {
    if( data[columnCount] != (int)columnCount ) FOUND_ERROR = true;
  }
  BOOST_CHECK( !FOUND_ERROR );

  // close device
  ad.close();

}
