#include <boost/test/included/unit_test.hpp>
using namespace boost::unit_test_framework;
#include <boost/lambda/lambda.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>

#include "ad16.h"
using namespace mtca4u;

#define TEST_MAPPING_FILE "ad16dummy.map"

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

    void testSoftwareTriggeredMode();
    void testExceptions();

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
      add( BOOST_CLASS_TEST_CASE( &Ad16Test::testSoftwareTriggeredMode, ad16Test ) );
    }
};

/**********************************************************************************************************************/
test_suite* init_unit_test_suite( int /*argc*/, char* /*argv*/ [] ) {
  framework::master_test_suite().p_name.value = "AD16 DummyDevice test suite";
  framework::master_test_suite().add(new Ad16TestSuite);

  return NULL;
}

/**********************************************************************************************************************/
void Ad16Test::testSoftwareTriggeredMode() {

  // open device
  ad.open(TEST_MAPPING_FILE,TEST_MAPPING_FILE);

  // set number of samples per block
  //ad.setSamplesPerBlock(16);

  // trigger conversion
  ad.startConversion();

  // wait until conversion is complete
  while(!ad.conversionComplete()) usleep(1);

  // read data
  ad.read();

  // compare first channel data
  std::vector<int> data = ad.getChannelData(0);
  for (uint columnCount = 0; columnCount < data.size(); ++columnCount) {
    BOOST_CHECK( data[columnCount] == (int)columnCount );
  }

  // close device
  ad.close();

}

/**********************************************************************************************************************/
void Ad16Test::testExceptions() {

  // tests before opening the device: should all throw exceptions
  // close device
  BOOST_CHECK_THROW( ad.close() , ad16Exception);
  try {
    ad.close();
  }
  catch(ad16Exception &a) {
    BOOST_CHECK( a.getID() == ad16Exception::NOT_OPENED);
  }
  // trigger conversion
  BOOST_CHECK_THROW( ad.startConversion() , ad16Exception);
  try {
    ad.startConversion();
  }
  catch(ad16Exception &a) {
    BOOST_CHECK( a.getID() == ad16Exception::NOT_OPENED);
  }

  // open device
  ad.open(TEST_MAPPING_FILE,TEST_MAPPING_FILE);

  // open a second time: should throw exception
  BOOST_CHECK_THROW( ad.open(TEST_MAPPING_FILE,TEST_MAPPING_FILE) , ad16Exception);
  try {
    ad.open(TEST_MAPPING_FILE,TEST_MAPPING_FILE);
  }
  catch(ad16Exception &a) {
    BOOST_CHECK( a.getID() == ad16Exception::ALREADY_OPENED);
  }

  // trigger conversion
  ad.startConversion();

  // immediately trigger 2nd conversion: should throw exception
  BOOST_CHECK_THROW( ad.startConversion() , ad16Exception);
  try {
    ad.startConversion();
  }
  catch(ad16Exception &a) {
    BOOST_CHECK(a.getID() == ad16Exception::CONVERSION_RUNNING);
  }

  // immediately trigger read: should throw exception
  BOOST_CHECK_THROW( ad.read() , ad16Exception);
  try {
    ad.read();
  }
  catch(ad16Exception &a) {
    BOOST_CHECK(a.getID() == ad16Exception::CONVERSION_RUNNING);
  }

  // wait until conversion is complete
  while(!ad.conversionComplete()) usleep(10000);

  // read data
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

