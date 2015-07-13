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

/** The TestableAd16 is derived from ad16 DummyDevice to get access to the protected members.
 *  This is done by declaring DummyLibTest as a friend.
 */
class TestableAd16 : public ad16
{
    friend class Ad16Test;
};

class Ad16Test{
  public:
    Ad16Test() {
    }

    void testSoftwareTriggeredMode();

  private:
    TestableAd16 ad;
    friend class Ad16TestSuite;

};

class Ad16TestSuite : public test_suite{
  public:
    Ad16TestSuite() : test_suite("DummyDevice test suite"){
      boost::shared_ptr<Ad16Test> ad16Test( new Ad16Test );
      add( BOOST_CLASS_TEST_CASE( &Ad16Test::testSoftwareTriggeredMode, ad16Test ) );
    }};

test_suite* init_unit_test_suite( int /*argc*/, char* /*argv*/ [] )
{
  framework::master_test_suite().p_name.value = "AD16 DummyDevice test suite";
  framework::master_test_suite().add(new Ad16TestSuite);

  return NULL;
}

void Ad16Test::testSoftwareTriggeredMode(){

  // open device
  std::cout << "ad.open()" << std::endl;
  ad.open(TEST_MAPPING_FILE,TEST_MAPPING_FILE);

  // trigger conversion and read datra
  std::cout << "ad.startConversion()"<< std::endl;
  ad.startConversion();
  std::cout << "ad.read()" << std::endl;
  ad.read();

  // print data
  for (uint rowCount = 0; rowCount < 16; ++rowCount) {
    std::vector<int> data = ad.getChannelData(rowCount);
    std::cout << "Channel " << rowCount << " with " << data.size() << " samples:" << std::endl;
    for (uint columnCount = 0; columnCount < data.size(); ++columnCount) {
      std::cout << data[columnCount] << std::endl;
    }
    std::cout << std::endl;
  }

}

