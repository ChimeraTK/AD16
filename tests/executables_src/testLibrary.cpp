#include <boost/test/included/unit_test.hpp>
using namespace boost::unit_test_framework;

#include <boost/lambda/lambda.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>

#include "ad16.h"
#include "ad16dummy.h"

using namespace mtca4u;

#define DUMMY_ALIAS "DUMMY"
#define DUMMY_ALIAS_WRONG "WRONG_DUMMY"

/**********************************************************************************************************************/
class Ad16Test {
  public:
    Ad16Test() {
      backend = boost::static_pointer_cast<ad16dummy>(BackendFactory::getInstance().createBackend(DUMMY_ALIAS));
    }

    void testExceptions();
    void testTriggerModes();
    void testSoftwareTriggeredMode();
    void testPeriodicTriggeredMode();

  private:
    ad16 ad;
    boost::shared_ptr<ad16dummy> backend;
    friend class Ad16TestSuite;

};

/**********************************************************************************************************************/
class Ad16TestSuite: public test_suite {
  public:
    Ad16TestSuite()
        : test_suite("DummyDevice test suite") {
      boost::shared_ptr<Ad16Test> ad16Test(new Ad16Test);
      add(BOOST_CLASS_TEST_CASE(&Ad16Test::testExceptions, ad16Test));
      add(BOOST_CLASS_TEST_CASE(&Ad16Test::testTriggerModes, ad16Test));
      add(BOOST_CLASS_TEST_CASE(&Ad16Test::testSoftwareTriggeredMode, ad16Test));
      add(BOOST_CLASS_TEST_CASE(&Ad16Test::testPeriodicTriggeredMode, ad16Test));
    }
};

/**********************************************************************************************************************/
test_suite* init_unit_test_suite(int /*argc*/, char* /*argv*/[]) {
  framework::master_test_suite().p_name.value = "AD16 DummyDevice test suite";
  framework::master_test_suite().add(new Ad16TestSuite);

  return NULL;
}

/**********************************************************************************************************************/
class TestSignalGenerator {
  public:
    boost::shared_ptr<SignalSource> sampleCounter;

    TestSignalGenerator() {
      sampleCounter = boost::make_shared<SignalSource>();
      sampleCounter->setCallback( boost::bind(&TestSignalGenerator::sampleCounterCallback, this, _1) );
      sampleCount = 0;
      maxVoltage = 5.;
    }

    double sampleCounterCallback(double) {
      return (double)(sampleCount++) / pow(2., 17) * maxVoltage;
    }

    int sampleCount;
    double maxVoltage;
};

/**********************************************************************************************************************/
void Ad16Test::testExceptions() {
  std::cout << "testExceptions" << std::endl;

  // tests before opening the device: should all throw exceptions
  // close device
  BOOST_CHECK_THROW(ad.close(), ad16Exception);
  try {
    ad.close();
  }
  catch(ad16Exception &a) {
    BOOST_CHECK(a.getID() == ad16Exception::NOT_OPENED);
  }

  // send user trigger
  BOOST_CHECK_THROW(ad.sendUserTrigger(), ad16Exception);
  try {
    ad.sendUserTrigger();
  }
  catch(ad16Exception &a) {
    BOOST_CHECK(a.getID() == ad16Exception::NOT_OPENED);
  }

  // open dummy-PCIe device (wrong type of PCIe device)
  BOOST_CHECK_THROW(ad.open(DUMMY_ALIAS_WRONG), ad16Exception);
  ad.close();
  try {
    ad.open(DUMMY_ALIAS_WRONG);
  }
  catch(ad16Exception &a) {
    BOOST_CHECK(a.getID() == ad16Exception::CANNOT_OPEN);
  }
  ad.close();

  // open device
  ad.open(DUMMY_ALIAS);

  // open a second time: should throw exception
  BOOST_CHECK_THROW(ad.open(DUMMY_ALIAS), ad16Exception);
  try {
    ad.open(DUMMY_ALIAS);
  }
  catch(ad16Exception &a) {
    BOOST_CHECK(a.getID() == ad16Exception::ALREADY_OPENED);
  }

  // set illegal sampling rate
  BOOST_CHECK_THROW(ad.setSamplingRate(100000.1), ad16Exception);
  try {
    ad.setSamplingRate(100000.1);
  }
  catch(ad16Exception &a) {
    BOOST_CHECK(a.getID() == ad16Exception::IMPOSSIBLE_TIMING_CONFIGURATION);
  }

  // set illegal combination of sampling rate and oversampling ratio
  BOOST_CHECK_THROW(ad.setSamplingRate(100000., ad16::RATIO_4), ad16Exception);
  try {
    ad.setSamplingRate(100000., ad16::RATIO_4);
  }
  catch(ad16Exception &a) {
    BOOST_CHECK(a.getID() == ad16Exception::IMPOSSIBLE_TIMING_CONFIGURATION);
  }

  // trigger-type argument for PERIODIC missing
  BOOST_CHECK_THROW(ad.setTriggerMode(ad16::PERIODIC), ad16Exception);
  try {
    ad.setTriggerMode(ad16::PERIODIC);
  }
  catch(ad16Exception &a) {
    BOOST_CHECK(a.getID() == ad16Exception::ILLEGAL_PARAMETER);
  }

  // trigger-type argument for EXTERNAL of wrong type
  BOOST_CHECK_THROW(ad.setTriggerMode(ad16::EXTERNAL, std::string("test")), ad16Exception);
  try {
    ad.setTriggerMode(ad16::EXTERNAL, std::string("test"));
  }
  catch(ad16Exception &a) {
    BOOST_CHECK(a.getID() == ad16Exception::ILLEGAL_PARAMETER);
  }

  // trigger-type argument for EXTERNAL out of range
  BOOST_CHECK_THROW(ad.setTriggerMode(ad16::EXTERNAL, -1), ad16Exception);
  try {
    ad.setTriggerMode(ad16::EXTERNAL, -1);
  }
  catch(ad16Exception &a) {
    BOOST_CHECK(a.getID() == ad16Exception::INCORRECT_TRIGGER_SETTING);
  }

  // timeout when sending a user trigger without enabled DAQ
  BOOST_CHECK_THROW(ad.sendUserTrigger(), ad16Exception);
  try {
    ad.sendUserTrigger();
  }
  catch(ad16Exception &a) {
    BOOST_CHECK(a.getID() == ad16Exception::TIMEOUT);
  }

  // read data (otherwise getChannelData() does not work)
  ad.read();

  // get channel out of range: should throw exception
  BOOST_CHECK_THROW(ad.getChannelData(16), ad16Exception);
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
  RegisterInfoMap::RegisterInfo elem;

  // open device
  ad.open(DUMMY_ALIAS);

  // test periodic trigger mode
  ad.setTriggerMode(ad16::PERIODIC, 1.);
  BOOST_CHECK(backend->regTrigSel == 0);
  BOOST_CHECK((backend->regTrigIntEna & (1 << 0)) != 0);
  BOOST_CHECK(backend->regTrigFreq == 49999999);               // 1 Hz

  // test user trigger mode
  ad.setTriggerMode(ad16::USER);
  BOOST_CHECK(backend->regTrigSel == 8);
  BOOST_CHECK((backend->regTrigIntEna & (1 << 8)) == 0);

  // test external trigger mode on channel 3
  ad.setTriggerMode(ad16::EXTERNAL, 3);
  BOOST_CHECK(backend->regTrigSel == 3);
  BOOST_CHECK((backend->regTrigIntEna & (1 << 3)) == 0);

  // test external trigger mode on channel 0 after the PERIODIC trigger was enabled (they are on the same channel)
  ad.setTriggerMode(ad16::PERIODIC, 1.);
  ad.setTriggerMode(ad16::EXTERNAL, 0);
  BOOST_CHECK(backend->regTrigSel == 0);
  BOOST_CHECK((backend->regTrigIntEna & (1 << 0)) == 0);

  // close device
  ad.close();

}

/**********************************************************************************************************************/
void Ad16Test::testSoftwareTriggeredMode() {
  std::cout << "testSoftwareTriggeredMode" << std::endl;

  // open device
  ad.open(DUMMY_ALIAS);

  // create and connect signal generator
  TestSignalGenerator generator;
  generator.maxVoltage = 5.;
  backend->sinks[1]->connect(generator.sampleCounter);

  // setup user trigger
  ad.setTriggerMode(ad16::USER);
  ad.setVoltageRange(ad16::RANGE_5Vpp);

  // enable the DAQ
  ad.enableDaq();

  // wait until conversion is complete
  for(int i = 0; i < 65536; i++) backend->timers.advance("strobe");

  // send another trigger to swap the buffers
  ad.sendUserTrigger();

  // read data
  ad.read();

  // compare second channel's data
  std::vector<int> data = ad.getChannelData(1);
  bool FOUND_ERROR = false;
  for(uint columnCount = 0; columnCount < data.size(); ++columnCount) {
    if(data[columnCount] != (int) columnCount) FOUND_ERROR = true;
  }
  BOOST_CHECK(!FOUND_ERROR);

  // close device
  ad.close();

}

/**********************************************************************************************************************/
void Ad16Test::testPeriodicTriggeredMode() {
  std::cout << "testPeriodicTriggeredMode" << std::endl;

  // open device
  ad.open(DUMMY_ALIAS);

  // create and connect signal generator
  TestSignalGenerator generator;
  generator.maxVoltage = 10.;
  backend->sinks[1]->connect(generator.sampleCounter);

  // setup software trigger and voltage range
  ad.setTriggerMode(ad16::PERIODIC, 1.);    // 1 Hz
  ad.setVoltageRange(ad16::RANGE_10Vpp);

  // enable the DAQ
  ad.enableDaq();

  // wait until conversion is complete
  BOOST_CHECK( ad.conversionComplete() == false );
  backend->timers.advance("trigger");
  BOOST_CHECK( ad.conversionComplete() == true );

  // read data
  ad.read();

  // compare second channel's data
  std::vector<int> data = ad.getChannelData(1);
  bool FOUND_ERROR = false;
  for(uint columnCount = 0; columnCount < data.size(); ++columnCount) {
    if(data[columnCount] != (int) columnCount) FOUND_ERROR = true;
  }
  BOOST_CHECK(!FOUND_ERROR);

  // close device
  ad.close();

}
