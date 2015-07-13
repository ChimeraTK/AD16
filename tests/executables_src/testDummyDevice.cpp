#include <boost/test/included/unit_test.hpp>
using namespace boost::unit_test_framework;
#include <boost/lambda/lambda.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>

#include "ad16DummyDevice.h"
#include <MtcaMappedDevice/devMap.h>
using namespace mtca4u;

#define TEST_MAPPING_FILE "ad16dummy.map"

// forward declaration so we can declare it friend
// of TestableDummyDevice.
class DummyDeviceTest;

/** The TestableDummyDevice is derived from 
 *  DummyDevice to get access to the protected members.
 *  This is done by declaring DummyDeviceTest as a friend.
 */
class TestableDummyDevice : public ad16DummyDevice
{
    friend class DummyDeviceTest;
};

class DummyDeviceTest{
  public:
    DummyDeviceTest() {
      _dummyDevice = boost::shared_ptr<TestableDummyDevice>( new TestableDummyDevice() );
    }

    void testSoftwareTriggeredMode();

  private:
    //TestableDummyDevice _dummyDevice;
    boost::shared_ptr<TestableDummyDevice> _dummyDevice;
    devMap<devBase> _dummyMapped;
    void freshlyOpenDevice();
    friend class DummyDeviceTestSuite;

};

class  DummyDeviceTestSuite : public test_suite{
  public:
    DummyDeviceTestSuite() : test_suite("DummyDevice test suite"){
      boost::shared_ptr<DummyDeviceTest> dummyDeviceTest( new DummyDeviceTest );

      add( BOOST_CLASS_TEST_CASE( &DummyDeviceTest::testSoftwareTriggeredMode, dummyDeviceTest ) );
    }};

test_suite* init_unit_test_suite( int /*argc*/, char* /*argv*/ [] )
{
  framework::master_test_suite().p_name.value = "AD16 DummyDevice test suite";
  framework::master_test_suite().add(new DummyDeviceTestSuite);

  return NULL;
}

void DummyDeviceTest::freshlyOpenDevice(){
  try{
    _dummyDevice->openDev(TEST_MAPPING_FILE);
    _dummyMapped.openDev( _dummyDevice, _dummyDevice->_registerMapping);
  }
  catch(DummyDeviceException &){
    // make sure the device was freshly opened, so
    // registers are set to 0.
    _dummyMapped.closeDev();
    _dummyDevice->closeDev();
    _dummyDevice->openDev(TEST_MAPPING_FILE);
    _dummyMapped.openDev( _dummyDevice, _dummyDevice->_registerMapping);
  }
}

void DummyDeviceTest::testSoftwareTriggeredMode(){
  freshlyOpenDevice();

  // start conversion
  int32_t val = 1;
  _dummyMapped.writeReg("START_CONVERSION", "AD16", &val);

  // create accessor for multiplexed data
  boost::shared_ptr< mtca4u::MultiplexedDataAccessor<int32_t> > dataDemuxed =
      _dummyMapped.getCustomAccessor< mtca4u::MultiplexedDataAccessor<int32_t> >("DMA", "AD16");
  dataDemuxed->read();

  // Return the number of sequences found: should be 16
  uint numberOfDataSequences = dataDemuxed->getNumberOfDataSequences();
  std::cout << "Number Of dataSequences extracted: " << numberOfDataSequences << std::endl;

  // The accessor expects that all sequences are of the same length, and that a
  // described region should have at least one sequence.
  uint lengthOfaSequence = (*dataDemuxed)[0].size();
  std::cout << "Length of each sequence: " << lengthOfaSequence << std::endl;

  // print data
  for (uint rowCount = 0; rowCount < numberOfDataSequences; ++rowCount) {
    std::cout << "Sequence: " << rowCount << std::endl;
    for (uint columnCount = 0; columnCount < lengthOfaSequence; ++columnCount) {
      std::cout << (*dataDemuxed)[rowCount][columnCount] << std::endl;
    }
    std::cout << std::endl;
  }

}

