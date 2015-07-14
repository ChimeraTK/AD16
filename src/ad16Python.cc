#include <ad16.h>
#include <boost/python.hpp>
#include <boost/noncopyable.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <vector>

using namespace mtca4u;
using namespace boost::python;

BOOST_PYTHON_MODULE(libad16)
{

  class_< std::vector<int> >("VectorInt")
     .def(vector_indexing_suite< std::vector<int> >() );

  class_<ad16, boost::noncopyable>("ad16")
    .def("open",&ad16::open)
    .def("close",&ad16::close)
    .def("startConversion",&ad16::startConversion)
    .def("conversionComplete",&ad16::conversionComplete)
    .def("setSamplingRate",&ad16::setSamplingRate)
    .def("setSamplesPerBlock",&ad16::setSamplesPerBlock)
    .def("read",&ad16::read)
    .def("getChannelData",&ad16::getChannelData)
    ;

  enum_<ad16::rate>("rate")
    .value("10000Hz",ad16::RATE_10000Hz)
    .value("5000Hz", ad16::RATE_5000Hz)
    .value("3333Hz", ad16::RATE_3333Hz)
    .value("2500Hz", ad16::RATE_2500Hz)
    .value("2000Hz", ad16::RATE_2000Hz)
    .value("1000Hz", ad16::RATE_1000Hz)
    .value("100Hz", ad16::RATE_100Hz)
  ;
}
