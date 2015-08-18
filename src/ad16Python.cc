#include <ad16.h>
#include <boost/python.hpp>
#include <boost/noncopyable.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <vector>

using namespace mtca4u;
using namespace boost::python;

BOOST_PYTHON_MODULE(libad16)
{

  class_<ad16, boost::noncopyable>("ad16")
    .def("open",&ad16::open)
    .def("close",&ad16::close)
    .def("setSamplingRate",&ad16::setSamplingRate)
    .def("getSamplingRate",&ad16::getSamplingRate)
    .def("setVoltageRange",&ad16::setVoltageRange)
    .def("setTriggerMode",&ad16::setTriggerModePy1)
    .def("setTriggerMode",&ad16::setTriggerModePy2)
    .def("setTriggerMode",&ad16::setTriggerModePy3)
    .def("enableDaq",&ad16::enableDaq)
    .def("sendUserTrigger",&ad16::sendUserTrigger)
    .def("conversionComplete",&ad16::conversionComplete)
    .def("read",&ad16::read)
    .def("getChannelData",&ad16::getChannelDataNumpy)
    ;

  enum_<ad16::oversampling>("oversampling")
    .value("NO_OVERSAMPLING",ad16::NO_OVERSAMPLING)
    .value("RATIO_2", ad16::RATIO_2)
    .value("RATIO_4", ad16::RATIO_4)
    .value("RATIO_8", ad16::RATIO_8)
    .value("RATIO_16", ad16::RATIO_16)
    .value("RATIO_32", ad16::RATIO_32)
    .value("RATIO_64", ad16::RATIO_64)
  ;

  enum_<ad16::voltageRange>("voltageRange")
    .value("RANGE_5Vpp",ad16::RANGE_5Vpp)
    .value("RANGE_10Vpp", ad16::RANGE_10Vpp)
  ;

  enum_<ad16::trigger>("trigger")
    .value("PERIODIC",ad16::PERIODIC)
    .value("USER", ad16::USER)
    .value("EXTERNAL", ad16::EXTERNAL)
  ;

  numeric::array::set_module_and_type("numpy", "ndarray");
}
