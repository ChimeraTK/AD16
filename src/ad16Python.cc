#include <vector>
#include <boost/python.hpp>
#include <boost/noncopyable.hpp>
#include <boost/python/overloads.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/python/exception_translator.hpp>

#include <ad16.h>

using namespace mtca4u;
using namespace boost::python;

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(setSamplingRate_overloads, ad16::setSamplingRate, 1,2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(enableDaq_overloads, ad16::enableDaq, 0,1)

PyObject *ad16ExceptionType = NULL;

void translateAd16Exception(const ad16Exception &e)
{
    assert(ad16ExceptionType != NULL);
    PyErr_SetObject(ad16ExceptionType, boost::python::object(e).ptr());
}


BOOST_PYTHON_MODULE(libad16)
{

  class_<ad16, boost::noncopyable>("ad16")
    .def("open",&ad16::open)
    .def("close",&ad16::close)
    .def("setSamplingRate",&ad16::setSamplingRate, setSamplingRate_overloads(args("rate","oversampling")) )
    .def("getSamplingRate",&ad16::getSamplingRate)
    .def("setVoltageRange",&ad16::setVoltageRange)
    .def("setTriggerMode",&ad16::setTriggerModePy1)
    .def("setTriggerMode",&ad16::setTriggerModePy2)
    .def("setTriggerMode",&ad16::setTriggerModePy3)
    .def("enableDaq",&ad16::enableDaq, enableDaq_overloads(args("enable")) )
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

  class_<ad16Exception> ad16ExceptionClass("ad16Exception", init<std::string,unsigned int>());
  ad16ExceptionClass.add_property("what",&ad16Exception::what);
  ad16ExceptionClass.add_property("getID",&ad16Exception::getID);
  ad16ExceptionType = ad16ExceptionClass.ptr();

  enum_<ad16Exception::exceptionId>("exceptionId")
    .value("NOT_OPENED", ad16Exception::NOT_OPENED)
    .value("ALREADY_OPENED", ad16Exception::ALREADY_OPENED)
    .value("NO_DATA_AVAILABLE", ad16Exception::NO_DATA_AVAILABLE)
    .value("ILLEGAL_PARAMETER", ad16Exception::ILLEGAL_PARAMETER)
    .value("CHANNEL_OUT_OF_RANGE", ad16Exception::CHANNEL_OUT_OF_RANGE)
    .value("IMPOSSIBLE_TIMING_CONFIGURATION", ad16Exception::IMPOSSIBLE_TIMING_CONFIGURATION)
    .value("INCORRECT_TRIGGER_SETTING", ad16Exception::INCORRECT_TRIGGER_SETTING)
    .value("TIMEOUT", ad16Exception::TIMEOUT)
  ;

  register_exception_translator<ad16Exception>(&translateAd16Exception);

}
