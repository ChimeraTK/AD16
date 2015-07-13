#include <ad16.h>
#include <boost/python.hpp>
#include <boost/noncopyable.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <vector>

using namespace mtca4u;
using namespace boost::python;

BOOST_PYTHON_MODULE(libad16)
{

  class_<DataVector>("DataVector")
     .def(vector_indexing_suite<DataVector>() );

  class_<ad16, boost::noncopyable>("ad16")
    .def("open",&ad16::open)
    .def("close",&ad16::close)
    .def("startConversion",&ad16::startConversion)
    .def("read",&ad16::read)
    .def("getChannelData",&ad16::getChannelData)
    ;
}
