#ifndef MTCA4U_AD16_EXCEPTION_H
#define MTCA4U_AD16_EXCEPTION_H

#include <MtcaMappedDevice/exBase.h>

namespace mtca4u
{
  /** The type of exception thrown by ad16.
   */
  class ad16Exception: public mtca4u::exBase {
    public:

      /** The different error types of the ad16Exception.
       */
      enum exceptionId {
        NOT_OPENED,                             // function called which required the device to be opened before it has been opened
        ALREADY_OPENED,                         // tried to open device twice
        CANNOT_OPEN,                            // cannot open the device
        NO_DATA_AVAILABLE,                      // data has been requested before transfer from hardware was performed
        ILLEGAL_PARAMETER,                      // wrong type of parameter given
        CHANNEL_OUT_OF_RANGE,                   // given channel number out of range
        IMPOSSIBLE_TIMING_CONFIGURATION,        // sampling frequency, oversampling etc. chosen in an impossible way
        INCORRECT_TRIGGER_SETTING,              // trigger setting is not possible (e.g. trigger channel out of range)
        TIMEOUT                                 // timout during hardware communication
      };

      ad16Exception(const std::string & message, unsigned int id):
        exBase(message, id) {}
      virtual ~ad16Exception() throw(){}
  };

}// namespace mtca4u

#endif //MTCA4U_AD16_EXCEPTION_H
