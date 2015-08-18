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
      enum { NOT_OPENED, ALREADY_OPENED, ILLEGAL_PARAMETER, ILLEGAL_OVERSAMPLING_RATIO, CHANNEL_OUT_OF_RANGE,
            IMPOSSIBLE_TIMING_CONFIGURATION, INCORRECT_TRIGGER_SETTING, NOT_IMPLEMENTED, TIMEOUT };

      ad16Exception(const std::string & message, unsigned int id):
        exBase(message, id) {}
      virtual ~ad16Exception() throw(){}
  };

}// namespace mtca4u

#endif //MTCA4U_AD16_EXCEPTION_H
