#ifndef _MDC_ACKNOWLEDGE_MESSAGE_
#define _MDC_ACKNOWLEDGE_MESSAGE_

#include "Message.hpp"

typedef struct AcknowledgeMessageType
{
    //  timestamp longtime mod 65,536
    uint16_t timestamp;
} acknowledge_message_t;

typedef sb::CANMessage<acknowledge_message_t> AcknowledgeMessage;

#endif