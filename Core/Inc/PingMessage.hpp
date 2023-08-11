#ifndef _MDC_PING_MESSAGE_
#define _MDC_PING_MESSAGE_

#include "Message.hpp"

typedef struct PingMessageType
{
    uint8_t v;
} ping_message_t;

typedef sb::CANMessage<ping_message_t> PingMessage;

#endif