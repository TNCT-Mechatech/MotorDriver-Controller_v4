#ifndef _MDC_TARGET_MESSAGE_
#define _MDC_TARGET_MESSAGE_

#include "Message.hpp"

typedef struct TargetMessageType
{
    float target[4];
} target_message_t;

typedef sb::CANMessage<target_message_t> TargetMessage;

#endif