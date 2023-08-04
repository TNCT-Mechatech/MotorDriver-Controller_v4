#ifndef _MDC_COMMAND_MESSAGE_
#define _MDC_COMMAND_MESSAGE_

#include "Message.hpp"

typedef struct CommandMessageType
{
    uint16_t command;
    uint32_t value;
} command_message_t;

typedef sb::CANMessage<command_message_t> CommandMessage;

#endif