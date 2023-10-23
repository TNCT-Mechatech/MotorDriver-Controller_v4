#ifndef _MDC_COMMAND_MESSAGE_
#define _MDC_COMMAND_MESSAGE_

#include "Message.hpp"

enum class Command : uint8_t
{
    NO_OPERATION
};

typedef struct CommandMessageType
{
    Command command;
    uint32_t value;
    //  timestamp longtime mod 256
    uint8_t timestamp;
} command_message_t;

typedef sb::CANMessage<command_message_t> CommandMessage;

#endif