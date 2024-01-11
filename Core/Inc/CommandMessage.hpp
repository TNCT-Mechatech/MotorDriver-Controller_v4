#ifndef _MDC_COMMAND_MESSAGE_
#define _MDC_COMMAND_MESSAGE_

#include "Message.hpp"

enum class Command : uint8_t
{
    RESET = 0U,
    PING = 1U,
    SET_PWM_FREQUENCY_AS_20KHZ = 2U,
    SET_PWM_FREQUENCY_AS_4KHZ = 2U,
    ENCODER_1_COUNT_RESET = 10U,
    ENCODER_2_COUNT_RESET = 11U,
    ENCODER_3_COUNT_RESET = 12U,
    ENCODER_4_COUNT_RESET = 13U,
};

typedef struct CommandMessageType
{
    Command command;
    //  timestamp longtime mod 65,536
    uint16_t timestamp;
} command_message_t;

typedef sb::CANMessage<command_message_t> CommandMessage;

#endif