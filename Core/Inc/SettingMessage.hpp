#ifndef _MDC_SETTING_MESSAGE_
#define _MDC_SETTING_MESSAGE_

#include "Message.hpp"

enum class OperatorMode : uint8_t
{
    NO_OPERATOR,
    MD_OPERATOR,
    PID_OPERATOR
};

typedef struct SettingMessageType
{
    uint8_t nodeId;
    OperatorMode mode;
    uint32_t scale;
    //  normal pid
    float kp;
    float ki;
    float kd;
    //  current pid
    float cp;
    float ci;
    float cd;
    //  timestamp longtime mod 256
    uint8_t timestamp;
} setting_message_t;

typedef sb::CANMessage<setting_message_t> SettingMessage;

#endif