#ifndef _MDC_SETTING_MESSAGE_
#define _MDC_SETTING_MESSAGE_

#include "Message.hpp"

typedef struct SettingMessageType
{
    uint8_t nodeId;
    uint8_t mode;
    uint32_t scale;
    //  normal pid
    float kp;
    float ki;
    float kd;
    //  current pid
    float cp;
    float ci;
    float cd;
} setting_message_t;

typedef sb::CANMessage<setting_message_t> SettingMessage;

#endif