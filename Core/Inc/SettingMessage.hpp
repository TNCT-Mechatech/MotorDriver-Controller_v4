#ifndef _MDC_SETTING_MESSAGE_
#define _MDC_SETTING_MESSAGE_

#include "Message.hpp"
#include "EncoderType.hpp"

enum class Motor : uint8_t
{
    MOTOR1 = 0U,
    MOTOR2 = 1U,
    MOTOR3 = 2U,
    MOTOR4 = 3U
};

enum class OperatorMode : uint8_t
{
    NO_OPERATOR = 0U,
    MD_OPERATOR = 1U,
    PID_OPERATOR = 2U
};

typedef struct SettingMessageType
{
    Motor motorId;
    OperatorMode mode;
    EncoderType encoderType;
    float scale;
    bool reverse;
    //  normal pid
    float kp;
    float ki;
    float kd;
    float forward_gain;
    //  timestamp longtime mod 65,536
    uint16_t timestamp;
} setting_message_t;

typedef sb::CANMessage<setting_message_t> SettingMessage;

#endif