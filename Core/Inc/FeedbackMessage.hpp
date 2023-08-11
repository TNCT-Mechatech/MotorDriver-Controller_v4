#ifndef _MDC_FEEDBACK_MESSAGE_
#define _MDC_FEEDBACK_MESSAGE_

#include "Message.hpp"

typedef struct NodeFeedbackMessageType
{
    float angle;
    float velocity;
    float current;
} node_feedback_message_t;

typedef struct FeedbackMessageType
{
    node_feedback_message_t node[4];
} feedback_message_t;

typedef sb::CANMessage<feedback_message_t> FeedbackMessage;

#endif