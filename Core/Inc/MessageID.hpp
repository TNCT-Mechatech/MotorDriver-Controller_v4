//
// Created by owner on 2023/08/04.
//

#ifndef MOTORDRIVER_CONTROLLER_V4_MESSAGEID_HPP
#define MOTORDRIVER_CONTROLLER_V4_MESSAGEID_HPP

#include "stdint.h"

/**
 * MDC reserved 10,000-10170(==10200)
 */
#define MDC_ID_RANGE_START 1000
#define MDC_ID_NODE_RANGE 10

enum class MessageID: uint32_t {
    PING,
    COMMAND,
    ACKNOWLEDGE,
    SETTING,
    TARGET,
    FEEDBACK
};

uint32_t resolve_id(uint8_t device_id, MessageID message) {
    return MDC_ID_RANGE_START + MDC_ID_NODE_RANGE * device_id + (uint32_t)message;
}

#endif //MOTORDRIVER_CONTROLLER_V4_MESSAGEID_HPP
