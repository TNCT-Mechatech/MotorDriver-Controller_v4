//
// Created by owner on 2023/07/16.
//

#ifndef MOTORDRIVER_CONTROLLER_V4_PIDOPERATOR_HPP
#define MOTORDRIVER_CONTROLLER_V4_PIDOPERATOR_HPP

#include "Operator.hpp"
#include "MotorDriver.hpp"
#include "QEI.hpp"
#include "PID.hpp"
#include "EncoderType.hpp"

class PIDOperator : public Operator
{
public:
    PIDOperator(MotorDriver* md, QEI* qei, PID* pid, PID::ctrl_variable_t* cv, EncoderType encoder_type);

    void step(double dt) override;
    void stop() override;
    void start() override;
    void reset() override;
    OperatorMode mode() override {
        return OperatorMode::PID_OPERATOR;
    };

private:
    MotorDriver* _md;
    QEI* _qei;
    PID* _pid;
    PID::ctrl_variable_t* _cv;
    EncoderType _encoder_type;
};

#endif //MOTORDRIVER_CONTROLLER_V4_PIDOPERATOR_HPP
