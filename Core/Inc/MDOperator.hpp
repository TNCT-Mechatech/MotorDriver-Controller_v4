//
// Created by owner on 2023/08/03.
//

#ifndef MOTORDRIVER_CONTROLLER_V4_MDOPERATOR_HPP
#define MOTORDRIVER_CONTROLLER_V4_MDOPERATOR_HPP

#include "Operator.hpp"
#include "MotorDriver.hpp"
#include "QEI.hpp"
#include "PID.hpp"
#include "EncoderType.hpp"

class MDOperator : public Operator
{
public:
    MDOperator(MotorDriver* md, QEI* qei, PID::ctrl_variable_t* cv, EncoderType encoder_type);

    void step(double dt) override;
    void stop() override;
    void start() override;
    void reset() override;

private:
    MotorDriver* _md;
    QEI* _qei;
    PID::ctrl_variable_t* _cv;
    EncoderType _encoder_type;
};


#endif //MOTORDRIVER_CONTROLLER_V4_MDOPERATOR_HPP
