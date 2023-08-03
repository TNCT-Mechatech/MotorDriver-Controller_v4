//
// Created by testusuke on 2023/08/03.
//

#ifndef MOTORDRIVER_CONTROLLER_V4_ANGLEOPERATOR_HPP
#define MOTORDRIVER_CONTROLLER_V4_ANGLEOPERATOR_HPP

#include "Operator.hpp"
#include "MotorDriver.hpp"
#include "QEI.hpp"
#include "PID.hpp"

class AngleOperator : public Operator
{
public:
    AngleOperator(MotorDriver* md, QEI* qei, PID* pid, PID::ctrl_variable_t* cv);

    virtual void step(double dt) = 0;
    virtual void stop() = 0;
    virtual void start() = 0;
    virtual void reset() = 0;

private:
    MotorDriver* _md;
    QEI* _qei;
    PID* _pid;
    PID::ctrl_variable_t* _cv;
};


#endif //MOTORDRIVER_CONTROLLER_V4_ANGLEOPERATOR_HPP
