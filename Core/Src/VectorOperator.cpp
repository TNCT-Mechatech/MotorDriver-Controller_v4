//
// Created by testusuke on 2023/08/03.
//

#include "VectorOperator.hpp"


VectorOperator::VectorOperator(MotorDriver* md, QEI* qei, PID* pid, PID::ctrl_variable_t* cv)
    :   _md(md), _qei(qei), _pid(pid), _cv(cv)
{
    //  enable
    _is_enabled = true;
}

void VectorOperator::step(double dt) {
    //  check if module is enabled
    if (!_is_enabled) {
        return;
    }

    //  get velocity from QEI
    _cv->feedback = _qei->get_velocity(dt);
    //  step pid control
    _pid->step(dt);
    //  set output to MD
    _md->set(_cv->output);
}

void VectorOperator::stop() {
    //  set zero
    _md->set(0.0);

    //  disable
    _is_enabled = false;
}

void VectorOperator::start() {
    _is_enabled = true;
}

void VectorOperator::reset() {
    //  set zero
    _md->set(0.0);
    _qei->reset_count();
    _pid->reset();
}