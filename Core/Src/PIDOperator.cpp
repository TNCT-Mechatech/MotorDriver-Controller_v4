//
// Created by testusuke on 2023/08/03.
//

#include "PIDOperator.hpp"


PIDOperator::PIDOperator(MotorDriver* md, QEI* qei, PID* pid, PID::ctrl_variable_t* cv, EncoderType encoder_type)
    :   _md(md), _qei(qei), _pid(pid), _cv(cv), _encoder_type(encoder_type)
{
    //  enable
    _is_enabled = true;
}

void PIDOperator::step(double dt) {
    //  check if module is enabled
    if (!_is_enabled) {
        return;
    }

    if (_encoder_type == EncoderType::VELOCITY) {
        //  get velocity from QEI
        _cv->feedback = _qei->get_velocity(dt);
    } else {
        //  get angle from QEI
        _cv->feedback = _qei->get_angle();
    }

    //  step pid control
    _pid->step(dt);
    //  set output to MD
    _md->set(_cv->output);
}

void PIDOperator::stop() {
    //  set zero
    _md->set(0.0);

    //  disable
    _is_enabled = false;
}

void PIDOperator::start() {
    _is_enabled = true;
}

void PIDOperator::reset() {
    //  set zero
    _md->set(0.0);
    _qei->reset_count();
    _pid->reset();
}