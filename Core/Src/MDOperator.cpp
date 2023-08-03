//
// Created by testusuke on 2023/08/03.
//

#include "MDOperator.hpp"


MDOperator::MDOperator(MotorDriver* md, QEI* qei, PID::ctrl_variable_t* cv, EncoderType encoder_type)
        :   _md(md), _qei(qei), _cv(cv), _encoder_type(encoder_type)
{
    //  enable
    _is_enabled = true;
}

void MDOperator::step(double dt) {
    //  check if module is enabled
    if (!_is_enabled) {
        return;
    }

    if (_encoder_type == VELOCITY) {
        //  get velocity from QEI
        _cv->feedback = _qei->get_velocity(dt);
    } else if (_encoder_type == ANGLE) {
        //  get velocity from QEI
        _cv->feedback = _qei->get_angle();
    }

    //  set output to MD
    _md->set(_cv->target);
}

void MDOperator::stop() {
    //  set zero
    _md->set(0.0);

    //  disable
    _is_enabled = false;
}

void MDOperator::start() {
    _is_enabled = true;
}

void MDOperator::reset() {
    //  set zero
    _md->set(0.0);
    _qei->reset_count();
}