//
// Created by testusuke on 2023/07/16.
//

#ifndef MOTORDRIVER_CONTROLLER_V4_OPERATOR_HPP
#define MOTORDRIVER_CONTROLLER_V4_OPERATOR_HPP

class Operator
{
public:
    virtual void step(double dt) = 0;
    virtual void stop() = 0;
    virtual void start() = 0;
    virtual void reset() = 0;

protected:
    bool _is_enabled;
};

#endif //MOTORDRIVER_CONTROLLER_V4_OPERATOR_HPP
