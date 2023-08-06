//
// Created by owner on 2023/08/03.
//

#ifndef MOTORDRIVER_CONTROLLER_V4_NOOPERATOR_HPP
#define MOTORDRIVER_CONTROLLER_V4_NOOPERATOR_HPP

#include "Operator.hpp"

class NoOperator : public Operator
{
public:
    NoOperator() {};

    void step(double dt) override {};
    void stop() override {};
    void start() override {};
    void reset() override {};
    OperatorMode mode() override {
        return OperatorMode::NO_OPERATOR;
    };
};

#endif //MOTORDRIVER_CONTROLLER_V4_NOOPERATOR_HPP
