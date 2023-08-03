//
// Created by owner on 2023/08/03.
//

#ifndef MOTORDRIVER_CONTROLLER_V4_NOOPERATOR_HPP
#define MOTORDRIVER_CONTROLLER_V4_NOOPERATOR_HPP

#include "Operator.hpp"

class NoOperator : public Operator
{
public:
    NoOperator();

    void step(double dt){};
    void stop(){};
    void start(){};
    void reset(){};
};

#endif //MOTORDRIVER_CONTROLLER_V4_NOOPERATOR_HPP
