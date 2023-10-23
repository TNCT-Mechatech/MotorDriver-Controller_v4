//
// Created by testusuke on 2023/07/22.
//

#ifndef _SIMPLE_MOVING_AVERAGE_HPP_
#define _SIMPLE_MOVING_AVERAGE_HPP_

template <typename T>
class SimpleMovingAverage
{
public:
    SimpleMovingAverage(int buffer_size, T default_value)
            : _index(0), _buffer_size(buffer_size), _default_value(default_value),
              _array(new T[buffer_size]), _sum(default_value), _num(0)
    {
        //  init reset
        reset();
    };

    ~SimpleMovingAverage()
    {
        delete[] _array;
    };

    void push(T value)
    {
        //  sub
        if(_num >= _buffer_size) {
            _sum -= _array[_index % _buffer_size];
        } else {
            _num++;
        }

        //  insert
        _array[_index % _buffer_size] = value;
        //  sum
        _sum += value;

        //  index increment
        _index++;
    };

    T calculate()
    {
        //  calculate average
        return _sum / _num;
    };

    void reset()
    {
        //  buffer
        for(int i = 0; i < _buffer_size; i++) {
            _array[i] = _default_value;
        }
        //  sum
        _sum = _default_value;
        _num = 0;

        _index = 0;
    };

private:
    const int _buffer_size;
    T *_array;
    T _default_value;

    T _sum;
    int _num;

    int _index;
};

static int window_number(int k)
{
    return 2 * k + 1;
}

#endif //_SIMPLE_MOVING_AVERAGE_HPP_
