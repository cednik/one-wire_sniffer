#pragma once

template <class Timer>
static typename Timer::value_t INTR_ATTR wait(typename Timer::value_t t) {
    const typename Timer::value_t tStart = Timer::value();
    t += tStart;
    while(Timer::value() < t);
    return tStart;
}
