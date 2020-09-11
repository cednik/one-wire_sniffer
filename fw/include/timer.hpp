#pragma once

#include <cstdint>

#include "FreeRTOS.h"

#include "driver/timer.h"

class Timer {
public:
    typedef uint16_t divider_t;
    typedef uint64_t value_t;

    Timer(timer_group_t group_num, timer_idx_t timer_num, divider_t divider = 1);

    value_t INTR_ATTR value() const;

private:
    const timer_group_t m_group_num;
    const timer_idx_t m_timer_num;

    portMUX_TYPE& m_lock;
    volatile uint32_t& m_update;
    volatile uint32_t& m_cnt_high;
    volatile uint32_t& m_cnt_low;

    static portMUX_TYPE s_lock[TIMER_GROUP_MAX][TIMER_MAX];
    static timg_dev_t* const s_group[TIMER_GROUP_MAX];
};
