#include "timer.hpp"

Timer::Timer(timer_group_t group_num, timer_idx_t timer_num, divider_t divider)
    : m_group_num(group_num),
      m_timer_num(timer_num),
      m_lock(s_lock[m_group_num][m_timer_num]),
      m_update  (s_group[m_group_num]->hw_timer[m_timer_num].update  ),
      m_cnt_high(s_group[m_group_num]->hw_timer[m_timer_num].cnt_high),
      m_cnt_low (s_group[m_group_num]->hw_timer[m_timer_num].cnt_low )
{
    timer_config_t cfg = {
        .alarm_en = TIMER_ALARM_DIS,
        .counter_en = TIMER_PAUSE,
        .intr_type = TIMER_INTR_LEVEL,
        .counter_dir = TIMER_COUNT_UP,
        .auto_reload = TIMER_AUTORELOAD_DIS,
        .divider = divider < 2UL ? 2UL : divider
    };
    timer_init(m_group_num, m_timer_num, &cfg);
    timer_set_counter_value(m_group_num, m_timer_num, 0ULL);
    timer_set_alarm_value(m_group_num, m_timer_num, -1ULL);
    if (divider != 0)
        timer_start(m_group_num, m_timer_num);
}

Timer::value_t Timer::value() const {
    portENTER_CRITICAL_ISR(&m_lock);
    m_update = 1;
    uint64_t h = m_cnt_high;
    uint64_t l = m_cnt_low;
    portEXIT_CRITICAL_ISR(&m_lock);
    return (h << 32) | l;
}

portMUX_TYPE Timer::s_lock[TIMER_GROUP_MAX][TIMER_MAX] = {
    { portMUX_INITIALIZER_UNLOCKED,
      portMUX_INITIALIZER_UNLOCKED },
    { portMUX_INITIALIZER_UNLOCKED,
      portMUX_INITIALIZER_UNLOCKED }
 };
timg_dev_t* const Timer::s_group[TIMER_GROUP_MAX] = { &TIMERG0, &TIMERG1 };
