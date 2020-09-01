#include <Arduino.h>

#include "driver/timer.h"

#define INTR_ATTR IRAM_ATTR

#define DUMMY_PIN 40

#include <format.h>

class Pin {
public:
    typedef uint8_t pin_num_t;
    typedef void* intr_callback_arg_t;
    typedef void (*intr_callback_t)(intr_callback_arg_t);
    typedef uint8_t pin_mode_t;
    typedef int intr_mode_t;
    static const pin_mode_t KEEP_CURRENT = 255;
    Pin(pin_num_t pin, pin_mode_t mode = KEEP_CURRENT, bool inverted = false)
        : m_pin(pin),
          m_inverted(inverted)
    {
        if (mode != KEEP_CURRENT)
            pinMode(m_pin, mode);
    }
    Pin(const Pin& pin)
        : m_pin(pin.m_pin),
          m_inverted(pin.m_inverted)
    {}
    void INTR_ATTR setHigh() { digitalWrite(m_pin, m_inverted ? LOW : HIGH); }
    void INTR_ATTR setLow()  { digitalWrite(m_pin, m_inverted ? HIGH : LOW); }
    void INTR_ATTR setValue(bool value) { if (value) setHigh(); else setLow(); }
    void setOutput() { pinMode(m_pin, OUTPUT); }
    void setInput(bool pullup = false) { pinMode(m_pin, INPUT | (pullup ? PULLUP : 0)); }
    void setOpenDrain(bool pullup = false) { pinMode(m_pin, OUTPUT_OPEN_DRAIN | (pullup ? PULLUP : 0)); }
    bool INTR_ATTR read() const { return (digitalRead(m_pin) == HIGH) ^ m_inverted; }
    void attachInterrupt(intr_callback_t callback, intr_callback_arg_t arg, int mode) { attachInterruptArg(m_pin, callback, arg, mode); }
    void detachInterrupt() { ::detachInterrupt(m_pin); }
    pin_num_t pin() const { return m_pin; }
private:
    const pin_num_t m_pin;
    const bool m_inverted;
};

class Timer {
public:
    typedef uint16_t divider_t;
    typedef uint64_t value_t;

    Timer(timer_group_t group_num, timer_idx_t timer_num, divider_t divider = 1)
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
    value_t INTR_ATTR value() const {
        portENTER_CRITICAL_ISR(&m_lock);
        m_update = 1;
        uint64_t h = m_cnt_high;
        uint64_t l = m_cnt_low;
        portEXIT_CRITICAL_ISR(&m_lock);
        return (h << 32) | l;
    }
    
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
portMUX_TYPE Timer::s_lock[TIMER_GROUP_MAX][TIMER_MAX] = {
    { portMUX_INITIALIZER_UNLOCKED,
      portMUX_INITIALIZER_UNLOCKED },
    { portMUX_INITIALIZER_UNLOCKED,
      portMUX_INITIALIZER_UNLOCKED }
 };
timg_dev_t* const Timer::s_group[TIMER_GROUP_MAX] = { &TIMERG0, &TIMERG1 };

class OneWire {
public:
    struct Timing {
        typedef uint16_t time_t; // 1 = 0.5 us
        // based on https://www.maximintegrated.com/en/design/technical-documents/app-notes/1/126.html
        time_t A,B,C,D,E,F,G,H,I,J;
    };
    static const Timing Standard;
    static const Timing Overdrive;
    OneWire(Pin input, Pin output, Pin boost, const Timing* timing = &Standard)
        : m_input(input),
          m_output(output),
          m_boost(boost),
          m_timing(timing),
          m_lock(portMUX_INITIALIZER_UNLOCKED)
    {}

private:
    void _writeBit(bool v)
    {
        portENTER_CRITICAL(&m_lock);
        m_output.setLow();
        if (v) {

        }
        portEXIT_CRITICAL(&m_lock);
    }

    Pin m_input;
    Pin m_output;
    Pin m_boost;
    const Timing* m_timing;
    portMUX_TYPE m_lock;
};
// based on https://www.maximintegrated.com/en/design/technical-documents/app-notes/1/126.html
const OneWire::Timing OneWire::Standard = {
    .A =  12,
    .B = 128,
    .C = 120,
    .D =  20,
    .E =  18,
    .F = 110,
    .G =   0,
    .H = 960,
    .I = 140,
    .J = 820
};
const OneWire::Timing OneWire::Overdrive = {
    .A =   2,
    .B =  15,
    .C =  15,
    .D =   5,
    .E =   2,
    .F =  14,
    .G =   5,
    .H = 140,
    .I =  17,
    .J =  80
};

Timer tmr0(TIMER_GROUP_0, TIMER_0, 80);

void setup() {
    Serial.begin(115200);
    fmt::print("OneWire sniffer\n");
    const size_t l = 1000;
    uint64_t* t = new uint64_t[l];
    for (size_t i = 0; i != l; ++i)
        t[i] = tmr0.value();
    for (size_t i = 0; i != l; ++i)
        fmt::print("{:8}\n", t[i]);
    fmt::print("{:8} - {} = {} / {}\n", t[l-1], t[0], t[l-1] - t[0], l);
    delete t;
}

void loop() {
    uint64_t v = tmr0.value();
    fmt::print("{:8}\n", v);
    delay(1000);
}