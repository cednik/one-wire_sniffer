#include <Arduino.h>

#include "driver/timer.h"

#define INTR_ATTR IRAM_ATTR

#define DUMMY_PIN 40

#include <format.h>

using fmt::print;

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

class HWTimer {
public:
    typedef uint16_t divider_t;
    typedef uint64_t value_t;

    HWTimer(timer_group_t group_num, timer_idx_t timer_num, divider_t divider = 1)
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
portMUX_TYPE HWTimer::s_lock[TIMER_GROUP_MAX][TIMER_MAX] = {
    { portMUX_INITIALIZER_UNLOCKED,
      portMUX_INITIALIZER_UNLOCKED },
    { portMUX_INITIALIZER_UNLOCKED,
      portMUX_INITIALIZER_UNLOCKED }
 };
timg_dev_t* const HWTimer::s_group[TIMER_GROUP_MAX] = { &TIMERG0, &TIMERG1 };

class fastestTimer {
public:
    typedef uint32_t value_t;
    inline static value_t INTR_ATTR value() { return xthal_get_ccount(); }
};

template <class Timer>
void INTR_ATTR wait(typename Timer::value_t t) {
    t += Timer::value();
    while(Timer::value() < t);
}

class OneWire {
    typedef fastestTimer Timer;
public:
    struct Timing {
        typedef Timer::value_t time_t; // in CPU ticks -> 1 / 240 MHz
        // based on https://www.maximintegrated.com/en/design/technical-documents/app-notes/1/126.html
        time_t A,B,C,D,E,F,G,H,I,J;
    };
    static const Timing Standard;
    static const Timing Overdrive;
    OneWire(Pin input, Pin output, Pin boost = Pin(DUMMY_PIN), const Timing* timing = &Standard)
        : m_input(input),
          m_output(output),
          m_boost(boost),
          m_timing(timing),
          m_lock(portMUX_INITIALIZER_UNLOCKED)
    {}

//private:
    void _writeBit(const bool v) {
        const Timer::value_t pre  = v ? m_timing->A : m_timing->C;
        const Timer::value_t post = v ? m_timing->B : m_timing->D;
        portENTER_CRITICAL(&m_lock);
        m_output.setLow();
        wait<Timer>(pre);
        m_output.setHigh();
        wait<Timer>(post);
        portEXIT_CRITICAL(&m_lock);
    }
    bool _readBit() {
        portENTER_CRITICAL(&m_lock);
        m_output.setLow();
        wait<Timer>(m_timing->A);
        m_output.setHigh();
        wait<Timer>(m_timing->E);
        const bool v = m_input.read();
        wait<Timer>(m_timing->F);
        portEXIT_CRITICAL(&m_lock);
        return v;
    }
    bool reset() {
        portENTER_CRITICAL(&m_lock);
        wait<Timer>(m_timing->G);
        m_output.setLow();
        wait<Timer>(m_timing->H);
        m_output.setHigh();
        wait<Timer>(m_timing->I);
        const bool v = m_input.read();
        wait<Timer>(m_timing->J);
        portEXIT_CRITICAL(&m_lock);
        return !v;
    }
    void _write(uint8_t v) {
        for (uint8_t i = 1; i != 0; i<<=1)
            _writeBit(v & i);
    }
    uint8_t _read() {
        uint8_t v = 0;
        for(uint8_t i = 0; i != 8; ++i) {
            v |= _readBit();
            v <<= 1;
        }
        return v;
    }

    Pin m_input;
    Pin m_output;
    Pin m_boost;
    const Timing* m_timing;
    portMUX_TYPE m_lock;
};
#define US2TICKS(us) OneWire::Timing::time_t(us * 240)
// based on https://www.maximintegrated.com/en/design/technical-documents/app-notes/1/126.html
const OneWire::Timing OneWire::Standard = {
    .A = US2TICKS(  6),
    .B = US2TICKS( 64),
    .C = US2TICKS( 60),
    .D = US2TICKS( 10),
    .E = US2TICKS(  9),
    .F = US2TICKS( 55),
    .G = US2TICKS(  0),
    .H = US2TICKS(480),
    .I = US2TICKS( 70),
    .J = US2TICKS(410)
};
const OneWire::Timing OneWire::Overdrive = {
    .A = US2TICKS(  1.0),
    .B = US2TICKS(  7.5),
    .C = US2TICKS(  7.5),
    .D = US2TICKS(  2.5),
    .E = US2TICKS(  1.0),
    .F = US2TICKS(  7.0),
    .G = US2TICKS(  2.5),
    .H = US2TICKS( 70.0),
    .I = US2TICKS(  8.5),
    .J = US2TICKS( 40.0)
};

void setup() {
    Serial.begin(115200);
    print("OneWire sniffer\n");
    const Pin::pin_num_t onewirePin = 21;
    OneWire ow(Pin(onewirePin, OUTPUT_OPEN_DRAIN | PULLUP), Pin(onewirePin));
    for(;;) {
        bool presence = ow.reset();
        ow._write(0xAA);
        print("presence: {}\n", presence);
        delay(1000);
    }
}

void loop() {}