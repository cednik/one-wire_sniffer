#include <Arduino.h>

#include "driver/timer.h"

#define INTR_ATTR IRAM_ATTR

#define DUMMY_PIN 40

class Pin {
public:
    typedef uint8_t pin_num_t;
    typedef void* intr_callback_arg_t;
    typedef void (*intr_callback_t)(intr_callback_arg_t);
    typedef uint8_t pin_mode_t;
    typedef int intr_mode_t;
    static const pin_mode_t KEEP_CURRENT = 255;
    Pin(pin_num_t pin, bool inverted = false, pin_mode_t mode = KEEP_CURRENT)
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

class ISRGuard {
public:
    INTR_ATTR ISRGuard() { portENTER_CRITICAL_ISR(&mux); }
    INTR_ATTR ~ISRGuard() { portEXIT_CRITICAL_ISR(&mux); }
private:
    static portMUX_TYPE mux;
};
portMUX_TYPE ISRGuard::mux = portMUX_INITIALIZER_UNLOCKED;

#include "allocator.hpp"

template<class T, class Alloc = allocator<T> >
class SortedList {
    struct Item {
        T* item;
        Item* prev;
        Item* next;
    };
public:
    typedef Alloc allocator_type;
    typedef uintptr_t handle_t;
    INTR_ATTR SortedList(const allocator_type& alloc = allocator_type())
        : m_allocator(alloc),
          m_first(nullptr),
          m_last(nullptr)
    {}
    handle_t INTR_ATTR add() {
        return 0;
    }
private:
    allocator_type m_allocator;
    Item* m_first;
    Item* m_last;
};

class Timer {
public:
    typedef void* callback_arg_t;
    typedef void (*callback_t)(callback_arg_t);
    typedef uint16_t divider_t;
    typedef uint64_t value_t;
private:
    struct ScheduleItem {
        INTR_ATTR ScheduleItem (value_t time, callback_t callback, callback_arg_t callback_arg)
            : m_time(time),
              m_callback(callback),
              m_callback_arg(callback_arg)
        {}
        void INTR_ATTR operator ()() { m_callback(m_callback_arg); }
        bool INTR_ATTR operator < (const ScheduleItem& rhs) const { return m_time < rhs.m_time; }
    private:
        const value_t m_time;
        callback_t m_callback;
        callback_arg_t m_callback_arg;
    };
    typedef SortedList<ScheduleItem> ScheduleItems;
public:
    typedef ScheduleItems::handle_t handle_t;
    Timer(timer_group_t group_num, timer_idx_t timer_num, divider_t divider = 2)
        : m_group_num(group_num),
          m_timer_num(timer_num),
          m_scheduled_items()
    {
        timer_config_t cfg = {
            .alarm_en = TIMER_ALARM_EN,
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
    value_t INTR_ATTR value() const { return 0; } //timer_group_get_counter_value_in_isr(m_group_num, m_timer_num); }
    handle_t schedule(value_t when, callback_t callback, callback_arg_t callback_arg) {
        return m_scheduled_items.add();
    }
private:
    const timer_group_t m_group_num;
    const timer_idx_t m_timer_num;
    ScheduleItems m_scheduled_items;
};

class OneWire {
public:
    OneWire(
        Pin input, Pin output, Pin boost, Timer timer
    ){}
private:
    
};

void setup() {
    // put your setup code here, to run once:
}

void loop() {
    // put your main code here, to run repeatedly:
}