#pragma once

#include "callback.hpp"

#include "Arduino.h"

#ifndef CPUS_COUNT
#define CPUS_COUNT 2
#endif

class Pin {
public:
    typedef uint8_t pin_num_t;
    typedef uint8_t pin_mode_t;
    typedef int intr_mode_t;
    typedef uint8_t Trigger;

    static constexpr pin_mode_t KEEP_CURRENT = 255;
    static constexpr pin_num_t DUMMY_PIN = 40;

public:
    Pin(pin_num_t pin, pin_mode_t mode = KEEP_CURRENT, pin_mode_t value = KEEP_CURRENT, bool inverted = false);
    Pin(const Pin& pin);

    ~Pin();

    void INTR_ATTR setHigh();
    void INTR_ATTR setLow();
    void INTR_ATTR setValue(bool value);

    void setOutput();
    void setInput(bool pullup = false);
    void setOpenDrain(bool pullup = false);

    bool INTR_ATTR read() const;

    void attachInterrupt(Callback::callback_t callback, Callback::callback_arg_t arg, Trigger mode);
    void detachInterrupt();

    void INTR_ATTR interruptTrigger(Trigger trigger);
    void INTR_ATTR clearInterruptFlag();

    pin_num_t pin() const;

    static void initInterrupts();
    static void printInterrupts();

private:
    typedef uint8_t interruptIndex_t;

    static constexpr interruptIndex_t NONE_INTERRUPT = 255;
    static constexpr uint8_t ISR_SLOTS = GPIO_PIN_COUNT;

    struct InterruptHandler {
        uint64_t pinMask;
        Callback callback;
    };

    struct InterruptRecord {
        uint8_t cpu;
        interruptIndex_t index;
        InterruptRecord(uint8_t cpu, interruptIndex_t index);
    };

    static void _printInterruptSlots(uint8_t cpu, uint8_t first = 0, uint8_t last = ISR_SLOTS);
    static void _printPinsInterruptSettings(uint8_t first = 0, uint8_t last = GPIO_PIN_COUNT);
    void _enableInterruptOnCpu(bool en, uint8_t cpu);

    const pin_num_t m_pin;
    const bool m_inverted;
    portMUX_TYPE m_lock;
    InterruptRecord m_interruptRecord;

    static void INTR_ATTR ISR(void* args);

    static void initWorker(void* args);

    static portMUX_TYPE s_lock;
    static InterruptHandler s_interruptHandler[CPUS_COUNT][ISR_SLOTS];
    static intr_handle_t s_interruptHandle[CPUS_COUNT];
};
