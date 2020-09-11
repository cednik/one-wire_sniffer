#pragma once

#include <cstdint>
#include <cstddef>

#include <esp_attr.h>

#include <FreeRTOS.h>

class testQueue {
public:
    static void init(size_t capacity = 128);

    template<typename... Args>
    static void send(Args&&... args);

    template<typename... Args>
    static void INTR_ATTR sendFromISR(Args&&... args);

    static void print();

private:
    uint64_t _status;
    uint8_t _cpu;

    testQueue();
    ~testQueue();

    void _print();
    void INTR_ATTR _init(uint8_t cpu, uint64_t status);

    static QueueHandle_t s_handle;
    static volatile uint32_t s_index;
};
