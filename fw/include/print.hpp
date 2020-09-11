#pragma once

#include <esp_log.h>

#include <FreeRTOS.h>

#include <format.h>

template <typename S, typename... Args>
void print(const S& format_str, Args&&... args);

class Printer { // consider use of MessageBuffer, instead of queue
public:
    template <typename S, typename... Args>
    friend void print(const S& format_str, Args&&... args);

    static bool begin(size_t queueSize = 128, uint32_t stackSize = 1024, UBaseType_t priority = 4);
    static bool acquire(TickType_t timeout = portMAX_DELAY);
    static bool release();
private:
    static void process(void*);

    static QueueHandle_t queue;
    static SemaphoreHandle_t mutex;
};

template <typename S, typename... Args>
void print(const S& format_str, Args&&... args) {
    Printer::acquire();
    fmt::memory_buffer* buf = new fmt::memory_buffer;
    fmt::format_to(*buf, format_str, args...);
    if (xQueueSend(Printer::queue, &buf, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE("Printer::print", "Can not send message to the queue.");
        delete buf;
    }
    Printer::release();
}
