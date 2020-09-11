#include "test_queue.hpp"

#include "print.hpp"

#include <esp_log.h>

void testQueue::init(size_t capacity) {
    if (s_handle == nullptr) {
        s_handle = xQueueCreate(capacity, sizeof(testQueue));
        if (s_handle == nullptr) {
            ESP_LOGE("TestQueue:init", "Can not allocate queue.");
        }
    } else {
        ESP_LOGW("TestQueue:init", "Queue already initialised.");
    }
}

template<typename... Args>
void testQueue::send(Args&&... args) {
    testQueue item;
    item._init(args...);
    xQueueSend(s_handle, &item, 0);
}

template<typename... Args>
void INTR_ATTR testQueue::sendFromISR(Args&&... args) {
    testQueue item;
    item._init(args...);
    xQueueSendFromISR(s_handle, &item, 0);
}

void testQueue::print() {
    testQueue item;
    if (xQueueReceive(s_handle, &item, portMAX_DELAY) == pdTRUE)
        item._print();
}

void testQueue::_init(uint8_t cpu, uint64_t status) {
    _cpu = cpu;
    _status = status;
}

void testQueue::_print() {
    ::print("{:8} Interrupt on cpu {}: {:016X}\n", s_index++, _cpu, _status);
}

testQueue::testQueue() {}

testQueue::~testQueue() {}

QueueHandle_t testQueue::s_handle = nullptr;
volatile uint32_t testQueue::s_index = 0;
