#include "print.hpp"

#include <esp_log.h>

#include <Arduino.h>


bool Printer::begin(size_t queueSize, uint32_t stackSize, UBaseType_t priority) {
    if (queue != nullptr) {
        ESP_LOGE("Printer::init", "Already initialized.");
        return false;
    }
    queue = xQueueCreate(queueSize, sizeof(fmt::memory_buffer*));
    if (queue == nullptr) {
        ESP_LOGE("Printer::init", "Can not create queue.");
        return false;
    }
    mutex = xSemaphoreCreateRecursiveMutex();
    if (mutex == nullptr) {
        ESP_LOGE("Printer::init", "Can not create mutex.");
        vQueueDelete(queue);
        queue = nullptr;
        return false;
    }
    BaseType_t ret = xTaskCreate(process, "Printer::process", stackSize, nullptr, priority, nullptr);
    if (ret != pdPASS) {
        ESP_LOGE("Printer::init", "Can not create process task (error code %d).", ret);
        vSemaphoreDelete(mutex);
        vQueueDelete(queue);
        mutex = nullptr;
        queue = nullptr;
        return false;
    }
    return true;
}

bool Printer::acquire(TickType_t timeout) {
    return xSemaphoreTakeRecursive(mutex, timeout) == pdTRUE;
}
bool Printer::release() {
    return xSemaphoreGiveRecursive(mutex) == pdTRUE;
}

void Printer::process(void*) {
    fmt::memory_buffer* buf = nullptr;
    for(;;) {
        if (xQueueReceive(queue, &buf, portMAX_DELAY) == pdTRUE) {
            if (buf == nullptr) {
                ESP_LOGE("Printer::process", "Received nullptr from queue.");
                continue;
            }
            Serial.write(reinterpret_cast<uint8_t*>(buf->data()), buf->size());
            delete buf;
        }
    }
}

QueueHandle_t Printer::queue = nullptr;
SemaphoreHandle_t Printer::mutex = nullptr;
