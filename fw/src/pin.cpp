#include "pin.hpp"

#include "print.hpp"

extern Pin isrOut;

Pin::InterruptRecord::InterruptRecord(uint8_t cpu, interruptIndex_t index)
    : cpu(cpu),
      index(index)
{}

Pin::Pin(pin_num_t pin, pin_mode_t mode, pin_mode_t value, bool inverted)
    : m_pin(pin),
      m_inverted(inverted),
      m_lock(portMUX_INITIALIZER_UNLOCKED),
      m_interruptRecord(0, NONE_INTERRUPT)
{
    if (value != KEEP_CURRENT)
        setValue(value);
    if (mode != KEEP_CURRENT)
        pinMode(m_pin, mode);
}

Pin::Pin(const Pin& pin)
    : m_pin(pin.m_pin),
      m_inverted(pin.m_inverted),
      m_lock(portMUX_INITIALIZER_UNLOCKED),
      m_interruptRecord(0, NONE_INTERRUPT)
{}

Pin::~Pin() {
    detachInterrupt();
}

void Pin::setHigh() { digitalWrite(m_pin, m_inverted ? LOW : HIGH); }

void Pin::setLow()  { digitalWrite(m_pin, m_inverted ? HIGH : LOW); }

void Pin::setValue(bool value) { if (value) setHigh(); else setLow(); }

void Pin::setOutput() { pinMode(m_pin, OUTPUT); }

void Pin::setInput(bool pullup) { pinMode(m_pin, INPUT | (pullup ? PULLUP : 0)); }

void Pin::setOpenDrain(bool pullup) { pinMode(m_pin, OUTPUT_OPEN_DRAIN | (pullup ? PULLUP : 0)); }

bool Pin::read() const { return digitalRead(m_pin) ^ m_inverted; }

// Arduino attach interrupt crash ESP
//void Pin::attachInterrupt(Callback::callback_t callback, Callback::callback_arg_t arg, int mode) { attachInterruptArg(digitalPinToInterrupt(m_pin), callback, arg, mode); }
//void Pin::detachInterrupt() { ::detachInterrupt(digitalPinToInterrupt(m_pin)); }

void Pin::attachInterrupt(Callback::callback_t callback, Callback::callback_arg_t arg, Trigger mode) {
    portENTER_CRITICAL(&s_lock);
    uint8_t cpu = xPortGetCoreID();
    esp_intr_disable(s_interruptHandle[cpu]);
    if (m_interruptRecord.index != NONE_INTERRUPT) {
        if (cpu != m_interruptRecord.cpu) {
            _enableInterruptOnCpu(false, m_interruptRecord.cpu);
            esp_intr_disable(s_interruptHandle[m_interruptRecord.cpu]);
            s_interruptHandler[m_interruptRecord.cpu][m_interruptRecord.index].pinMask = 0;
            esp_intr_enable(s_interruptHandle[m_interruptRecord.cpu]);
        }
    } else {
        for (uint8_t i = 0; i != ISR_SLOTS; ++i) {
            if (s_interruptHandler[cpu][i].pinMask == 0) {
                m_interruptRecord.index = i;
                break;
            }
        }
    }
    m_interruptRecord.cpu = cpu;
    if (m_interruptRecord.index == NONE_INTERRUPT) {
        ESP_LOGE("Pin::attachIntr", "No available slot.");
    } else {
        s_interruptHandler[m_interruptRecord.cpu][m_interruptRecord.index].callback = Callback(callback, arg);
        s_interruptHandler[m_interruptRecord.cpu][m_interruptRecord.index].pinMask = (1ULL<<m_pin);
        interruptTrigger(mode);
        _enableInterruptOnCpu(true, m_interruptRecord.cpu);
    }
    esp_intr_enable(s_interruptHandle[m_interruptRecord.cpu]);
    portEXIT_CRITICAL(&s_lock);
}

void Pin::detachInterrupt() {
    portENTER_CRITICAL(&s_lock);
    if (m_interruptRecord.index != NONE_INTERRUPT) {
        _enableInterruptOnCpu(false, m_interruptRecord.cpu);
        esp_intr_disable(s_interruptHandle[m_interruptRecord.cpu]);
        s_interruptHandler[m_interruptRecord.cpu][m_interruptRecord.index].pinMask = 0;
        m_interruptRecord.index = NONE_INTERRUPT;
        esp_intr_enable(s_interruptHandle[m_interruptRecord.cpu]);
    }
    portEXIT_CRITICAL(&s_lock);
}

void Pin::interruptTrigger(Trigger trigger) {
    GPIO.pin[m_pin].int_type = trigger;
}

void Pin::clearInterruptFlag() {
    if (m_pin < 32)
        GPIO.status_w1tc = 1<<m_pin;
    else
        GPIO.status1_w1tc.val = 1<<(m_pin-32);
}

Pin::pin_num_t Pin::pin() const { return m_pin; }

void Pin::initInterrupts() {
    for (uint8_t i = 0; i != CPUS_COUNT; ++i) {
        std::string taskName = fmt::format("Pin:initIntr{}",i);
        xTaskCreatePinnedToCore(Pin::initWorker, taskName.c_str(), 2*1024, xTaskGetCurrentTaskHandle(), 4, nullptr, i);
        ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
    }
}

void Pin::printInterrupts() {
    Printer::acquire();
    portENTER_CRITICAL(&s_lock);
    print("Registered interrupts\n");
    for (uint8_t cpu = 0; cpu != CPUS_COUNT; ++cpu) {
        _printInterruptSlots(cpu);
    }
    _printPinsInterruptSettings();
    portEXIT_CRITICAL(&s_lock);
    Printer::release();
}

void Pin::_printInterruptSlots(uint8_t cpu, uint8_t first, uint8_t last) {
    print("\tCPU {}\n", cpu);
        print("\t\tslot       mask           fcn          arg\n", cpu);
        for (uint8_t slot = first; slot != last; ++slot) {
            InterruptHandler& rec = s_interruptHandler[cpu][slot];
            if (rec.pinMask || rec.callback.getCallback() || rec.callback.getArg())
                print("\t\t{:4} {:016X} {:08X} {:08X}\n", slot, rec.pinMask, uintptr_t(rec.callback.getCallback()), uintptr_t(rec.callback.getArg()));
        }
}

void Pin::_printPinsInterruptSettings(uint8_t first, uint8_t last) {
    print("\n\tpin enable(PRO_NM, PRO, -, APP_NM, APP), type\n");
    for(uint8_t i = first; i != last; ++i) {
        std::string type;
        switch (GPIO.pin[i].int_type) {
            case  0: type = "disabled"; break;
            case  1: type = "rising"; break;
            case  2: type = "falling"; break;
            case  3: type = "both"; break;
            case  4: type = "low"; break;
            case  5: type = "high"; break;
            default: type = fmt::format("unknown {:02x}", uint8_t(GPIO.pin[i].int_type)); break;
        }
        if (GPIO.pin[i].int_ena || GPIO.pin[i].int_type)
            print("\t{:3} {:05b} {}\n", i, uint8_t(GPIO.pin[i].int_ena), type);
    }
}

void Pin::_enableInterruptOnCpu(bool en, uint8_t cpu) {
    switch (cpu) {
        case 0: GPIO.pin[m_pin].int_ena = (GPIO.pin[m_pin].int_ena & ~GPIO_PRO_CPU_INTR_ENA) | (en ? GPIO_PRO_CPU_INTR_ENA : 0); break;
        case 1: GPIO.pin[m_pin].int_ena = (GPIO.pin[m_pin].int_ena & ~GPIO_APP_CPU_INTR_ENA) | (en ? GPIO_APP_CPU_INTR_ENA : 0); break;
        default:
            ESP_LOGE("Pin::enIntr", "ESP32 has no core %d.\n", cpu);
            break;
    }

}

void Pin::ISR(void* args) {
    isrOut.setHigh();
    uint32_t gpio_intr_status_l = GPIO.status;
    uint32_t gpio_intr_status_h = GPIO.status1.val;
    GPIO.status_w1tc = gpio_intr_status_l;
    GPIO.status1_w1tc.val = gpio_intr_status_h;
    uint8_t cpu = intptr_t(args);
    uint64_t status = (uint64_t(gpio_intr_status_h) << 32) | gpio_intr_status_l;
    for (uint8_t i = 0; i != ISR_SLOTS; ++i) {
        if ((status & s_interruptHandler[cpu][i].pinMask) != 0) {
            s_interruptHandler[cpu][i].callback();
            break;
        }
    }
    isrOut.setLow();
}

void Pin::initWorker(void* args) {
    uint8_t cpu = xPortGetCoreID();
    if (s_interruptHandle[cpu] == nullptr) {
        esp_err_t ret = esp_intr_alloc(ETS_GPIO_INTR_SOURCE, (int)ESP_INTR_FLAG_IRAM, Pin::ISR, reinterpret_cast<void*>(cpu), &s_interruptHandle[cpu]);
        switch (ret) {
        case ESP_OK: break;
        case ESP_ERR_INVALID_ARG: ESP_LOGE("Pin::InitWorker", "Invalid combination of esp_intr_alloc arguments on cpu %d, strange.", cpu); break;
        case ESP_ERR_NOT_FOUND: ESP_LOGE("Pin::InitWorker", "No available interrupt slot on cpu %d.", cpu); break;
        default: ESP_LOGE("Pin::InitWorker", "esp_intr_alloc failed with code %d on cpu %d, strange.", uint32_t(ret), cpu); break;
        }
    } else {
        ESP_LOGW("Pin::InitWorker", "Interrupt already initialised on cpu %d.", cpu);
    }
    xTaskNotifyGive(args);
    vTaskDelete(nullptr);
}

portMUX_TYPE Pin::s_lock = portMUX_INITIALIZER_UNLOCKED;
Pin::InterruptHandler Pin::s_interruptHandler[CPUS_COUNT][Pin::ISR_SLOTS] = { 0 };
intr_handle_t Pin::s_interruptHandle[CPUS_COUNT] = { nullptr };
