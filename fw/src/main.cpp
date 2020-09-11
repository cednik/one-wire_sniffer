#define LOG_LOCAL_LEVEL 5
#include "esp_log.h"
#include <Arduino.h>
#include "driver/timer.h"
#include <initializer_list>

#define CPUS_COUNT 2

#define ONEWIRE_MAX_ROMS 8

#define INTR_ATTR IRAM_ATTR

#define DUMMY_PIN 40

#define ONEWIRE_MASTER_PIN 22
#define ONEWIRE_SLAVE_PIN  23

#define OW_SLAVE_TEST_PIN   21
#define OW_MASTER_TEST_PIN  18
#define ISR_TEST_PIN        19

#include <format.h>

template <typename S, typename... Args>
static inline void print(const S& format_str, Args&&... args);

class Printer { // consider use of MessageBuffer, instead of queue
public:
    static bool begin(size_t queueSize = 128, uint32_t stackSize = 1024, UBaseType_t priority = 4) {
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
    template <typename S, typename... Args>
    friend void print(const S& format_str, Args&&... args);
    static bool acquire(TickType_t timeout = portMAX_DELAY) {
        return xSemaphoreTakeRecursive(mutex, timeout) == pdTRUE;
    }
    static bool release() {
        return xSemaphoreGiveRecursive(mutex) == pdTRUE;
    }
private:
    static void process(void*) {
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
    static QueueHandle_t queue;
    static SemaphoreHandle_t mutex;
};
QueueHandle_t Printer::queue = nullptr;
SemaphoreHandle_t Printer::mutex = nullptr;

template <typename S, typename... Args>
static inline void print(const S& format_str, Args&&... args) {
    Printer::acquire();
    fmt::memory_buffer* buf = new fmt::memory_buffer;
    fmt::format_to(*buf, format_str, args...);
    if (xQueueSend(Printer::queue, &buf, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE("Printer::print", "Can not send message to the queue.");
        delete buf;
    }
    Printer::release();
}

class Callback {
public:
    typedef void* callback_arg_t;
    typedef void (*callback_t)(callback_arg_t);
    Callback(callback_t callback = nullptr, callback_arg_t arg = nullptr)
        : m_callback(callback),
          m_callback_arg(arg)
    {}
    void INTR_ATTR operator ()() { if (m_callback) m_callback(m_callback_arg); }
    const callback_t& getCallback() const { return m_callback; }
    const callback_arg_t& getArg() const { return m_callback_arg; }
private:
    callback_t m_callback;
    callback_arg_t m_callback_arg;
};

class testQueue {
    uint64_t _status;
    uint8_t _cpu;
    void _print() {
        ::print("{:8} Interrupt on cpu {}: {:016X}\n", s_index++, _cpu, _status);
    }
    void INTR_ATTR _init(uint8_t cpu, uint64_t status) {
        _cpu = cpu;
        _status = status;
    }
    testQueue(){}
    ~testQueue(){}
public:
    static void init(size_t capacity = 128) {
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
    static void send(Args&&... args) {
        testQueue item;
        item._init(args...);
        xQueueSend(s_handle, &item, 0);
    }
    template<typename... Args>
    static void INTR_ATTR sendFromISR(Args&&... args) {
        testQueue item;
        item._init(args...);
        xQueueSendFromISR(s_handle, &item, 0);
    }
    static void print() {
        testQueue item;
        if (xQueueReceive(s_handle, &item, portMAX_DELAY) == pdTRUE)
            item._print();
    }
private:
    static QueueHandle_t s_handle;
    static volatile uint32_t s_index;
};
QueueHandle_t testQueue::s_handle = nullptr;
volatile uint32_t testQueue::s_index = 0;

#undef GPIO_PRO_CPU_INTR_ENA
#define GPIO_PRO_CPU_INTR_ENA (BIT(3)) // Errornously defined as (BIT(2)) in arduino-esp32

class Pin {
public:
    typedef uint8_t pin_num_t;
    typedef uint8_t pin_mode_t;
    typedef int intr_mode_t;
    typedef uint8_t Trigger;
private:
    typedef uint8_t interruptIndex_t;

    static const interruptIndex_t NONE_INTERRUPT = 255;
    static const pin_mode_t KEEP_CURRENT = 255;
public:
    static const uint8_t ISR_SLOTS = GPIO_PIN_COUNT;
private:
    struct InterruptHandler {
        uint64_t pinMask;
        Callback callback;
    };
    struct InterruptRecord {
        uint8_t cpu;
        interruptIndex_t index;
        InterruptRecord(uint8_t cpu, interruptIndex_t index)
            : cpu(cpu),
              index(index)
        {}
    };
    
public:   
    Pin(pin_num_t pin, pin_mode_t mode = KEEP_CURRENT, pin_mode_t value = KEEP_CURRENT, bool inverted = false)
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
    Pin(const Pin& pin)
        : m_pin(pin.m_pin),
          m_inverted(pin.m_inverted),
          m_lock(portMUX_INITIALIZER_UNLOCKED),
          m_interruptRecord(0, NONE_INTERRUPT)
    {}
    ~Pin() {
        detachInterrupt();
    }
    void INTR_ATTR setHigh() { digitalWrite(m_pin, m_inverted ? LOW : HIGH); }
    void INTR_ATTR setLow()  { digitalWrite(m_pin, m_inverted ? HIGH : LOW); }
    void INTR_ATTR setValue(bool value) { if (value) setHigh(); else setLow(); }
    void setOutput() { pinMode(m_pin, OUTPUT); }
    void setInput(bool pullup = false) { pinMode(m_pin, INPUT | (pullup ? PULLUP : 0)); }
    void setOpenDrain(bool pullup = false) { pinMode(m_pin, OUTPUT_OPEN_DRAIN | (pullup ? PULLUP : 0)); }
    bool INTR_ATTR read() const { return digitalRead(m_pin) ^ m_inverted; }
    // Arduino attach interrupt crash ESP
    //void attachInterrupt(Callback::callback_t callback, Callback::callback_arg_t arg, int mode) { attachInterruptArg(digitalPinToInterrupt(m_pin), callback, arg, mode); }
    //void detachInterrupt() { ::detachInterrupt(digitalPinToInterrupt(m_pin)); }
    void attachInterrupt(Callback::callback_t callback, Callback::callback_arg_t arg, Trigger mode) {
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
    void detachInterrupt() {
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
    void INTR_ATTR interruptTrigger(Trigger trigger) {
        GPIO.pin[m_pin].int_type = trigger;
    }
    void INTR_ATTR clearInterruptFlag() {
        if (m_pin < 32)
            GPIO.status_w1tc = 1<<m_pin;
        else
            GPIO.status1_w1tc.val = 1<<(m_pin-32);
    }
    pin_num_t pin() const { return m_pin; }

    static void initInterrupts() {
        for (uint8_t i = 0; i != CPUS_COUNT; ++i) {
            std::string taskName = fmt::format("Pin:initIntr{}",i);
            xTaskCreatePinnedToCore(Pin::initWorker, taskName.c_str(), 2*1024, xTaskGetCurrentTaskHandle(), 4, nullptr, i);
            ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
        }
    }
    static void printInterrupts() {
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
private:
    static void _printInterruptSlots(uint8_t cpu, uint8_t first = 0, uint8_t last = ISR_SLOTS) {
        print("\tCPU {}\n", cpu);
            print("\t\tslot       mask           fcn          arg\n", cpu);
            for (uint8_t slot = first; slot != last; ++slot) {
                InterruptHandler& rec = s_interruptHandler[cpu][slot];
                if (rec.pinMask || rec.callback.getCallback() || rec.callback.getArg())
                    print("\t\t{:4} {:016X} {:08X} {:08X}\n", slot, rec.pinMask, uintptr_t(rec.callback.getCallback()), uintptr_t(rec.callback.getArg()));
            }
    }
    static void _printPinsInterruptSettings(uint8_t first = 0, uint8_t last = GPIO_PIN_COUNT) {
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
    void _enableInterruptOnCpu(bool en, uint8_t cpu) {
        switch (cpu) {
            case 0: GPIO.pin[m_pin].int_ena = (GPIO.pin[m_pin].int_ena & ~GPIO_PRO_CPU_INTR_ENA) | (en ? GPIO_PRO_CPU_INTR_ENA : 0); break;
            case 1: GPIO.pin[m_pin].int_ena = (GPIO.pin[m_pin].int_ena & ~GPIO_APP_CPU_INTR_ENA) | (en ? GPIO_APP_CPU_INTR_ENA : 0); break;
            default:
                ESP_LOGE("Pin::enIntr", "ESP32 has no core %d.\n", cpu);
                break;
        }

    }

    const pin_num_t m_pin;
    const bool m_inverted;
    portMUX_TYPE m_lock;
    InterruptRecord m_interruptRecord;

    static void INTR_ATTR ISR(void* args);

    static void initWorker(void* args) {
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

    static portMUX_TYPE s_lock;
    static InterruptHandler s_interruptHandler[CPUS_COUNT][ISR_SLOTS];
    static intr_handle_t s_interruptHandle[CPUS_COUNT];
};
portMUX_TYPE Pin::s_lock = portMUX_INITIALIZER_UNLOCKED;
Pin::InterruptHandler Pin::s_interruptHandler[CPUS_COUNT][Pin::ISR_SLOTS] = { 0 };
intr_handle_t Pin::s_interruptHandle[CPUS_COUNT] = { nullptr };

Pin isrOut(ISR_TEST_PIN, OUTPUT);

void INTR_ATTR Pin::ISR(void* args) {
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
static typename Timer::value_t INTR_ATTR wait(typename Timer::value_t t) {
    const typename Timer::value_t tStart = Timer::value();
    t += tStart;
    while(Timer::value() < t);
    return tStart;
}

template <typename T, size_t Capacity>
class Buffer {
public:
    typedef size_t index_type;
	typedef T value_type;
	static const index_type capacity = Capacity;
    Buffer()
        : m_lock(portMUX_INITIALIZER_UNLOCKED),
          m_rptr(0),
          m_wptr(0)
    {}

    void clear()
	{
		m_rptr = m_wptr;
	}

	bool empty() const
	{
		return m_wptr == m_rptr;
	}

	bool full() const
	{
		return this->next(m_wptr) == m_rptr;
	}

	bool push(value_type v)
	{
		portENTER_CRITICAL_ISR(&m_lock);
        index_type wptr = m_wptr;
		m_buffer[wptr] = v;
		m_wptr = this->next(wptr);
        bool overflow = m_wptr != m_rptr;
        portEXIT_CRITICAL_ISR(&m_lock);
		return overflow;
	}

	value_type top() const
	{
		return m_buffer[m_rptr];
	}
	
	volatile value_type const & top_ref() const
	{
		return m_buffer[m_rptr];
	}
	
	volatile value_type & top_ref()
	{
		return m_buffer[m_rptr];
	}

	index_type size() const
	{
		return this->dist(m_wptr, m_rptr);
	}

	value_type operator[](index_type i) const
	{
		return m_buffer[this->next(m_rptr, i)];
	}

    void pop()
	{
		portENTER_CRITICAL_ISR(&m_lock);
        m_rptr = this->next(m_rptr);
        portEXIT_CRITICAL_ISR(&m_lock);
	}

    bool try_pop(value_type & v)
	{
		portENTER_CRITICAL_ISR(&m_lock);
        index_type rptr = m_rptr;
		if (m_wptr == rptr) {
            portEXIT_CRITICAL_ISR(&m_lock);
			return false;
        }
		v = m_buffer[rptr];
		m_rptr = this->next(rptr);
        portEXIT_CRITICAL_ISR(&m_lock);
		return true;
	}

private:
    portMUX_TYPE m_lock;
    volatile index_type m_rptr;
    volatile index_type m_wptr;
    volatile value_type m_buffer[capacity];

    static index_type next(index_type v)
	{
		return index_type(v + 1) % Capacity;
	}

	static index_type next(index_type v, index_type len)
	{
		return index_type(v + len) % Capacity;
	}

	static index_type dist(index_type ptr, index_type base)
	{
		return index_type(ptr - base) % Capacity;
	}
};

class OneWire {
    typedef fastestTimer Timer;
    enum class State: uint8_t { MASTER, CHECK_RESET, READ, WRITE, SEARCH };
    enum class SlaveSearchState: uint8_t { IDLE, READY, SEND_BIT, SEND_COMPLEMENT, READ_BIT };
    typedef uint8_t roms_count_t;
    static constexpr roms_count_t MAX_ROMS = ONEWIRE_MAX_ROMS;

    void INTR_ATTR _pinISR();

public:
    struct Timing {
        typedef Timer::value_t time_t; // in CPU ticks -> 1 / 240 MHz
        // based on https://www.maximintegrated.com/en/design/technical-documents/app-notes/1/126.html
        //     plus slave timing - see StandardTiming definition below
        time_t A,B,C,D,E,F,G,H,I,J,K,L,M,N;
    };
    static const DRAM_ATTR Timing StandardTiming;
    static const DRAM_ATTR Timing OverdriveTiming;
    struct event_t {
        enum class type_t: uint8_t {
            RECEIVED,
            RESET,
            SEARCH_START,
            SEARCH_SEND_BIT,
            SEARCH_SEND_COMPLEMENT,
            SEARCH_READ_BIT,
            SEARCH_COMPLETE,
            ACTIVATE_ROM,
            DEACTIVATE_ROM
        };
        typedef uint8_t value_t;
        typedef Timer::value_t time_t;
        time_t time;
        type_t type;
        value_t value;
        void INTR_ATTR operator =(volatile event_t& e) volatile {
            time = e.time;
            type = e.type;
            value = e.value;
        }
    };
    class Rom {
        friend void INTR_ATTR OneWire::_pinISR();
        Rom(const Rom&) = delete;
    public:
        union rom_t {
            uint64_t value;
            uint8_t byte[8];
            struct {
                uint8_t family;
                uint8_t addr[6];
                uint8_t crc;
            };
            bool operator == (const rom_t& other) { return value == other.value; }
        };
        Rom(const uint8_t* addr) {
            _init(addr);
        }
        Rom(uint8_t familyCode, const uint8_t* addr) {
            _init(familyCode, addr);
        }
        Rom(std::initializer_list<uint8_t> addr) {
            _init(addr.begin());
            if (addr.size() != 7) {
                ESP_LOGW("OneWire:Rom:init", "Invalid ROM address length %d. Will be filled in by zeros.", addr.size());
                for(auto i = addr.size(); i < 7; ++i)
                    m_value.byte[i] = 0;
                m_value.crc = crc(m_value.byte, 7);
            }
        }
        Rom(uint8_t familyCode, std::initializer_list<uint8_t> addr) {
            _init(familyCode, addr.begin());
            if (addr.size() != 6) {
                ESP_LOGW("OneWire:Rom:init", "Invalid ROM address length %d. Will be filled in by zeros.", addr.size());
                for(auto i = addr.size(); i < 7; ++i)
                    m_value.addr[i] = 0;
                m_value.crc = crc(m_value.byte, 7);
            }
        }
        
        const rom_t& value() const { return m_value; }
        const uint8_t* begin() const { return m_value.byte; }
        const uint8_t* end() const { return m_value.byte+8; }
        bool getBit(uint8_t bit) const { return m_value.byte[bit>>3] & (1<<(bit&7)); }

        void enable(bool en = true) { m_enabled = en; }
        void disable() { m_enabled = false; }
        bool isEnabled() const { return m_enabled; }

        void setAlarm(bool v = true) { m_alarm = v; }
        void clearAlarm() { m_alarm = false; }
        bool alarm() const { return m_alarm; }

        bool isActive() const { return m_enabled && m_active; }

        void registerCallback(Callback callback) { m_onEvent = callback; }
        const event_t& getLastEvent() const { return const_cast<const event_t&>(m_buffer.top_ref()); }
        void popEvent() { m_buffer.pop(); }
        size_t available() const { return m_buffer.size(); }

    private:
        void _init(const uint8_t* addr) {
            m_enabled = false;
            m_active = true;
            m_alarm = false;
            for (uint8_t i = 0; i !=7; ++i)
                m_value.byte[i] = addr[i];
            m_value.crc = crc(m_value.byte, 7);
        }
        void _init(uint8_t familyCode, const uint8_t* addr) {
            m_enabled = false;
            m_active = true;
            m_alarm = false;
            m_value.family = familyCode;
            for (uint8_t i = 0; i !=6; ++i)
                m_value.addr[i] = addr[i];
            m_value.crc = crc(m_value.byte, 7);
        }

        void INTR_ATTR event(const event_t& e) {
            m_buffer.push(e);
            m_onEvent();
        }
        void INTR_ATTR event(event_t::type_t type, event_t::value_t value) {
            event_t e;
            e.time = Timer::value();
            e.type = type;
            e.value = value;
            event(e);
        }

        void setActiveState() {
            m_active = true;
            //event(event_t::type_t::ACTIVATE_ROM, 0);
        }
        void setWaitState(uint8_t bit) {
            m_active = false;
            event(event_t::type_t::DEACTIVATE_ROM, bit);
        }

        volatile bool m_enabled;
        volatile bool m_active;
        volatile bool m_alarm;
        rom_t m_value;
        Buffer<event_t, 256> m_buffer;
        Callback m_onEvent;

    public:
        static uint8_t INTR_ATTR crc(uint8_t& _crc, bool bit) {
            const bool doXor = (_crc ^ bit) & 1;
            _crc >>= 1;
            if (doXor)
                _crc ^= 0x8C;
            return _crc;
        }
        static uint8_t crc(uint8_t& _crc, uint8_t byte) {
            for(uint8_t bit = 0; bit != 8; ++bit) {
                crc(_crc, bool(byte & 1));
                byte >>= 1;
            }
            return _crc;
        }
        static uint8_t crc(const uint8_t* data, size_t length) {
            uint8_t _crc = 0;
            for (const uint8_t* const stop = data + length; data != stop; ++data)
                crc(_crc, *data);
            return _crc;
        }
    };
    enum class CMD: uint8_t {
            SEARCH              = 0xF0,
            SEARCH_ALARM        = 0xEC
        };
private:
    struct search_state_t {
        Rom::rom_t rom;
        uint8_t LastDiscrepancy;
        uint8_t LastFamilyDiscrepancy;
        bool LastDeviceFlag;
    };
public:

    OneWire(Pin input,
            Pin output,
            Pin boost = Pin(DUMMY_PIN),
            const Timing* timing = &StandardTiming,
            Callback onEvent = Callback())
        : m_input(input),
          m_output(output),
          m_boost(boost),
          m_timing(timing),
          m_lock(portMUX_INITIALIZER_UNLOCKED),
          m_state(State::MASTER),
          m_bitIndex(0),
          m_receivingByte(0),
          m_edgeTime(0),
          m_selfAdvertising(false),
          m_slaveSearchState(SlaveSearchState::IDLE),
          m_searchState(),
          m_buffer(),
          m_onEvent(onEvent)
    {
        m_input.setHigh();
        m_output.setHigh();
        m_boost.setLow();
        for (roms_count_t i = 0; i != MAX_ROMS; ++i)
            m_rom[i] = nullptr;
        becomeSlave();
    }

    void becomeMaster() {
        if (m_state == State::MASTER)
            return;
        m_input.detachInterrupt();
        m_buffer.clear();
        m_state = State::MASTER;
    }
    void becomeSlave() {
        if (m_state != State::MASTER)
            return;
        m_receivingByte = 0;
        m_bitIndex = 0;
        m_state = State::READ;
        m_slaveSearchState = SlaveSearchState::IDLE;
        m_input.attachInterrupt(pinISR, this, FALLING);
    }

    void resetSearch() {
        m_searchState.rom.value = 0;
        m_searchState.LastDiscrepancy = 0;
        m_searchState.LastFamilyDiscrepancy = 0;
        m_searchState.LastDeviceFlag = false;
    }
    bool verify(const Rom::rom_t& rom) {
        search_state_t stateBackup = m_searchState;
        bool res = false;
        m_searchState.rom = rom;
        m_searchState.LastDiscrepancy = 64;
        m_searchState.LastDeviceFlag = false;
        if (search())
            res = (m_searchState.rom == rom);
        m_searchState = stateBackup;
        return res;
    }
    void searchForFamily(uint8_t family) {
        m_searchState.rom.value = family;
        m_searchState.LastDiscrepancy = 64;
        m_searchState.LastFamilyDiscrepancy = 0;
        m_searchState.LastDeviceFlag = false;
    }
    void skipThisFamily() {
        m_searchState.LastDiscrepancy = m_searchState.LastFamilyDiscrepancy;
        m_searchState.LastFamilyDiscrepancy = 0;

        // check for end of list
        if (m_searchState.LastDiscrepancy == 0)
            m_searchState.LastDeviceFlag = true;
    }
    bool search(bool alarmOnly = false) {
        uint8_t id_bit_number = 1;
        uint8_t last_zero = 0;
        uint8_t rom_byte_number = 0;
        uint8_t rom_byte_mask = 1;
        uint8_t crc = 0;
        bool search_result = false;
        bool search_direction = false;
        bool id_bit = false;
        bool cmp_id_bit = false;

        // if the last call was not the last one
        if (!m_searchState.LastDeviceFlag) {
            // 1-Wire reset
            print("    Search: start\n");
            if (!reset()) {
                // reset the search
                resetSearch();
                print("    Search: no device available\n");
                return false;
            }

            // issue the search command 
            write(alarmOnly ? CMD::SEARCH_ALARM : CMD::SEARCH);
            wait<Timer>(240*250);

            // loop to do the search
            do {
                // read a bit and its complement
                id_bit = _readBit();
                cmp_id_bit = _readBit();
                //print("    Search: bit {:2} = {} | {}\n", id_bit_number, id_bit, cmp_id_bit);

                // check for no devices on 1-wire
                if ((id_bit == 1) && (cmp_id_bit == 1)) {
                    print("    Search: both read bits {} are true - no devices\n", id_bit_number);
                    break;
                } else {
                    // all devices coupled have 0 or 1
                    if (id_bit != cmp_id_bit) {
                        search_direction = id_bit;  // bit write value for search
                    } else {
                        // if this discrepancy if before the Last Discrepancy
                        // on a previous next then pick the same as last time
                        if (id_bit_number < m_searchState.LastDiscrepancy)
                            search_direction = ((m_searchState.rom.byte[rom_byte_number] & rom_byte_mask) > 0);
                        else
                            // if equal to last pick 1, if not then pick 0
                            search_direction = (id_bit_number == m_searchState.LastDiscrepancy);

                        // if 0 was picked then record its position in LastZero
                        if (search_direction == 0) {
                            last_zero = id_bit_number;

                            // check for Last discrepancy in family
                            if (last_zero < 9)
                                m_searchState.LastFamilyDiscrepancy = last_zero;
                        }
                    }

                    // set or clear the bit in the ROM byte rom_byte_number
                    // with mask rom_byte_mask
                    if (search_direction == 1)
                        m_searchState.rom.byte[rom_byte_number] |= rom_byte_mask;
                    else
                        m_searchState.rom.byte[rom_byte_number] &= ~rom_byte_mask;

                    // serial number search direction write bit
                    _writeBit(search_direction);

                    // increment the byte counter id_bit_number
                    // and shift the mask rom_byte_mask
                    id_bit_number++;
                    rom_byte_mask <<= 1;
        
                    // if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
                    if (rom_byte_mask == 0) {
                        Rom::crc(crc, m_searchState.rom.byte[rom_byte_number]);  // accumulate the CRC
                        rom_byte_number++;
                        rom_byte_mask = 1;
                    }
                }
            } while(rom_byte_number < 8);  // loop until through all ROM bytes 0-7
      
            // if the search was successful then
            if (!((id_bit_number < 65) || (crc != 0))) {
                // search successful so set m_searchState.LastDiscrepancy,m_searchState.LastDeviceFlag,search_result
                m_searchState.LastDiscrepancy = last_zero;
 
                // check for last device
                if (m_searchState.LastDiscrepancy == 0) {
                    m_searchState.LastDeviceFlag = true;
                    print("    Search: found last device\n");
                }
                
                search_result = true;
            }
        }
     
        // if no device found then reset counters so next 'search' will be like a first
        if (!search_result || m_searchState.rom.family == 0) {
            resetSearch();
            search_result = false;
        }
     
        return search_result;
    }
    const Rom::rom_t& LastFoundRom() const { return m_searchState.rom; }

    bool reset() {
        portENTER_CRITICAL(&m_lock);
        becomeMaster();
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
    uint8_t read() {
        uint8_t v = 0;
        if (m_state == State::MASTER) {
            for(uint8_t i = 0; i != 8; ++i) {
                v |= _readBit();
                v <<= 1;
            }
        }
        return v;
    }
    void write(CMD cmd) { write(uint8_t(cmd)); }
    void write(uint8_t v) {
        for (uint8_t i = 1; i != 0; i<<=1)
            _writeBit(v & i);
    }
    
    const event_t& getLastEvent() const { return const_cast<const event_t&>(m_buffer.top_ref()); }
    void popEvent() { m_buffer.pop(); }
    size_t available() const { return m_buffer.size(); }

    void selfAdvertising(bool enable) { m_selfAdvertising = enable; }
    bool selfAdvertising() const { return m_selfAdvertising; }

    bool addRom(Rom* rom) {
        for (roms_count_t i = 0; i != MAX_ROMS; ++i) {
            if (m_rom[i] == nullptr) {
                m_rom[i] = rom;
                return true;
            }
        }
        return false;
    }

private:
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
        m_boost.setHigh();
        m_output.setLow();
        wait<Timer>(m_timing->A);
        m_output.setHigh();
        wait<Timer>(m_timing->E);
        const bool v = m_input.read();
        m_boost.setLow();
        wait<Timer>(m_timing->F);
        portEXIT_CRITICAL(&m_lock);
        return v;
    }

    void INTR_ATTR _slaveWriteBit(const bool v) {
        if (!v) {
            m_output.setLow();
            wait<Timer>(m_timing->N);
            m_output.setHigh();
            m_input.clearInterruptFlag();
        }
    }
    void INTR_ATTR _slaveReadBit() { // reads to m_receivingByte
        m_edgeTime = wait<Timer>(m_timing->K);
        m_receivingByte >>= 1;
        if (m_input.read()) {
            m_receivingByte |= 0x80;
        } else {
            m_state = State::CHECK_RESET;
            m_input.interruptTrigger(RISING);
        }
    }
    void INTR_ATTR _slaveSearchSendBit(bool complement) {
        m_receivingByte = 1;
        for (roms_count_t i = 0; i != MAX_ROMS; ++i) {
            if (m_rom[i] == nullptr)
                break;
            if (m_rom[i]->isActive() && (m_rom[i]->getBit(m_bitIndex) ^ !complement)) {
                m_receivingByte =  0;
                break;
            }
        }
        _slaveWriteBit(m_receivingByte);
        _event(complement ? event_t::type_t::SEARCH_SEND_COMPLEMENT : event_t::type_t::SEARCH_SEND_BIT, m_bitIndex | (m_receivingByte<<7));
    }

    void INTR_ATTR _event(event_t::type_t type, event_t::value_t value) {
        event_t e;
        e.time = Timer::value();
        e.type = type;
        e.value = value;
        _event(e);
    }
    void INTR_ATTR _event(const event_t&e) {
        m_buffer.push(e);
        m_onEvent();
    }

    Pin m_input;
    Pin m_output;
    Pin m_boost;
    const Timing* m_timing;
    portMUX_TYPE m_lock;
    volatile State m_state;
    volatile uint8_t m_bitIndex;
    volatile uint8_t m_receivingByte;
    volatile Timer::value_t m_edgeTime;
    volatile bool m_selfAdvertising;
    volatile SlaveSearchState m_slaveSearchState;
    search_state_t m_searchState;
    Rom* m_rom[MAX_ROMS];
    Buffer<event_t, 256> m_buffer;
    Callback m_onEvent;

    static void INTR_ATTR pinISR(void* args) {
        reinterpret_cast<OneWire*>(args)->_pinISR();
    }
};

void INTR_ATTR OneWire::_pinISR() {
    m_boost.setHigh();
    switch (m_state) {
    case State::MASTER:
        break;
    case State::CHECK_RESET:
        if ((Timer::value() - m_edgeTime) > m_timing->H) {
            bool isAnyRomEnabled = false;
            for (roms_count_t i = 0; i != MAX_ROMS; ++i) {
                if (m_rom[i] == nullptr)
                    break;
                if (m_rom[i]->isEnabled()) {
                    isAnyRomEnabled =  true;
                    break;
                }
            }
            if (m_selfAdvertising && isAnyRomEnabled) {
                wait<Timer>(m_timing->L);
                m_output.setLow();
                wait<Timer>(m_timing->M);
                m_output.setHigh();
            } else {
                wait<Timer>(m_timing->I);
            }
            m_receivingByte = 0;
            m_bitIndex = 0;
            m_slaveSearchState = SlaveSearchState::READY;
            const event_t e = {
                .time = Timer::value(),
                .type = event_t::type_t::RESET,
                .value = 0
            };
            for (roms_count_t i = 0; i != MAX_ROMS; ++i) {
                if (m_rom[i] == nullptr)
                    break;
                m_rom[i]->setActiveState();
                m_rom[i]->event(e);
            }
            _event(e);
            m_state = State::READ;
        } else {
            m_state = ( m_slaveSearchState == SlaveSearchState::IDLE
                        || m_slaveSearchState == SlaveSearchState::READY ) ? State::READ : State::SEARCH;
        }
        m_input.interruptTrigger(FALLING);
        m_input.clearInterruptFlag();
        break;
    case State::READ:
        _slaveReadBit();
        if (++m_bitIndex == 8) {
            if (m_slaveSearchState == SlaveSearchState::READY) {
                if ((m_receivingByte == uint8_t(CMD::SEARCH) || m_receivingByte == uint8_t(CMD::SEARCH_ALARM))) {
                    m_slaveSearchState = SlaveSearchState::SEND_BIT;
                    m_state = State::SEARCH;
                } else {
                    m_slaveSearchState = SlaveSearchState::IDLE;
                }
            }
            const event_t e = {
                .time = Timer::value(),
                .type = event_t::type_t::RECEIVED,
                .value = m_receivingByte
            };
            for (roms_count_t i = 0; i != MAX_ROMS; ++i) {
                if (m_rom[i] == nullptr)
                    break;
                m_rom[i]->event(e);
            }
            _event(e);
            m_receivingByte = 0;
            m_bitIndex = 0;
        }
        break;
    case State::WRITE:
        _slaveWriteBit(m_buffer.top_ref().value & (1<<m_bitIndex));
        if (++m_bitIndex == 8) {
            if (!m_buffer.empty())
                m_buffer.pop();
            m_bitIndex = 0;
        }
        break;
    case State::SEARCH:
        switch (m_slaveSearchState) {
        case SlaveSearchState::IDLE:
        case SlaveSearchState::READY:
            break;
        case SlaveSearchState::SEND_BIT:
            _slaveSearchSendBit(false);
            m_slaveSearchState = SlaveSearchState::SEND_COMPLEMENT;
            break;
        case SlaveSearchState::SEND_COMPLEMENT:
            _slaveSearchSendBit(true);
            m_slaveSearchState = SlaveSearchState::READ_BIT;
            break;
        case SlaveSearchState::READ_BIT:
            m_receivingByte = 0;
            _slaveReadBit(); // read value is in m_receivingByte
            _event(event_t::type_t::SEARCH_READ_BIT, m_bitIndex | m_receivingByte);
            for (roms_count_t i = 0; i != MAX_ROMS; ++i) {
                if (m_rom[i] == nullptr)
                    break;
                if (m_rom[i]->isActive() && (bool(m_receivingByte) != m_rom[i]->getBit(m_bitIndex))) {
                    m_rom[i]->setWaitState(m_bitIndex+1);
                }
            }
            if (++m_bitIndex == 64) {
                if (m_receivingByte != 0) {
                    m_receivingByte = 0;
                    m_state = State::READ;
                }
                m_bitIndex = 0;
                m_slaveSearchState = SlaveSearchState::IDLE;
                _event(event_t::type_t::SEARCH_COMPLETE, 0);
            } else {
                m_slaveSearchState = SlaveSearchState::SEND_BIT;
            }
            break;
        }
        break;
    }
    m_boost.setLow();
}

#define US2TICKS(us) OneWire::Timing::time_t((us) * 240)
// based on https://www.maximintegrated.com/en/design/technical-documents/app-notes/1/126.html
//     plus slave timing
//     - ESP32 processing offset
const DRAM_ATTR OneWire::Timing OneWire::StandardTiming = {
    .A = US2TICKS(  6 - 0.0), // master write zero first
    .B = US2TICKS( 64 - 0.0), // master write zero second
    .C = US2TICKS( 60 - 0.0), // master write one first
    .D = US2TICKS( 10 - 0.0), // master write one second
    .E = US2TICKS(  9 - 0.0), // master read sample
    .F = US2TICKS( 55 - 0.0), // master read pause
    .G = US2TICKS(  0 - 0.0), // master reset before
    .H = US2TICKS(480 - 0.0), // master reset low
    .I = US2TICKS( 70 - 0.0), // master reset sample
    .J = US2TICKS(410 - 0.0), // master reset pause
    .K = US2TICKS( 35 - 2.5), // slave read sample
    .L = US2TICKS( 35 - 0.0), // slave presence responce
    .M = US2TICKS( 70 - 0.0), // slave presence release
    .N = US2TICKS( 30 - 0.0)  // slave write zero
};
const DRAM_ATTR OneWire::Timing OneWire::OverdriveTiming = { // Too fast for ESP32
    .A = US2TICKS(  1.0),
    .B = US2TICKS(  7.5),
    .C = US2TICKS(  7.5),
    .D = US2TICKS(  2.5),
    .E = US2TICKS(  1.0),
    .F = US2TICKS(  7.0),
    .G = US2TICKS(  2.5),
    .H = US2TICKS( 70.0),
    .I = US2TICKS(  8.5),
    .J = US2TICKS( 40.0),
    .K = US2TICKS(  4.5),
    .L = US2TICKS(  4.5),
    .M = US2TICKS(  8.5),
    .N = US2TICKS(  3.0)
};

static void INTR_ATTR notifyTaskFromISR(Callback::callback_arg_t args) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR( reinterpret_cast<TaskHandle_t>(args), &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE)
        portYIELD_FROM_ISR();
}

static constexpr uint8_t ROMS_COUNT = 3;
static constexpr uint8_t ROM_ADDRESS[ROMS_COUNT][7] = {
    { 0x10,0x50,0xA9,0x0A,0x02,0x08,0x00 },
    { 0x10,0x51,0xA9,0x0A,0x02,0x08,0x00 },
    { 0x10,0x52,0xA9,0x0A,0x02,0x08,0x00 }
};

static OneWire* slaveOw = nullptr;
static OneWire::Rom* slaveRom[ROMS_COUNT] = { nullptr };

#define onSlave(ptr, fcn, msg, msgArgs...) \
    if (ptr) { \
        print(msg, msgArgs); \
        ptr->fcn; \
    } else { \
        print("No slave available.\n");\
    }

static void oneWireMaster(void* args = nullptr) {
    const Pin::pin_num_t onewirePin = ONEWIRE_MASTER_PIN;
    OneWire ow(
        Pin(onewirePin, OUTPUT_OPEN_DRAIN | PULLUP, HIGH),
        Pin(onewirePin),
        Pin(OW_MASTER_TEST_PIN, OUTPUT) );
    ow.becomeMaster();
    uint8_t v = 0xAA;
    for(;0;) {
        //bool presence = ow.reset();
        print("write 0x{:02X}\n", v);
        ow.write(v++);
        if (v % 4 == 0) {
            print("reset\n");
            ow.reset();
            delay(1000);    
        }
        //print("presence: {}\n", presence);
        delay(1000);
    }
    for(;;) {
        if (Serial.available()) {
            char c = Serial.read();
            switch(c) {
            case '\r':
                print("\n");
                break;
            case 'R':
                print("reset: {} device available.\n", ow.reset() ? "any" : "no");
                break;
            case 'S':
                print("Searching bus:\n");
                ow.resetSearch();
                while(ow.search()) {
                    print("  Found 0x{:016X}\n", ow.LastFoundRom().value);
                }
                print("Search complete.\n");
                break;
            case 'E':
                onSlave(slaveOw, selfAdvertising(true), "Enable slave selfadvertising.\n", 0);
                break;
            case 'D':
                onSlave(slaveOw, selfAdvertising(false), "Disable slave selfadvertising.\n", 0);
                break;
            case '0' ... '2':
                c -= '0';
                onSlave(slaveRom[uint8_t(c)], enable(!slaveRom[uint8_t(c)]->isEnabled()), "{} slave ROM{} advertising.\n", !slaveRom[uint8_t(c)]->isEnabled(), uint8_t(c));
                break;
            default:
                print("write 0x{:02X}\n", c);
                ow.write(c);
                break;
            }
        } else {
            delay(100);
        }
    }
}

static void oneWireSlave(void* args = nullptr) {
    const Pin::pin_num_t onewirePin = ONEWIRE_SLAVE_PIN;
    OneWire ow(
        Pin(onewirePin, OUTPUT_OPEN_DRAIN | PULLUP, HIGH),
        Pin(onewirePin),
        Pin(OW_SLAVE_TEST_PIN, OUTPUT),
        &OneWire::StandardTiming,
        Callback(notifyTaskFromISR, xTaskGetCurrentTaskHandle()) );
    // this is pretty nasty
    slaveOw = &ow;
    for (uint8_t i = 0; i != ROMS_COUNT; ++i) {
        slaveRom[i] = new OneWire::Rom(&ROM_ADDRESS[i][0]);
        slaveRom[i]->registerCallback(Callback(notifyTaskFromISR, xTaskGetCurrentTaskHandle()));
        ow.addRom(slaveRom[i]);
        slaveRom[i]->enable();
        print("ROM{}: {:02X}\n", i, fmt::join(*slaveRom[i], " "));
    }
    ow.selfAdvertising(true);
    for(;;) {
        ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
        while (ow.available()) {
            OneWire::event_t e = ow.getLastEvent();
            OneWire::event_t::time_t eventTime   = e.time;
            OneWire::event_t::type_t eventType   = e.type;
            OneWire::event_t::value_t eventValue = e.value;
            ow.popEvent();
            Printer::acquire();
            print("{:10}\tOneWireSlave: ", eventTime);
            switch(eventType) {
            case OneWire::event_t::type_t::RECEIVED:
                print("receive 0x{:02X}\n", eventValue);
                break;
            case OneWire::event_t::type_t::RESET:
                print("reset\n");
                break;
            case OneWire::event_t::type_t::SEARCH_START:
                print("search start\n");
                break;
            case OneWire::event_t::type_t::SEARCH_SEND_BIT:
                print("search send  bit {}: {}\n", eventValue & 0x7F, bool(eventValue & 0x80));
                break;
            case OneWire::event_t::type_t::SEARCH_SEND_COMPLEMENT:
                print("search send ~bit {}: {}\n", eventValue & 0x7F, bool(eventValue & 0x80));
                break;
            case OneWire::event_t::type_t::SEARCH_READ_BIT:
                print("search READ  bit {}: {}\n", eventValue & 0x7F, bool(eventValue & 0x80));
                break;
            case OneWire::event_t::type_t::SEARCH_COMPLETE:
                print("search complete\n");
                break;
            default:
                print("unknown 0x{:02X} = 0x{:02X}\n", uint8_t(eventType), eventValue);
                break;
            }
            Printer::release();
        }
        for (auto rom: slaveRom) {
            if (rom == nullptr)
                continue;
            while (rom->available()) {
                OneWire::event_t e = rom->getLastEvent();
                OneWire::event_t::time_t eventTime   = e.time;
                OneWire::event_t::type_t eventType   = e.type;
                OneWire::event_t::value_t eventValue = e.value;
                rom->popEvent();
                Printer::acquire();
                print("{:10}\tOneWireSlave::Rom 0x{:016X}: ", eventTime, rom->value().value);
                switch(eventType) {
                case OneWire::event_t::type_t::RECEIVED:
                    print("receive 0x{:02X}\n", eventValue);
                    break;
                case OneWire::event_t::type_t::RESET:
                    print("reset\n");
                    break;
                case OneWire::event_t::type_t::ACTIVATE_ROM:
                    print("activated\n");
                    break;
                case OneWire::event_t::type_t::DEACTIVATE_ROM:
                    print("deactivated on bit {}\n", eventValue);
                    break;
                default:
                    print("unknown 0x{:02X} = 0x{:02X}\n", uint8_t(eventType), eventValue);
                    break;
                }
                Printer::release();
            }
        }
    }
}

inline static void checkReset() {
    esp_reset_reason_t resetReason = esp_reset_reason();
    switch (resetReason) {
    case ESP_RST_UNKNOWN:
        print("\tUnknown reset - strange\n");
        break;
    case ESP_RST_POWERON:
        print("\tPoweron reset\n");
        break;
    case ESP_RST_EXT:
        print("\tExternal reset\n");
        break;
    case ESP_RST_SW:
        print("\tSoftware reset\n");
        break;
    case ESP_RST_PANIC:
        print("\tReset due to core panic - stop program\n");
        vTaskSuspend(nullptr);
        break;
    case ESP_RST_INT_WDT:
        print("\tReset due to interrupt watchdog - stop program\n");
        vTaskSuspend(nullptr);
        break;
    case ESP_RST_TASK_WDT:
        print("\tReset due to task watchdog - stop program\n");
        vTaskSuspend(nullptr);
        break;
    case ESP_RST_WDT:
        print("\tReset due to some watchdog - stop program\n");
        vTaskSuspend(nullptr);
        break;
    case ESP_RST_DEEPSLEEP:
        print("\tWaked from deep sleep\n");
        break;
    case ESP_RST_BROWNOUT:
        print("\tBrownout reset - please check power\n");
        break;
    case ESP_RST_SDIO:
        print("\tSDIO reset - strange\n");
        break;
    }
}

void setup() {
    Serial.begin(115200);
    Printer::begin();
    print("OneWire sniffer\n");
    checkReset();

    testQueue::init();

    Pin::initInterrupts();

    xTaskCreatePinnedToCore(oneWireMaster, "OneWireMaster", 4*1024, nullptr, 4, nullptr, 0);
    xTaskCreatePinnedToCore(oneWireSlave , "OneWireSlave" , 8*1024, nullptr, 4, nullptr, 1);
    delay(500);
}

void loop() {
    testQueue::print();
}