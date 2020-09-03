#include "esp_log.h"
#include <Arduino.h>

#include "driver/timer.h"

#define INTR_ATTR IRAM_ATTR

#define DUMMY_PIN 40

#define ONEWIRE_MASTER_PIN 22
#define ONEWIRE_SLAVE_PIN  23

#include <format.h>

template <typename S, typename... Args>
static inline void print(const S& format_str, Args&&... args);

class Printer { // constide use of MessageBuffer, instead of queue
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
        BaseType_t ret = xTaskCreate(process, "Printer::process", stackSize, nullptr, priority, nullptr);
        if (ret != pdPASS) {
            ESP_LOGE("Printer::init", "Can not create process task (error code %d).", ret);
            //xQueueDelete(queue); // not supported?!
            queue = nullptr;
            return false;
        }
        return true;
    }
    template <typename S, typename... Args>
    friend void print(const S& format_str, Args&&... args);
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
};
QueueHandle_t Printer::queue = nullptr;

template <typename S, typename... Args>
static inline void print(const S& format_str, Args&&... args) {
    fmt::memory_buffer* buf = new fmt::memory_buffer;
    fmt::format_to(*buf, format_str, args...);
    if (xQueueSend(Printer::queue, &buf, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE("Printer::print", "Can not send message to the queue.");
        delete buf;
    }
}

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
    void attachInterrupt(intr_callback_t callback, intr_callback_arg_t arg, int mode) { attachInterruptArg(digitalPinToInterrupt(m_pin), callback, arg, mode); }
    void detachInterrupt() { ::detachInterrupt(digitalPinToInterrupt(m_pin)); }
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
static void INTR_ATTR wait(typename Timer::value_t t) {
    t += Timer::value();
    while(Timer::value() < t);
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

class Callback {
public:
    typedef void* callback_arg_t;
    typedef void (*callback_t)(callback_arg_t);
    Callback(callback_t callback = nullptr, callback_arg_t arg = nullptr)
        : m_callback(callback),
          m_callback_arg(arg)
    {}
    void INTR_ATTR operator ()() { if (m_callback) m_callback(m_callback_arg); }
private:
    callback_t m_callback;
    callback_arg_t m_callback_arg;
};

class OneWire {
    typedef fastestTimer Timer;
    enum State { MASTER, READ, WRITE, SCANNED, WAIT };
public:
    struct Timing {
        typedef Timer::value_t time_t; // in CPU ticks -> 1 / 240 MHz
        // based on https://www.maximintegrated.com/en/design/technical-documents/app-notes/1/126.html
        //     plus slave timing - see StandardTiming definition below
        time_t A,B,C,D,E,F,G,H,I,J,K;
    };
    static const Timing StandardTiming;
    static const Timing OverdriveTiming;
    OneWire(Pin input,
            Pin output,
            Pin boost = Pin(DUMMY_PIN),
            const Timing* timing = &StandardTiming,
            Callback onReceive = Callback())
        : m_input(input),
          m_output(output),
          m_boost(boost),
          m_timing(timing),
          m_lock(portMUX_INITIALIZER_UNLOCKED),
          m_state(MASTER),
          m_bitIndex(0),
          m_receivingByte(0),
          m_buffer(),
          m_onReceive(onReceive)
    {
        becomeSlave();
    }

    void becomeMaster() {
        if (m_state == MASTER)
            return;
        m_input.detachInterrupt();
        m_buffer.clear();
        m_state = MASTER;
    }

    void becomeSlave() {
        if (m_state != MASTER)
            return;
        m_state = READ;
        m_input.attachInterrupt(pinISR, this, FALLING);
    }

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
    void write(uint8_t v) {
        for (uint8_t i = 1; i != 0; i<<=1)
            _writeBit(v & i);
    }
    uint8_t read() {
        uint8_t v = 0;
        if (m_state == MASTER) {
            for(uint8_t i = 0; i != 8; ++i) {
                v |= _readBit();
                v <<= 1;
            }
        } else {
            m_buffer.try_pop(v);
        }
        return v;
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
        m_output.setLow();
        wait<Timer>(m_timing->A);
        m_output.setHigh();
        wait<Timer>(m_timing->E);
        const bool v = m_input.read();
        wait<Timer>(m_timing->F);
        portEXIT_CRITICAL(&m_lock);
        return v;
    }

    Pin m_input;
    Pin m_output;
    Pin m_boost;
    const Timing* m_timing;
    portMUX_TYPE m_lock;
    State m_state;
    volatile uint8_t m_bitIndex;
    volatile uint8_t m_receivingByte;
    Buffer<uint8_t, 64> m_buffer;
    Callback m_onReceive;
    static void INTR_ATTR pinISR(void* args) {
        OneWire* self = reinterpret_cast<OneWire*>(args);
        switch (self->m_state) {
        case MASTER:
            break;
        case READ:
            wait<Timer>(self->m_timing->K);
            self->m_receivingByte = (self->m_receivingByte << 1) | self->m_input.read();
            if (++self->m_bitIndex == 8) {
                self->m_buffer.push(self->m_receivingByte);
                self->m_receivingByte = 0;
                self->m_bitIndex = 0;
                self->m_onReceive();
            }
            break;
        case WRITE:
            self->m_output.setValue(self->m_buffer.top() & (1<<self->m_bitIndex));
            if (++self->m_bitIndex == 8) {
                if (!self->m_buffer.empty())
                    self->m_buffer.pop();
                self->m_bitIndex = 0;
            }
            break;
        case SCANNED:
            break;
        case WAIT:
            break;
        }
    }
};
#define US2TICKS(us) OneWire::Timing::time_t(us * 240)
// based on https://www.maximintegrated.com/en/design/technical-documents/app-notes/1/126.html
//     plus slave timing
const OneWire::Timing OneWire::StandardTiming = {
    .A = US2TICKS(  6), // master write zero first
    .B = US2TICKS( 64), // master write zero second
    .C = US2TICKS( 60), // master write one first
    .D = US2TICKS( 10), // master write one first
    .E = US2TICKS(  9), // master read sample
    .F = US2TICKS( 55), // master read pause
    .G = US2TICKS(  0), // master reset before
    .H = US2TICKS(480), // master reset low
    .I = US2TICKS( 70), // master reset sample
    .J = US2TICKS(410), // master reset pause
    .K = US2TICKS( 35)  // slave read sample
};
const OneWire::Timing OneWire::OverdriveTiming = {
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
    .K = US2TICKS(  4.5)
};

static void oneWireMaster(void* args = nullptr) {
    const Pin::pin_num_t onewirePin = ONEWIRE_MASTER_PIN;
    OneWire ow(Pin(onewirePin, OUTPUT_OPEN_DRAIN | PULLUP), Pin(onewirePin));
    uint8_t v = 0xAA;
    for(;;) {
        //bool presence = ow.reset();
        print("write 0x{:02X}\n", v);
        ow.write(v);
        //print("presence: {}\n", presence);
        delay(1000);
    }
}

static void INTR_ATTR notifyTaskFromISR(Callback::callback_arg_t args) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR( reinterpret_cast<TaskHandle_t>(args), &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE)
        portYIELD_FROM_ISR();
}

static void oneWireSlave(void* args = nullptr) {
    const Pin::pin_num_t onewirePin = ONEWIRE_SLAVE_PIN;
    OneWire ow(
        Pin(onewirePin,OUTPUT_OPEN_DRAIN | PULLUP),
        Pin(onewirePin),
        Pin(DUMMY_PIN),
        &OneWire::StandardTiming,
        Callback(notifyTaskFromISR, xTaskGetCurrentTaskHandle()) );
    for(;;) {
        ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
        print("OneWireSlave:receive {:02X}\n", ow.read());
    }
}

void setup() {
    Serial.begin(115200);
    Printer::begin();
    print("OneWire sniffer\n");
    xTaskCreatePinnedToCore(oneWireMaster, "OneWireMaster", 2*1024, nullptr, 4, nullptr, 0);
    xTaskCreatePinnedToCore(oneWireSlave , "OneWireSlave" , 2*1024, nullptr, 4, nullptr, 1);
}

void loop() {}