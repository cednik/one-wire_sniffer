#pragma once

#include "pin.hpp"
#include "buffer.hpp"
#include "cpu_timer.hpp"

#include <initializer_list>

class OneWire {
    typedef CPUTimer Timer;
    enum class State: uint8_t { MASTER, CHECK_RESET, READ, WRITE, SEARCH };
    enum class SlaveSearchState: uint8_t { IDLE, READY, SEND_BIT, SEND_COMPLEMENT, READ_BIT };
    typedef uint8_t roms_count_t;
    static constexpr roms_count_t MAX_ROMS = 8;

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
        void INTR_ATTR operator =(volatile event_t& e) volatile;
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
            bool INTR_ATTR operator == (const rom_t& other);
        };

        Rom(const uint8_t* addr);
        Rom(uint8_t familyCode, const uint8_t* addr);
        Rom(std::initializer_list<uint8_t> addr);
        Rom(uint8_t familyCode, std::initializer_list<uint8_t> addr);
        
        const rom_t& INTR_ATTR value() const;
        
        const uint8_t* begin() const;
        const uint8_t*   end() const;

        bool INTR_ATTR getBit(uint8_t bit) const;

        void INTR_ATTR enable(bool en = true);
        void INTR_ATTR disable();
        bool INTR_ATTR isEnabled() const;

        void INTR_ATTR setAlarm(bool v = true);
        void INTR_ATTR clearAlarm();
        bool INTR_ATTR alarm() const;

        bool INTR_ATTR isActive() const;

        void registerCallback(Callback callback);
        const event_t& getLastEvent() const;
        void popEvent();
        size_t available() const;

    private:
        void _init(const uint8_t* addr);
        void _init(uint8_t familyCode, const uint8_t* addr);

        void INTR_ATTR event(const event_t& e);
        void INTR_ATTR event(event_t::type_t type, event_t::value_t value);

        void INTR_ATTR setActiveState();
        void INTR_ATTR setWaitState(uint8_t bit);

        volatile bool m_enabled;
        volatile bool m_active;
        volatile bool m_alarm;
        rom_t m_value;
        Buffer<event_t, 256> m_buffer;
        Callback m_onEvent;

    public:
        static uint8_t INTR_ATTR crc(uint8_t& _crc, bool bit);
        static uint8_t crc(uint8_t& _crc, uint8_t byte);
        static uint8_t crc(const uint8_t* data, size_t length);
    };

    enum class CMD: uint8_t {
        SEARCH              = 0xF0,
        SEARCH_ALARM        = 0xEC
    };

    OneWire(Pin input,
            Pin output,
            Pin boost = Pin(Pin::DUMMY_PIN),
            const Timing* timing = &StandardTiming,
            Callback onEvent = Callback() );

    void becomeMaster();
    void becomeSlave();

    void resetSearch();
    bool verify(const Rom::rom_t& rom);
    void searchForFamily(uint8_t family);
    void skipThisFamily();
    bool search(bool alarmOnly = false);
    const Rom::rom_t& LastFoundRom() const;

    bool reset();
    uint8_t read();
    void write(CMD cmd);
    void write(uint8_t v);
    
    const event_t& getLastEvent() const;
    void popEvent();
    size_t available() const;

    void selfAdvertising(bool enable);
    bool selfAdvertising() const;

    bool addRom(Rom* rom);

private:
    struct search_state_t {
        Rom::rom_t rom;
        uint8_t LastDiscrepancy;
        uint8_t LastFamilyDiscrepancy;
        bool LastDeviceFlag;
    };

    void _writeBit(const bool v);
    bool _readBit();

    void INTR_ATTR _slaveWriteBit(const bool v);
    void INTR_ATTR _slaveReadBit();  // reads to m_receivingByte
    void INTR_ATTR _slaveSearchSendBit(bool complement);

    void INTR_ATTR _event(event_t::type_t type, event_t::value_t value);
    void INTR_ATTR _event(const event_t&e);

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

    static void INTR_ATTR pinISR(void* args);
};
