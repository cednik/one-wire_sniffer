#include "onewire.hpp"

#include "print.hpp"
#include "wait.hpp"

//******************************** OneWire::event_t ********************************

void OneWire::event_t::operator =(volatile event_t& e) volatile {
    time = e.time;
    type = e.type;
    value = e.value;
}

//******************************** OneWire::Rom ********************************

bool OneWire::Rom::rom_t::operator == (const rom_t& other) { return value == other.value; }

OneWire::Rom::Rom(const uint8_t* addr) {
    _init(addr);
}

OneWire::Rom::Rom(uint8_t familyCode, const uint8_t* addr) {
    _init(familyCode, addr);
}

OneWire::Rom::Rom(std::initializer_list<uint8_t> addr) {
    _init(addr.begin());
    if (addr.size() != 7) {
        ESP_LOGW("OneWire:Rom:init", "Invalid ROM address length %d. Will be filled in by zeros.", addr.size());
        for(auto i = addr.size(); i < 7; ++i)
            m_value.byte[i] = 0;
        m_value.crc = crc(m_value.byte, 7);
    }
}

OneWire::Rom::Rom(uint8_t familyCode, std::initializer_list<uint8_t> addr) {
    _init(familyCode, addr.begin());
    if (addr.size() != 6) {
        ESP_LOGW("OneWire:Rom:init", "Invalid ROM address length %d. Will be filled in by zeros.", addr.size());
        for(auto i = addr.size(); i < 7; ++i)
            m_value.addr[i] = 0;
        m_value.crc = crc(m_value.byte, 7);
    }
}

const OneWire::Rom::rom_t& OneWire::Rom::value() const { return m_value; }
const uint8_t* OneWire::Rom::begin()             const { return m_value.byte; }
const uint8_t* OneWire::Rom::end()               const { return m_value.byte+8; }
bool OneWire::Rom::getBit(uint8_t bit)           const { return m_value.byte[bit>>3] & (1<<(bit&7)); }

void OneWire::Rom::enable(bool en)   { m_enabled = en; }
void OneWire::Rom::disable()         { m_enabled = false; }
bool OneWire::Rom::isEnabled() const { return m_enabled; }

void OneWire::Rom::setAlarm(bool v) { m_alarm = v; }
void OneWire::Rom::clearAlarm()     { m_alarm = false; }
bool OneWire::Rom::alarm() const    { return m_alarm; }

bool OneWire::Rom::isActive() const { return m_enabled && m_active; }

void OneWire::Rom::registerCallback(Callback callback)     { m_onEvent = callback; }
const OneWire::event_t& OneWire::Rom::getLastEvent() const { return const_cast<const event_t&>(m_buffer.top_ref()); }
void OneWire::Rom::popEvent()                              { m_buffer.pop(); }
size_t OneWire::Rom::available()                     const { return m_buffer.size(); }

void OneWire::Rom::_init(const uint8_t* addr) {
    m_enabled = false;
    m_active = true;
    m_alarm = false;
    for (uint8_t i = 0; i !=7; ++i)
        m_value.byte[i] = addr[i];
    m_value.crc = crc(m_value.byte, 7);
}

void OneWire::Rom::_init(uint8_t familyCode, const uint8_t* addr) {
    m_enabled = false;
    m_active = true;
    m_alarm = false;
    m_value.family = familyCode;
    for (uint8_t i = 0; i !=6; ++i)
        m_value.addr[i] = addr[i];
    m_value.crc = crc(m_value.byte, 7);
}

void OneWire::Rom::event(const event_t& e) {
    m_buffer.push(e);
    m_onEvent();
}
void OneWire::Rom::event(event_t::type_t type, event_t::value_t value) {
    event_t e;
    e.time = Timer::value();
    e.type = type;
    e.value = value;
    event(e);
}

void OneWire::Rom::setActiveState() {
    m_active = true;
}

void OneWire::Rom::setWaitState(uint8_t bit) {
    m_active = false;
    event(event_t::type_t::DEACTIVATE_ROM, bit);
}

uint8_t OneWire::Rom::crc(uint8_t& _crc, bool bit) {
    const bool doXor = (_crc ^ bit) & 1;
    _crc >>= 1;
    if (doXor)
        _crc ^= 0x8C;
    return _crc;
}
uint8_t OneWire::Rom::crc(uint8_t& _crc, uint8_t byte) {
    for(uint8_t bit = 0; bit != 8; ++bit) {
        crc(_crc, bool(byte & 1));
        byte >>= 1;
    }
    return _crc;
}
uint8_t OneWire::Rom::crc(const uint8_t* data, size_t length) {
    uint8_t _crc = 0;
    for (const uint8_t* const stop = data + length; data != stop; ++data)
        crc(_crc, *data);
    return _crc;
}

//******************************** OneWire ********************************

OneWire::OneWire(
    Pin input,
    Pin output,
    Pin boost,
    const Timing* timing,
    Callback onEvent )
    : m_input(input),
      m_output(output),
      m_boost(boost),
      m_timing(timing),
      m_lock(portMUX_INITIALIZER_UNLOCKED),
      m_searchState(),
      m_state(State::MASTER),
      m_slaveSearchState(SlaveSearchState::IDLE),
      m_processBit(nullptr),
      m_bitIndex(0),
      m_transferedByte(0),
      m_edgeTime(0),
      m_selfAdvertising(false),
      m_slaveSearchRom(),
      m_romsCount(0),
      m_rom(),
      m_buffer(),
      m_onEvent(onEvent)
{
    m_input.setHigh();
    m_output.setHigh();
    m_boost.setLow();
    for (roms_count_t i = 0; i != MAX_ROMS; ++i)
        m_rom[i] = nullptr;
    becomeSlave();
    m_input.attachInterrupt(pinISR, this, FALLING);
}

void OneWire::becomeMaster() {
    if (m_state == State::MASTER)
        return;
    m_input.detachInterrupt();
    m_buffer.clear();
    m_state = State::MASTER;
    m_boost.setHigh();
}

void OneWire::becomeSlave() {
    if (m_state != State::MASTER)
        return;
    m_transferedByte = 0;
    m_bitIndex = 0;
    m_state = State::READ_START;
    m_slaveSearchState = SlaveSearchState::IDLE;
    m_processBit = &OneWire::_slaveReadBit;
}

void OneWire::resetSearch() {
    m_searchState.rom.value = 0;
    m_searchState.LastDiscrepancy = 0;
    m_searchState.LastFamilyDiscrepancy = 0;
    m_searchState.LastDeviceFlag = false;
}

bool OneWire::verify(const Rom::rom_t& rom) {
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

void OneWire::searchForFamily(uint8_t family) {
    m_searchState.rom.value = family;
    m_searchState.LastDiscrepancy = 64;
    m_searchState.LastFamilyDiscrepancy = 0;
    m_searchState.LastDeviceFlag = false;
}

void OneWire::skipThisFamily() {
    m_searchState.LastDiscrepancy = m_searchState.LastFamilyDiscrepancy;
    m_searchState.LastFamilyDiscrepancy = 0;

    // check for end of list
    if (m_searchState.LastDiscrepancy == 0)
        m_searchState.LastDeviceFlag = true;
}

bool OneWire::search(bool alarmOnly) {
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
                print("    Search: both read bits {} are 1 - no devices\n", id_bit_number - 1);
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

const OneWire::Rom::rom_t& OneWire::LastFoundRom() const { return m_searchState.rom; }

bool OneWire::reset() {
    portENTER_CRITICAL(&m_lock);
    becomeMaster();
    wait<Timer>(m_timing->G);
    m_boost.setLow();
    m_output.setLow();
    wait<Timer>(m_timing->H);
    m_output.setHigh();
    m_boost.setHigh();
    wait<Timer>(m_timing->I);
    const bool v = m_input.read();
    m_boost.setLow();
    wait<Timer>(m_timing->J);
    m_boost.setHigh();
    portEXIT_CRITICAL(&m_lock);
    return !v;
}

uint8_t OneWire::read() {
    uint8_t v = 0;
    if (m_state == State::MASTER) {
        for(uint8_t i = 0; i != 8; ++i) {
            v |= _readBit();
            v <<= 1;
        }
    }
    return v;
}

void OneWire::write(CMD cmd) { write(uint8_t(cmd)); }

void OneWire::write(uint8_t v) {
    for (uint8_t i = 1; i != 0; i<<=1)
        _writeBit(v & i);
}

const OneWire::event_t& OneWire::getLastEvent() const { return const_cast<const event_t&>(m_buffer.top_ref()); }

void OneWire::popEvent() { m_buffer.pop(); }

size_t OneWire::available() const { return m_buffer.size(); }

void OneWire::selfAdvertising(bool enable) { m_selfAdvertising = enable; }

bool OneWire::selfAdvertising() const { return m_selfAdvertising; }

bool OneWire::addRom(Rom* rom) {
    if (m_romsCount == MAX_ROMS)
        return false;
    m_rom[m_romsCount++] = rom;
    return true;
}

void OneWire::_writeBit(const bool v) {
    const Timer::value_t pre  = v ? m_timing->A : m_timing->C;
    const Timer::value_t post = v ? m_timing->B : m_timing->D;
    portENTER_CRITICAL(&m_lock);
    m_boost.setLow();
    m_output.setLow();
    wait<Timer>(pre);
    m_output.setHigh();
    m_boost.setHigh();
    wait<Timer>(post);
    portEXIT_CRITICAL(&m_lock);
}

bool OneWire::_readBit() {
    portENTER_CRITICAL(&m_lock);
    m_boost.setLow();
    m_output.setLow();
    wait<Timer>(m_timing->A);
    m_output.setHigh();
    m_boost.setHigh();
    wait<Timer>(m_timing->E);
    const bool v = m_input.read();
    m_boost.setLow();
    wait<Timer>(m_timing->F);
    m_boost.setHigh();
    portEXIT_CRITICAL(&m_lock);
    return v;
}

void OneWire::_slaveWriteBit(const bool v) {
}

void OneWire::_slaveReadBit(const bool v) {
    m_transferedByte >>= 1;
    m_transferedByte |= 0x80 * v;
    event_t e = {
        .time = m_edgeTime,
        .type = event_t::type_t::RECEIVED_BIT,
        .value = event_t::value_t(m_bitIndex | (m_transferedByte & 0x80))
    };
    _event(e);
    if (++m_bitIndex == 8) {
        e.type = event_t::type_t::RECEIVED;
        e.value = m_transferedByte;
        _event(e);
        m_bitIndex = 0;
        if (m_slaveSearchState == SlaveSearchState::READY) {
            if (m_transferedByte == uint8_t(CMD::SEARCH)
             || m_transferedByte == uint8_t(CMD::SEARCH_ALARM)) {
                m_transferedByte = 0x03;
                for (roms_count_t i = 0; i != m_romsCount; ++i) {
                    if (m_rom[i]->isActive())
                        m_rom[i]->event(e);
                    if (m_rom[i]->isActive()) {
                        if (m_rom[i]->getBit(0))
                            m_transferedByte &= ~0x02;
                        else
                            m_transferedByte &= ~0x01;
                    }
                }
                e.type = event_t::type_t::SEARCH_START;
                _event(e);
                m_slaveSearchRom.value = 0;
                m_processBit = &OneWire::_slaveSearchWriteBit;
                m_state = State::WRITE;
                return;
            }
            m_slaveSearchState = SlaveSearchState::IDLE;
        }
        for (roms_count_t i = 0; i != m_romsCount; ++i) {
            if (m_rom[i]->isActive())
                m_rom[i]->event(e);
        }
        m_transferedByte = 0;
    }
    m_state = State::READ_START;
}

void OneWire::_slaveReset() {
    m_transferedByte = 0;
    m_bitIndex = 0;
    m_processBit = &OneWire::_slaveReadBit;
    m_slaveSearchState = SlaveSearchState::READY;
    m_state = State::READ_START;
    event_t e = {
        .time = m_edgeTime,
        .type = event_t::type_t::RESET,
        .value = 0
    };
    bool isAnyRomEnabled = false;
    for (roms_count_t i = 0; i != m_romsCount; ++i) {
        if (m_rom[i]->isEnabled()) {
            isAnyRomEnabled = true;
            m_rom[i]->event(e);
            m_rom[i]->setActiveState();
        }
    }
    const Timer::value_t dt = Timer::value() - m_edgeTime;
    if (m_selfAdvertising && isAnyRomEnabled) {
        if (m_timing->M > dt)
            wait<Timer>(m_timing->M - dt);
        m_output.setLow();
        wait<Timer>(m_timing->N);
        m_output.setHigh();
        e.value = 1;
    } else {
        if (m_timing->I > dt)
            wait<Timer>(m_timing->I - dt);
        if (!m_input.read()) {
            e.value = 1;
            for (roms_count_t i = 0; i != m_romsCount; ++i) {
                if (m_rom[i]->isEnabled()) {
                    m_rom[i]->m_buffer.last_ref().value = 1;
                }
            }
        }
    }
    _event(e);
    m_input.clearInterruptFlag();
}

void OneWire::_slaveSearchWriteBit(const bool v) {
    m_transferedByte >>= 1;
    m_processBit = &OneWire::_slaveSearchWriteComplementBit;
    _event(event_t::type_t::SEARCH_SEND_BIT, m_bitIndex | (v<<7));
}

void OneWire::_slaveSearchWriteComplementBit(const bool v) {
    m_processBit = &OneWire::_slaveSearchReadBit;
    m_state = State::READ_START;
    _event(event_t::type_t::SEARCH_SEND_COMPLEMENT, m_bitIndex | (v<<7));
}

void OneWire::_slaveSearchReadBit(const bool v) {
    event_t e = {
        .time = m_edgeTime,
        .type = event_t::type_t::SEARCH_READ_BIT,
        .value = event_t::value_t(m_bitIndex | (v<<7))
    };
    _event(e);
    m_slaveSearchRom.value |= uint64_t(v)<<m_bitIndex;
    m_transferedByte = 0x03;
    for (roms_count_t i = 0; i != m_romsCount; ++i) {
        if (m_rom[i]->isActive()) {
            if (m_rom[i]->getBit(m_bitIndex) != v) {
                m_rom[i]->setWaitState(m_bitIndex);
            } else {
                if (m_rom[i]->getBit(m_bitIndex + 1))
                    m_transferedByte &= ~0x02;
                else
                    m_transferedByte &= ~0x01;
            }
        }
    }
    if (++m_bitIndex == 64) {
        e.type = event_t::type_t::SEARCH_COMPLETE;
        e.value = m_slaveSearchRom.value;
        _event(e);
        m_transferedByte = 0;
        m_bitIndex = 0;
        m_processBit = &OneWire::_slaveReadBit;
        m_slaveSearchState = SlaveSearchState::IDLE;
        m_state = State::READ_START;
    } else {
        m_state = State::WRITE;
        m_processBit = &OneWire::_slaveSearchWriteBit;
    }
}

void OneWire::_event(event_t::type_t type, event_t::value_t value) {
    event_t e;
    e.time = m_edgeTime;
    e.type = type;
    e.value = value;
    _event(e);
}

void OneWire::_event(const event_t&e) {
    m_buffer.push(e);
    m_onEvent();
}

void OneWire::pinISR(void* args) {
    reinterpret_cast<OneWire*>(args)->_pinISR();
}

void OneWire::_slaveProcessBit(const bool v) {
    (this->*m_processBit)(v);
}

void OneWire::_pinISR() {
    m_boost.setHigh();
    m_edgeTime = Timer::value();
    //_event(event_t::type_t::STATE_CHECK, uint8_t(m_state));
    switch (m_state) {
    case State::MASTER:
        becomeSlave();
        _event(event_t::type_t::GET_SLAVED, 0);
        [[fallthrough]];
    case State::READ_START:
        m_input.interruptTrigger(RISING);
        m_state = State::READ_FINISH;
        break;
    case State::READ_FINISH:
        m_input.interruptTrigger(FALLING);
        {
            const Timer::value_t dt = m_edgeTime - m_lastEdgeTime;
            if (dt < m_timing->K) {
                _slaveProcessBit(1);
            } else if (dt < m_timing->L) {
                _slaveProcessBit(0);
            } else {
                _slaveReset();
            }
        }
        break;
    case State::WRITE:
        if ((m_transferedByte & 0x01) == 0) {
            m_output.setLow();
            wait<Timer>(m_timing->O);
            m_output.setHigh();
            m_input.clearInterruptFlag();
            _slaveProcessBit(0);
        } else {
            _slaveProcessBit(1);
        }
        break;
    }
    m_lastEdgeTime = m_edgeTime;
    m_boost.setLow();
}

/*void OneWire::_pinISR() {
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
            m_transferedByte = 0;
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
                if (m_rom[i]->isActive())
                    m_rom[i]->event(e);
            }
            _event(e);
            m_state = State::READ;
        } else {
            if (m_slaveSearchState == SlaveSearchState::IDLE
             || m_slaveSearchState == SlaveSearchState::READY) {
                _event(event_t::type_t::RECEIVED_BIT, m_bitIndex);
                m_state = State::READ;
                _slaveProcessReceivedByte();
            } else {
                m_state = State::SEARCH;
            }
        }
        m_input.interruptTrigger(FALLING);
        m_input.clearInterruptFlag();
        break;
    case State::READ:
        if (_slaveReadBit()) {
            _event(event_t::type_t::RECEIVED_BIT, m_bitIndex | 0x80);
            _slaveProcessReceivedByte();
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
            _slaveSearchSendBit(0);
            m_slaveSearchState = SlaveSearchState::SEND_COMPLEMENT;
            break;
        case SlaveSearchState::SEND_COMPLEMENT:
            _slaveSearchSendBit(1);
            m_slaveSearchState = SlaveSearchState::READ_BIT;
            break;
        case SlaveSearchState::READ_BIT:
            m_transferedByte = 0;
            _slaveReadBit(); // read value is in m_transferedByte
            _event(event_t::type_t::SEARCH_READ_BIT, m_bitIndex | m_transferedByte);
            for (roms_count_t i = 0; i != MAX_ROMS; ++i) {
                if (m_rom[i] == nullptr)
                    break;
                if (m_rom[i]->isActive() && (bool(m_transferedByte) != m_rom[i]->getBit(m_bitIndex))) {
                    m_rom[i]->setWaitState(m_bitIndex+1);
                }
            }
            if (++m_bitIndex == 64) {
                if (m_transferedByte != 0) {
                    m_transferedByte = 0;
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
}*/

#define US2TICKS(us) OneWire::Timing::time_t((us) * 240)
// based on https://www.maximintegrated.com/en/design/technical-documents/app-notes/1/126.html
//     plus slave timing
//     - ESP32 processing offset
const DRAM_ATTR OneWire::Timing OneWire::StandardTiming = {
    .A = US2TICKS(  6), // master write zero first
    .B = US2TICKS( 64), // master write zero second
    .C = US2TICKS( 60), // master write one first
    .D = US2TICKS( 10), // master write one second
    .E = US2TICKS(  9), // master read sample
    .F = US2TICKS( 55), // master read pause
    .G = US2TICKS(  0), // master reset before
    .H = US2TICKS(480), // master reset low
    .I = US2TICKS( 70), // master reset sample
    .J = US2TICKS(410), // master reset pause
    .K = US2TICKS( 35), // slave read one threshold
    .L = US2TICKS(100), // slave reset threshold
    .M = US2TICKS( 35), // slave presence responce delay
    .N = US2TICKS( 70), // slave presence release
    .O = US2TICKS( 30)  // slave write zero
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
