#define LOG_LOCAL_LEVEL 5

#include "print.hpp"
#include "test_queue.hpp"
#include "onewire.hpp"
#include "util.hpp"

static constexpr Pin::pin_num_t ONEWIRE_MASTER_PIN = 22;
static constexpr Pin::pin_num_t ONEWIRE_SLAVE_PIN  = 23;
static constexpr Pin::pin_num_t OW_SLAVE_TEST_PIN  = 21;
static constexpr Pin::pin_num_t OW_MASTER_TEST_PIN = 18;
static constexpr Pin::pin_num_t ISR_TEST_PIN       = 19;

Pin isrOut(ISR_TEST_PIN, OUTPUT);

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

[[maybe_unused]]
static void oneWireMaster(void* args = nullptr) {
    const Pin::pin_num_t onewirePin = ONEWIRE_MASTER_PIN;
    OneWire ow(
        Pin(onewirePin, OUTPUT_OPEN_DRAIN | PULLUP, HIGH),
        Pin(onewirePin),
        Pin(OW_MASTER_TEST_PIN, OUTPUT) );
    ow.becomeMaster();
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

[[maybe_unused]]
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