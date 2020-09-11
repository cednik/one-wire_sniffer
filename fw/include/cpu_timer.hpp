#pragma once

class CPUTimer {
public:
    typedef uint32_t value_t;
    inline static value_t INTR_ATTR value() { return xthal_get_ccount(); }
};
