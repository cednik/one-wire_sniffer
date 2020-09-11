#pragma once

#include "callback.hpp"

void checkReset();

void INTR_ATTR notifyTaskFromISR(Callback::callback_arg_t args);
