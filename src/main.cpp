#include <Arduino.h>

// Determine which code to compile based on build flags
#if defined(BUILD_MASTER)
    #include "hub_master.h"
#elif defined(BUILD_SLOT)
    #include "function.h"
    // Forward declarations for slot functionality that will be implemented later
    class Sensors;
    class Acquire;
    class ShiftRegister;
    class Logging;
#else
    #error "Please define either BUILD_MASTER or BUILD_SLOT in build flags"
#endif

void setup() {
#if defined(BUILD_MASTER)
    setup_master();
#elif defined(BUILD_SLOT)
    setup_slot();
#endif
}

void loop() {
#if defined(BUILD_MASTER)
    loop_master();
#elif defined(BUILD_SLOT)
    loop_slot();
#endif
}

#if defined(BUILD_SLOT)
void setup1() {
    setup1_slot();
}

void loop1() {
    loop1_slot();
}
#endif