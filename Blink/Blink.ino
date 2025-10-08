#include <Arduino.h>
#include <cstdint>

const auto interval_ms = uint32_t{500};
auto next_ms = uint32_t{0};
auto state = uint8_t{LOW};

uint8_t blink(uint8_t state) {
    digitalWrite(LED_BUILTIN, state);
    return (state == LOW) ? HIGH : LOW;
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    next_ms = millis();
}

void loop() {
    if (millis() >= next_ms) {
        state = blink(state);
        next_ms += interval_ms;
    }
}
