#include "SoftwareEncoder.h"

SoftwareEncoder* SoftwareEncoder::instances[MAX_ENCODERS] = {nullptr};
uint8_t SoftwareEncoder::nextIndex = 0;

const int8_t SoftwareEncoder::STATE_TABLE[16] = {
    0, -1, 1, 0,
    1, 0, 0, -1,
    -1, 0, 0, 1,
    0, 1, -1, 0
};

void IRAM_ATTR SoftwareEncoder::isrHandler(void* arg) {
    uint8_t idx = (uint8_t)(uint32_t)arg;
    if (instances[idx]) {
        instances[idx]->updateEncoder();
    }
}

void IRAM_ATTR SoftwareEncoder::updateEncoder() {
    uint8_t currentState = (digitalRead(pinB) << 1) | digitalRead(pinA);
    uint8_t combined = (state << 2) | currentState;
    position += STATE_TABLE[combined];
    state = currentState;
}

SoftwareEncoder::SoftwareEncoder(uint8_t encoderPinA, uint8_t encoderPinB) {
    if (nextIndex >= MAX_ENCODERS) return;
    
    index = nextIndex++;
    instances[index] = this;

    pinA = encoderPinA;
    pinB = encoderPinB;
    
    state = (digitalRead(pinB) << 1) | digitalRead(pinA);
    
    attachInterruptArg(digitalPinToInterrupt(pinA), isrHandler, (void*)(uint32_t)index, CHANGE);
    attachInterruptArg(digitalPinToInterrupt(pinB), isrHandler, (void*)(uint32_t)index, CHANGE);
}

int32_t SoftwareEncoder::getCount() {
    return position;
}

void SoftwareEncoder::setCount(int32_t newPosition) {
    position = newPosition;
}