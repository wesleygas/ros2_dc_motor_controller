#include <Arduino.h>

class SoftwareEncoder {
private:
    static const uint8_t MAX_ENCODERS = 2;
    static const int8_t STATE_TABLE[16];
    static SoftwareEncoder* instances[MAX_ENCODERS];
    static uint8_t nextIndex;

    volatile int32_t position = 0;
    volatile int8_t state = 0;
    uint8_t pinA;
    uint8_t pinB;
    uint8_t index;

    static SoftwareEncoder* instance;
    static void IRAM_ATTR isrHandler(void* arg);
    void IRAM_ATTR updateEncoder();

public:
    SoftwareEncoder(uint8_t encoderPinA, uint8_t encoderPinB);
    int32_t getCount();
    void setCount(int32_t newPosition);
};