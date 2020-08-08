#ifndef LOCKET_API_LED_H
#define LOCKET_API_LED_H

#include "sequences.h"

// Interface allowing to control RGB LED by using LedRGBChunk sequences.
//

class RgbLed {
public:
    virtual ~RgbLed() {};
    virtual void StartOrRestart(const LedRGBChunk* sequence) = 0;
};

#endif //LOCKET_API_LED_H
