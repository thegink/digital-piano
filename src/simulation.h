#ifndef SIMULATION_H
#define SIMULATION_H

#include "main.h"

typedef struct {
    uint32_t timestampMs;
    uint32_t durationMs;
    uint8_t key;
} SimulationStep_t;

#ifdef SIMULATE_KEYS

void processSimulate();
uint8_t readKey(uint8_t address);

#endif  //SIMULATE_KEYS

#endif //SIMULATION_H