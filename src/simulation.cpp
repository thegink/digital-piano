#include "simulation.h"
#include "main.h"
#include "KeyTable.h"

#ifdef SIMULATE_KEYS

uint8_t simulationS[21];
uint32_t lastStepTime;
uint32_t lastTimestamp;
uint32_t startTime;

static const SimulationStep_t SimulationSteps[] =
{
    {1,    1000, 40},
    {1000, 1000, 42},
    {2000, 1000, 44},
    {3000, 1000, 45},
    {4000, 1000, 47},
    {5000, 1000, 49},
    {6000, 1000, 51},
    {7000, 1000, 52},
    {8000, 1000, 51},
    {9000, 1000, 49},
    {10000, 1000, 47},
    {11000, 1000, 45},
    {12000, 1000, 44},
    {13000, 1000, 42},
    {14000, 1000, 40}
};

void processSimulate()
{
    uint32_t now = millis();
    if (now != lastStepTime)      //only update every ms
    {
        uint32_t thisTimestamp = (now - startTime);
        bool loopFinished = false;

        for (uint32_t timestamp = lastTimestamp+1; timestamp <= thisTimestamp; timestamp++)
        {
            //we only care about the last time
            loopFinished = true;
            
            for (uint32_t i = 0; i < COUNTOF(SimulationSteps); i++)
            {
                if (timestamp == SimulationSteps[i].timestampMs)            //half press
                {
                    //update the current state!
                    uint8_t a = KeyTable[SimulationSteps[i].key].address;
                    uint8_t s = 1<< KeyTable[SimulationSteps[i].key].s1;
                    simulationS[a] |= s;
                }

                if (timestamp == SimulationSteps[i].timestampMs + 100)      //half-press -> full press
                {
                    uint8_t a = KeyTable[SimulationSteps[i].key].address;
                    uint8_t s = 1 << KeyTable[SimulationSteps[i].key].s2;
                    simulationS[a] |= s;
                }

                if (timestamp == SimulationSteps[i].timestampMs + 
                            SimulationSteps[i].durationMs)                  //release
                {
                    uint8_t a = KeyTable[SimulationSteps[i].key].address;
                    uint8_t s = (1 << KeyTable[SimulationSteps[i].key].s1) |
                                (1 << KeyTable[SimulationSteps[i].key].s2);
                    simulationS[a] &= ~s;
                    
                    loopFinished = false;
                }
                else if (timestamp < SimulationSteps[i].timestampMs + 
                            SimulationSteps[i].durationMs)
                {
                    loopFinished = false;
                }
            }
        }

        lastTimestamp = thisTimestamp;
        lastStepTime = now;

        if (loopFinished)
        {
            startTime = now;
        }
    }
}



uint8_t readKey(uint8_t address)
{
    if (address <= (sizeof(simulationS)/sizeof(simulationS[0])))
        return simulationS[address];

    return 0xFF;
}

#endif