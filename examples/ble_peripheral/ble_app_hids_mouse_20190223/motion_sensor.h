#include <stdint.h>
#include "boards.h"

bool getMotion(int16_t *x, int16_t *y, int8_t maxReads);
void usrPaw3205ReSynchronousSPI(void);
void writeReg(uint8_t regAddress, int8_t val);
int8_t readReg(uint8_t regAddress);
void turnOn(void);
bool verifyProductId(void);
void usrPaw3205setCPI(uint8_t CPIValue);
void enterOrExitSleepmode(uint8_t operation);
void enterPowerDownMode(uint8_t operation);




