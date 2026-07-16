/*
 * power_monitor.h - TB6612 module ADC monitor on PB19 / ADC1 channel 6
 *
 * The carrier exposes an ADC signal, but its external divider ratio is not
 * encoded in the schematic.  The firmware therefore reports both the raw
 * 12-bit code and the voltage present at the MCU pin.  It deliberately does
 * not label that pin voltage as battery voltage.
 */

#ifndef POWER_MONITOR_H
#define POWER_MONITOR_H

#include <stdint.h>

void POWER_Init(void);
void POWER_Task10ms(void);
void POWER_ResetWindow(void);

uint8_t POWER_IsReady(void);
uint16_t POWER_GetLatestRaw(void);
uint16_t POWER_GetMinimumRaw(void);
uint16_t POWER_GetMaximumRaw(void);
uint32_t POWER_GetWindowSampleCount(void);
uint16_t POWER_RawToPinMillivolts(uint16_t raw);

#endif
