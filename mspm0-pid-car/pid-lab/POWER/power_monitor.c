/*
 * power_monitor.c - asynchronous ADC sampling for the TB6612 ADC output
 */

#include "power_monitor.h"

#include "ti_msp_dl_config.h"

#include <stdint.h>

#define POWER_ADC_FULL_SCALE       4095U
#define POWER_ADC_REFERENCE_MV     3300U

static volatile uint16_t g_latestRaw = 0U;
static volatile uint16_t g_minimumRaw = POWER_ADC_FULL_SCALE;
static volatile uint16_t g_maximumRaw = 0U;
static volatile uint32_t g_windowSamples = 0U;
static volatile uint8_t g_ready = 0U;
static volatile uint8_t g_conversionComplete = 0U;

void POWER_Init(void)
{
    g_latestRaw = 0U;
    g_minimumRaw = POWER_ADC_FULL_SCALE;
    g_maximumRaw = 0U;
    g_windowSamples = 0U;
    g_ready = 0U;
    g_conversionComplete = 0U;

    NVIC_ClearPendingIRQ(POWER_ADC_INST_INT_IRQN);
    NVIC_EnableIRQ(POWER_ADC_INST_INT_IRQN);
    DL_ADC12_startConversion(POWER_ADC_INST);
}

void POWER_Task10ms(void)
{
    /* A single conversion completes long before the next 10 ms call.  Do not
     * busy-wait in the control loop; only arm the next conversion after the
     * ISR has consumed the previous result. */
    if (g_conversionComplete != 0U) {
        g_conversionComplete = 0U;
        DL_ADC12_enableConversions(POWER_ADC_INST);
        DL_ADC12_startConversion(POWER_ADC_INST);
    }
}

void POWER_ResetWindow(void)
{
    uint16_t latest;

    /* Keep the first post-reset window internally consistent if a conversion
     * completes while the command is being handled.  Any pending conversion
     * is serviced immediately after the IRQ is re-enabled. */
    NVIC_DisableIRQ(POWER_ADC_INST_INT_IRQN);
    latest = g_latestRaw;
    g_minimumRaw = latest;
    g_maximumRaw = latest;
    g_windowSamples = 0U;
    NVIC_EnableIRQ(POWER_ADC_INST_INT_IRQN);
}

uint8_t POWER_IsReady(void)
{
    return g_ready;
}

uint16_t POWER_GetLatestRaw(void)
{
    return g_latestRaw;
}

uint16_t POWER_GetMinimumRaw(void)
{
    return g_minimumRaw;
}

uint16_t POWER_GetMaximumRaw(void)
{
    return g_maximumRaw;
}

uint32_t POWER_GetWindowSampleCount(void)
{
    return g_windowSamples;
}

uint16_t POWER_RawToPinMillivolts(uint16_t raw)
{
    return (uint16_t)((((uint32_t)raw * POWER_ADC_REFERENCE_MV) +
                       (POWER_ADC_FULL_SCALE / 2U)) /
                      POWER_ADC_FULL_SCALE);
}

void POWER_ADC_INST_IRQHandler(void)
{
    if (DL_ADC12_getPendingInterrupt(POWER_ADC_INST) ==
        DL_ADC12_IIDX_MEM0_RESULT_LOADED) {
        uint16_t raw = DL_ADC12_getMemResult(
            POWER_ADC_INST, POWER_ADC_ADCMEM_0);

        g_latestRaw = raw;
        if ((g_ready == 0U) || (raw < g_minimumRaw)) {
            g_minimumRaw = raw;
        }
        if ((g_ready == 0U) || (raw > g_maximumRaw)) {
            g_maximumRaw = raw;
        }
        g_windowSamples++;
        g_ready = 1U;
        g_conversionComplete = 1U;
    }
}
