#include "adc_midi.h"

#include "main.h"
#include "tusb.h"

extern ADC_HandleTypeDef hadc1;

/* We read 2 potentiometers in a round-robin ADC sequence. */
#define NUM_POTS      2U
/* Ignore tiny ADC fluctuations to avoid MIDI jitter/noise. */
#define HYSTERESIS    2U

/* DMA continuously updates latest ADC samples for each pot (8-bit mode). */
static volatile uint8_t adc_buffer[NUM_POTS];
/* Last value sent over MIDI per potentiometer. */
static uint8_t last_midi_value[NUM_POTS];
/* Pot 0 -> CC7 (Volume), Pot 1 -> CC8 (Balance by convention). */
static const uint8_t midi_cc_map[NUM_POTS] = {7U, 8U};

static void TIM6_Trigger_1kHz_Start(void)
{
    uint32_t pclk1_hz;
    uint32_t tim_clk_hz;
    uint32_t psc;

    __HAL_RCC_TIM6_CLK_ENABLE();

    pclk1_hz = HAL_RCC_GetPCLK1Freq();

    if ((RCC->CFGR2 & RCC_CFGR2_PPRE1) != RCC_HCLK_DIV1)
    {
        /* On STM32, timer clock is doubled when APB prescaler is not DIV1. */
        tim_clk_hz = pclk1_hz * 2U;
    }
    else
    {
        tim_clk_hz = pclk1_hz;
    }

    /* Target timer tick at 1 MHz, then ARR = 999 gives 1 kHz update/TRGO. */
    psc = (tim_clk_hz / 1000000U);
    if (psc > 0U)
    {
        psc -= 1U;
    }

    TIM6->CR1 = 0U;
    /* Prescaler gives 1 MHz timer tick (1 tick = 1 us). */
    TIM6->PSC = (uint16_t)psc;
    /* 1000 ticks -> 1 ms period -> 1 kHz trigger rate. */
    TIM6->ARR = 999U;
    TIM6->EGR = TIM_EGR_UG;

    /* TRGO on update event: each timer period triggers one ADC sequence. */
    TIM6->CR2 &= ~TIM_CR2_MMS;
    TIM6->CR2 |= TIM_CR2_MMS_1;

    TIM6->CR1 |= TIM_CR1_CEN;
}

void ADC_MIDI_Init(void)
{
    /* Ensure a clean start before enabling DMA again. */
    HAL_ADC_Stop_DMA(&hadc1);

    /* Timer generates periodic ADC trigger events. */
    TIM6_Trigger_1kHz_Start();

    /* Start ADC in DMA mode: adc_buffer gets updated in background. */
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, NUM_POTS) != HAL_OK)
    {
        Error_Handler();
    }
}

void ADC_MIDI_Process(void)
{
    uint8_t i;

    for (i = 0U; i < NUM_POTS; i++)
    {
        uint8_t new_value;
        int16_t diff;

        /* ADC is configured at 8-bit; shift maps 0..255 to MIDI-friendly 0..127. */
        new_value = adc_buffer[i] >> 1;
        diff = (int16_t)new_value - (int16_t)last_midi_value[i];

        if (diff < 0)
        {
            diff = -diff;
        }

        /* Send only when movement is meaningful (hysteresis filter). */
        if ((uint16_t)diff >= HYSTERESIS)
        {
            if (tud_midi_mounted())
            {
                /* MIDI CC message: status, controller number, controller value. */
                uint8_t msg[3] = {0xB0U, midi_cc_map[i], new_value};
                tud_midi_stream_write(0U, msg, 3U);
            }

            /* Update reference even if host is disconnected, to avoid jumps later. */
            last_midi_value[i] = new_value;
        }
    }
}
