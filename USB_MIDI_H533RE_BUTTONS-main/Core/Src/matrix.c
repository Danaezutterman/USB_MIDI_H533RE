#include "matrix.h"
#include "mcp23s17.h"
#include "tusb.h"

extern SPI_HandleTypeDef hspi2;

/* Column drive values: pull one column LOW, rest HIGH */
static const uint8_t COL_MASK[4] = { 0x0E, 0x0D, 0x0B, 0x07 };

/* Previous row state per column (0x0F = all released) */
static uint8_t prev_state[4] = { 0x0F, 0x0F, 0x0F, 0x0F };

/* ~1 µs delay using a simple spin loop (96 MHz SYSCLK, ~4 cycles/iteration) */
static void delay_us(uint32_t us)
{
    volatile uint32_t count = us * 24U;
    while (count--) {}
}

static void send_note_event(uint8_t col, uint8_t row, uint8_t pressed)
{
    /* Flatten 4x4 matrix coordinate to button index 0..15. */
    uint8_t button = col * 4U + row;
    /* Map buttons to MIDI notes C2..D#3 (36..51). */
    uint8_t note = 36U + button;   /* C2 = 36, range 36-51 */

    /* Only transmit when the USB host has enumerated the MIDI interface. */
    if (tud_midi_mounted())
    {
        uint8_t cable = 0;
        /* MIDI channel 1 in human terms (0 in protocol encoding). */
        uint8_t ch = 0;
        if (pressed)
        {
            /* Note On with fixed velocity. */
            uint8_t msg[3] = { 0x90U | ch, note, 100U };
            tud_midi_stream_write(cable, msg, 3);
        }
        else
        {
            /* Note Off for the same note. */
            uint8_t msg[3] = { 0x80U | ch, note, 0U };
            tud_midi_stream_write(cable, msg, 3);
        }
    }
}

void Matrix_Init(void)
{
    /* Configure MCP23S17 directions and pull-ups once at startup. */
    MCP23S17_Init(&hspi2);
}

void Matrix_Scan(void)
{
    /* Scan one column at a time to detect pressed intersections. */
    for (uint8_t col = 0; col < 4; col++)
    {
        /* Drive one column LOW */
        MCP23S17_WriteReg(&hspi2, MCP_GPIOA, COL_MASK[col]);
        delay_us(1);

        /* Read rows from GPIOB — LOW bit = pressed (pull-up active) */
        uint8_t rows = MCP23S17_ReadReg(&hspi2, MCP_GPIOB) & 0x0F;

        /* Detect changes */
        uint8_t changed = rows ^ prev_state[col];
        if (changed)
        {
            /* Only changed row bits generate MIDI events. */
            for (uint8_t row = 0; row < 4; row++)
            {
                if (changed & (1U << row))
                {
                    uint8_t pressed = !(rows & (1U << row));
                    send_note_event(col, row, pressed);
                }
            }
            prev_state[col] = rows;
        }
    }

    /* Release all columns between scans to minimize ghost effects. */
    MCP23S17_WriteReg(&hspi2, MCP_GPIOA, 0x0F);
}
