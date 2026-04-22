# ADC MIDI 2 Potentiometers

Deze notities horen bij de 2-potentiometer implementatie met ADC1 + GPDMA + USB MIDI CC.

## Pin toewijzing

- Pot 1: PA1 -> ADC1_INP1 -> CC7
- Pot 2: PB1 -> ADC1_INP5 -> CC8

## MIDI details

- Status byte: 0xB0 (MIDI kanaal 1)
- Data byte 1: controller nummer (CC)
- Data byte 2: waarde 0..127

## Config samenvatting

- ADC resolutie: 8-bit
- ADC scan: aan (2 conversies)
- Trigger: TIM6 TRGO op 1 kHz
- DMA: peripheral-to-memory, circular, destination increment aan
- Hysteresis: 2

## Test checklist

- Pot 1 en pot 2 aangesloten op 3.3V, GND, en wiper op respectievelijk PA1 en PB1
- Firmware geflasht
- MIDI monitor geopend
- Pot 1 stuurt CC7 met bereik 0..127
- Pot 2 stuurt CC8 met bereik 0..127
- Geen duidelijke jitter bij stilstaande potmeter
