/*
 * USB MIDI Descriptors
 *
 * Beschrijving: Dit bestand definieert HOE het apparaat zichzelf aan de computer voorstelt.
 * Wanneer je de USB-kabel aansluit, vraagt de computer: "Wat ben jij?"
 * De descriptors zijn het antwoord: type apparaat, fabrikant, naam, USB-versie, etc.
 *
 * Volgorde van communicatie bij aansluiting:
 *   1. Computer vraagt Device Descriptor  -> wie ben je? (VID, PID, klasse)
 *   2. Computer vraagt Configuration Descriptor -> welke interfaces heb je?
 *   3. Computer vraagt String Descriptors -> naam, fabrikant als tekst
 */

#include "tusb.h"
#include <string.h>

/* Een combinatie van interfaces moet een unieke product-ID hebben, anders onthoudt Windows
 * de verkeerde driver. Als je VID/PID verandert, herkent Windows het als nieuw apparaat.
 */

// VID = Vendor ID: identificeert de fabrikant (normaal aangevraagd bij USB-IF, hier 0xCAFE voor dev)
#define USB_VID   0xCAFE  // Fabrikant-ID (0xCAFE = veelgebruikt test-VID)
// PID = Product ID: identificeert dit specifieke product van de fabrikant
#define USB_PID   0x4002  // Product-ID (veranderd van 0x4001 zodat Windows opnieuw installeert)
// BCD = Binary Coded Decimal: geeft de USB-versie aan (0x0200 = USB 2.0)
#define USB_BCD   0x0200  // USB versie 2.0

//--------------------------------------------------------------------+
// Device Descriptor (Apparaat Descriptor)
//--------------------------------------------------------------------+
// Dit is het eerste wat de computer opvraagt na aansluiting.
// Het vertelt: wat voor apparaat is dit, welke USB-versie, VID/PID, etc.
tusb_desc_device_t const desc_device =
{
    .bLength            = sizeof(tusb_desc_device_t), // Grootte van deze structuur in bytes
    .bDescriptorType    = TUSB_DESC_DEVICE,           // Type = Device Descriptor (0x01)
    .bcdUSB             = USB_BCD,                    // USB versie: 0x0200 = USB 2.0
    .bDeviceClass       = 0x00,  // Klasse in device descriptor: 0x00 = klasse per interface
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,     // Max pakketgrootte van controle-eindpunt (EP0)

    .idVendor           = USB_VID,    // Fabrikant-ID (0xCAFE)
    .idProduct          = USB_PID,    // Product-ID (0x4002)
    .bcdDevice          = 0x0100,     // Versie van dit apparaat (1.0)

    .iManufacturer      = 0x01,  // Index naar string descriptor 1 ("STMicroelectronics")
    .iProduct           = 0x02,  // Index naar string descriptor 2 ("Danae's MIDI Controller")
    .iSerialNumber      = 0x03,  // Index naar string descriptor 3 ("123456")

    .bNumConfigurations = 0x01   // Dit apparaat heeft 1 configuratie
};

// Callback: wordt aangeroepen door TinyUSB wanneer de computer de Device Descriptor opvraagt
// We geven een pointer terug naar onze descriptor hierboven
uint8_t const * tud_descriptor_device_cb(void)
{
  return (uint8_t const *) &desc_device;
}

//--------------------------------------------------------------------+
// Configuration Descriptor (Configuratie Descriptor)
//--------------------------------------------------------------------+
// De configuratie descriptor beschrijft WELKE interfaces het apparaat aanbiedt.
// Hier hebben we 2 interfaces: een MIDI Audio Control interface en een MIDI Streaming interface.
// De computer gebruikt dit om te weten hoe hij met het apparaat moet praten.

// Interface nummers (de volgorde is belangrijk voor USB)
enum
{
  ITF_NUM_MIDI = 0,           // Interface 0: MIDI Audio Control (verplicht, maar bijna leeg)
  ITF_NUM_MIDI_STREAMING,     // Interface 1: MIDI Streaming (hier gaan de echte MIDI-data over)
  ITF_NUM_TOTAL               // Totaal aantal interfaces = 2
};

// Totale lengte van de configuratie descriptor = config deel + MIDI deel
#define CONFIG_TOTAL_LEN    (TUD_CONFIG_DESC_LEN + TUD_MIDI_DESC_LEN)

// Eindpunt adressen (endpoints = communicatiekanalen op USB)
// OUT = van computer naar apparaat (computer stuurt MIDI naar ons)
// IN  = van apparaat naar computer (wij sturen MIDI naar de computer)
// Bit 7 hoog (0x80) = IN richting
#define EPNUM_MIDI_OUT   0x01  // Eindpunt 1 OUT: ontvangen van MIDI-data van de computer
#define EPNUM_MIDI_IN    0x81  // Eindpunt 1 IN:  sturen van MIDI-data naar de computer

uint8_t const desc_fs_configuration[] =
{
  // Configuratie descriptor: configuratienummer=1, 2 interfaces, geen string, totale lengte, geen speciale attributen, 100mA stroom
  TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0, 100),

  // MIDI interface descriptor: interfacenummer, geen string, eindpunt OUT, eindpunt IN, pakketgrootte 64 bytes
  TUD_MIDI_DESCRIPTOR(ITF_NUM_MIDI, 0, EPNUM_MIDI_OUT, EPNUM_MIDI_IN, 64)
};

// Callback: wordt aangeroepen door TinyUSB wanneer de computer de Configuration Descriptor opvraagt
uint8_t const * tud_descriptor_configuration_cb(uint8_t index)
{
  (void) index; // index wordt gebruikt bij meerdere configuraties, wij hebben er maar 1
  return desc_fs_configuration;
}

//--------------------------------------------------------------------+
// String Descriptors (Tekst Descriptors)
//--------------------------------------------------------------------+
// Dit zijn de leesbare teksten die de computer opvraagt voor weergave.
// Index 0 = taalcode, index 1 = fabrikant, index 2 = productnaam, etc.

// Array van pointers naar de tekst-descriptors (index 0-4)
char const* string_desc_arr [] =
{
  (const char[]) { 0x09, 0x04 }, // 0: Taalcode = Engels (0x0409 = US English)
  "STMicroelectronics",           // 1: Fabrikantnaam (zichtbaar in Apparaatbeheer)
  "Danae's MIDI Controller",      // 2: Productnaam (zichtbaar in Apparaatbeheer en DAW)
  "123456",                       // 3: Serienummer (ideaal: uniek chip-ID gebruiken)
  "TinyUSB MIDI",                 // 4: Naam van de MIDI-interface
};

// Interne buffer voor de Unicode (UTF-16) string die teruggestuurd wordt
// USB string descriptors gebruiken UTF-16 formaat, niet ASCII
static uint16_t _desc_str[32];

// Callback: wordt aangeroepen door TinyUSB wanneer de computer een String Descriptor opvraagt
// index = welke string wil de computer (0=taal, 1=fabrikant, 2=product, ...)
uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
  (void) langid;  // Taalcode negeren (we hebben maar 1 taal)

  uint8_t chr_count;

  if ( index == 0)
  {
    // Index 0 = taalcode: kopieer de 2-byte taalcode rechtstreeks
    memcpy(&_desc_str[1], string_desc_arr[0], 2);
    chr_count = 1;
  }
  else
  {
    // Controleer of de gevraagde index wel bestaat in onze array
    if ( !(index < sizeof(string_desc_arr)/sizeof(string_desc_arr[0])) ) return NULL;

    const char* str = string_desc_arr[index];  // Haal de ASCII-tekst op

    // Beperk de lengte tot maximaal 31 tekens (buffer is 32 uint16_t groot, 1 voor de header)
    chr_count = (uint8_t) strlen(str);
    if ( chr_count > 31 ) chr_count = 31;

    // Converteer ASCII naar UTF-16: elke ASCII-byte wordt een 16-bit waarde
    // (ASCII-tekens passen allemaal in 1 byte, dus hoge byte blijft 0)
    for(uint8_t i=0; i<chr_count; i++)
    {
      _desc_str[1+i] = str[i];  // ASCII byte -> uint16_t (automatisch zero-extended)
    }
  }

  // Vul de descriptor header in: byte 0 = totale lengte, byte 1 = type (String Descriptor = 0x03)
  // Lengte = 2 bytes header + 2 bytes per teken
  _desc_str[0] = (uint16_t) ((TUSB_DESC_STRING << 8 ) | (2*chr_count + 2));

  return _desc_str;  // Geef pointer naar de ingevulde buffer terug
}
