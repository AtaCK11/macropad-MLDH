#pragma once

// TinyUSB common configuration
#define CFG_TUSB_MCU                OPT_MCU_RP2040   // RP2350 is RP2-series; Pico SDK maps this appropriately
#define CFG_TUSB_RHPORT0_MODE       (OPT_MODE_DEVICE)

#define CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_ALIGN      __attribute__ ((aligned(4)))

// Device configuration
#define CFG_TUD_ENDPOINT0_SIZE      64

// Class configuration
#define CFG_TUD_CDC                 1
#define CFG_TUD_MSC                 0
#define CFG_TUD_HID                 1
#define CFG_TUD_MIDI                0
#define CFG_TUD_VENDOR              0
#define CFG_TUD_BTH                 0
#define CFG_TUD_NET                 0

// HID
#define CFG_TUD_HID_EP_BUFSIZE      8

// HID
#define CFG_TUD_HID_EP_BUFSIZE  64

// CDC
#define CFG_TUD_CDC_RX_BUFSIZE  256
#define CFG_TUD_CDC_TX_BUFSIZE  256
#define CFG_TUD_CDC_EP_BUFSIZE  64