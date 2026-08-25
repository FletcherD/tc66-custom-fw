/* usb.h — minimal polled USB CDC-ACM for the GD32F150 (STM32F0-style USB_FS device).
 * Public interface used by main.c. Enumeration + CDC data path run entirely by
 * polling usb_poll() from the main loop (no USB IRQ, IRQs stay masked). */
#ifndef USB_H
#define USB_H
#include <stdint.h>

/* Progress markers — read over SWD with: openocd ... mdw 0x20001000 <n> (NO halt). */
#define MK_BOOT     (*(volatile uint32_t *)0x20001000)  /* 0xB007B007 once main() runs   */
#define MK_STAGE    (*(volatile uint32_t *)0x20001004)  /* furthest milestone (see below) */
/* 0x20001008..0x20001017 reserved for startup.s HardFault capture */
#define MK_HEART    (*(volatile uint32_t *)0x20001020)  /* usb_poll() heartbeat           */
#define MK_CTR      (*(volatile uint32_t *)0x20001024)  /* CTR (transfer-complete) count  */
#define MK_RESET    (*(volatile uint32_t *)0x20001028)  /* USB bus-reset count            */
#define MK_SETUP    (*(volatile uint32_t *)0x2000102c)  /* last SETUP: bmReqType<<8|bReq  */
#define MK_CMD      (*(volatile uint32_t *)0x20001030)  /* last CDC command (first 4 B)   */
#define MK_ENUM     (*(volatile uint32_t *)0x20001034)  /* assigned USB address (EF set)  */
#define MK_CMDCNT   (*(volatile uint32_t *)0x20001038)  /* CDC commands dispatched        */
#define MK_PROBE    (*(volatile uint32_t *)0x2000103c)  /* last code point reached (freeze locator) */
#define MK_ISTR     (*(volatile uint32_t *)0x20001040)  /* last ISTR value observed       */
#define MK_RA       (*(volatile uint32_t *)0x20001044)  /* usb_poll return address (corruption detector) */
#define MK_SPEXIT   (*(volatile uint32_t *)0x20001048)  /* SP at usb_poll exit            */

/* MK_STAGE milestone codes */
#define STAGE_CLOCK      1   /* 48 MHz clock + USB clock configured */
#define STAGE_USB_INIT   2   /* USB peripheral powered + connected  */
#define STAGE_RESET      3   /* first USB bus reset serviced        */
#define STAGE_ADDRESS    4   /* SET_ADDRESS handled                 */
#define STAGE_CONFIG     5   /* SET_CONFIGURATION handled (enumerated) */
#define STAGE_CMD        6   /* first CDC command received          */
#define STAGE_QUERY      7   /* answered query->firm                */
#define STAGE_UPDAT      8   /* updat received, launching flasher   */

void usb_init(void);
void usb_poll(void);

/* CDC command hook: main.c implements this. Called with the 5-byte-matched RX
 * packet (buf,len). Return 1 if it was the 'updat' command (caller must NOT
 * re-arm EP3 and should launch the flasher), else 0. */
int  cdc_on_packet(const uint8_t *buf, uint32_t len);

/* Send up to 64 bytes on the CDC IN endpoint (EP1). Blocks until the previous
 * IN packet has been fetched by the host. */
void usb_cdc_send(const uint8_t *buf, uint32_t len);

#endif
