/* main.c — TC66 custom firmware, Milestone 0.
 *
 * Brings up the 48 MHz clock + USB CDC-ACM (usb.c) and runs a polled main loop
 * that services USB and dispatches the bare-5-byte serial commands:
 *   query -> "firm"      (proves the app is running; host tools gate on this)
 *   getva -> 192-B static AES packet (so tc66.py --info connects end to end)
 *   updat -> copy the SRAM streaming flasher to 0x20001400 and jump to it
 *            (app-side self-update; no K1). Mirrors firmware/updater/stub.s.
 *   rotat/nextp/lastp -> silent no-ops (no display in M0)
 *
 * Interrupts stay masked the whole time (startup.s does cpsid i); USB is polled.
 */
#include <stdint.h>
#include "usb.h"

extern const uint8_t *usb_getva_reply(void);

/* Flasher blob (flasher.s, linked at 0x20001400) embedded via flasher_blob.s */
extern const uint8_t  _flasher_blob_start[];
extern const uint8_t  _flasher_blob_end[];
#define FLASHER_SRAM  ((void *)0x20001400u)

static void launch_flasher(void){
    MK_STAGE = STAGE_UPDAT;
    /* copy the position-fixed flasher into SRAM */
    volatile uint8_t *d = (volatile uint8_t *)FLASHER_SRAM;
    const uint8_t *s = _flasher_blob_start;
    while (s < _flasher_blob_end) *d++ = *s++;
    __asm volatile("dsb; isb");
    /* jump (thumb). The flasher sets its own SP, arms EP3, streams, resets. */
    void (*entry)(void) = (void (*)(void))((uint32_t)FLASHER_SRAM | 1u);
    __asm volatile("cpsid i");
    entry();
    for (;;) { }   /* not reached */
}

static int match5(const uint8_t *b, uint32_t len, const char *cmd){
    if (len < 5) return 0;
    for (int i = 0; i < 5; i++) if (b[i] != (uint8_t)cmd[i]) return 0;
    return 1;
}

/* Called by usb.c when a CDC OUT packet arrives. Return 1 iff it was 'updat'. */
int cdc_on_packet(const uint8_t *buf, uint32_t len){
    if (match5(buf, len, "query")){
        static const uint8_t firm[4] = { 'f','i','r','m' };
        usb_cdc_send(firm, 4);
        MK_STAGE = STAGE_QUERY;
        return 0;
    }
    if (match5(buf, len, "getva")){
        const uint8_t *r = usb_getva_reply();
        usb_cdc_send(r,       64);   /* pac1 */
        usb_cdc_send(r + 64,  64);   /* pac2 */
        usb_cdc_send(r + 128, 64);   /* pac3 */
        return 0;
    }
    if (match5(buf, len, "updat")){
        launch_flasher();            /* does not return */
        return 1;
    }
    /* rotat / nextp / lastp / unknown -> silent (side-effect-only verbs reply nothing) */
    return 0;
}

void main(void){
    MK_BOOT = 0xB007B007u;
    usb_init();
    for (;;){
        usb_poll();
    }
}
