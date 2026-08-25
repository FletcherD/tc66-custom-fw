/* usb.c — polled USB CDC-ACM device for the GD32F150G8 (TC66 custom firmware M0).
 *
 * The GD32F1x0 USBD is the STM32F0/F1-style "USB_FS device" IP: EPnR/CNTR/ISTR/
 * DADDR/BTABLE at 0x40005C00 and a 16-bit packet buffer memory (PMA) at 0x40006000
 * that is *doubled-addressed* (each 16-bit PMA slot occupies 4 CPU-address bytes).
 *
 * Everything host-visible matches the meter's own USB identity so the host
 * binds it as the same device (same VID/PID/strings/endpoints):
 *   device/config/string descriptors  (the meter's USB identity)
 *   EP map: EP1 IN bulk(0x81), EP2 IN interrupt(0x82), EP3 OUT bulk(0x03)
 *   PMA:  EP1 TX buf @0xC0, EP3 RX buf @0x100  (the app-update SRAM stub depends on
 *         exactly these; see firmware/updater/stub.s)
 * Clock/USB power-up sequence for this board:
 *   RCU_CFG0[23:22]=01 (USBDPSC=CK_PLL/1=48MHz), RCU_APB1EN|=(1<<23) (USBD clock),
 *   CNTR=FRES then 0, ISTR=0, BTABLE=0.
 */
#include "usb.h"

/* ---------------- register map ---------------- */
#define RCU_BASE     0x40021000u
#define RCU_CTL0     (*(volatile uint32_t *)(RCU_BASE + 0x00))
#define RCU_CFG0     (*(volatile uint32_t *)(RCU_BASE + 0x04))
#define RCU_APB1EN   (*(volatile uint32_t *)(RCU_BASE + 0x1C))

#define RCU_AHBEN    (*(volatile uint32_t *)(RCU_BASE + 0x14))

/* GPIOA — PA9 is the USB D+ soft-connect (PA9 --resistor--> PA12/USB_DP). Driving
 * PA9 low pulls D+ low (host sees a disconnect); high presents the FS pull-up so the
 * host enumerates. Hold PA9 low through init, then raise it
 * after usb bring-up (main @0x0800b1f2 clear / @0x0800bb0c set). */
#define GPIOA_BASE   0x48000000u
#define GPIOA_MODER  (*(volatile uint32_t *)(GPIOA_BASE + 0x00))
#define GPIOA_OTYPER (*(volatile uint32_t *)(GPIOA_BASE + 0x04))
#define GPIOA_OSPEED (*(volatile uint32_t *)(GPIOA_BASE + 0x08))
#define GPIOA_PUPDR  (*(volatile uint32_t *)(GPIOA_BASE + 0x0C))
#define GPIOA_BOP    (*(volatile uint32_t *)(GPIOA_BASE + 0x18))  /* set bits   */
#define GPIOA_BC     (*(volatile uint32_t *)(GPIOA_BASE + 0x28))  /* clear bits */
#define PA9_MASK     (1u<<9)

#define USB_BASE     0x40005C00u
#define USB_EPR(n)   (*(volatile uint32_t *)(USB_BASE + 4u*(n)))
#define USB_CNTR     (*(volatile uint32_t *)(USB_BASE + 0x40))
#define USB_ISTR     (*(volatile uint32_t *)(USB_BASE + 0x44))
#define USB_DADDR    (*(volatile uint32_t *)(USB_BASE + 0x4C))
#define USB_BTABLE   (*(volatile uint32_t *)(USB_BASE + 0x50))

/* PMA: 16-bit slots, doubled. CPU addr of PMA byte-offset `po` = 0x40006000 + 2*po. */
#define PMA_BASE     0x40006000u
#define PMA_U16(po)  ((volatile uint16_t *)(PMA_BASE + 2u*(uint32_t)(po)))
/* BTABLE entry for EP n (ADDR_TX/COUNT_TX/ADDR_RX/COUNT_RX at btable pma offset 8n) */
#define BT(n)        ((volatile uint16_t *)(PMA_BASE + 16u*(uint32_t)(n)))

/* CNTR bits */
#define CNTR_FRES    (1u<<0)
#define CNTR_PDWN    (1u<<1)
/* ISTR bits */
#define ISTR_CTR     (1u<<15)
#define ISTR_RESET   (1u<<10)
#define ISTR_EPID    (0x0Fu)
#define ISTR_DIR     (1u<<4)
/* EPnR fields */
#define EP_CTR_RX    (1u<<15)
#define EP_SETUP     (1u<<11)
#define EP_CTR_TX    (1u<<7)
#define EP_TYPE_BULK    (0u<<9)
#define EP_TYPE_CONTROL (1u<<9)
#define EP_TYPE_INTR    (3u<<9)
/* STAT values */
#define STAT_DISABLED 0u
#define STAT_STALL    1u
#define STAT_NAK      2u
#define STAT_VALID    3u

/* ---------------- PMA buffer layout (pma byte offsets) ---------------- */
#define PMA_EP0_TX  0x20
#define PMA_EP0_RX  0x60
#define PMA_EP1_TX  0xC0     /* MUST match firmware/updater/stub.s */
#define PMA_EP3_RX  0x100    /* MUST match firmware/updater/stub.s */
#define PMA_EP2_TX  0x140
#define COUNT_RX_64 0x8400   /* BL_SIZE=1 (32-B blocks), NUM_BLOCK=1 -> 64 bytes */

/* ---------------- EPnR toggle-safe helpers ----------------
 * STAT/DTOG bits are "toggle" type: written value XORs the current bit. CTR bits
 * are rc_w0. These masks preserve the *other* fields (write 0 = no change / no
 * toggle) while forcing the target STAT field to `s`. */
static inline void ep_set_stat_rx(uint32_t n, uint32_t s){
    USB_EPR(n) = (USB_EPR(n) & 0xBF8Fu) ^ (s << 12);
}
static inline void ep_set_stat_tx(uint32_t n, uint32_t s){
    USB_EPR(n) = (USB_EPR(n) & 0x8FBFu) ^ (s << 4);
}
/* Clear a CTR bit (rc_w0) WITHOUT toggling the STAT/DTOG bits. The mask must have
 * every toggle bit = 0 (so writing 0 leaves them unchanged) and the *other* CTR bit
 * = 1 (rc_w0: writing 1 preserves it). Cf. ST _ClearEP_CTR_RX/TX. */
static inline void ep_clear_ctr_rx(uint32_t n){ USB_EPR(n) &= 0x0F8Fu; } /* clr CTR_RX, keep CTR_TX */
static inline void ep_clear_ctr_tx(uint32_t n){ USB_EPR(n) &= 0x8F0Fu; } /* clr CTR_TX, keep CTR_RX */
/* Set EP_TYPE + EA on a freshly-reset (0) register without toggling STAT bits. */
static inline void ep_set_type_ea(uint32_t n, uint32_t type, uint32_t ea){
    USB_EPR(n) = type | ea;   /* toggle/CTR bits written 0 => no change */
}

/* ---------------- PMA copy (doubled addressing) ---------------- */
static void pma_write(uint32_t po, const uint8_t *src, uint32_t n){
    volatile uint16_t *d = PMA_U16(po);
    uint32_t i;
    for (i = 0; i + 1 < n; i += 2){ *d = (uint16_t)(src[i] | (src[i+1] << 8)); d += 2; }
    if (i < n){ *d = src[i]; }
}
static void pma_read(uint32_t po, uint8_t *dst, uint32_t n){
    volatile uint16_t *s = PMA_U16(po);
    uint32_t i;
    for (i = 0; i + 1 < n; i += 2){ uint16_t h = *s; dst[i] = (uint8_t)h; dst[i+1] = (uint8_t)(h>>8); s += 2; }
    if (i < n){ dst[i] = (uint8_t)(*s); }
}

/* ================= descriptors (the meter USB identity) ================= */
static const uint8_t desc_device[18] = {
    0x12,0x01,0x00,0x02,0x02,0x00,0x00,0x40,
    0xe9,0x28,0x8a,0x01,0x00,0x01,0x01,0x02,0x03,0x01
};
static const uint8_t desc_config[67] = {
    0x09,0x02,0x43,0x00,0x02,0x01,0x00,0x80,0x32,
    0x09,0x04,0x00,0x00,0x01,0x02,0x02,0x01,0x00,
    0x05,0x24,0x00,0x10,0x01,
    0x05,0x24,0x01,0x00,0x01,
    0x04,0x24,0x02,0x02,
    0x05,0x24,0x06,0x00,0x01,
    0x07,0x05,0x82,0x03,0x08,0x00,0x0a,
    0x09,0x04,0x01,0x00,0x02,0x0a,0x00,0x00,0x00,
    0x07,0x05,0x03,0x02,0x40,0x00,0x00,
    0x07,0x05,0x81,0x02,0x40,0x00,0x00
};
static const uint8_t str_langid[4]  = { 0x04,0x03,0x09,0x04 };
/* UTF-16LE strings (bLength, 0x03, then chars) */
static const uint8_t str_mfr[16]    = { 16,3,'R',0,'u',0,'i',0,'D',0,'e',0,'n',0,'g',0 };
static const uint8_t str_prod[44]   = { 44,3,'T',0,'C',0,'6',0,'6',0,' ',0,'U',0,'S',0,'B',0,
                                        ' ',0,'T',0,'y',0,'p',0,'e',0,'-',0,'C',0,' ',0,
                                        'M',0,'e',0,'t',0,'e',0,'r',0 };
static const uint8_t str_serial[10] = { 10,3,'T',0,'C',0,'6',0,'6',0 };

/* getva static reply (AES-256-ECB precomputed offline; decodes to a valid TC66 packet) */
#include "getva_reply.inc"

/* ================= EP0 control state ================= */
static uint8_t  setup[8];
static const uint8_t *ep0_in_ptr;    /* remaining IN data to send  */
static uint32_t ep0_in_len;          /* bytes remaining            */
static uint8_t  ep0_zlp_after;       /* send terminating ZLP when in_len hits 0 and last==64 */
static uint8_t  pending_addr;        /* SET_ADDRESS value to apply after status */
static uint8_t  addr_pending;
static uint8_t  ep0_out_data;        /* 1 => next EP0 OUT is a real data stage (SET_LINE_CODING) */
static volatile uint8_t usb_configured; /* set on SET_CONFIGURATION; gates the CDC dispatcher */

/* line coding (7 bytes) — accepted and echoed back, otherwise ignored */
static uint8_t  line_coding[7] = { 0x00,0xC2,0x01,0x00, 0x00,0x00,0x08 }; /* 115200 8N1 */

static void ep0_arm_tx(uint32_t n){                 /* generic TX arm from ep0_in_* */
    MK_PROBE = 0x630;
    uint32_t chunk = ep0_in_len > 64 ? 64 : ep0_in_len;
    pma_write(PMA_EP0_TX, ep0_in_ptr, chunk);
    BT(0)[2] = (uint16_t)chunk;                     /* COUNT_TX */
    ep0_in_ptr += chunk;
    ep0_in_len -= chunk;
    (void)n;
    ep_set_stat_tx(0, STAT_VALID);
}

static void ep0_start_in(const uint8_t *p, uint32_t len, uint32_t wLength){
    MK_PROBE = 0x610;
    if (len > wLength) len = wLength;               /* never send more than asked */
    ep0_in_ptr = p; ep0_in_len = len;
    ep0_zlp_after = (len && (len % 64) == 0 && len < wLength) ? 1 : 0;
    ep0_arm_tx(0);
    ep_set_stat_rx(0, STAT_VALID);                  /* allow status-OUT */
}
static void ep0_send_zlp_status(void){              /* no-data or status stage IN */
    MK_PROBE = 0x620;
    ep0_in_ptr = 0; ep0_in_len = 0; ep0_zlp_after = 0;
    BT(0)[2] = 0;                                   /* COUNT_TX = 0 */
    ep_set_stat_tx(0, STAT_VALID);
}
static void ep0_stall(void){
    MK_PROBE = 0x6f0;
    ep_set_stat_tx(0, STAT_STALL);
    ep_set_stat_rx(0, STAT_STALL);
}

static void handle_setup(void){
    MK_PROBE = 0x400 | setup[1];
    uint32_t bmReq = setup[0];
    uint32_t bReq  = setup[1];
    uint32_t wVal  = setup[2] | (setup[3] << 8);
    uint32_t wLen  = setup[6] | (setup[7] << 8);
    MK_SETUP = (bmReq << 8) | bReq;

    if ((bmReq & 0x60) == 0x00){          /* ---- standard requests ---- */
        switch (bReq){
        case 0x06: {                      /* GET_DESCRIPTOR */
            uint32_t type = wVal >> 8, idx = wVal & 0xff;
            const uint8_t *p = 0; uint32_t l = 0;
            if (type == 1){ p = desc_device; l = sizeof desc_device; }
            else if (type == 2){ p = desc_config; l = sizeof desc_config; }
            else if (type == 3){
                if (idx == 0){ p = str_langid; l = sizeof str_langid; }
                else if (idx == 1){ p = str_mfr; l = sizeof str_mfr; }
                else if (idx == 2){ p = str_prod; l = sizeof str_prod; }
                else if (idx == 3){ p = str_serial; l = sizeof str_serial; }
            }
            if (p) ep0_start_in(p, l, wLen); else ep0_stall();   /* devqual etc -> STALL */
            break; }
        case 0x05:                        /* SET_ADDRESS (apply after status IN) */
            pending_addr = (uint8_t)(wVal & 0x7f);
            addr_pending = 1;
            MK_STAGE = STAGE_ADDRESS;
            ep0_send_zlp_status();
            break;
        case 0x09:                        /* SET_CONFIGURATION */
            usb_configured = (wVal & 0xff) ? 1 : 0;
            if (usb_configured) MK_STAGE = STAGE_CONFIG;
            ep0_send_zlp_status();
            break;
        case 0x08: {                      /* GET_CONFIGURATION */
            static const uint8_t one = 1;
            ep0_start_in(&one, 1, wLen); break; }
        case 0x00: {                      /* GET_STATUS */
            static const uint8_t st[2] = {0,0};
            ep0_start_in(st, 2, wLen); break; }
        case 0x01:                        /* CLEAR_FEATURE */
        case 0x03:                        /* SET_FEATURE */
            ep0_send_zlp_status(); break;
        default:
            ep0_stall(); break;
        }
    } else if ((bmReq & 0x60) == 0x20){   /* ---- CDC class requests ---- */
        switch (bReq){
        case 0x20:                        /* SET_LINE_CODING (7-byte OUT data stage) */
            ep0_out_data = 1;             /* the next EP0 OUT is real data, not status */
            ep_set_stat_rx(0, STAT_VALID);
            break;
        case 0x21:                        /* GET_LINE_CODING */
            ep0_start_in(line_coding, sizeof line_coding, wLen); break;
        case 0x22:                        /* SET_CONTROL_LINE_STATE (no data) */
            ep0_send_zlp_status(); break;
        default:
            ep0_stall(); break;
        }
    } else {
        ep0_stall();
    }
}

static void handle_ep0(void){
    MK_PROBE = 0x300;
    uint32_t epr = USB_EPR(0);
    if (epr & EP_CTR_RX){
        uint32_t is_setup = epr & EP_SETUP;
        uint32_t cnt = BT(0)[6] & 0x3FF;             /* COUNT_RX */
        ep_clear_ctr_rx(0);
        if (is_setup){
            MK_PROBE = 0x310;
            ep0_out_data = 0;
            pma_read(PMA_EP0_RX, setup, 8);
            handle_setup();
        } else if (ep0_out_data){
            /* real OUT data stage (SET_LINE_CODING payload) -> ack with status IN */
            MK_PROBE = 0x320;
            ep0_out_data = 0;
            if (cnt >= 7) pma_read(PMA_EP0_RX, line_coding, 7);
            ep0_send_zlp_status();
        } else {
            /* status-OUT ZLP after a control-read: just accept + re-arm for next SETUP */
            MK_PROBE = 0x330;
            ep_set_stat_rx(0, STAT_VALID);
        }
    }
    if (USB_EPR(0) & EP_CTR_TX){
        MK_PROBE = 0x380;
        ep_clear_ctr_tx(0);
        if (addr_pending){                           /* SET_ADDRESS status done */
            USB_DADDR = 0x80u | pending_addr;
            addr_pending = 0;
            MK_ENUM = pending_addr;
        }
        if (ep0_in_len){
            ep0_arm_tx(0);                            /* next IN chunk */
        } else if (ep0_zlp_after){
            ep0_zlp_after = 0;
            BT(0)[2] = 0;
            ep_set_stat_tx(0, STAT_VALID);           /* terminating ZLP */
        }
        /* re-arm RX for the next SETUP */
        ep_set_stat_rx(0, STAT_VALID);
    }
}

/* ---------------- bus reset: (re)configure all endpoints ---------------- */
static void usb_reset(void){
    MK_PROBE = 0x200;
    MK_RESET = MK_RESET + 1;
    USB_BTABLE = 0;
    addr_pending = 0; ep0_in_len = 0;

    /* EP0: control, EA0, buffers */
    BT(0)[0] = PMA_EP0_TX; BT(0)[2] = 0;
    BT(0)[4] = PMA_EP0_RX; BT(0)[6] = COUNT_RX_64;
    ep_set_type_ea(0, EP_TYPE_CONTROL, 0);
    ep_set_stat_rx(0, STAT_VALID);
    ep_set_stat_tx(0, STAT_NAK);

    /* EP1: bulk IN (0x81) */
    BT(1)[0] = PMA_EP1_TX; BT(1)[2] = 0;
    ep_set_type_ea(1, EP_TYPE_BULK, 1);
    ep_set_stat_tx(1, STAT_NAK);
    ep_set_stat_rx(1, STAT_DISABLED);

    /* EP2: interrupt IN (0x82) */
    BT(2)[0] = PMA_EP2_TX; BT(2)[2] = 0;
    ep_set_type_ea(2, EP_TYPE_INTR, 2);
    ep_set_stat_tx(2, STAT_NAK);
    ep_set_stat_rx(2, STAT_DISABLED);

    /* EP3: bulk OUT (0x03) */
    BT(3)[4] = PMA_EP3_RX; BT(3)[6] = COUNT_RX_64;
    ep_set_type_ea(3, EP_TYPE_BULK, 3);
    ep_set_stat_rx(3, STAT_VALID);
    ep_set_stat_tx(3, STAT_DISABLED);

    USB_DADDR = 0x80u;                                /* EF=1, address 0 */
    if (MK_STAGE < STAGE_RESET) MK_STAGE = STAGE_RESET;
    MK_PROBE = 0x2ff;
}

/* ---------------- CDC bulk OUT (EP3) ---------------- */
static uint8_t rxbuf[64];
static void handle_ep3(void){
    if (!(USB_EPR(3) & EP_CTR_RX)) return;
    MK_PROBE = 0x500;
    uint32_t cnt = BT(3)[6] & 0x3FF;
    if (cnt > 64) cnt = 64;
    pma_read(PMA_EP3_RX, rxbuf, cnt);
    ep_clear_ctr_rx(3);
    MK_CMD = rxbuf[0] | (rxbuf[1]<<8) | (rxbuf[2]<<16) | (rxbuf[3]<<24);
    MK_CMDCNT = MK_CMDCNT + 1;
    if (MK_STAGE < STAGE_CMD) MK_STAGE = STAGE_CMD;

    /* Only dispatch once configured, so stray/foreign packets before enumeration
     * completes can't trigger a CDC reply (and its bounded TX wait). */
    int is_updat = usb_configured ? cdc_on_packet(rxbuf, cnt) : 0;
    /* On updat we deliberately DO NOT re-arm EP3 (leave it NAK) so the SRAM
     * flasher can arm it once, exactly as the flasher expects. */
    if (!is_updat)
        ep_set_stat_rx(3, STAT_VALID);
}

/* ---------------- public API ---------------- */
void usb_cdc_send(const uint8_t *buf, uint32_t len){
    if (len > 64) len = 64;
    pma_write(PMA_EP1_TX, buf, len);
    BT(1)[2] = (uint16_t)len;                        /* COUNT_TX */
    ep_clear_ctr_tx(1);
    ep_set_stat_tx(1, STAT_VALID);
    /* wait (bounded) until the host fetches it (CTR_TX) so back-to-back sends don't
     * clobber. Bounded so a confused/absent host can never wedge the main loop. */
    for (volatile uint32_t t = 0; t < 2000000u; t++){
        if (USB_EPR(1) & EP_CTR_TX) break;
    }
    ep_clear_ctr_tx(1);
}

const uint8_t *usb_getva_reply(void){ return getva_reply; }   /* used by main.c */

static void clock_init(void){
    /* Target: PLL x12 from HSI/2 => 48 MHz, USBDPSC=/1 (48 MHz USB clock).
     * The bootloader hands off with its OWN clock already running (PLL as system
     * clock), so we must switch back to HSI and disable PLL before we can change
     * PLLMF — a running PLL ignores config writes. This is robust from any state. */
    RCU_CTL0 |= (1u<<0);                              /* HSIEN */
    while (!(RCU_CTL0 & (1u<<1))) { }                 /* wait HSISTB */
    RCU_CFG0 &= ~0x3u;                                /* SCS = HSI */
    while ((RCU_CFG0 & 0xCu) != 0x0u) { }             /* wait SCSS = HSI */
    RCU_CTL0 &= ~(1u<<24);                            /* PLLEN = 0 (now allowed) */
    while (RCU_CTL0 & (1u<<25)) { }                   /* wait PLLSTB clear */
    RCU_CFG0 &= ~0x003C0000u;                         /* clear PLLMF[3:0] (bits 21:18) */
    RCU_CFG0 &= ~(1u<<27);                            /* clear PLLMF[4] */
    RCU_CFG0 &= ~(1u<<16);                            /* PLLSEL = HSI/2 */
    RCU_CFG0 |= (10u<<18);                            /* PLLMF=10 -> x12 (=48MHz) */
    RCU_CFG0 = (RCU_CFG0 & ~0x00C00000u) | 0x00400000u; /* USBDPSC = 01 (CK_PLL/1) */
    RCU_CTL0 |= (1u<<24);                             /* PLLEN */
    while (!(RCU_CTL0 & (1u<<25))) { }                /* wait PLLSTB */
    RCU_CFG0 = (RCU_CFG0 & ~0x3u) | 0x2u;             /* SCS = PLL */
    while ((RCU_CFG0 & 0xCu) != 0x8u) { }             /* wait SCSS = PLL */
    RCU_APB1EN |= (1u<<23);                           /* USBD clock enable */
    MK_STAGE = STAGE_CLOCK;
}

/* crude busy-wait milliseconds at 48 MHz (~12k loop iters/ms; generous) */
static void delay_ms(uint32_t ms){
    for (volatile uint32_t n = 0; n < ms * 16000u; n++) { __asm volatile("nop"); }
}

static void pa9_config_output(void){
    RCU_AHBEN |= (1u<<17);                            /* GPIOA clock (AHBEN bit17) */
    GPIOA_MODER  = (GPIOA_MODER  & ~(3u<<18)) | (1u<<18); /* PA9 = output */
    GPIOA_OTYPER &= ~(1u<<9);                         /* push-pull */
    GPIOA_OSPEED = (GPIOA_OSPEED & ~(3u<<18)) | (1u<<18); /* medium speed */
    GPIOA_PUPDR  &= ~(3u<<18);                        /* no pull */
}
static inline void pa9_low(void){  GPIOA_BC  = PA9_MASK; }  /* D+ low  -> disconnect */
static inline void pa9_high(void){ GPIOA_BOP = PA9_MASK; }  /* D+ pull-up -> connect */

void usb_init(void){
    /* init the runtime markers so we can read absolute counts over SWD */
    MK_HEART = 0; MK_CTR = 0; MK_RESET = 0; MK_CMDCNT = 0; MK_ENUM = 0;
    usb_configured = 0;

    clock_init();

    /* Force a fresh USB enumeration of THIS app. The bootloader jumped to us with
     * D+ still pulled up (PA9 high) and the host enumerated the *bootloader*; a bus
     * reset would otherwise never arrive. Pull PA9 (=D+ soft-connect) low to signal
     * an electrical disconnect, hold it, bring the USB core up, then reconnect. */
    pa9_config_output();
    pa9_low();                                        /* USB disconnect */
    delay_ms(120);                                    /* let the host notice */

    /* power-on / core reset (mirror usbd_core_poweron_reset @0x08007698):
     * clear PDWN but hold FRES, wait tSTARTUP, then release FRES. */
    USB_CNTR = CNTR_FRES;                             /* PDWN=0, FRES=1 */
    for (volatile int i = 0; i < 2000; i++) { }       /* tSTARTUP settle */
    USB_CNTR = 0;                                     /* release reset */
    USB_ISTR = 0;                                     /* clear pending */
    USB_BTABLE = 0;
    USB_DADDR = 0x80u;                                /* EF set, address 0 */
    MK_STAGE = STAGE_USB_INIT;

    pa9_high();                                       /* reconnect -> host enumerates us */
}

void usb_poll(void){
    MK_HEART = MK_HEART + 1;
    MK_PROBE = 0x100;
    uint32_t istr = USB_ISTR;
    MK_ISTR = istr;
    MK_PROBE = 0x110;
    if (istr & ISTR_RESET){
        USB_ISTR = ~ISTR_RESET;                       /* clear (rc_w0) */
        usb_reset();
        return;
    }
    /* Acknowledge (clear) the non-CTR status flags so they can't accumulate or wedge
     * the peripheral: SUSP(11) WKUP(12) SOF(9) ESOF(8) ERR(13) PMAOVR(14). We take no
     * power-management action (stay powered) but must clear them (rc_w0). */
    if (istr & 0x7B00u){
        USB_ISTR = (uint32_t)~0x7B00u;
    }
    if (istr & ISTR_CTR){
        MK_CTR = MK_CTR + 1;
        uint32_t ep = istr & ISTR_EPID;
        if (ep == 0) handle_ep0();
        else if (ep == 3) handle_ep3();
        else {
            /* EP1/EP2 IN complete — just clear CTR_TX */
            if (USB_EPR(ep) & EP_CTR_TX) ep_clear_ctr_tx(ep);
            if (USB_EPR(ep) & EP_CTR_RX) ep_clear_ctr_rx(ep);
        }
    }
    /* corruption detectors: if usb_poll's return address ever reads back as a
     * non-Thumb/garbage value, the freeze is a return-address smash (visible here
     * before the faulting pop). */
    MK_RA = (uint32_t)__builtin_return_address(0);
    { register uint32_t _sp __asm__("sp"); MK_SPEXIT = _sp; }
}
