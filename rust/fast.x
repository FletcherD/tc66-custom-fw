/* Fast-flash placement (QUIRKS §4 / audit 2026-08-25): the GD32F150 fetches
 * zero-wait only below 0x0800_8000 ("the preceding 32 pages"); above that the
 * manual promises "a long delay" per fetch (~10x, bench-measured via AES).
 * .fasttext pins the hot path directly after the vector table; ordinary
 * .text follows and spills past the boundary; .slowtext force-demotes cold
 * machinery that the hot globs would otherwise catch (mangled generic names
 * contain the type-parameter crates, so crate globs over-match).
 *
 * MECHANICS (all load-bearing):
 * - This script must be passed BEFORE link.x (-Tfast.x -Tlink.x): input
 *   sections are claimed in script order, and these sections must steal
 *   their functions before link.x's .text glob takes everything.
 * - Claim order inside this file: .slowtext first (steals the cold
 *   contested symbols), then .fasttext (hot globs get the rest); INSERT
 *   clauses place them vector_table -> .fasttext -> .text -> .slowtext.
 * - .fasttext PATTERNS ARE PRIORITY-ORDERED: if the list outgrows the fast
 *   region the TAIL crosses into slow flash — keep the least critical
 *   entries last. `make fast-gate` verifies the criticals made it.
 * - `_stext = __efasttext` overrides link.x's PROVIDE so .text is placed
 *   immediately after .fasttext (contiguous LMA for objcopy). */

SECTIONS
{
  /* Enum-time / one-shot code the hot globs must not claim. */
  .slowtext : ALIGN(4)
  {
    *(.text.*14get_descriptor*);
    *(.text.*16DescriptorWriter*);
    *(.text.*9BosWriter*);
    *(.text.*29get_configuration_descriptors*);
    *(.text.*17StringDescriptors*);
    *(.text.*8alloc_ep*);
    *(.text.*11control_out* .text.*10control_in*);
    *(.text.*9set_error* .text.*10handle_out* .text.*14write_in_chunk*);
    *(.text.*11accept_with* .text.*6accept*);
    *(.text.*7Request5parse*);
    *(.text.*11set_stalled* .text.*10is_stalled* .text.*7unstall*);
    *(.text.*19aes256_key_schedule*);   /* one-shot, boot (getva::init) */
    *(.text.*7try_cal*);                /* boot + 0.25 s retries until cal */
    *(.text.*8UsbError*);
    /* getva::build (encrypted compat packet, 1.2 KB): the first casualty of
     * the documented squeeze order — getva = compat, spills first. The AES
     * inner loops and build_raw (rawva streaming) stay in tier 4. Does NOT
     * match 5getva9build_raw (length-prefixed mangling). */
    *(.text.*5getva5build*);
  } > FLASH
} INSERT AFTER .text;

SECTIONS
{
  .fasttext : ALIGN(4)
  {
    __sfasttext = .;

    /* --- tier 1: the two ISRs + everything they call --- */
    *(.text.USBD_LP .text.TIMER5_DAC);
    *(.text.*7tc66_fw3ina*);            /* sampler tick, cal math */
    *(.text.*7tc66_fw3i2c*);            /* I2C read inside the ISR */
    *(.text.*12drain_serial*);
    *(.text.*17ep_events_pending*); /* the ISR drain-loop condition */
    *(.text.*7tc66_fw6powerz*);         /* poll_cmd/arm/set_payload */
    *(.text.*6UsbBus4poll*);            /* stm32_usbd bus poll */
    *(.text.*10stm32_usbd8endpoint*);   /* endpoint + PMA access */
    *(.text.*10stm32_usbd15endpoint_memory*);
    *(.text.*6UsbBus5write* .text.*6UsbBus4read*);
    *(.text.*11usbd_serial*);
    *(.text.*7tc66_fw7usb_bus*);
    *(.text.*10usb_device*4poll*);      /* UsbDevice::poll dispatch */
    *(.text.*8Endpoint*5write*);
    *(.text.*8Endpoint*4read*);
    *(.text.*18handle_in_complete*);
    *(.text.*8with_usb*);               /* incl. its closure instantiations */

    /* --- tier 2: builtins used from the ISRs / everywhere --- */
    *(.text.*20copy_from_slice_impl*); /* the byte-copy loop: 18% of PC
                                        * samples when it sat in slow flash */
    *(.text.memcpy .text.*3mem6memcpy* .text.memcmp);
    *(.text.__aeabi_memcpy*);
    *(.text.__aeabi_memclr* .text.*3mem6memset*);
    *(.text.*11u64_div_rem*);           /* cal_v/cal_i u64 divides (ISR) */
    *(.text.__aeabi_uldivmod .text.__udivmoddi4);
    *(.text.__aeabi_uidiv* .text.__udivsi3 .text.__umodsi3);

    /* --- tier 3: main loop shell + the powerz data path (powerz is the
     * PRIORITY protocol — its 250 Hz payload refresh and the loop that
     * paces it outrank the compatibility verbs below) --- */
    *(.text.*7tc66_fw3run*);
    *(.text.*22refresh_powerz_payload*);
    *(.text.*14powerz_payload*);
    *(.text.*16run_pending_cmds*);
    *(.text.*9write_all*);
    *(.text.*9on_packet* .text.*6match5*);

    /* --- tier 4: the getva/rawva reply path (COMPATIBILITY — the ~20 ms
     * AES fix lives here; ordered so a future squeeze spills do_getva/
     * do_rawva's thin wrappers before the AES inner loops) --- */
    *(.text.*14aes256_encrypt*);
    *(.text.*9sub_bytes*);
    *(.text.*13mix_columns_*);
    *(.text.*11xor_columns*);
    *(.text.*10memshift32*);
    *(.text.*7tc66_fw5getva*);          /* build, build_raw (init is 56 B) */
    *(.text.*8do_getva* .text.*8do_rawva*);

    /* --- tier 4b: the TFT pixel engine (glyph/fill DMA pump). In slow
     * flash its row loop missed on nearly every call and its speed swung
     * with layout (full redraw 0.15 s .. 1.3 s across unrelated commits).
     * Init, backlight, rotation and scroll commands stay slow. --- */
    *(.text.*7tc66_fw3tft7service*);
    *(.text.*3tft9stage_row* .text.*3tft8dma_kick* .text.*3tft9pump_idle*);
    *(.text.*3tft10start_char* .text.*3tft10start_fill*);
    *(.text.*3tft19pixel_session_begin* .text.*3tft17pixel_session_end*);
    *(.text.*3tft10set_window* .text.*3tft11spi_frame16*);
    *(.text.*3tft13write_command* .text.*3tft10write_data*);
    *(.text.*3tft14spi_write_byte* .text.*3tft13spi_wait_idle*);
    *(.text.*7tc66_fw2ui7service*);

    /* --- tier 5: per-command helpers (spill fodder — fine either side) --- */
    *(.text.*11live_fields* .text.*11reply_ok_er*);
    *(.text.*11i2c_command* .text.*8do_i2crd* .text.*8do_adcrd*);
    *(.text.*12find_bulk_ep*);

    . = ALIGN(8); /* .text is 8-aligned; a 4-aligned end draws an lld warning */
    __efasttext = .;
  } > FLASH
} INSERT AFTER .vector_table;

/* Place .text right after .fasttext (overrides link.x's PROVIDE). */
_stext = __efasttext;
