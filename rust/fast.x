/* Fast-flash placement (QUIRKS §4, docs/FIRMWARE.md "Fast-flash placement"):
 * the GD32F150 fetches zero-wait only below 0x0800_8000; above it every
 * cold fetch costs tens of µs. .fasttext pins the hot path directly after
 * the vector table; ordinary .text follows and spills past the boundary;
 * .slowtext force-demotes cold machinery that the globs below would
 * otherwise catch (mangled generic names contain the type-parameter
 * crates, so crate globs over-match).
 *
 * WHAT GOES FAST, two ways:
 * - Our own functions: `#[link_section = ".fasttext.<unique name>"]` on
 *   the fn (claimed by `*(.fasttext.*)` below). Callees the compiler does
 *   not inline stay slow unless annotated too; closures get their own
 *   symbols and need a pattern here.
 * - Other crates' code, generic fns with closures, and whole modules:
 *   mangled-name patterns below.
 *
 * NO TIERS: `make fast-gate` FAILS the build if .fasttext crosses
 * 0x0800_8000 (nothing spills silently) or if a critical symbol — e.g. one
 * whose pattern stopped matching after a rename — lands slow. On overflow,
 * stop and decide what to demote; the rough priority (most critical first)
 * is ISR chains > builtins > main loop/powerz > getva/AES > TFT engine >
 * per-command helpers.
 *
 * MECHANICS (all load-bearing):
 * - This script must be passed BEFORE link.x (-Tfast.x -Tlink.x): input
 *   sections are claimed in script order, and these sections must steal
 *   their functions before link.x's .text glob takes everything.
 * - Claim order inside this file: .slowtext first (steals the cold
 *   contested symbols), then .fasttext; INSERT clauses place them
 *   vector_table -> .fasttext -> .text -> .slowtext.
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

    /* our #[link_section = ".fasttext.*"] functions: USBD_LP, the TIMER5
     * veneer (ina.rs asm), the main loop shell + powerz refresh, command
     * dispatch, do_getva/do_rawva, the TFT pixel engine + ui::service */
    *(.fasttext.*);

    /* ISR chains: whole modules + other crates' USB stack */
    *(.text.*7tc66_fw3ina*);            /* sampler tick, cal math */
    *(.text.*7tc66_fw3ntc*);            /* adc_burst temperature (log2 loop) */
    *(.text.*7tc66_fw3i2c*);            /* I2C read inside the ISR */
    *(.text.*7tc66_fw6powerz*);         /* poll_cmd/arm/set_payload */
    *(.text.*7tc66_fw7usb_bus*);
    *(.text.*8with_usb*);               /* generic: incl. its closures */
    *(.text.*6UsbBus4poll*);            /* stm32_usbd bus poll */
    *(.text.*10stm32_usbd8endpoint*);   /* endpoint + PMA access */
    *(.text.*10stm32_usbd15endpoint_memory*);
    *(.text.*6UsbBus5write* .text.*6UsbBus4read*);
    *(.text.*11usbd_serial*);
    *(.text.*10usb_device*4poll*);      /* UsbDevice::poll dispatch */
    *(.text.*8Endpoint*5write*);
    *(.text.*8Endpoint*4read*);
    *(.text.*18handle_in_complete*);

    /* builtins used from the ISRs / everywhere */
    *(.text.*20copy_from_slice_impl*); /* the byte-copy loop: 18% of PC
                                        * samples when it sat in slow flash */
    *(.text.memcpy .text.*3mem6memcpy* .text.memcmp);
    *(.text.__aeabi_memcpy*);
    *(.text.__aeabi_memclr* .text.*3mem6memset*);
    *(.text.*11u64_div_rem*);           /* cal_v/cal_i u64 divides (ISR) */
    *(.text.__aeabi_uldivmod .text.__udivmoddi4);
    *(.text.__aeabi_uidiv* .text.__udivsi3 .text.__umodsi3);

    /* getva/rawva reply path: the aes crate's inner loops (QUIRKS §4
     * fix) + our getva module (getva::build is demoted in .slowtext) */
    *(.text.*14aes256_encrypt*);
    *(.text.*9sub_bytes*);
    *(.text.*13mix_columns_*);
    *(.text.*11xor_columns*);
    *(.text.*10memshift32*);
    *(.text.*7tc66_fw5getva*);

    . = ALIGN(8); /* .text is 8-aligned; a 4-aligned end draws an lld warning */
    __efasttext = .;
  } > FLASH
} INSERT AFTER .vector_table;

/* Place .text right after .fasttext (overrides link.x's PROVIDE).
 * Computed from SIZEOF, NOT `= __efasttext`: this assignment is evaluated
 * before .fasttext is laid out, so on lld's first pass __efasttext reads 0,
 * .text looks out of BL range, and lld emits a permanent movw/movt long-branch
 * thunk for every cross-region call (~900 B). memory.x is not yet included
 * here, so the FLASH origin is spelled out; flasher.x asserts the result.
 * fast-gate fails on any Thumbv7ABSLongThunk. */
_stext = 0x08002C00 + SIZEOF(.vector_table) + SIZEOF(.fasttext);
