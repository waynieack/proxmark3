//-----------------------------------------------------------------------------
// Copyright (C) Proxmark3 contributors. See AUTHORS.md for details.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// See LICENSE.txt for the text of the license.
//-----------------------------------------------------------------------------
// Standalone mode: SRT512 tag simulator with internal flash persistence.
//
// Setup (from PM3 client):
//   hf 14b simsrx -f <dump.json> [-s <chipid>] [-0] [-n] [-d <0-4>] [--trace] --saveconfig
//
// Button behaviour (standalone, no PC required):
//   Power-on  → starts in SIM mode (simulates stored tag)
//   Tap       → toggles SIM ↔ SCAN mode
//   Hold 1s   → exits standalone
//
// SCAN mode:
//   Searches for a real SRT512 tag and reads all blocks.
//   On success, saves ONLY the dump page (UID + blocks).
//   Config settings (flags, debug level, etc.) are NEVER changed by a scan.
//   After a successful save, automatically switches back to SIM mode.
//
// LED codes:
//   LED_A only  → SIM mode active
//   LED_B only  → SCAN mode searching
//   LED_A+B     → flash write in progress / success blink
//   All off     → idle / exiting
//-----------------------------------------------------------------------------

#include "standalone.h"
#include "proxmark3_arm.h"
#include "appmain.h"
#include "iso14443b.h"
#include "fpgaloader.h"
#include "util.h"
#include "dbprint.h"
#include "ticks.h"
#include "string.h"

// ============================================================
// SRT512 standalone flash storage (AT91SAM7S internal EFC)
// ============================================================
// Storage occupies the last 2 pages of internal flash.
// Addresses are computed dynamically so they work on both
// AT91SAM7S256 (256 KB) and AT91SAM7S512 (512 KB).
//
// AT91SAM7S512 (512 KB):  config=0x0017FE00  dump=0x0017FF00  (EFC1)
// AT91SAM7S256 (256 KB):  config=0x0013FE00  dump=0x0013FF00  (EFC0)
//
// AT91SAM7S has two EFC controllers (each manages one 256 KB plane):
//   EFC0 (0xFFFFFF60): addresses 0x00100000 – 0x0013FFFF
//   EFC1 (0xFFFFFF70): addresses 0x00140000 – 0x0017FFFF  (512 KB only)
//
// START_PROG (0x01) auto-erases the page before writing.
// RAMFUNC + IRQ masking required: firmware may execute from the same
// flash plane being programmed; ISR fetch must not occur during PROG.
//
// NOTE: These pages are never linked by the firmware binary — they live
// beyond _etext/_edata.  If the firmware ever grows this large the magic
// check will fail gracefully and standalone will report "no config".
//
// NOTE: The RDV4 external SPI flash (W25Q128 / SPIFFS) is a completely
// separate chip on a different bus.  EFC only touches internal flash.
// WITH_FLASH / SPIFFS code is never called from this module.

#define SRT512_SA_CONFIG_ADDR  (AT91C_IFLASH + AT91C_IFLASH_SIZE - 512UL)
#define SRT512_SA_DUMP_ADDR    (AT91C_IFLASH + AT91C_IFLASH_SIZE - 256UL)

// Prototypes for non-static storage functions defined below.
// appmain.c has matching declarations inside #ifdef WITH_STANDALONE_HF_SRT512SA.
bool srt512_sa_is_valid(void);
bool srt512_sa_config_store(const srt512_sa_config_t *cfg);
bool srt512_sa_config_load(srt512_sa_config_t *cfg);
bool srt512_sa_dump_store(const srt512_sa_dump_t *dump);
bool srt512_sa_dump_load(srt512_sa_dump_t *dump);

// Write one 256-byte flash page from SRAM.  Must be a RAMFUNC.
// flash_addr must be 256-byte aligned.
static RAMFUNC void srt512_efc_prog_page(uint32_t flash_addr, const uint32_t *data32) {
    volatile uint32_t *page_buf = (volatile uint32_t *)flash_addr;
    for (uint32_t i = 0; i < 64; i++) {
        page_buf[i] = data32[i];
    }

    AT91PS_EFC efc;
    uint32_t page_in_plane;
    if (flash_addr < 0x00140000UL) {
        efc = AT91C_BASE_EFC0;
        page_in_plane = (flash_addr - 0x00100000UL) / 256UL;
    } else {
        efc = AT91C_BASE_EFC1;
        page_in_plane = (flash_addr - 0x00140000UL) / 256UL;
    }

    uint32_t cpsr;
    __asm__ volatile ("MRS %0, cpsr" : "=r"(cpsr));
    __asm__ volatile ("MSR cpsr_c, %0" :: "r"(cpsr | 0xC0U));

    efc->EFC_FCR = MC_FLASH_COMMAND_KEY | MC_FLASH_COMMAND_PAGEN(page_in_plane) | AT91C_MC_FCMD_START_PROG;

    while (!(efc->EFC_FSR & AT91C_MC_FRDY)) {}

    __asm__ volatile ("MSR cpsr_c, %0" :: "r"(cpsr));
}

// Safety guard: only allow writes to the two reserved storage pages.
static bool srt512_sa_addr_ok(uint32_t flash_addr) {
    return (flash_addr == SRT512_SA_CONFIG_ADDR || flash_addr == SRT512_SA_DUMP_ADDR);
}

bool srt512_sa_is_valid(void) {
    const srt512_sa_config_t *cfg = (const srt512_sa_config_t *)SRT512_SA_CONFIG_ADDR;
    return (cfg->magic == SRT512_SA_MAGIC);
}

bool srt512_sa_config_store(const srt512_sa_config_t *cfg) {
    if (!srt512_sa_addr_ok(SRT512_SA_CONFIG_ADDR)) return false;
    srt512_efc_prog_page(SRT512_SA_CONFIG_ADDR, (const uint32_t *)cfg);
    const srt512_sa_config_t *stored = (const srt512_sa_config_t *)SRT512_SA_CONFIG_ADDR;
    return (stored->magic == SRT512_SA_MAGIC);
}

bool srt512_sa_config_load(srt512_sa_config_t *cfg) {
    const srt512_sa_config_t *stored = (const srt512_sa_config_t *)SRT512_SA_CONFIG_ADDR;
    if (stored->magic != SRT512_SA_MAGIC) return false;
    memcpy(cfg, stored, sizeof(srt512_sa_config_t));
    return true;
}

bool srt512_sa_dump_store(const srt512_sa_dump_t *dump) {
    if (!srt512_sa_addr_ok(SRT512_SA_DUMP_ADDR)) return false;
    srt512_efc_prog_page(SRT512_SA_DUMP_ADDR, (const uint32_t *)dump);
    const srt512_sa_dump_t *stored = (const srt512_sa_dump_t *)SRT512_SA_DUMP_ADDR;
    return (stored->magic == SRT512_SA_MAGIC);
}

bool srt512_sa_dump_load(srt512_sa_dump_t *dump) {
    const srt512_sa_dump_t *stored = (const srt512_sa_dump_t *)SRT512_SA_DUMP_ADDR;
    if (stored->magic != SRT512_SA_MAGIC) return false;
    memcpy(dump, stored, sizeof(srt512_sa_dump_t));
    return true;
}

// Scan once for a real SRT512 tag (~47 ms timeout per attempt).
// Caller must call iso14443b_setup() before and switch_off() after the scan loop.
// Returns PM3_SUCCESS and fills *out on success.
static int srt512_sa_scan_once(srt512_sa_dump_t *out) {
    iso14b_set_timeout(5000);   // 5000 ETU ≈ 47 ms

    iso14b_card_select_t card;
    memset(&card, 0, sizeof(card));

    int res = iso14443b_select_srx_card(&card);
    iso14b_set_timeout(9000);   // restore default

    if (res != PM3_SUCCESS)
        return res;

    memset(out, 0, sizeof(*out));
    uint8_t num_blocks = 0;
    for (int blk = 0; blk <= 16; blk++) {
        uint8_t tmp[4] = {0};
        if (read_14b_srx_block((uint8_t)blk, tmp) != PM3_SUCCESS)
            return PM3_ECARDEXCHANGE;
        memcpy(out->blocks[blk], tmp, 4);
        num_blocks = (uint8_t)(blk + 1);
    }

    out->magic      = SRT512_SA_MAGIC;
    out->num_blocks = num_blocks;
    memcpy(out->uid, card.uid, 8);
    return PM3_SUCCESS;
}

void ModInfo(void) {
    DbpString(" HF SRT512 SA - SRT512 standalone simulator with flash persistence");
    DbpString("   Setup: hf 14b simsrx -f <dump.json> [-n] [-0] [-s <id>] --saveconfig");
    DbpString("   Tap button: toggle SIM/SCAN  |  Hold 1s: exit");
}

void RunMod(void) {
    StandAloneMode();

    Dbprintf(_YELLOW_("HF SRT512 SA") " started");

    // Require a valid config saved by --saveconfig
    if (!srt512_sa_is_valid()) {
        Dbprintf("No valid config found. From PM3 client run:");
        Dbprintf("  hf 14b simsrx -f <dump.json> --saveconfig");
        LED_A_ON(); SpinDelay(200); LED_A_OFF();
        LED_B_ON(); SpinDelay(200); LED_B_OFF();
        LED_A_ON(); SpinDelay(200); LED_A_OFF();
        return;
    }

    srt512_sa_config_t config;
    srt512_sa_dump_t   dump;

    if (!srt512_sa_config_load(&config) || !srt512_sa_dump_load(&dump)) {
        Dbprintf("Failed to load stored config/dump");
        return;
    }

    Dbprintf("Config loaded: flags=0x%08X chipid=0x%02X debug=%d trace=%d",
             config.flags, config.static_chipid, config.debug_level, config.tracing);
    Dbprintf("Dump loaded: uid=%02X%02X%02X%02X%02X%02X%02X%02X blocks=%d",
             dump.uid[0], dump.uid[1], dump.uid[2], dump.uid[3],
             dump.uid[4], dump.uid[5], dump.uid[6], dump.uid[7],
             dump.num_blocks);

    bool sim_mode = true;  // start in SIM mode
    bool running  = true;

    while (running) {

        if (sim_mode) {
            // ---- SIM mode: simulate the stored tag ----
            LED_A_ON();
            LED_B_OFF();

            // Backward compatibility: old configs stored tracing in cfg.tracing
            // but may not have SRT512_FLAG_TRACE set in cfg.flags.
            uint32_t sim_flags = config.flags;
            if (config.tracing)
                sim_flags |= SRT512_FLAG_TRACE;

            Dbprintf("SRT512 SA: SIM - uid=%02X%02X%02X%02X%02X%02X%02X%02X",
                     dump.uid[0], dump.uid[1], dump.uid[2], dump.uid[3],
                     dump.uid[4], dump.uid[5], dump.uid[6], dump.uid[7]);

            SimulateSRT512Tag(dump.uid,
                              (const uint8_t *)dump.blocks,
                              dump.num_blocks,
                              sim_flags,
                              config.static_chipid);

            LEDsoff();

            // Distinguish tap (→ SCAN) from hold (→ exit)
            int btn = BUTTON_HELD(1000);
            if (btn == BUTTON_HOLD) {
                Dbprintf("SRT512 SA: exit");
                running = false;
            } else if (data_available()) {
                Dbprintf("SRT512 SA: exit");
                running = false;
            } else {
                Dbprintf("SRT512 SA: switching to SCAN mode");
                sim_mode = false;
            }

        } else {
            // ---- SCAN mode: search for a real SRT512 tag ----
            // Setup field ONCE at scan mode entry; keep it on between attempts.
            // srt512_sa_scan_once() uses a short ~47ms timeout per attempt so the
            // loop is responsive — no 5-second hang when no tag is present.
            LED_B_ON();
            LED_A_OFF();

            Dbprintf("SRT512 SA: SCAN - tap button to return to SIM, hold 1s to exit");

            iso14443b_setup();

            bool found = false;
            while (!found) {
                WDT_HIT();
                if (data_available()) {
                    Dbprintf("SRT512 SA: exit");
                    running = false;
                    break;
                }

                srt512_sa_dump_t new_dump;
                int res = srt512_sa_scan_once(&new_dump);

                if (res == PM3_SUCCESS) {
                    Dbprintf("SRT512 SA: tag found uid=%02X%02X%02X%02X%02X%02X%02X%02X",
                             new_dump.uid[0], new_dump.uid[1], new_dump.uid[2], new_dump.uid[3],
                             new_dump.uid[4], new_dump.uid[5], new_dump.uid[6], new_dump.uid[7]);

                    // Save ONLY the dump page — config settings untouched
                    LED_A_ON();
                    bool saved = srt512_sa_dump_store(&new_dump);
                    if (saved) {
                        srt512_sa_dump_load(&dump);
                        Dbprintf("SRT512 SA: saved to flash");
                    } else {
                        Dbprintf("SRT512 SA: flash write failed");
                    }

                    // Success blink
                    for (int i = 0; i < 3; i++) {
                        LEDsoff();      SpinDelay(80);
                        LED_A_ON(); LED_B_ON(); SpinDelay(80);
                    }
                    LEDsoff();

                    sim_mode = true;
                    found = true;

                } else {
                    // No tag this attempt — check button
                    if (BUTTON_PRESS()) {
                        int btn = BUTTON_HELD(800);
                        if (btn == BUTTON_HOLD) {
                            Dbprintf("SRT512 SA: exit from scan");
                            running = false;
                        } else {
                            Dbprintf("SRT512 SA: tap, returning to SIM");
                            sim_mode = true;
                        }
                        break;
                    }
                    SpinDelay(50);
                }
            }

            switch_off();
        }
    }

    LEDsoff();
    switch_off();
    Dbprintf("SRT512 SA: done");
}
