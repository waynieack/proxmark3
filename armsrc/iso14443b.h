//-----------------------------------------------------------------------------
// Copyright (C) Jonathan Westhues, Nov 2006
// Copyright (C) Gerhard de Koning Gans - May 2008
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
// Routines to support ISO 14443 type B.
//-----------------------------------------------------------------------------

#ifndef __ISO14443B_H
#define __ISO14443B_H

#include "common.h"

#include "iso14b.h"
#include "pm3_cmd.h"

#ifndef AddCrc14A
# define AddCrc14A(data, len) compute_crc(CRC_14443_A, (data), (len), (data)+(len), (data)+(len)+1)
#endif

#ifndef AddCrc14B
# define AddCrc14B(data, len) compute_crc(CRC_14443_B, (data), (len), (data)+(len), (data)+(len)+1)
#endif

#ifndef AddCrc15
#define AddCrc15(data, len) compute_crc(CRC_ICLASS, (data), (len), (data)+(len), (data)+(len)+1)
#endif

void iso14443b_setup(void);
int iso14443b_apdu(uint8_t const *msg, size_t msg_len, bool send_chaining, void *rxdata, uint16_t rxmaxlen, uint8_t *response_byte, uint16_t *responselen);

int iso14443b_select_card(iso14b_card_select_t *card);

void SimulateIso14443bTag(const uint8_t *pupi);
void SimulateSRT512Tag(const uint8_t *uid, const uint8_t *blocks, uint8_t num_blocks, uint32_t flags, uint8_t static_chipid);
void read_14b_st_block(uint8_t blocknr);
int read_14b_srx_block(uint8_t blocknr, uint8_t *block);
int iso14443b_select_srx_card(iso14b_card_select_t *card);
void SniffIso14443b(void);
void SendRawCommand14443B(iso14b_raw_cmd_t *p);
void ST25TB_TearOff(const uint8_t *data);
void CodeAndTransmit14443bAsReader(const uint8_t *cmd, int len, uint32_t *start_time, uint32_t *eof_time, bool framing);

// 14b config
void printHf14bConfig(void);
void setHf14bConfig(const hf14b_config_t *hc);
hf14b_config_t *getHf14bConfig(void);

// States for 14B SIM command
#define SIM_POWER_OFF   0
#define SIM_IDLE        1
#define SIM_READY       2
#define SIM_HALT        3
#define SIM_ACTIVE      4

// ---- SRT512 standalone flash storage ------------------------------------
// Structs shared between appmain.c dispatch and hf_srt512_sa.c storage code.
// Address defines and function implementations live in hf_srt512_sa.c.
#define SRT512_SA_MAGIC  0x53525431UL  // "SRT1"

// Persisted simulation settings.  Written by --saveconfig.
// NEVER overwritten by a standalone scan — scan updates only the dump page.
typedef struct {
    uint32_t magic;           // SRT512_SA_MAGIC
    uint32_t flags;           // SRT512_FLAG_* bits
    uint8_t  static_chipid;   // used when SRT512_FLAG_STATIC_CHIPID set
    uint8_t  debug_level;     // 0-4
    uint8_t  tracing;         // 1 = enable tracing
    uint8_t  reserved[245];   // pad to exactly 256 bytes
} srt512_sa_config_t;         // sizeof == 256

// Persisted tag data.  Written by --saveconfig AND overwritten by standalone scan.
typedef struct {
    uint32_t magic;           // SRT512_SA_MAGIC
    uint8_t  uid[8];          // 8-byte UID
    uint8_t  blocks[17][4];   // up to 17 blocks x 4 bytes = 68 bytes
    uint8_t  num_blocks;      // actual number stored (1-17)
    uint8_t  reserved[175];   // pad to exactly 256 bytes
} srt512_sa_dump_t;           // sizeof == 256

// Timeout setter — used by both iso14443b.c and hf_srt512_sa.c
void iso14b_set_timeout(uint32_t timeout_etu);

#endif /* __ISO14443B_H */
