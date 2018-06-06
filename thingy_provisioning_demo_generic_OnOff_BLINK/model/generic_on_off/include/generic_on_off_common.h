/* Copyright (c) 2010 - 2018, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef GENERIC_ON_OFF_COMMON_H__
#define GENERIC_ON_OFF_COMMON_H__

#include <stdint.h>
#include "access.h"

/*lint -align_max(push) -align_max(1) */

#define GENERIC_ON_OFF_COMPANY_ID    (ACCESS_COMPANY_ID_NONE)

/** Generic OnOff opcodes. Model specification P.298 */
typedef enum
{
    GENERIC_ON_OFF_OPCODE_GET = 0x8201,            /**< Generic OnOff Acknowledged Set. */
    GENERIC_ON_OFF_OPCODE_SET = 0x8202,            /**< Generic OnOff Get. */
    GENERIC_ON_OFF_OPCODE_SET_UNRELIABLE = 0x8203, /**< Generic OnOff Set Unreliable. */
    GENERIC_ON_OFF_OPCODE_STATUS = 0x8204          /**< Generic OnOff Status. */
} generic_on_off_opcode_t;

/** Message format for the Generic OnOff Set message. P.43 */
typedef struct __attribute((packed))
{
    uint8_t on_off; /**< State to set. */
    uint8_t tid;    /**< Transaction number. */
} generic_on_off_msg_set_t;

/** Message format for th Generic OnOff Set Unreliable message. P.43 */
typedef struct __attribute((packed))
{
    uint8_t on_off; /**< State to set. */
    uint8_t tid;    /**< Transaction number. */
} generic_on_off_msg_set_unreliable_t;

/** Message format for the Generic OnOff Status message. P.44 */
typedef struct __attribute((packed))
{
    uint8_t present_on_off; /**< Current state. */
} generic_on_off_msg_status_t;

/*lint -align_max(pop) */

/** @} end of GENERIC_ON_OFF_COMMON */
/** @} end of GENERIC_ON_OFF_MODEL */
#endif /* GENERIC_ON_OFF_COMMON_H__ */
