/* Copyright (c) 2010 - 2019, Nordic Semiconductor ASA
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

#ifndef LIGHT_SWITCH_EXAMPLE_COMMON_H__
#define LIGHT_SWITCH_EXAMPLE_COMMON_H__

/**
 * @defgroup LIGHT_SWITCH_EXAMPLE_COMMON Common definitions for the Light switch example
 * @{
 */

/** Number of active servers.
 * Note: If the value of SERVER_NODE_COUNT is increased, you may need to scale up the the replay
 * protection list size (@ref REPLAY_CACHE_ENTRIES), by the appropriate amount, for the provisioner and
 * client examples. For the provisioner example to work as expected, its replay protection list size should
 * be greater than or equal to the total number of nodes it is going to configure after provisioning them.
 * The replay protection list size of the client example should be greater than or equal to the total
 * number of unicast addresses in the network that it can receive a message from.
 */
#define SERVER_NODE_COUNT (30)
#if SERVER_NODE_COUNT > 30
#error Maximum 30 servers currently supported by client example.
#endif

/** Number of active clients nodes. */
#define CLIENT_NODE_COUNT            (1)

/** Number of On-Off client models on the Switch Node */
#define CLIENT_MODEL_INSTANCE_COUNT  (2)

/** Number of group address being used in this example */
#define GROUP_ADDR_COUNT             (2)

/** Static authentication data */
#define STATIC_AUTH_DATA {0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88}


/** @} end of Common definitions for the Light switch example */

#endif /* LIGHT_SWITCH_EXAMPLE_COMMON_H__ */
