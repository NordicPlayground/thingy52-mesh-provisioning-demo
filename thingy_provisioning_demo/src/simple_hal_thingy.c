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

#include "simple_hal_thingy.h"
#include "nrf_mesh_config_examples.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "nrf.h"
#include "nrf_error.h"
#include "boards.h"
#include "nrf_delay.h"

#include "nrf_mesh_defines.h"
#include "timer.h"
#include "app_timer.h"
#include "app_error.h"
#include "drv_ext_light.h"
#include "drv_ext_gpio.h"
#include "m_ui.h"
/*****************************************************************************
 * Definitions
 *****************************************************************************/
#define LED_PIN_CONFIG ((GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)   | \
                        (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)       | \
                        (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)     | \
                        (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) | \
                        (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos))


#define GPIOTE_IRQ_LEVEL NRF_MESH_IRQ_PRIORITY_LOWEST

/*****************************************************************************
 * Static variables
 *****************************************************************************/


APP_TIMER_DEF(m_blink_timer);
APP_TIMER_DEF(m_charge_timer);

static uint32_t m_blink_count;
static uint32_t m_blink_mask;
static uint32_t m_prev_state;
static led_state=0;

/*****************************************************************************
 * Public API
 *****************************************************************************/


static void led_timeout_handler(void * p_context)
{
    APP_ERROR_CHECK_BOOL(m_blink_count > 0);
    if (led_state)
    {
        APP_ERROR_CHECK(drv_ext_light_off(1));
        led_state=0;
    }
    else
    {
        APP_ERROR_CHECK(drv_ext_light_on(1));
        led_state=1;
    }
    m_blink_count--;
    if (m_blink_count == 0)
    {
        (void) app_timer_stop(m_blink_timer);
        //if(led_state) APP_ERROR_CHECK(drv_ext_light_off(1));
    }
}

static void charge_timeout_handler(void * p_context)
{  m_blink_count=2;
   app_timer_start(m_blink_timer, APP_TIMER_TICKS(20), NULL);
}

void hal_led_blink_ms( uint32_t delay_ms, uint32_t blink_count)
{
    if (blink_count == 0 || delay_ms < HAL_LED_BLINK_PERIOD_MIN_MS)
    {
        return;
    }


    m_blink_count = blink_count * 2 - 1;
    if (app_timer_start(m_blink_timer, APP_TIMER_TICKS(delay_ms), NULL) == NRF_SUCCESS)
    {
          APP_ERROR_CHECK(drv_ext_light_on(1));
          led_state=1;
    }
}

void hal_led_blink_stop(void)
{
    (void) app_timer_stop(m_blink_timer);
    APP_ERROR_CHECK(drv_ext_light_off(1));
}
bool hal_led_pin_get(void)
{
    return led_state;
}

void hal_leds_init(void)
{
    APP_ERROR_CHECK(app_timer_create(&m_blink_timer, APP_TIMER_MODE_REPEATED, led_timeout_handler));
    APP_ERROR_CHECK(app_timer_create(&m_charge_timer, APP_TIMER_MODE_REPEATED, charge_timeout_handler));
   
}

void hal_led_pin_set(bool value)
{
    if (value)
    {
        APP_ERROR_CHECK(drv_ext_light_on(1));
        led_state= 1;
    }
    else
    {
        APP_ERROR_CHECK(drv_ext_light_off(1));
        led_state= 0;
    }
}
void led_breath_red(void)
{
    drv_ext_light_rgb_sequence_t seq=SEQUENCE_DEFAULT_VALUES;
    seq.color = DRV_EXT_LIGHT_COLOR_RED;
    seq.sequence_vals.on_time_ms = 500;
    seq.sequence_vals.on_intensity = 0xFF;
    seq.sequence_vals.off_time_ms = 40;
    seq.sequence_vals.off_intensity =5;
    seq.sequence_vals.fade_in_time_ms = 400;
    seq.sequence_vals.fade_out_time_ms = 400; 
    APP_ERROR_CHECK(drv_ext_light_rgb_sequence(1, &seq));
        
}
void led_breath_yellow(void)
{
    drv_ext_light_rgb_sequence_t seq=SEQUENCE_DEFAULT_VALUES;
    seq.color = DRV_EXT_LIGHT_COLOR_YELLOW;
    seq.sequence_vals.on_time_ms = 500;
    seq.sequence_vals.on_intensity = 0xFF;
    seq.sequence_vals.off_time_ms = 40;
    seq.sequence_vals.off_intensity =5;
    seq.sequence_vals.fade_in_time_ms = 400;
    seq.sequence_vals.fade_out_time_ms = 400; 
    APP_ERROR_CHECK(drv_ext_light_rgb_sequence(1, &seq));
        
}

void charging_indicate_start(void)
{
    (void) app_timer_start(m_charge_timer, APP_TIMER_TICKS(2000), NULL);
}
void charging_indicate_stop(void)
{
    (void) app_timer_stop(m_charge_timer);
}