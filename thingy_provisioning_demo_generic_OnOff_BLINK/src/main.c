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

#include "boards.h"
//#include "simple_hal.h"
#include "log.h"
#include "access_config.h"
#include "generic_on_off_server.h"
#include "generic_on_off_client.h"
#include "light_switch_example_common.h"
#include "mesh_app_utils.h"
#include "net_state.h"
#include "rtt_input.h"
#include "mesh_stack.h"
#include "my_mesh_provisionee.h"

#include "nrf_mesh_configure.h"
#include "nrf_mesh_config_examples.h"
#include "mesh_adv.h"
#include "nrf_delay.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "sdk_config.h"
#include "proxy.h"
#include "drv_ext_light.h"
#include "drv_ext_gpio.h"
#include "twi_manager.h"
#include "m_ui.h"
#include "support_func.h"
#include "pca20020.h"
#include "app_timer.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#define THINGY_BUTTON 11
#define DEVICE_NAME "Thingy"
#define MIN_CONN_INTERVAL MSEC_TO_UNITS(250, UNIT_1_25_MS)
#define MAX_CONN_INTERVAL MSEC_TO_UNITS(1000, UNIT_1_25_MS)
#define SLAVE_LATENCY     0
#define CONN_SUP_TIMEOUT  MSEC_TO_UNITS(4000, UNIT_10_MS)
#define GROUP_MSG_REPEAT_COUNT   (5)
#define BUTTON_PIN_CONFIG ((GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos)     | \
                           (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)    | \
                           (BUTTON_PULL << GPIO_PIN_CNF_PULL_Pos)                 | \
                           (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) | \
                           (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos))
#define GPIOTE_IRQ_LEVEL NRF_MESH_IRQ_PRIORITY_LOWEST   
#define HAL_BUTTON_PRESS_FREQUENCY  HAL_MS_TO_RTC_TICKS(400)
static uint32_t m_last_button_press;
static int8_t number_of_blink = 0;
static void on_sd_evt(uint32_t sd_evt, void * p_context)
{
    (void) nrf_mesh_on_sd_evt(sd_evt);
}


APP_TIMER_DEF(m_blink_timer_id);                                  /**< Battery timer. */

#define BLINK_INTERVAL         APP_TIMER_TICKS(700)                   /**< Battery level measurement interval (ticks). */

NRF_SDH_SOC_OBSERVER(mesh_observer, NRF_SDH_BLE_STACK_OBSERVER_PRIO, on_sd_evt, NULL);

#define RTT_INPUT_POLL_PERIOD_MS (100)
#define LED_PIN_NUMBER (BSP_LED_0)
#define LED_PIN_MASK   (1u << LED_PIN_NUMBER)
#define LED_BLINK_INTERVAL_MS    (200)
#define LED_BLINK_CNT_START      (2)
#define LED_BLINK_CNT_RESET      (3)
#define LED_BLINK_CNT_PROV       (4)

static generic_on_off_server_t m_server;
static generic_on_off_client_t m_client;
static bool                   m_device_provisioned;
static bool m_led_flag= 0;
static bool m_on_off_button_flag= 0;
int LED_blink(void)
{
    ERROR_CHECK(drv_ext_light_on(1));
    nrf_delay_ms(100);  
   ERROR_CHECK(drv_ext_light_off(1));
}
static void provisioning_complete_cb(void)
{
    dsm_local_unicast_address_t node_address;
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Successfully provisioned\n");
    dsm_local_unicast_addresses_get(&node_address);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Node Address: 0x%04x \n", node_address.address_start);
      drv_ext_light_rgb_sequence_t seq=SEQUENCE_DEFAULT_VALUES;
    seq.color = DRV_EXT_LIGHT_COLOR_GREEN;
    seq.sequence_vals.on_time_ms = 500;
    seq.sequence_vals.on_intensity = 0x80;
    seq.sequence_vals.off_time_ms = 40;
    seq.sequence_vals.off_intensity =5;
    seq.sequence_vals.fade_in_time_ms = 400;
    seq.sequence_vals.fade_out_time_ms = 400; 
    ERROR_CHECK(drv_ext_light_rgb_sequence(1, &seq));
    nrf_delay_ms(300);  

}

static void provisioning_failed_cb(void)
{
    drv_ext_light_rgb_sequence_t seq=SEQUENCE_DEFAULT_VALUES;
    seq.color = DRV_EXT_LIGHT_COLOR_RED;
    seq.sequence_vals.on_time_ms = 500;
    seq.sequence_vals.on_intensity = 0xFF;
    seq.sequence_vals.off_time_ms = 40;
    seq.sequence_vals.off_intensity =5;
    seq.sequence_vals.fade_in_time_ms = 400;
    seq.sequence_vals.fade_out_time_ms = 400; 
    
    ERROR_CHECK(drv_ext_light_rgb_sequence(1, &seq));
   
}

static void provisioning_start_cb(void)
{
  ERROR_CHECK(drv_ext_light_off(1));
   
}

static void provisioning_blink_output_cb(uint8_t * number)

{   uint32_t err_code;
    ERROR_CHECK(drv_ext_light_off(1));
     __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Blink OOB %u\n", number[15]);
     //The OOB data only use last byte to set the number of blink
     //Start the blinking timer
    number_of_blink= number[15];
    err_code = app_timer_start(m_blink_timer_id, BLINK_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}
static bool on_off_server_get_cb(const generic_on_off_server_t * p_server)
{
  return m_led_flag;
}

static bool on_off_server_set_cb(const generic_on_off_server_t * p_server, bool value)
{
   uint32_t err_code;
   __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Got SET command to %u\n", value);
   if (value)
   {
       ERROR_CHECK(drv_ext_light_on(1));
       m_led_flag=1;
    }
    else
    {
       ERROR_CHECK(drv_ext_light_off(1));
       m_led_flag=0;
    }
    
    return value;
}

static void node_reset(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- Node reset  -----\n");
    /* This function may return if there are ongoing flash operations. */
    mesh_stack_device_reset();
    
}

static void config_server_evt_cb(const config_server_evt_t * p_evt)
{
    if (p_evt->type == CONFIG_SERVER_EVT_NODE_RESET)
    {
        node_reset();
    }
    if (p_evt->type == CONFIG_SERVER_EVT_MODEL_SUBSCRIPTION_ADD)
    {
        LED_blink();
    }
    if (p_evt->type == CONFIG_SERVER_EVT_MODEL_PUBLICATION_SET)
    {
        LED_blink();
    }
}
static bool client_publication_configured(void)
{
    dsm_handle_t pub_addr_handle;
  
        if (access_model_publish_address_get(m_client.model_handle, &pub_addr_handle) == NRF_SUCCESS)
        {
            if (pub_addr_handle == DSM_HANDLE_INVALID)
            {
                return false;
            }
        }
        else
        {
            return false;
        }
    

    return true;
}
static void button_event_handler(uint32_t button_number)
{
      __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Button %u pressed\n", button_number);
    if (client_publication_configured())
    {
        uint32_t status = NRF_SUCCESS;
        switch (button_number)
        {
            case 0:
            case 1:
              
               
               /* send a group message to the ODD group, with flip the current button flag value */
                m_on_off_button_flag=!m_on_off_button_flag; 
                status = generic_on_off_client_set_unreliable(&m_client,
                                                             m_on_off_button_flag,
                                                            GROUP_MSG_REPEAT_COUNT);
                                                         
               // hal_led_pin_set(BSP_LED_0 + button_number, !hal_led_pin_get(BSP_LED_0 + button_number));
                break;
            default:
                break;

        }

    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Ignored. Node is not configured.\n");
    }
   
}

static void app_rtt_input_handler(int key)
{
    if (key >= '0' && key <= '4')
    {
        uint32_t button_number = key - '0';
        button_event_handler(button_number);
    }
}
static void client_status_cb(const generic_on_off_client_t * p_self, generic_on_off_status_t status, uint16_t src)
{

    __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "server status received \n");
    switch (status)
    {
        case GENERIC_ON_OFF_STATUS_ON:
            m_on_off_button_flag=1;   

            break;

        case GENERIC_ON_OFF_STATUS_OFF:
            m_on_off_button_flag=0;

            break;

        case GENERIC_ON_OFF_STATUS_ERROR_NO_REPLY:

            break;

        case GENERIC_ON_OFF_STATUS_CANCELLED:
        default:
            __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Unknown status \n");
            break;
    }
}
static void client_publish_timeout_cb(access_model_handle_t handle, void * p_self)
{
     __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Acknowledged send timedout\n");
}

static void models_init_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing and adding models\n");
    m_server.get_cb = on_off_server_get_cb;
    m_server.set_cb = on_off_server_set_cb;
    ERROR_CHECK(generic_on_off_server_init(&m_server, 0));
    ERROR_CHECK(access_model_subscription_list_alloc(m_server.model_handle));
    //Initialize client on model 1
    m_client.status_cb = client_status_cb;
    m_client.timeout_cb = client_publish_timeout_cb;
    ERROR_CHECK(generic_on_off_client_init(&m_client, 1));
    ERROR_CHECK(access_model_subscription_list_alloc(m_client.model_handle));

}

static void mesh_init(void)
{
    uint8_t dev_uuid[NRF_MESH_UUID_SIZE];
    uint8_t node_uuid_prefix[SERVER_NODE_UUID_PREFIX_SIZE] = SERVER_NODE_UUID_PREFIX;

    ERROR_CHECK(mesh_app_uuid_gen(dev_uuid, node_uuid_prefix, SERVER_NODE_UUID_PREFIX_SIZE));
    mesh_stack_init_params_t init_params =
    {
        .core.irq_priority       = NRF_MESH_IRQ_PRIORITY_LOWEST,
        .core.lfclksrc           = DEV_BOARD_LF_CLK_CFG,
        .core.p_uuid             = dev_uuid,
        .models.models_init_cb   = models_init_cb,
        .models.config_server_cb = config_server_evt_cb
    };
    ERROR_CHECK(mesh_stack_init(&init_params, &m_device_provisioned));
}

static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;
    
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

static void initialize(void)
{
    __LOG_INIT(LOG_SRC_APP | LOG_SRC_ACCESS, LOG_LEVEL_DBG3, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Light Switch Proxy Server Demo -----\n");

   // hal_leds_init();
#if BUTTON_BOARD
  //  ERROR_CHECK(hal_buttons_init(button_event_handler));
#endif
    uint32_t err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    uint32_t ram_start = 0;
    /* Set the default configuration (as defined through sdk_config.h). */
    err_code = nrf_sdh_ble_default_cfg_set(MESH_SOFTDEVICE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    gap_params_init();

    mesh_init();
}

static void start(void)
{
    rtt_input_enable(app_rtt_input_handler, RTT_INPUT_POLL_PERIOD_MS);
    ERROR_CHECK(mesh_stack_start());
    
    if (!m_device_provisioned)
    {
        static const uint8_t static_auth_data[NRF_MESH_KEY_SIZE] = STATIC_AUTH_DATA;
        mesh_provisionee_start_params_t prov_start_params =
        {
            .p_static_data    = static_auth_data,
            .prov_complete_cb = provisioning_complete_cb,
            .auth_output_cb = provisioning_blink_output_cb, 
            .prov_started_cb = provisioning_start_cb,
            .prov_failed_cb= provisioning_failed_cb
        };
        ERROR_CHECK(mesh_provisionee_prov_start(&prov_start_params));
               
        drv_ext_light_rgb_sequence_t seq=SEQUENCE_DEFAULT_VALUES;
        seq.color = DRV_EXT_LIGHT_COLOR_RED;
        seq.sequence_vals.on_time_ms = 500;
        seq.sequence_vals.on_intensity = 0xFF;
        seq.sequence_vals.off_time_ms = 40;
        seq.sequence_vals.off_intensity =5;
        seq.sequence_vals.fade_in_time_ms = 400;
        seq.sequence_vals.fade_out_time_ms = 400; 
        
        ERROR_CHECK(drv_ext_light_rgb_sequence(1, &seq));
        
        
    }else
    {  
        //erase bond information if button pressed when reset
        nrf_gpio_cfg_input(THINGY_BUTTON,GPIO_PIN_CNF_PULL_Pullup);
        if (nrf_gpio_pin_read(THINGY_BUTTON)==0)
        {
      
            proxy_disable();
            mesh_stack_config_clear();
            nrf_delay_ms(500);
            node_reset();
        }
        ERROR_CHECK(drv_ext_light_on(1));
        nrf_delay_ms(100);
       ERROR_CHECK(drv_ext_light_off(1));
 
    }
    const uint8_t *p_uuid = nrf_mesh_configure_device_uuid_get();
    __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "Device UUID ", p_uuid, NRF_MESH_UUID_SIZE);
}


void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
   
    
    // On assert, the system can only recover with a reset.
    #ifndef DEBUG
        NVIC_SystemReset();
    #endif

    app_error_save_and_stop(id, pc, info);
}
static const nrf_drv_twi_t     m_twi_sensors = NRF_DRV_TWI_INSTANCE(TWI_SENSOR_INSTANCE);

uint32_t hal_buttons_init(void)
{


    
    NRF_GPIO->PIN_CNF[THINGY_BUTTON] = BUTTON_PIN_CONFIG;
   

    NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Msk;
    NRF_GPIOTE->EVENTS_PORT  = 0;

    NVIC_SetPriority(GPIOTE_IRQn, GPIOTE_IRQ_LEVEL);
    NVIC_EnableIRQ(GPIOTE_IRQn);
    return NRF_SUCCESS;

}
void GPIOTE_IRQHandler(void)
{
    NRF_GPIOTE->EVENTS_PORT = 0;
   
        /* Check that the event was generated by a button press, and reject if it's too soon (debounce).
         * NOTE: There is a bug with this at the wrap-around for the RTC0 where the button could be
         * pressed before HAL_BUTTON_PRESS_FREQUENCY has passed a single time. It doesn't matter practically.
         */

   if( TIMER_DIFF(m_last_button_press, NRF_RTC0->COUNTER) > HAL_BUTTON_PRESS_FREQUENCY)
    {
        m_last_button_press = NRF_RTC0->COUNTER;
        button_event_handler(0);
    }
    
}

uint32_t m_my_ui_init( void)
{
    uint32_t                        err_code;
    static drv_sx1509_cfg_t         sx1509_cfg;
    drv_ext_light_init_t            led_init;
    //lint --e{651} Potentially confusing initializer
    static const drv_ext_light_conf_t led_conf[DRV_EXT_LIGHT_NUM] = DRV_EXT_LIGHT_CFG;

    static const nrf_drv_twi_config_t twi_config =
    {
        .scl                = TWI_SCL,
        .sda                = TWI_SDA,
        .frequency          = NRF_TWI_FREQ_100K,
        .interrupt_priority = APP_IRQ_PRIORITY_LOW
    };
    sx1509_cfg.twi_addr       = SX1509_ADDR;
    sx1509_cfg.p_twi_instance = &m_twi_sensors;
    sx1509_cfg.p_twi_cfg      = &twi_config;



    led_init.p_light_conf        = led_conf;
    led_init.num_lights          = DRV_EXT_LIGHT_NUM;
    led_init.clkx_div            = DRV_EXT_LIGHT_CLKX_DIV_8;
    led_init.p_twi_conf          = &sx1509_cfg;
    led_init.resync_pin          = SX_RESET;

    err_code = drv_ext_light_init(&led_init, false);
    APP_ERROR_CHECK(err_code);

    ERROR_CHECK(drv_ext_light_off(DRV_EXT_RGB_LED_SENSE));
    ERROR_CHECK(drv_ext_light_off(DRV_EXT_RGB_LED_LIGHTWELL));
    
    nrf_gpio_cfg_output(MOS_1);
    nrf_gpio_cfg_output(MOS_2);
    nrf_gpio_cfg_output(MOS_3);
    nrf_gpio_cfg_output(MOS_4);
    nrf_gpio_pin_clear(MOS_1);
    nrf_gpio_pin_clear(MOS_2);
    nrf_gpio_pin_clear(MOS_3);
    nrf_gpio_pin_clear(MOS_4);
    
 
    return NRF_SUCCESS;
}


/**
 * @brief Function for application main entry.
 */

static void board_init(void)
{
    uint32_t            err_code;
    drv_ext_gpio_init_t ext_gpio_init;


    static const nrf_drv_twi_config_t twi_config =
    {
        .scl                = TWI_SCL,
        .sda                = TWI_SDA,
        .frequency          = NRF_TWI_FREQ_400K,
        .interrupt_priority = APP_IRQ_PRIORITY_LOW
    };

    static const drv_sx1509_cfg_t sx1509_cfg =
    {
        .twi_addr       = SX1509_ADDR,
        .p_twi_instance = &m_twi_sensors,
        .p_twi_cfg      = &twi_config
    };

    ext_gpio_init.p_cfg = &sx1509_cfg;
    
    err_code = support_func_configure_io_startup(&ext_gpio_init);
    APP_ERROR_CHECK(err_code);
}

static void blink_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    
    ERROR_CHECK(drv_ext_light_on(1));
    
    nrf_delay_ms(100);  
    ERROR_CHECK(drv_ext_light_off(1));
    number_of_blink--;
    if (number_of_blink<=0) 
    {
     
         APP_ERROR_CHECK(app_timer_stop(m_blink_timer_id));
     }
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    ret_code_t err_code;

    // Initialize timer module.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers.
    err_code = app_timer_create(&m_blink_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                blink_timeout_handler);
    APP_ERROR_CHECK(err_code);

  
}

int main(void)
{
    uint32_t err_code;
    

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    timers_init();
    board_init();
    err_code= m_my_ui_init();
    APP_ERROR_CHECK(err_code);
    initialize();
    execution_start(start);
    hal_buttons_init();
   
    for (;;)
    {
        (void)sd_app_evt_wait();
    }
}
