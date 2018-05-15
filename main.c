/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
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
 * 
 */
/** @file
 *
 * @defgroup ble_sdk_app_beacon_main main.c
 * @{
 * @ingroup ble_sdk_app_beacon
 * @brief Beacon Transmitter Sample Application main file.
 *
 * This file contains the source code for an Beacon transmitter sample application.
 */

#include <stdbool.h>
#include <stdint.h>
#include "nordic_common.h"
#include "bsp.h"
#include "nrf_soc.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "ble_advdata.h"
#include "app_timer.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_delay.h"
#include "mem_manager.h"
#include "dali.h"
#include "common.h"


#define APP_BLE_CONN_CFG_TAG            1                                 /**< A tag identifying the SoftDevice BLE configuration. */

#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(30, UNIT_0_625_MS) /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */

#define DEAD_BEEF                       0xDEADBEEF                        /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */


static ble_gap_adv_params_t m_adv_params;                                 /**< Parameters to be passed to the stack when starting advertising. */

static uint8_t m_beacon_info[24];
 APP_TIMER_DEF(m_dali_timer_id);  
 
 


extern int test_ecc(void);
extern int init_dali(void);
extern uint32_t dali_write(const char *buf, size_t count);
extern int dali_read(char *buf, size_t count);


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

#if 1
#define ECB_KEY_LENGTH     (16)
#define SHARED_KEY_LENGTH  (32)
#define MAX_HASH_SIZE      (32)
#define NONCE_LENGTH       (16)
#define MAX_DATA_LENGTH    (16)

typedef struct __attribute__ ((__packed__))
{ 
    uint8_t site_key[ECB_KEY_LENGTH];       /* Site/AES-128 key */
} security_keys_t;

static security_keys_t m_sec; /* security context variable */
static void aes128_hw_ecb(uint8_t const *in, uint16_t len, const uint8_t *key, uint8_t *encr) 
{
    nrf_ecb_hal_data_t aes_data;    /* Data for HW AES Block(via SoftDevice) */

    for (uint32_t i = 0; i < ECB_KEY_LENGTH; i++)
        aes_data.key[i] = key[ECB_KEY_LENGTH - 1 - i];

    if(len > SOC_ECB_CLEARTEXT_LENGTH)
        len = SOC_ECB_CLEARTEXT_LENGTH;

    memset(aes_data.cleartext, 0, len);

    for (uint32_t i = 0; i < len; i++)
        aes_data.cleartext[len - 1 - i] = in[i];

    // Can only return NRF_SUCCESS.
    (void) sd_ecb_block_encrypt(&aes_data);

    for (uint32_t i = 0; i < len; i++)
        encr[i] = aes_data.ciphertext[len - 1 - i];
}

static ret_code_t __encrypt_decrypt(const uint8_t *data, int data_length, uint8_t * out, uint32_t nonce)
{
    uint8_t nonce_128[NONCE_LENGTH]   = {0};
    uint8_t encr[MAX_DATA_LENGTH] = {0};

    // copy the nonce and append the rest with zero's
    memcpy(nonce_128, &nonce, sizeof(nonce));

    if(data_length > MAX_DATA_LENGTH)
        data_length = MAX_DATA_LENGTH;

#ifdef TINY_AES128
    AES128_ECB_encrypt(nonce, m_sec.site_key, encr);
#else
    aes128_hw_ecb(nonce_128, data_length, m_sec.site_key, encr);
#endif

    /* Hash the aes-128 with the data to encrypt */
    for(int i = 0; i < data_length; i++)
        out[i] = encr[i] ^ data[i];

    return NRF_SUCCESS;
}
#endif


unsigned char crc8(unsigned char *bytes, unsigned int length)
{
  const unsigned char generator = 0x1D;
  unsigned char crc = 0; /* start with 0 so first byte can be 'xored' in */

  for (unsigned char i = 0; i < length; i++)
  {
    crc ^= bytes[i]; /* XOR-in the next input byte */

    for (unsigned char i = 0; i < 8; i++)
    {
      if ((crc & 0x80) != 0)
        crc = (unsigned char)((crc << 1) ^ generator);
      else
        crc <<= 1;
    }
  }

  return crc;
}

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    uint8_t       flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;
    uint8_t encrypted[16] = {0};

    ble_advdata_manuf_data_t manuf_specific_data;
    manuf_specific_data.company_identifier = 0x0346;

    memset(m_beacon_info, 0, sizeof(m_beacon_info));
    uint8_t index = 0;

    // src
    m_beacon_info[index++] = 0x01;
    m_beacon_info[index++] = 0x00;
    
    // dst
    m_beacon_info[index++] = 0x00;
    m_beacon_info[index++] = 0xC0;
    
    // ttl + channel
    m_beacon_info[index++] = 0x6A;
    
    // nonce
    uint32_t nonce = 0x1236;
    m_beacon_info[index++] = nonce;
    m_beacon_info[index++] = (uint8_t)(nonce >> 8);
    m_beacon_info[index++] = (uint8_t)(nonce >> 16);
    
    //lighting control command
    m_beacon_info[index++] = 0x00;
    m_beacon_info[index++] = 0x04;
    float *ptr = (float*)&m_beacon_info[index];
    *ptr = (float)5.0;
    index += 4;
    while(index < 22) m_beacon_info[index++] = 00;
    m_beacon_info[23] = crc8(&m_beacon_info[8], 15);

    // Print Mesh message
    //NRF_LOG_INFO("Message Data");
    //for(int i = 0; i < 16; i++)
        //NRF_LOG_INFO(" %02x", m_beacon_info[8+i]);

    //dummy AES key;
    for(int i = 0; i < 16; i++)
        m_sec.site_key[i] = i+1;

    // Encrypt the message
    __encrypt_decrypt(&m_beacon_info[8], 16, &encrypted[0], nonce);
    memset(&m_beacon_info[8], 0, 16);
    memcpy(&m_beacon_info[8], &encrypted[0], 16);

    // Verify the message decryption 
    memset(&encrypted[8], 0, 16);
    __encrypt_decrypt(&m_beacon_info[8], 16, &encrypted[0], nonce);
    //NRF_LOG_INFO("Decrypted Data");
    //for(int i = 0; i < 16; i++)
       // NRF_LOG_INFO(" %02x", encrypted[i]); 

    // Fill the Advertisment packet
    manuf_specific_data.data.p_data = (uint8_t *) m_beacon_info;
    manuf_specific_data.data.size   = 24;

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type             = BLE_ADVDATA_NO_NAME;
    advdata.flags                 = flags;
    advdata.p_manuf_specific_data = &manuf_specific_data;

    err_code = ble_advdata_set(&advdata, NULL);
    APP_ERROR_CHECK(err_code);

    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_NONCONN_IND;
    m_adv_params.p_peer_addr = NULL;    // Undirected advertisement.
    m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval    = NON_CONNECTABLE_ADV_INTERVAL;
    m_adv_params.timeout     = 0;       // Never time out.   
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    ret_code_t err_code;

    err_code = sd_ble_gap_adv_start(&m_adv_params, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing logging. */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing LEDs. */
static void leds_init(void)
{
    ret_code_t err_code = bsp_init(BSP_INIT_LED, NULL);
    APP_ERROR_CHECK(err_code);
}

static void DALI_timeout_handler(void * p_context)
{
    //const char word[] = {0x00, 0xFF};
    //dali_write(&word[0], 2);

    address = 0x00;
    command = 0xFF;
    dali_SendData();
}


/**@brief Function for initializing timers. */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create Security Request timer.
    err_code = app_timer_create(&m_dali_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                DALI_timeout_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for doing power management. */
static void power_manage(void)
{
    return;
    ret_code_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{

    // Initialize.
    log_init();
    timers_init();
    leds_init();
    ble_stack_init();
    advertising_init();

    advertising_start();

    // Start execution.
    NRF_LOG_INFO("Beacon example started.");

    /*
     * Initialize memory manager
     */

    //init_dali();
    dali_Init(); 

    ret_code_t err_code;
    err_code = app_timer_start(m_dali_timer_id, APP_TIMER_TICKS(1500), NULL);
    APP_ERROR_CHECK(err_code);

    // Enter main loop.
    for (;; )
    {
        if (NRF_LOG_PROCESS() == false)
        {
            power_manage();
        }
        __WFI();
    }
}


/**
 * @}
 */
