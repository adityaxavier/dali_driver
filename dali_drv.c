/*
 *  dali_drv.c
 *
 *  Created on: Jun 21, 2013
 *  Author: Florian Feurstein
 *
 *  Description:
 *
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
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "mem_manager.h"
#include "nrf_drv_timer.h"

#define DALI_IN_PORT        12
#define DALI_OUT_PORT       13

//#define ENABLE_THIS

#ifdef ENABLE_THIS

/*
 * Microseconds from sending one bit to the next.
 * Frequency for DALI is 1200 Hz
 * -> 1s/1200 = 0.000833 s = 833 us = 833333 ns
 * Because of the Manchester (Bi-Phase) Coding where
 * two states are sent for one Bit, the ferquency
 * for the states doubles to 2400 Hz
 * -> 1s/2400 = 0.000416 s = 416 us = 416666 ns
 *
 */
#define DALI_DATA_SIZE      2
#define DALI_STOPBIT_VAL    3

typedef struct manchesterBitValList_t {
  struct manchesterBitValList_t *pNext;
  uint8_t bitVal;
}manchesterBitValList_t;

static manchesterBitValList_t* pBitValRoot = NULL;

const nrf_drv_timer_t TIMER_LED = NRF_DRV_TIMER_INSTANCE(1);

/*
 * dali_timerCB
 *
 * Timer callback function. Sends bit if available and restarts
 * timer if there are more bits to send in manchesterBitValList.
 *
 * @param hrtimer is the timer to be restarted
 * @return returns if the timer was restarted or not
 */
static void dali_timerCB(void)
{
  manchesterBitValList_t* pTemp = NULL;

  if(pBitValRoot != NULL)
  {
    nrf_gpio_pin_write(DALI_OUT_PORT, pBitValRoot->bitVal);
    NRF_LOG_INFO("Bit val %x", pBitValRoot->bitVal);

    if(pBitValRoot->pNext != NULL)
    {
      pTemp = pBitValRoot;
      pBitValRoot = pTemp->pNext;
      nrf_free(pTemp);
      pTemp = NULL;
      
      /*now = hrtimer_cb_get_time(&high_res_timer);
      hrtimer_forward(&high_res_timer,now , dali_freq_time);
      return HRTIMER_RESTART;*/
    }
    else
    {
  	  nrf_drv_timer_disable(&TIMER_LED);
      nrf_free(pBitValRoot);
      pBitValRoot = NULL;
    }
  }
}

/**
 * @brief Handler for timer events.
 */
void timer_led_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
            dali_timerCB();
            break;

        default:
            //Do nothing.
            break;
    }
}

/*
 * dali_manchesterListAddVal
 *
 * Adds a logical bit to the manchester list.
 *
 * @param val value of the logical bit to add to manchester list
 */
static void dali_manchesterListAddVal(uint8_t val)
{
  manchesterBitValList_t* pTemp = nrf_malloc(sizeof(manchesterBitValList_t));
  
  if(pTemp == NULL) {
  	NRF_LOG_INFO("f1");
  	return;
  }
  pTemp->pNext = nrf_malloc(sizeof(manchesterBitValList_t));

  if(pTemp->pNext == NULL) {
  	NRF_LOG_INFO("f2");
  	return;
  }

  pTemp->pNext->pNext = NULL;
  switch(val)
    {
      case 0:
        /*
         * logical 0 is the transition from 1 to 0
         * because we
         */
        pTemp->bitVal = 0;
        pTemp->pNext->bitVal = 1;
        break;

      case DALI_STOPBIT_VAL:
        pTemp->bitVal = 1;
        pTemp->pNext->bitVal = 1;
        break;

      default: //if it's bigger than 0, it's a one
        pTemp->bitVal = 1;
        pTemp->pNext->bitVal = 0;
      break;
    }

  if(pBitValRoot != NULL){
    pTemp->pNext->pNext = pBitValRoot;
  }
  pBitValRoot = pTemp;

}

/*
 * dali_manchesterListAddByte
 *
 * Manchester encode each bit of the byte and add it to
 * the manchesterBitValList
 *
 * @param byte data to encode and send
 *
 */
static void dali_manchesterListAddByte(char byte)
{
  int i = 0;

  for(i = 0; i < 8; i++)
  {
    dali_manchesterListAddVal(byte & (0x1 << i));
  }
}

/*
 * dali_write
 *
 * Write bytes from buffer to manchester list.
 *
 * @param F file to write to
 * @param buf buffer that should be written to manchester list
 * @param count bytes that should be written
 * @param f_pos indicates the file position to write from
 * @return size that was written
 */
uint32_t dali_write(const char *buf, size_t count)
{
 	char writeBuff[DALI_DATA_SIZE];

	memcpy(writeBuff, buf, DALI_DATA_SIZE);

	NRF_LOG_INFO("dali_write");

	/*
	 * Fill the Manchester list from behind (stop bits first, then data byte and address byte, start
	 * bit last) to get a properly ordered list (adding a node to the
	 * list adds it to the front!)
	 *
	 */

	/*
   * Send 2 stop bits (idle), no phase change for stop bits
   */
	dali_manchesterListAddVal(DALI_STOPBIT_VAL);
	dali_manchesterListAddVal(DALI_STOPBIT_VAL);

	/*
   * Send data byte
   */
	dali_manchesterListAddByte(writeBuff[1]);

	/*
   * Send address byte
   */
	dali_manchesterListAddByte(writeBuff[0]);

	/*
	 * Send start bit: logical 1 manchester code
	 */
	dali_manchesterListAddVal(1);

  nrf_drv_timer_enable(&TIMER_LED);

	return DALI_DATA_SIZE;
}

 
/*
 * dali_read
 *
 * Reads bits from GPIO defined in DALI_IN_PORT
 *
 * @param F file to be read
 * @param buf buffer to read data into
 * @param count bytes that should be read
 * @param f_pos indicates the file position the user is accessing
 * @return size that was read
 */
int dali_read(char *buf, size_t count)
{
	char buffer[10];
	 
	//implement as blocking read
	int temp = nrf_gpio_pin_read(DALI_IN_PORT);
	 
	sprintf( buffer, "%1d" , temp );
	 
	count = sizeof( buffer );
	 
	if( memcpy( buf, buffer, count ) )
	{
		return -1;
	}

	return 0;
 
} 

/*
 * init_dali
 *
 * Initialize the dali driver. Request and register device number,
 * create /sys/class, create device, init char device, init hrtimer,
 * init GPIO Ports
 *
 * @return success state, successfull if >= 0, else error
 */
int init_dali(void)
{

    nrf_mem_init();

    uint32_t time_us = 416; //Time(in miliseconds) between consecutive compare events.
    uint32_t time_ticks;
    uint32_t err_code = NRF_SUCCESS;

    //Configure all leds on board.
    bsp_board_leds_init();

    //Configure TIMER_LED for generating simple light effect - leds on board will invert his state one after the other.
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    err_code = nrf_drv_timer_init(&TIMER_LED, &timer_cfg, timer_led_event_handler);
    APP_ERROR_CHECK(err_code);

    time_ticks = nrf_drv_timer_us_to_ticks(&TIMER_LED, time_us);

    nrf_drv_timer_extended_compare(
         &TIMER_LED, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

	/*
	 * Initialize GPIO Ports
	 */
	nrf_gpio_cfg_output(DALI_OUT_PORT);
	nrf_gpio_cfg_input(DALI_IN_PORT, NRF_GPIO_PIN_NOPULL);

	return 0;
 
}
#endif
