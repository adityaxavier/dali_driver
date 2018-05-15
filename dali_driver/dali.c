// dali.c
//
// This module handles all tasks related to the dali port
//
// Prepared: Motorola AB
//
// Functional level: Hardware
//
// Revision: R1B
//
// Rev Date Reason/description
// P1A 001110 Initial version
// R1A 010212 Released version
// R1B 011010 Adapted to new clock speed, Reception improved to be more tolerant
//
// The timebase module (TBM) is used
#include "common.h"
//#include "iokx8.h"

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
#include "dali.h"

#define ENABLE_THIS

#ifdef ENABLE_THIS

// Address
#define BROADCAST 0xFF 				// Broadcast address for the DALI network

// Normal commands
#define GO_TO_SCENE 0x10 			// Command for changing scene
#define DEMO 0xFF 					// Command for starting the demo mode (not included in the DALI standard)

// Special commands
#define INITIALISE 0xA5 			// Command for starting initialization mode
#define RANDOMISE 0xA7 				// Command for generating a random address

static unsigned char send_position; // Position in the data to transfer
static unsigned char send_active; 	// True if transfer has started
static unsigned char send_value; 	// Holds the logic level to transfer
static unsigned char rec_position; 	// Position in the data to receive
static unsigned char rec_active; 	// True if reception has started
static unsigned char rec_value; 	// Holds the received logic level
static unsigned char rec_bit; 		// Number of received bits
static unsigned char demo_scene; 	// Holds the current scene
static unsigned int demo_tick; 		// Number of ticks since last scene change
static unsigned char repeat_active; // True if a command shall be repeated


#define DALI_IN_PORT        12
#define DALI_OUT_PORT       13

// Global variables declaration
unsigned char address; 	// The DALI address
unsigned char command; 	// The DALI command
unsigned char answer; 	// The DALI answer
unsigned char error; 	// Stores the last error code
unsigned int flag; 		// Keep track of all events

const nrf_drv_timer_t DALI_TIMER = NRF_DRV_TIMER_INSTANCE(1);

// Interrupt handler for timebase module
void dali_Tick(void)
{
	unsigned char temp_value;

	//TBCR |= 0x08; // Acknowledge the interrupt
	if (rec_active == TRUE)
	{
		//temp_value = PTB & 0x40;
		temp_value = nrf_gpio_pin_read(DALI_IN_PORT);
		rec_position++;
		if (temp_value != rec_value)
		{
			// An edge has been detected
			switch (rec_bit)
			{
				case 0:
					// Start bit
					rec_bit++;
					rec_position = 0;
					break;
				case 9:
					// First stop bit
					if (rec_position > 6)
					{
					// Stop bit error, no edge should exist, stop receiving
					rec_active = FALSE;
					//ISCR = 0x04; // Enable IRQ1 interrupt
					}
				break;
				case 10:
					// Second stop bit
					// Stop bit error, no edge should exist, stop receiving
					rec_active = FALSE;
					//ISCR = 0x04; // Enable IRQ1 interrupt
					break;
				default:
					// The address and command bits
					if (rec_position > 6)
					{
						// Store the values
						if (temp_value)
						{
							answer |= (1 << (8 - rec_bit));
						}
						rec_bit++;
						rec_position = 0;
					}
					break;
			}
			rec_value = temp_value;
		}
		else
		{
			// Signal level stable
			switch (rec_bit)
			{
				case 0:
					// Start bit
					if (rec_position == 8)
					{
						// Start bit error, too long delay before edge, stop receiving
						rec_active = FALSE;
						//ISCR = 0x04; // Enable IRQ1 interrupt
					}
					break;
				case 9:
					// First stop bit
					if (rec_position == 8)
					{
						if (temp_value == 0)
						{
							// Stop bit error, wrong level, stop receiving
							rec_active = FALSE;
							//ISCR = 0x04; // Enable IRQ1 interrupt
						}
						else
						{
							rec_bit++;
							rec_position = 0;
						}
					}
					break;
				case 10:
					// Second stop bit
					if (rec_position == 8)
					{
						// Receive ready
						flag |= ANSWER_EVENT;
						rec_active = FALSE;
						//ISCR = 0x04; // Enable IRQ1 interrupt
					}
					break;
				default:
					// The address and command bits
					if (rec_position == 10)
					{
						// Data bit error, too long delay before edge, stop receiving
						rec_active = FALSE;
						//ISCR = 0x04; // Enable IRQ1 interrupt
					}
					break;
			}
		}
	}
	if (send_active == TRUE)
	{
		if ((send_position & 0x03) == 0)
		{
			//PTB = (PTB & ~0x80) | send_value;
    		nrf_gpio_pin_write(DALI_OUT_PORT, send_value);

			if (send_position == 0)
			{
				send_value = 0x80; // Second half of start bit
			}
			if (send_position >= 4 && send_position <= 128)
			{
				// Extract bit level
				// Check if address or command
				if (send_position < 68)
				{
					// Address
					temp_value = (address >> ((64 - send_position) / 8)) & 0x01;
				}
				else
				{
					// Command
					temp_value = (command >> ((128 - send_position) / 8)) & 0x01;
				}

				// Check if first or second half of data bit
				if (send_position & 0x04)
				{
					// First half
					if (temp_value == 0x00)
					{
						send_value = 0x80;
					}
					else
					{
						send_value = 0x00;
					}
				}
				else
				{
					// Second half
					if (temp_value == 0x00)
					{
						send_value = 0x00;
					}
					else
					{
						send_value = 0x80;
					}
				}
			}
			if (send_position == 132)
			{
				send_value = 0x80; // Start of stop bit and settling time
			}
			if (send_position == 160)
			{
				//ISCR = 0x04; // Enable IRQ1 interrupt
			}
			if (send_position == 236)
			{
				send_active = FALSE;
				if (repeat_active == TRUE)
				{
					flag |= SEND_DATA;
				}
			}
		}
		send_position++;
	}

	if (flag & DEMO_MODE)
	{
		demo_tick++;
		
		// Change scene after 3 seconds
		if (demo_tick == 28800)
		{
			demo_tick = 0;
			demo_scene++;
			if (demo_scene>15)
			{
				demo_scene = 0;
			}
			address = BROADCAST;
			command = GO_TO_SCENE + demo_scene;
			flag |= NEW_DATA | SEND_DATA;
		}
	}
}

/**
 * @brief Handler for timer events.
 */
void dali_timer_cb(nrf_timer_event_t event_type, void* p_context)
{
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
            dali_Tick();
            break;

        default:
            //Do nothing.
            break;
    }
}

// Initialize
void dali_Init(void)
{
	send_active = FALSE;
	rec_active = FALSE;
	repeat_active = FALSE;
	//TBCR = 0x2E; // Interrupt 9600 times/second
	//ISCR = 0x04; // Enable IRQ1 interrupt

	/*
	 * Initialize GPIO Ports
	 */
	nrf_gpio_cfg_output(DALI_OUT_PORT);
	nrf_gpio_cfg_input(DALI_IN_PORT, NRF_GPIO_PIN_NOPULL);

    uint32_t time_us = 105; //Time(in microseconds) between consecutive compare events.
    uint32_t time_ticks;
    uint32_t err_code = NRF_SUCCESS;

    //Configure DALI_TIMER for generating simple light effect - leds on board will invert his state one after the other.
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    err_code = nrf_drv_timer_init(&DALI_TIMER, &timer_cfg, dali_timer_cb);
    APP_ERROR_CHECK(err_code);

    time_ticks = nrf_drv_timer_us_to_ticks(&DALI_TIMER, time_us);

    nrf_drv_timer_extended_compare(
         &DALI_TIMER, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

  	nrf_drv_timer_enable(&DALI_TIMER);
}

// Sends the DALI address and command on the dali port
void dali_SendData(void)
{
	// Check if it is the special command for starting the demo mode
	if (address == BROADCAST && command == DEMO)
	{
		demo_tick = 0;
		demo_scene = 0;
		address = BROADCAST;
		command = GO_TO_SCENE;
		flag |= DEMO_MODE | NEW_DATA;
	}

	// Check if some commands shall be repeated
	if (repeat_active == TRUE)
	{
		repeat_active = FALSE;
	}
	else
	{
		if (address & 0x01)
		{
			// Command
			if ((address & 0xE0) == 0xA0 || (address & 0xE0) == 0xC0)
			{
				// Special command
				if (address == INITIALISE || address == RANDOMISE)
				{
				// Command shall be repeated within 100 ms
				repeat_active = TRUE;
				}
			}
			else
			{
				// Normal command
				if (command >= 0x20 && command <= 0x80)
				{
					// Command shall be repeated within 100 ms
					repeat_active = TRUE;
				}
			}
		}
	}

	// Wait until dali port is idle
	while (send_active || rec_active);
	
	//ISCR = 0x06; // Disable IRQ1 interrupt
	send_value = 0x00;
	send_position = 0;
	send_active = TRUE; // Activate the timer module to transfer
}

// Interrupt handler for incoming data on the dali port
//@interrupt void dali_Start(void)
void dali_Start(void)
{
	//ISCR = 0x06; // Disable IRQ1 interrupt
	answer = 0x00; // Clear answer
	rec_bit = 0; // No bit has been received
	rec_value = 0x00; // Value is low when starting to receive
	rec_position = 0;
	rec_active = TRUE; // Activate the timer module to receive
}

#endif