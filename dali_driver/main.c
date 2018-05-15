// main.c
//
// This module contains the main function
//
// Prepared: Motorola AB
//
// Functional level: Application
//
// Revision: R1A
//
// Rev Date Reason/description
// P1A 001023 Initial version
// R1A 010212 Released version
#include "common.h"
#include "iokx8.h"
#include "cpu.h"
#include "lcd.h"
#include "keys.h"
#include "rs232.h"
#include "dali.h"
// Global variables declaration
unsigned char address; // The DALI address
unsigned char command; // The DALI command
unsigned char answer; // The DALI answer
unsigned char error; // Stores the last error code
unsigned int flag; // Keep track of all events

// Main function
void main(void)
{
	address = 0x00; // Initialize when starting
	command = 0x00; // Initialize when starting
	answer = 0x00; // Initialize when starting
	error = 0x00; // Initialize when starting
	flag = 0x00; // Initialize when starting

	cpu_Init(); // Initialize the cpu module
	lcd_Init(); // Initialize the lcd module
	keys_Init(); // Initialize the keys module

	rs232_Init(); // Initialize the keys module
	dali_Init(); // Initialize the dali module
	ei(); // Enable interrupt
	
	while (1==1) // Loop forever
	{
		COPCTL = 0x00; // Clear COP counter
		// NEW_DATA flag can be set from the keys module,
		// the rs232 module or the dali module
		if (flag & NEW_DATA)
		{
			flag &= ~NEW_DATA; // Clear NEW_DATA flag
			lcd_ShowData();
			// Update lcd with new address and command
		}
		// SEND_DATA flag can be set from the keys module,
		// the rs232 module or the dali module
		if (flag & SEND_DATA)
		{
			flag &= ~SEND_DATA; // Clear SEND_DATA flag
			dali_SendData();
			// Send address and command on the dali port
		}
		// ANSWER_EVENT flag can be set from the dali module
		if (flag & ANSWER_EVENT)
		{
			flag &= ~ANSWER_EVENT; // Clear ANSWER_EVENT flag
			lcd_ShowAnswer(); // Update the lcd with the answer
			rs232_SendAnswer(); // Send the answer on the rs232 port
		}
		// ERROR_EVENT flag can be set from any module
		if (flag & ERROR_EVENT)
		{
			flag &= ~ERROR_EVENT; // Clear ERROR_EVENT flag
			lcd_ShowError(); // Update the lcd with the error code
		}
	}
}