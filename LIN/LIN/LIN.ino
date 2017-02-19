/*
 Name:		LIN.ino
 Created:	2/7/2017 7:21:41 PM
 Author:	Beltramello
*/




#include "Arduino.h"
#include "variant.h"
#include <stdbool.h>
#include <lin.h>
#include <conf_lin.h>
#include "assert.h"
#include <stdint.h>
#include <LINtc.h>

/** LIN frame number */
#define LIN_FRAME_ID_12       0x12

/** First byte for test */
#define LIN_FIRST_BYTE        0xFF

/** Local Buffer for emission */
uint8_t lin_data_node[8];

/** LIN master node number */
#define LIN_MASTER_NODE_NUM   0

/** Timer counter channel used */
#define TC_CHANNEL            0

/** Timer counter frequency in Hz */
#define TC_FREQ               5

/** LIN slave node number */
#define LIN_SLAVE_NODE_NUM    0



/**
* \brief lin_master_task_ID12
*/
static void  lin_master_task_ID12(void)
{
	lin_data_node[0] = LIN_FIRST_BYTE;
	lin_send_cmd(LIN_MASTER_NODE_NUM, LIN_FRAME_ID_12, sizeof(lin_data_node));
}

/**
* \brief Interrupt handler for TC0. Record the number of bytes received,
* and then restart a read transfer on the USART if the transfer was stopped.
*/
void TC0_Handler(void)
{
	/* Read TC0 Status. */
	TC_GetStatus(TC0, 0);
	st_lin_message lin_desc;
	uint8_t uc_key;

	/* Enable the peripheral clock in the PMC. */
	pmc_enable_periph_clk(ID_USART0);

	/* Node 0:  LIN_MASTER_MODE */
	lin_init(USART0, true, LIN_MASTER_NODE_NUM, 19200, VARIANT_MCK);

	/* Configure lin_descriptor */
	/* Init LIN data Node 0 */
	/* Object 0 */
	lin_desc.uc_id = LIN_FRAME_ID_12;
	lin_desc.uc_dlc = sizeof(lin_data_node);
	lin_desc.lin_cmd = PUBLISH;
	lin_desc.uc_status = 0;
	lin_desc.uc_pt_data = lin_data_node;
	lin_register_descriptor(LIN_MASTER_NODE_NUM, 0, &lin_desc);
	lin_master_task_ID12();
	
}



/**
* \brief Configure Timer Counter 0 (TC0) to generate an interrupt every 200ms.
* This interrupt will be used to flush USART input and echo back.
*/
static void configure_tc(void)
{
	uint32_t ul_div;
	uint32_t ul_tcclks;
	static uint32_t ul_sysclk;

	/* Get system clock. */
	ul_sysclk = VARIANT_MCK;

	/* Configure PMC. */
	pmc_enable_periph_clk(ID_TC0);

	/* Configure TC for a 50Hz frequency and trigger on RC compare. */
	TC_FindMckDivisor(TC_FREQ, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	TC_Configure(TC0, 0, ul_tcclks | TC_CMR_CPCTRG);
	TC_SetRC(TC0, 0, (ul_sysclk / ul_div) / TC_FREQ);

	/* Configure and enable interrupt on RC compare. */
	NVIC_EnableIRQ((IRQn_Type)ID_TC0);
	tc_enable_interrupt(TC0, 0, TC_IER_CPCS);
}

/**
* \brief Interrupt handler for USART. Echo the bytes received and start the
* next receive.
*/
void USART0_Handler(void)
{
	usart_lin_handler(LIN_SLAVE_NODE_NUM);
}

	
/**
* \brief lin_slave_task_ID12
*/
//static void  lin_slave_task_ID12(uint8_t *uc_buf)
//{
//	if (uc_buf[0] == LIN_FIRST_BYTE) {
//		lin_led1_counter++;
//		if (lin_led1_counter == LIN_LED0_WAIT_COUNTER) {
//			LED_Toggle(LED0_GPIO);
//			lin_led1_counter = 0;
//		}
//	}
//
//	LED_Toggle(LED1_GPIO);
//}


// the setup function runs once when you press reset or power the board
void setup()
{
		st_lin_message lin_desc;
		uint8_t uc_key;

		/* Enable the peripheral clock in the PMC. */
		pmc_enable_periph_clk(ID_USART0);

		/* Node 0:  LIN_MASTER_MODE */
		lin_init(USART0, true, LIN_MASTER_NODE_NUM, 9600, VARIANT_MCK);
		
				/* Configure lin_descriptor */
				/* Init LIN data Node 0 */
				/* Object 0 */
		lin_desc.uc_id = LIN_FRAME_ID_12;
		lin_desc.uc_dlc = sizeof(lin_data_node);
		lin_desc.lin_cmd = PUBLISH;
		lin_desc.uc_status = 0;
		lin_desc.uc_pt_data = lin_data_node;
		lin_register_descriptor(LIN_MASTER_NODE_NUM, 0, &lin_desc);
		configure_tc();
//		/* In case of Master Mode, the timing transmission starts. */
		TC_Start(TC0, 0);
//				
//
//				///* Node 0:  LIN_SLAVE_MODE */
//				//lin_init(USART0, false, LIN_SLAVE_NODE_NUM, 9600,
//				//	sysclk_get_cpu_hz());
//
//				///* Configure lin_descriptor */
//				///* Init LIN data Node 0 */
//				///* Object 0 */
//				//lin_desc.uc_id = LIN_FRAME_ID_12;
//				//lin_desc.uc_dlc = sizeof(lin_data_node);
//				//lin_desc.lin_cmd = SUBSCRIBE;
//				//lin_desc.uc_status = 0;
//				//lin_desc.uc_pt_data = lin_data_node;
//				//lin_desc.pt_function = lin_slave_task_ID12;
//				//lin_register_descriptor(LIN_SLAVE_NODE_NUM, 0, &lin_desc);
//
//				///* Configure and enable interrupt of USART. */
//				//NVIC_EnableIRQ(USART0_IRQn);
//				//usart_enable_interrupt(USART0, US_IER_LINID);
//				///* In case of Slave Mode, only wait for ID reception. */
//	
		
		/* Node 0:  LIN_MASTER_MODE */
		
}

// the loop function runs over and over again until power down or reset
void loop() 
{
	
}





//
///**
//* \brief This is an example demonstrating the LIN mode of USART IP
//* functionalities using a dedicated USART LIN driver.
//*/
//int main(void)
//{
//	st_lin_message lin_desc;
//	uint8_t uc_key;
//
//	
//	/* Enable the peripheral clock in the PMC. */
//	pmc_enable_periph_clk(ID_USART0);
//
//	/* Enable LIN transceiver */
//	gpio_set_pin_high(PIN_USART0_CTS_IDX);
//	gpio_set_pin_high(PIN_USART0_RTS_IDX);
//
//	
//
//	while (true) {
//		while (uart_read(CONSOLE_UART, &uc_key)) {
//		}
//
//		switch (uc_key) {
//		case 'm':
//		case 'M':
//			puts("-- Set LIN to Master mode\n\r");
//
//			/* Node 0:  LIN_MASTER_MODE */
//			lin_init(USART0, true, LIN_MASTER_NODE_NUM, 9600,
//				sysclk_get_cpu_hz());
//
//			/* Configure lin_descriptor */
//			/* Init LIN data Node 0 */
//			/* Object 0 */
//			lin_desc.uc_id = LIN_FRAME_ID_12;
//			lin_desc.uc_dlc = sizeof(lin_data_node);
//			lin_desc.lin_cmd = PUBLISH;
//			lin_desc.uc_status = 0;
//			lin_desc.uc_pt_data = lin_data_node;
//			lin_register_descriptor(LIN_MASTER_NODE_NUM, 0, &lin_desc);
//
//			configure_tc();
//			/* In case of Master Mode, the timing transmission starts. */
//			tc_start(TC0, 0);
//			break;
//
//		case 's':
//		case 'S':
//			puts("-- Set LIN to Slave mode\n\r");
//			/* Node 0:  LIN_SLAVE_MODE */
//			lin_init(USART0, false, LIN_SLAVE_NODE_NUM, 9600,
//				sysclk_get_cpu_hz());
//
//			/* Configure lin_descriptor */
//			/* Init LIN data Node 0 */
//			/* Object 0 */
//			lin_desc.uc_id = LIN_FRAME_ID_12;
//			lin_desc.uc_dlc = sizeof(lin_data_node);
//			lin_desc.lin_cmd = SUBSCRIBE;
//			lin_desc.uc_status = 0;
//			lin_desc.uc_pt_data = lin_data_node;
//			lin_desc.pt_function = lin_slave_task_ID12;
//			lin_register_descriptor(LIN_SLAVE_NODE_NUM, 0, &lin_desc);
//
//			/* Configure and enable interrupt of USART. */
//			NVIC_EnableIRQ(USART0_IRQn);
//			usart_enable_interrupt(USART0, US_IER_LINID);
//			/* In case of Slave Mode, only wait for ID reception. */
//			break;
//
//		case 'h':
//		case 'H':
//			display_menu();
//			break;
//		}
//		
//	}
//}
