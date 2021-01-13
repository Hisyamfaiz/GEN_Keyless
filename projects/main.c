/*
* An nRF24LE1 REGISTER RETENTION TIMER ON example application
* Master
*/

// ======================================= Include header
#include <Nordic\reg24le1.h>
#include "hal_nrf.h"
#include "hal_clk.h"
#include "hal_rtc.h"
#include "hal_delay.h"
#include "hal_wdog.h"
#include "hal_aes.h"
#include "hal_rng.h"
#include "hal_flash.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// ======================================= Type definition
#define TRANSISTOR						P10
#define LED_1									P11
#define LED_2									P13
#define BTN_ALARM							P03
#define BTN_SEAT							P01

#define VADDR_VCU_ID					HAL_DATA_NV_BASE_ADDRESS
#define VADDR_AES_KEY					(VADDR_VCU_ID + sizeof(uint32_t))

#define DATA_LENGTH 					16
#define ADDR_LENGTH						5
#define DATA_PAIR_LENGTH		 (DATA_LENGTH + ADDR_LENGTH)

typedef enum {
	KLESS_CMD_PING 	= 0,
	KLESS_CMD_ALARM = 1,
	KLESS_CMD_SEAT 	= 2
} KLESS_CMD;

// ======================================= Global variable
static bool volatile radio_busy, received;
static uint8_t tx_address[ADDR_LENGTH] = {0x00, 0x00, 0x00, 0x00, 0xCD};
static uint8_t rx_address[ADDR_LENGTH] = {0x00, 0x00, 0x00, 0x00, 0xAB};
static uint8_t xdata payload[DATA_PAIR_LENGTH];
static uint8_t xdata payload_enc[DATA_LENGTH];
static uint32_t idata AES_Key[4];
static const uint8_t idata commands[3][8] = {
	{ 0x5e, 0x6c, 0xa7, 0x74, 0xfd, 0xe3, 0xdf, 0xbc },
	{ 0xf7, 0xda, 0x4a, 0x4f, 0x65, 0x2d, 0x6e, 0xf0 },
	{ 0xff, 0xa6, 0xe6, 0x5a, 0x84, 0x82, 0x66, 0x4f }
};

// ======================================= Function prototype 
void pin_init(void);
void rtc_init(void);
void clock_and_irq_init(void);
void nrf_init(void);
void sleep_mode(void);
void make_random_number(uint8_t *p);
void make_command(KLESS_CMD *cmd, hal_nrf_output_power_t *pwr);
void make_payload(uint8_t *payload, uint8_t cmd);
void send_payload(uint8_t *payload, uint8_t pwr, uint8_t retry);
void set_pairing_mode(void);
void receive_pairing(void);
void update_configuration(uint8_t *success);
void wait_button_released(void);
bool receive_ping(uint8_t timeout);
void set_normal_mode(void);
void save_flash(void);
void load_flash(void);

// ======================================= Main function 
void main(void){
	// local variable
	uint8_t pairing_success;
	KLESS_CMD command;
	hal_nrf_output_power_t power;
	
	// Initialise GPIO
	pin_init();
	// Initialise RTC
	rtc_init();
	// Initialise clock & irq
	clock_and_irq_init();
	
	// Load Flash
	load_flash();
	
	// Initialise RF module
	nrf_init();
	// Initialise watchdog
	hal_wdog_init(0x0300);

	while(1)	{						
		if(BTN_ALARM || BTN_SEAT){
			// handle bounce effect
			delay_ms(100);
				
			if(BTN_ALARM && BTN_SEAT){
				// Pairing Mode
				pairing_success = 0;
				
				// Set to pairing configuration
				set_pairing_mode();
				// Receive mode
				receive_pairing();
				// Check Payload
				update_configuration(&pairing_success);
				// indicator result
				if(pairing_success){
					LED_1 = 0;
					LED_2 = 0;
				} else {
					LED_1 = 1;
					LED_2 = 1;
				}
				
				set_normal_mode();
				
			// Wait until button released
			wait_button_released();
				
			} else if(BTN_ALARM){
				// Button Command Mode				
				command = KLESS_CMD_ALARM;
				power = HAL_NRF_0DBM;
				// Generate Random Number
				//make_random_number(&payload[DATA_LENGTH/2]);
				// Insert command to payload
				make_payload(payload, command);
				// Encrypt payload
				hal_aes_crypt(payload_enc, payload);
				// Send the payload
				send_payload(payload_enc, power, 1);		
				// indicator
				LED_2 = !LED_2;
			}
		} 
		
			// Normal Mode
			if (receive_ping(10) && !BTN_ALARM){
				// Generate Command				
				make_command(&command, &power);
				
				// Insert command to payload
				make_payload(payload, command);
				// Encrypt payload
				hal_aes_crypt(payload_enc, payload);
				// Send the payload
				send_payload(payload_enc, power, 1);
				
				// indicator
				LED_1 = !LED_1;
			}

		// reset wdog
		hal_wdog_restart();	
		
		// Enter sleep mode
		sleep_mode();
	};
}

// ======================================= Function declaration
void wait_button_released(void){
		while(BTN_ALARM || BTN_SEAT){
			hal_wdog_restart();
			delay_ms(50);
		}
		//delay_ms(100);	
}

void load_flash(void){
	uint8_t vcu_id[4];
	
	hal_flash_bytes_read(VADDR_VCU_ID, vcu_id, sizeof(uint32_t));
	hal_flash_bytes_read(VADDR_AES_KEY, (uint8_t*)AES_Key, DATA_LENGTH);
	
	// Apply address
	memcpy(tx_address, vcu_id, sizeof(uint32_t));
	memcpy(rx_address, vcu_id, sizeof(uint32_t));
	// Initialise AES
	hal_aes_setup(0, ECB, (uint8_t*)AES_Key, NULL);
}

void update_configuration(uint8_t *success){
	if(received && (payload[DATA_PAIR_LENGTH - 1] == 0xAB)) {
		//safe to flash
		save_flash();
		
		// Apply new aes key
		load_flash();
		
		//setting Tx address
		hal_nrf_set_address(HAL_NRF_TX, tx_address);
		//setting Rx address for pipe0
		hal_nrf_set_address(HAL_NRF_PIPE0, rx_address);
		
		*success = 1;
	}	
}

void save_flash(void){
	// Save to flash
	hal_flash_page_erase(HAL_DATA_NV_FLASH_PN0);
	hal_flash_bytes_write(VADDR_VCU_ID, &payload[DATA_LENGTH], sizeof(uint32_t));
	hal_flash_bytes_write(VADDR_AES_KEY, payload, DATA_LENGTH);
}

void receive_pairing(void){
	// Power up radio
	hal_nrf_set_power_mode(HAL_NRF_PWR_UP);
	// Configure radio as primary receiver 
	hal_nrf_set_operation_mode(HAL_NRF_PRX); 
	
	// Enable receiver
	CE_HIGH();
	received = false;
	while (!received && (BTN_ALARM && BTN_SEAT)){
		// Indicator
		LED_1 = !LED_1;
		LED_2 = !LED_1;

		hal_wdog_restart();
		delay_ms(100);
	}
	CE_LOW();
	
	// Power off radio	
	hal_nrf_set_power_mode(HAL_NRF_PWR_DOWN);
	//setting payload width back
	hal_nrf_set_rx_payload_width(HAL_NRF_PIPE0, DATA_LENGTH);	
}

bool receive_ping(uint8_t timeout){
	uint32_t ms = 0;
	// Power up radio
	hal_nrf_set_power_mode(HAL_NRF_PWR_UP);
	// Configure radio as primary receiver 
	hal_nrf_set_operation_mode(HAL_NRF_PRX); 
	
	// Enable receiver
	CE_HIGH();
	received = false;
	while (!received && ms < timeout){
		// Indicator
//		LED_1 = !LED_1;
//		LED_2 = !LED_1;

		delay_ms(1);
		ms++;
	}
	CE_LOW();
	
	// Power off radio	
	hal_nrf_set_power_mode(HAL_NRF_PWR_DOWN);
	//setting payload width back
	hal_nrf_set_rx_payload_width(HAL_NRF_PIPE0, DATA_LENGTH);	
	
	return received;
}

void set_pairing_mode(void) {
	// Set paring address
	memset(tx_address, 0x00, sizeof(uint32_t));
	memset(rx_address, 0x00, sizeof(uint32_t));
	
	//setting Tx address
	hal_nrf_set_address(HAL_NRF_TX, tx_address);
	//setting Rx address for pipe0
	hal_nrf_set_address(HAL_NRF_PIPE0, rx_address);
	//setting payload width
	hal_nrf_set_rx_payload_width(HAL_NRF_PIPE0, DATA_PAIR_LENGTH);
}

void set_normal_mode(void) {
	//setting payload width
	hal_nrf_set_rx_payload_width(HAL_NRF_PIPE0, DATA_LENGTH);
}

void send_payload(uint8_t *payload, uint8_t pwr, uint8_t retry){
	// Write payload to radio TX FIFO
	hal_nrf_write_tx_payload(payload, DATA_LENGTH);
	// Setting power output
	hal_nrf_set_output_power(pwr);
	// Set transistor based on Command
	TRANSISTOR = (pwr != HAL_NRF_0DBM);
	// Power up radio
	hal_nrf_set_power_mode(HAL_NRF_PWR_UP);
	// Configure radio as primary receiver (PTX)
	hal_nrf_set_operation_mode(HAL_NRF_PTX);
	
	while(retry--){// Start transmission
		CE_HIGH();
		radio_busy = true;
		while (radio_busy){}
		CE_LOW();
	}
		
	// Power off radio	
	hal_nrf_set_power_mode(HAL_NRF_PWR_DOWN);
	// Set transistor to Open
	TRANSISTOR = 1;	
}

void make_payload(uint8_t *payload, uint8_t cmd){	
	memcpy(payload, &commands[cmd], 8);
	//memcpy(payload+8, &commands[cmd], 8);
}

void make_command(KLESS_CMD *cmd, hal_nrf_output_power_t *pwr){		
	// handle each buttons
	*pwr = HAL_NRF_18DBM;
	if(BTN_SEAT)	{
		*cmd = KLESS_CMD_SEAT;
	} else 	{
		*cmd = KLESS_CMD_PING;
	}		
}

void make_random_number(uint8_t *p){
	uint8_t len = (DATA_LENGTH/2);
	
	// start
	hal_rng_power_up(1);
	while(len--){
		while(!hal_rng_data_ready()){};
		*(p++) = hal_rng_read();
	}
	hal_rng_power_up(0);
}

void pin_init(void){
	char i;
	
	// Disconnect unused GPIOs to avoid them floating in sleep
	for (i = 0; i < 8; i++){
			P0CON = 0x70 + i;
			P1CON = 0x70 + i;
	} 
	P0DIR = 0x0B; 
	P1DIR = 0x00;

	P1CON = 0x00 + 3; // Set P1.3 as outputb  again
	P1CON = 0x00 + 0; // Set P1.1 as output again
	P0CON = 0x10 + 1; // Set P0.1 as input again
	P0CON = 0x10 + 3; // Set P0.3 as input again
		
	P0 = 0x00;
	P1 = 0x00;
	
	WUOPC0 = 0x0B;	//set pin P0.3 & P0.1 as wake-up pin
	OPMCON = 0x00;	//latch open and wake-up pin active high
	
	// Set default 
	TRANSISTOR = 1;
	LED_1 = 0;
	LED_2 = 0;
}

void nrf_init(void){
	// Setting datarate
	hal_nrf_set_datarate(HAL_NRF_1MBPS);
	//setting crc
	hal_nrf_set_crc_mode(HAL_NRF_CRC_8BIT);
	//setting address
	hal_nrf_set_address_width(HAL_NRF_AW_5BYTES);
	//setting auto retransmitt
	hal_nrf_set_auto_retr(0x0F,0x0F);
	//settinf RF channel
	hal_nrf_set_rf_channel(110);
	//setting Tx address
	hal_nrf_set_address(HAL_NRF_TX, tx_address);
	//setting payload width
	hal_nrf_set_rx_payload_width(HAL_NRF_PIPE0, DATA_LENGTH);
	//open pipe
	hal_nrf_open_pipe(HAL_NRF_PIPE0, 1);
	//setting Rx address for pipe0
	hal_nrf_set_address(HAL_NRF_PIPE0, rx_address);
	//setting interupt mode
	hal_nrf_set_irq_mode(HAL_NRF_MAX_RT,1);
	hal_nrf_set_irq_mode(HAL_NRF_TX_DS,1);
	hal_nrf_set_irq_mode(HAL_NRF_RX_DR,1);
	//disable ack
	hal_nrf_enable_ack_payload(1);
	//clear interupt flag
	hal_nrf_clear_irq_flag(HAL_NRF_MAX_RT);
	hal_nrf_clear_irq_flag(HAL_NRF_TX_DS);
	hal_nrf_clear_irq_flag(HAL_NRF_RX_DR);
}

void rtc_init(void){
	hal_rtc_start(false);
	hal_clklf_set_source(HAL_CLKLF_RCOSC32K);
	hal_rtc_set_compare_mode(HAL_RTC_COMPARE_MODE_0);
	hal_rtc_set_compare_value(0xFFFF/2);
	hal_rtc_start(true);

	// Wait for the 32kHz to startup (change phase)
	while((CLKLFCTRL&0x80)==0x80);
	while((CLKLFCTRL&0x80)!=0x80);
	
	// Setting wake-up from TICK and IRQ
	IEN1 = 0x20|0x08;	
}

void clock_and_irq_init(void){
	// Wait until 16 MHz crystal oscillator is running
	#ifdef MCU_NRF24LE1
	while(hal_clk_get_16m_source() != HAL_CLK_XOSC16M){}
	#endif
	// Enable the radio clock
	RFCKEN = 1U;
	// Enable RF irq
	RF = 1U;
	// Enable global irq
	EA = 1U;
}

void sleep_mode(void){
	// Register retention mode
	PWRDWN = 0x04;
	// Standby mode (wait for irq)
	PWRDWN = 0x07;
	// Clear PWRDWN
	PWRDWN = 0x00;
	// Exit sleep mode
}

// ======================================= Interrupt Service Routine
// RTC wakeup by tick
void wakeup_tick() interrupt INTERRUPT_TICK {
	 //LED_2 = !LED_2; 
}

// RTC wakeup by button
void wakeup_irq() interrupt INTERRUPT_WUOPIRQ {
	// LED_1 = !LED_1;
}


// Radio irq
NRF_ISR() {
  uint8_t irq_flags;

  // Read and clear IRQ flags from radio
  irq_flags = hal_nrf_get_clear_irq_flags();
 
  switch(irq_flags)
  {
    // Transmission success
    case (1 << (uint8_t)HAL_NRF_TX_DS):
      radio_busy = false;
      // Data has been sent
      break;
		
    // Transmission failed (maximum re-transmits)
    case (1 << (uint8_t)HAL_NRF_MAX_RT):
      // When a MAX_RT interrupt occurs the TX payload will not be removed from the TX FIFO.
      // If the packet is to be discarded this must be done manually by flushing the TX FIFO.
      // Alternatively, CE_PULSE() can be called re-starting transmission of the payload.
      // (Will only be possible after the radio irq flags are cleared)
      hal_nrf_flush_tx();
      radio_busy = false;
      break;
		
		// Received success
		case (1 << (uint8_t)HAL_NRF_RX_DR):
			// Read payload
			while(!hal_nrf_rx_fifo_empty()){
				hal_nrf_read_rx_payload(payload);
			}
			received = true;
			break;
			
    default:
      break;
  }
	
}
/** @} */

