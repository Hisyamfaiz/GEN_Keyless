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
#include <string.h>
 
// ======================================= Type definition
#define TRANSISTOR						P14
#define LED_1									P02
#define LED_2									P03
#define BTN_ALARM							!P04
#define BTN_SEAT							!P06 

#define VADDR_VCU_ID					HAL_DATA_NV_BASE_ADDRESS
#define VADDR_AES_KEY					(VADDR_VCU_ID + sizeof(uint32_t))

#define DATA_LENGTH 					16
#define ADDR_LENGTH						5
#define DATA_PAIR_LENGTH		  (DATA_LENGTH + ADDR_LENGTH)

typedef enum {
  KLESS_CMD_PING 	= 0,
  KLESS_CMD_ALARM = 1,
  KLESS_CMD_SEAT 	= 2
} KLESS_CMD;

// ======================================= Global variable
static uint8_t volatile radio_busy, received, disable_radio = 0;
static uint8_t tx_address[ADDR_LENGTH] = {0x00, 0x00, 0x00, 0x00, 0xCD};
static uint8_t rx_address[ADDR_LENGTH] = {0x00, 0x00, 0x00, 0x00, 0xAB};
static uint8_t xdata payload[DATA_PAIR_LENGTH];
static uint8_t xdata payload_enc[DATA_LENGTH];
static uint32_t idata aes_key[4];
static const uint8_t idata commands[3][8] = {
  { 0x5e, 0x6c, 0xa7, 0x74, 0xfd, 0xe3, 0xdf, 0xbc },
  { 0xf7, 0xda, 0x4a, 0x4f, 0x65, 0x2d, 0x6e, 0xf0 },
  { 0xff, 0xa6, 0xe6, 0x5a, 0x84, 0x82, 0x66, 0x4f }
};

// ======================================= Function prototype 
void pin_init(void);
void rtc_init(void);
void nrf_init(void);
void clock_and_irq_init(void);
void sleep_mode(void);

void make_random_number(uint8_t *p);
void make_payload(uint8_t *payload, uint8_t cmd);
void send_payload(uint8_t *payload, uint8_t pwr, uint8_t retry);
void set_pairing_mode(void);
void set_normal_mode(void);
void transmit(KLESS_CMD cmd, hal_nrf_output_power_t power, uint8_t retry);
uint8_t receive_pairing(void);
uint8_t receive_ping(uint8_t timeout);
void update_flash(void);
void load_flash(void);
void led_blink(uint32_t ms);
void led_write(uint8_t state);
uint8_t led_read(void);
uint32_t wait_btn_seat(void);
uint32_t wait_btn_alarm(void);

// ======================================= Main function 
void main(void){
  uint8_t show_ping = 0;
  
  pin_init();
  rtc_init();
  clock_and_irq_init();
  load_flash();
  nrf_init();
  hal_wdog_init(0x0300);

  while(1)	{				
    if (BTN_ALARM || BTN_SEAT) {
      delay_ms(100);

      if (BTN_ALARM && BTN_SEAT) {				
        set_pairing_mode();

        if (receive_pairing()) {
          update_flash();
          led_write(0);
        } else 
          led_write(1);

        set_normal_mode();			
        wait_btn_alarm();
        wait_btn_seat();
      } 

      else if (BTN_ALARM) {
        if (wait_btn_alarm() > 3000) 
          show_ping = !show_ping;
        else {
          transmit(KLESS_CMD_ALARM, HAL_NRF_0DBM, 5);
          led_blink(20);
        }
      } 
      
      else if (BTN_SEAT) {
        if (wait_btn_seat() > 3000) 
          disable_radio = !disable_radio;
        else if (receive_ping(10)) {
          transmit(KLESS_CMD_SEAT, HAL_NRF_6DBM, 1);
          led_blink(20);
        }
      } 
    }
            
    else if (receive_ping(10)) {
      transmit(KLESS_CMD_PING, HAL_NRF_6DBM, 1);
      if (show_ping) led_blink(10);
    }

    hal_wdog_restart();	
    sleep_mode();
  };
}

// ======================================= Function declaration
void led_blink(uint32_t ms) {
  led_write(1);
  delay_ms(ms);
  led_write(0);
}

uint8_t led_read(void) {
  return disable_radio ? !LED_1 : !LED_2;
}

void led_write(uint8_t state) {
  if (disable_radio) {
    LED_1 = !state;
    LED_2 = 1;
  } else {
    LED_2 = !state;
    LED_1 = 1;
  }
}

void transmit(KLESS_CMD command, hal_nrf_output_power_t power, uint8_t retry) {
  if (disable_radio) return;

  make_payload(payload, command);
  hal_aes_crypt(payload_enc, payload);
  send_payload(payload_enc, power, retry);
}

void send_payload(uint8_t *payload, uint8_t pwr, uint8_t retry) {
  if (disable_radio) return;

  hal_nrf_write_tx_payload(payload, DATA_LENGTH);
  hal_nrf_set_output_power(pwr);
  hal_nrf_set_operation_mode(HAL_NRF_PTX);

  TRANSISTOR = (pwr != HAL_NRF_0DBM);
  hal_nrf_set_power_mode(HAL_NRF_PWR_UP);
  while(retry--){
    CE_HIGH();
    radio_busy = true;
    while (radio_busy){}
    CE_LOW();
  }	
  hal_nrf_set_power_mode(HAL_NRF_PWR_DOWN);
  TRANSISTOR = 1;	
}

uint8_t receive_pairing(void){
  if (disable_radio) return 0;
  
  hal_nrf_set_power_mode(HAL_NRF_PWR_UP);
  hal_nrf_set_operation_mode(HAL_NRF_PRX); 
  
  // Enable receiver
  CE_HIGH();
  received = false;
  while (!received && (BTN_ALARM && BTN_SEAT)){
    led_write(!led_read());
    hal_wdog_restart();
    delay_ms(100);
  }
  CE_LOW();
  
  hal_nrf_set_power_mode(HAL_NRF_PWR_DOWN);
  hal_nrf_set_rx_payload_width(HAL_NRF_PIPE0, DATA_LENGTH);	

  return (received && (payload[DATA_PAIR_LENGTH - 1] == 0xAB));
}

uint8_t receive_ping(uint8_t timeout){
  uint32_t ms = 0;
  
  if (disable_radio) return 0;
  
  hal_nrf_set_power_mode(HAL_NRF_PWR_UP);
  hal_nrf_set_operation_mode(HAL_NRF_PRX); 
  
  // Enable receiver
  CE_HIGH();
  received = false;
  while (!received && (ms < timeout)){
    delay_ms(1);
    ms++;
  }
  CE_LOW();
  
  hal_nrf_set_power_mode(HAL_NRF_PWR_DOWN);
  hal_nrf_set_rx_payload_width(HAL_NRF_PIPE0, DATA_LENGTH);	
  
  return received;
}

void set_pairing_mode(void) {
  memset(tx_address, 0x00, sizeof(uint32_t));
  memset(rx_address, 0x00, sizeof(uint32_t));
  
  hal_nrf_set_address(HAL_NRF_TX, tx_address);
  hal_nrf_set_address(HAL_NRF_PIPE0, rx_address);
  hal_nrf_set_rx_payload_width(HAL_NRF_PIPE0, DATA_PAIR_LENGTH);
}

void set_normal_mode(void) {
  load_flash();
  
  hal_nrf_set_address(HAL_NRF_TX, tx_address);
  hal_nrf_set_address(HAL_NRF_PIPE0, rx_address);
  hal_nrf_set_rx_payload_width(HAL_NRF_PIPE0, DATA_LENGTH);
}

void make_payload(uint8_t *payload, uint8_t cmd){	
  memcpy(payload, &commands[cmd], 8);
}

void make_random_number(uint8_t *p){
  uint8_t len = (DATA_LENGTH/2);
  
  hal_rng_power_up(1);
  while(len--){
    while(!hal_rng_data_ready()) {};
    *(p++) = hal_rng_read();
  }
  hal_rng_power_up(0);
}

uint32_t wait_btn_seat(void) {
  uint32_t ms = 0;

  while(BTN_SEAT) {
    hal_wdog_restart();
    delay_ms(50);
    ms+=50;
  }

  return ms;
}

uint32_t wait_btn_alarm(void) {
  uint32_t ms = 0;

  while(BTN_ALARM) {
    hal_wdog_restart();
    delay_ms(50);
    ms+=50;
  }

  return ms;
}

void update_flash(void){
  hal_flash_page_erase(HAL_DATA_NV_FLASH_PN0);
  hal_flash_bytes_write(VADDR_VCU_ID, &payload[DATA_LENGTH], sizeof(uint32_t));
  hal_flash_bytes_write(VADDR_AES_KEY, payload, DATA_LENGTH);
}

void load_flash(void){
  uint8_t vcu_size = sizeof(uint32_t);
  uint8_t vcu_id[sizeof(uint32_t)];
  
  hal_flash_bytes_read(VADDR_VCU_ID, vcu_id, vcu_size);
  hal_flash_bytes_read(VADDR_AES_KEY, (uint8_t*)aes_key, DATA_LENGTH);
  
  memcpy(tx_address, vcu_id, vcu_size);
  memcpy(rx_address, vcu_id, vcu_size);

  hal_aes_setup(0, ECB, (uint8_t*)aes_key, NULL);
}


void pin_init(void){
  char i;
  
  // Disconnect unused GPIOs to avoid them floating in sleep
  for (i = 0; i < 8; i++) {
      P0CON = 0x70 + i;
      P1CON = 0x70 + i;
  } 
  P0DIR = 0x50;
  P1DIR = 0x00;	

  P0CON = 0x00 + 3; // Set P0.3 as output again
  P0CON = 0x00 + 2; // Set P0.2 as output again
  P0CON = 0x50 + 6; // Set P0.6 as input again
  P0CON = 0x50 + 4; // Set P0.4 as input again
  P1CON = 0x00 + 4; // Set P1.4 as output again
    
  P0 = 0xff;
  P1 = 0x00;
  
  WUOPC0 = 0x00;	//set pin P0.4 & P0.6 as wake-up pin
//	OPMCON = 0x00;	//latch open and wake-up pin active high
  
  // Set default 
  TRANSISTOR = 1;
  LED_1 = 1;
  LED_2 = 1;
}

void nrf_init(void){
  hal_nrf_set_datarate(HAL_NRF_250KBPS);
  hal_nrf_set_crc_mode(HAL_NRF_CRC_8BIT);
  hal_nrf_set_address_width(HAL_NRF_AW_5BYTES);
  hal_nrf_set_auto_retr(0,0);
  hal_nrf_set_rf_channel(110);
  hal_nrf_set_address(HAL_NRF_TX, tx_address);
  hal_nrf_set_rx_payload_width(HAL_NRF_PIPE0, DATA_LENGTH);
  hal_nrf_open_pipe(HAL_NRF_PIPE0, 1);
  hal_nrf_set_address(HAL_NRF_PIPE0, rx_address);
  //setting interupt mode
  hal_nrf_set_irq_mode(HAL_NRF_MAX_RT,1);
  hal_nrf_set_irq_mode(HAL_NRF_TX_DS,1);
  hal_nrf_set_irq_mode(HAL_NRF_RX_DR,1);
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

