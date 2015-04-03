#include <stdint.h>
#include <stdio.h>
#include <string.h>
//#include "nordic_common.h"
//#include "nrf_sdm.h"
//#include "ble.h"
//#include "ble_db_discovery.h"
#include "softdevice_handler.h"
#include "app_util.h"
#include "app_error.h"
//#include "ble_advdata_parser.h"
//#include "boards.h"
////#include "nrf6350.h"
#include "nrf_gpio.h"
////#include "pstorage.h"
//#include "device_manager.h"
#include "app_trace.h"

//Add by Yelun
#include "simple_uart.h"
#include "app_timer.h"
#include "spi_master_old.h"
#include "spi_master_config.h"
#include "battery.h"

static ble_gap_scan_params_t        m_scan_param;                        /**< Scan parameters requested for scanning and connection. */
static app_timer_id_t           weakup_timer_id;                                   
static app_timer_id_t  					weakup_meantimer_id	;
#define APPL_LOG                        app_trace_log             /**< Debug logger macro that will be used in this file to do logging of debug information over UART. */
#define MAX_rx_count 522
#define LEN_record 40
#define GTM900_power_pin 4
#define TIME_PERIOD 20       //seconds
static uint8_t tx_data[MAX_rx_count]; /**< SPI TX buffer. */
static uint8_t rx_data[MAX_rx_count]; /**< SPI RX buffer. */
static uint32_t *spi_base_address;
static uint16_t spi_flash_pages=0;
static uint16_t spi_flash_pages_count=0;
static uint32_t timer_counter=0;
static uint16_t rx_count=4;
static uint16_t month_days[12]={0,31,31+28,31+28+31,31+28+31+30,31+28+31+30+31,
										31+28+31+30+31+30,31+28+31+30+31+30+31,31+28+31+30+31+30+31+31,
										31+28+31+30+31+30+31+31+30,31+28+31+30+31+30+31+31+30+31,
										31+28+31+30+31+30+31+31+30+31+30};

void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    APPL_LOG("[APPL]: ASSERT: %s, %d, error 0x%08x\r\n", p_file_name, line_num, error_code);
//						simple_uart_put( error_code);
//						simple_uart_put( 0xEE);

//    nrf_gpio_pin_set(ASSERT_LED_PIN_NO);

    // This call can be used for debug purposes during development of an application.
    // @note CAUTION: Activating this code will write the stack to flash on an error.
    //                This function should NOT be used in a final product.
    //                It is intended STRICTLY for development/debugging purposes.
    //                The flash write will happen EVEN if the radio is active, thus interrupting
    //                any communication.
    //                Use with care. Un-comment the line below to use.
    // ble_debug_assert_handler(error_code, line_num, p_file_name);

    // On assert, the system can only recover with a reset.
    NVIC_SystemReset();
}
void send_string_no_answer(char * S)
{
   while(*S)
    {
        simple_uart_put(*S++);
				}
}
bool send_string(char * S,char * Respond)
{
   bool ret = false;
	uint8_t Rcount=0;  
	 while (* (Respond+Rcount)) Rcount++;
		Rcount--;
	
//		Respond-=2;
//	 char * SR; 
		
		uint8_t r=0,i=Rcount;
		memset(rx_data,0,256);
   while(*S)
    {
        simple_uart_put(*S++);
				}

		while(simple_uart_get_with_timeout(10000,rx_data+i)){
//				i=10;
//				SR=Respond;rx_data[8]='4';rx_data[9]='O';rx_data[10]='K';
				if (i>3 && rx_data[i-4]=='E' && rx_data[i-3]=='R' && rx_data[i-2]=='R' && rx_data[i-1]=='O' && rx_data[i]=='R')	break ;
				for (r=0; r<Rcount; r++) 
						if (*(Respond+r) != (rx_data[i-Rcount+r])) break ;
				if (r==Rcount) break;
//				if (rx_data[i-3]=='O' && rx_data[i-2]=='K' && rx_data[i-1]==0x0D && rx_data[i]==0x0A) {ret=true; break ;}
//				if (rx_data[i-3]=='C' && rx_data[i-2]=='O' && rx_data[i-1]=='N' && rx_data[i]=='N') {ret=true; break ;}
				i++;
				}
		if (r==Rcount) {
			ret=true; 
			while(simple_uart_get_with_timeout(1,rx_data+i++));
			}
			else ret=false;
		return ret;
}

 char char_hex( char bHex){
		bHex&=0x0f;
    if((bHex<10))
        bHex += '0';
    else bHex += 'A'-10;
//    else bHex = 0xff;
    return bHex;
}
void gprs_gtm900()
{
//GTM900C 
//		simple_uart_config(NULL, 8, NULL, 9, false);
			send_string("AT\r\n","OK");
			send_string("AT\r\n","OK");
			send_string("ATE0\r\n","OK");
			if(!send_string("AT+CPIN?\r\n","READY")) return; //error
//////			send_string("AT+CLTS=1\r\n","OK");  //Get date time.
////////			if (send_string("","DST:")||send_string("","DST:"))	send_string("AT+CCLK?\r\n","OK");
//////			(send_string("","DST:")||send_string("","DST:"));  //wait for time stamp almost 2 timrs 10 Secs.
//////			send_string("AT+CLTS=0\r\n","OK");  //Get date time.
//			send_string("AT%SLEEP=0\r\n","OK");
//			send_string("AT+CGREG=1\r\n"); 
//connect smtp server
//			send_string("AT%IOMODE=1,1,0\r\n","OK");

			send_string("at+csq\r\n","OK");  	//ÐÅºÅÖÊÁ¿
//			send_string("at%ipclose=1\r\n","OK");
			send_string("at+cgatt=1\r\n","OK");
			send_string("AT+CIPCSGP=1,\"cmnet\",\"guest\",\"guest\"\r\n","OK");

			send_string("at+CSTT\r\n","OK");
			send_string("AT+CIICR\r\n","OK");

			send_string("AT+CIFSR\r\n",".");
			send_string("AT+CIPCLOSE=1\r\n","OK");

			if(!(send_string("AT+CIPSTART=\"TCP\",\"159.226.251.11\",25\r\n","cstnet")||send_string("","cstnet"))) return;

//send mail
			send_string("AT+CIPSPRT=1\r\n","OK");
			send_string("AT+CIPSEND\r\n",">");			
			send_string("ehlo helo\r\n\x1a","coremail");			//ehlo helo
			send_string("AT+CIPSEND\r\n",">");		
			send_string("auth login\r\n\x1a","SEND");		//auth login
			send_string("AT+CIPSEND\r\n",">");		
			send_string("Y2NkYw==\r\n\x1a","SEND");				//user name: ccdc
			send_string("AT+CIPSEND\r\n",">");		
			if(!send_string("Y2NkY2NzdG5ldA==\r\n\x1a","successful")) 
							return;//passwore: ********
			send_string("AT+CIPSEND\r\n",">");		
			send_string("MAIL FROM:<lye@cstnet.cn>\r\n\x1a","SEND");//mail from lye@cstnet.cn
			send_string("AT+CIPSEND\r\n",">");		
			send_string("RCPT to:<ccdc@cstnet.cn>\r\n\x1a","SEND");//RCPT to:<ccdc@cstnet.cn>
			send_string("AT+CIPSEND\r\n",">");		
			if(!send_string("DATA\r\n\x1a","SEND")) return;//DATA

			send_string("AT+CREG=2\r\n","OK");


			send_string("AT+CREG?\r\n","OK");

			char s,subject[40];
			for(s=3;s<27;s++) subject[s]=rx_data[s];
			battery_start(4);
			uint16_t batt_lvl_in_milli_volts=(((NRF_ADC->RESULT) * 1200) / 255) * 3 ;
			subject[27]=';';
			subject[28]=char_hex(batt_lvl_in_milli_volts>>12);
			subject[29]=char_hex((batt_lvl_in_milli_volts>>8));
			subject[30]=char_hex((batt_lvl_in_milli_volts>>4));
			subject[31]=char_hex(batt_lvl_in_milli_volts);
			battery_start(6);
			batt_lvl_in_milli_volts=(((NRF_ADC->RESULT) * 1200) / 255) * 3 ;
			subject[32]=';';
			subject[33]=char_hex(batt_lvl_in_milli_volts>>12);
			subject[34]=char_hex((batt_lvl_in_milli_volts>>8));
			subject[35]=char_hex((batt_lvl_in_milli_volts>>4));
			subject[36]=char_hex(batt_lvl_in_milli_volts);

			send_string("AT+CIPSEND\r\n",">");		


			send_string_no_answer("Subject: ");//Subject: 

			for (uint8_t i=3;i<37;i++) {
					simple_uart_put((subject[i]));//+CREG: 2,1,"11D4","2048"
					}

			send_string("\r\n\r\n\x1a","OK");
			send_string("AT+CREG=0\r\n","OK");
//send data area
			send_string("AT+CIPSEND\r\n",">");

			for (uint16_t i=4;i<(MAX_rx_count);){
					simple_uart_put(char_hex(tx_data[i]>>4));
					simple_uart_put(char_hex(tx_data[i]));
					i++;
					if (!((i-4)%LEN_record)){
									if (!send_string("\r\n\x1a","OK")) return;
									send_string("AT+CIPSEND\r\n",">");
					}
			}
			send_string("\r\n\x1a","OK");
			send_string("AT+CIPSEND\r\n",">");

			send_string("\r\n\x2e\r\n\x1a","OK");//end
			send_string("AT+CIPCLOSE=1\r\n","OK");

}

static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
//    uint32_t           retval;
    const ble_gap_evt_t   * p_gap_evt = &p_ble_evt->evt.gap_evt;

//  	simple_uart_put( p_ble_evt->header.evt_id);
//  	simple_uart_put( 0xFF);

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_ADV_REPORT:
        {
				if ((p_ble_evt->evt.gap_evt.params.adv_report.data[5] == 0x81 ) && (rx_count+LEN_record)<MAX_rx_count)
					{
						for (uint8_t i=6;i>0;i--){ 						
//							simple_uart_put( p_gap_evt->params.adv_report.peer_addr.addr[i-1] );
							tx_data[rx_count++]=p_gap_evt->params.adv_report.peer_addr.addr[i-1];
							}
						for (uint8_t i=0;i<BLE_GAP_ADV_MAX_SIZE;i++)
							{	
							tx_data[rx_count++]=p_ble_evt->evt.gap_evt.params.adv_report.data[i];
//							simple_uart_put( p_ble_evt->evt.gap_evt.params.adv_report.data[i]);
							}
						 *((uint32_t*)(tx_data+rx_count-1)) = timer_counter;
							rx_count+=3;
//						simple_uart_put( p_ble_evt->evt.gap_evt.params.adv_report.rssi);
						}
					}
				break;
				
			}


//			ble_adv_evt_handler(p_ble_evt);
//    dm_ble_evt_handler(p_ble_evt);
//    ble_db_discovery_on_ble_evt(&m_ble_db_discovery, p_ble_evt);
//    ble_hrs_c_on_ble_evt(&m_ble_hrs_c, p_ble_evt);
//    ble_bas_c_on_ble_evt(&m_ble_bas_c, p_ble_evt);
//    on_ble_evt(p_ble_evt);
}

static void sys_evt_dispatch(uint32_t sys_evt)
{
//  	simple_uart_put( sys_evt);
//  	simple_uart_put( 0x33);

}
static void weakup_timeout_handler(void * p_context)
{		
    UNUSED_PARAMETER(p_context);
		m_scan_param.active       = 0;            // Active scanning set.
		m_scan_param.selective    = 0;            // Selective scanning not set.
		m_scan_param.interval     = 0x10A0;     // Scan interval.
		m_scan_param.window       = 0x109E;   // Scan window.
		m_scan_param.p_whitelist  = NULL;         // No whitelist provided.
		m_scan_param.timeout      = 0x0000;       // No timeout.
		sd_ble_gap_scan_stop();
    uint32_t err_code = sd_ble_gap_scan_start(&m_scan_param);
    APP_ERROR_CHECK(err_code);
		timer_counter += TIME_PERIOD ;
    app_timer_start(weakup_meantimer_id,  APP_TIMER_TICKS(1010, 0), NULL);
//		NRF_UART0->POWER = (UART_POWER_POWER_Enabled << UART_POWER_POWER_Pos);
//		simple_uart_config(NULL, 8, NULL, 9, false);
		
//		if (rx_count>(MAX_rx_count-24)) rx_count=4;  //leave for tx command codes;

}
static void weakup_meantimeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    uint32_t err_code = sd_ble_gap_scan_stop();
    APP_ERROR_CHECK(err_code);
//AT45DB Flash writen
    tx_data[0]=0xAB;			//Resume from sleep
    spi_master_tx_rx(spi_base_address, 1, (const uint8_t *)tx_data, rx_data);
		nrf_delay_ms(1);

	  tx_data[0]=0x82;tx_data[1]=(spi_flash_pages>>6);tx_data[2]=(spi_flash_pages<<2);tx_data[3]=0;
    spi_master_tx_rx(spi_base_address, 256, (const uint8_t *)tx_data, rx_data);
		nrf_delay_ms(20);  //waiting for flash over.

    tx_data[0]=0xB9;				//power down sleep
    spi_master_tx_rx(spi_base_address, 1, (const uint8_t *)tx_data, rx_data);

		spi_flash_pages++;

		if (rx_count>(MAX_rx_count - 4*LEN_record )) {
				rx_count=4;
//				battery_start();		
				nrf_gpio_pin_clear(GTM900_power_pin);
				nrf_delay_ms(100);
				nrf_gpio_pin_set(GTM900_power_pin);

				NRF_UART0->POWER = (UART_POWER_POWER_Enabled << UART_POWER_POWER_Pos);
				simple_uart_config(NULL, 8, NULL, 9, false);

				gprs_gtm900();
				send_string("AT+CIPSHUT\r\n","OK");
				send_string("AT+CGATT=0\r\n","OK");
				send_string("AT+CPOWD=1\r\n","POWER");
////				send_string("AT%MSO\r\n","Shut down");
//				nrf_gpio_pin_set(GTM900_power_pin);
//				nrf_delay_ms(100);
				nrf_gpio_pin_clear(GTM900_power_pin);
				memset(tx_data,0,MAX_rx_count);
				NRF_UART0->POWER = (UART_POWER_POWER_Disabled << UART_POWER_POWER_Pos);

			}

//		NRF_UART0->POWER = (UART_POWER_POWER_Disabled << UART_POWER_POWER_Pos);


}

int main(void)
{
	uint32_t err_code;
	NRF_GPIO->PIN_CNF[GTM900_power_pin] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
                                            | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
                                            | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
                                            | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
                                            | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);

	nrf_gpio_pin_set(GTM900_power_pin);
	simple_uart_config(NULL, 8, NULL, 9, false);
			send_string("AT\r\n","OK");
			send_string("AT\r\n","OK");
			send_string("ATE1\r\n","OK");
			send_string("AT+CPIN?\r\n","READY");

			send_string("AT+CLTS=1\r\n","OK");  //Get date time.
			if ((send_string("","DST:")||send_string("","DST:"))	
						&& send_string("AT+CCLK?\r\n","OK") && (rx_data[21]=='1' && rx_data[22]>='5')){
					rx_data[24] = (rx_data[24]-'0')*10 + (rx_data[25]-'0');  	//month
					rx_data[27] = (rx_data[27]-'0')*10 + (rx_data[28]-'0');		//day
					rx_data[30] = (rx_data[30]-'0')*10 + (rx_data[31]-'0');		//hour
					rx_data[33] = (rx_data[33]-'0')*10 + (rx_data[34]-'0');		//minus
					rx_data[36] = (rx_data[36]-'0')*10 + (rx_data[37]-'0');		//second
					timer_counter = ((((month_days[rx_data[24]-1]+rx_data[27])*24 + rx_data[30])*60 + rx_data[33])*60 + rx_data[36]);
					}
			send_string("AT+CLTS=0\r\n","OK");  //Get date time.
			send_string("AT+CIPSHUT\r\n","OK");
			send_string("AT+CGATT=0\r\n","OK");
			send_string("AT+CPOWD=1\r\n","POWER");

//	while(!send_string("0D0A0D0A\"\r\n","OK"));
//GMT900 POWER on off control
	nrf_gpio_pin_clear(GTM900_power_pin);


//AT45DB161 sleep down
	 	nrf_delay_ms(20);  //There must be 20ms waitting for AT45DB161 ready from power on.
		spi_base_address=spi_master_init(0, 0, 0);
		tx_data[0]=0XB9;
    spi_master_tx_rx(spi_base_address, 1, (const uint8_t *)tx_data, rx_data);

//			simple_uart_config(NULL, 8, NULL, 9, false);
//softdevice enable
			SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, false);
			err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
			APP_ERROR_CHECK(err_code);
			err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
			APP_ERROR_CHECK(err_code);
//timer start
			APP_TIMER_INIT(0, 3, 4, false);
			err_code = app_timer_create(&weakup_timer_id,
																	APP_TIMER_MODE_REPEATED,
																	weakup_timeout_handler);
			err_code = app_timer_create(&weakup_meantimer_id,
																	APP_TIMER_MODE_SINGLE_SHOT,
																	weakup_meantimeout_handler);

			 APP_ERROR_CHECK(err_code);
			 app_timer_start(weakup_timer_id,  APP_TIMER_TICKS(TIME_PERIOD*1000, 0), NULL);
//evt_wait
			for (;;)
			{
					err_code = sd_app_evt_wait();
					APP_ERROR_CHECK(err_code);    
				}
}
