#include <stdint.h>
#include <stdio.h>
#include <string.h>

//#include "nordic_common.h"
//#include "nrf_sdm.h"
#include "ble.h"
//#include "ble_db_discovery.h"

//#include "softdevice_handler.h"
#include "app_util.h"
#include "app_error.h"
//#include "ble_advdata_parser.h"
//#include "boards.h"
////#include "nrf6350.h"
#include "nrf_gpio.h"
#include "pstorage.h"
//#include "device_manager.h"
#include "app_trace.h"

//Add by Yelun
#include "simple_uart.h"
#include "app_timer.h"
#include "battery.h"
#include "nrf_soc.h"
//#include "app_scheduler.h"
//#include "app_timer_appsh.h"
//#include "softdevice_handler_appsh.h"
#include "softdevice_handler.h"
//#include "ble_radio_notification.h"

static ble_gap_scan_params_t        m_scan_param;                        /**< Scan parameters requested for scanning and connection. */
static app_timer_id_t           weakup_timer_id;                                   
static app_timer_id_t  					weakup_meantimer_id	;
//static app_timer_id_t  					sim900a_timer_id	;
#define APPL_LOG                        app_trace_log             /**< Debug logger macro that will be used in this file to do logging of debug information over UART. */
#define MAX_rx_count 1024
#define LEN_record   32
//#define GTM900_power_pin 4
//#define GTM900_power_pin 8    //for GPS box PCB old Version PCB
#define GTM900_power_pin 18    //for GPS box PCB
#define LED_PIN 24   //22       //for GPS box PCB
#define TIME_PERIOD     1    //180   //300       //seconds
#define PSTORAGE_BLOCK_SIZE   1024 
#define SCAN_TIMER_STAR    app_timer_start(weakup_timer_id,  APP_TIMER_TICKS(TIME_PERIOD*1000, 0), NULL)
#define SCAN_TIMER_STOP    app_timer_stop(weakup_timer_id)

static uint8_t tx_data[32]; /**< SPI TX buffer. */
static uint8_t rx_data[256]; /**< Receive data buffer. */
static uint32_t timer_counter=0;
static uint32_t time_period_count = 0;
static uint16_t rx_count=0;
static uint16_t month_days[12]={365+0, 365+31, 365+60, 365+91, 365+121, 365+152,
				365+182, 212, 243, 273, 304, 334};

static pstorage_handle_t       flash_base_handle;
static pstorage_block_t pstorage_wait_handle = 0;
static uint8_t pstorage_wait_flag = 0;
static pstorage_size_t pstorage_block_id = 0;
static uint8_t  pstorage_clear_nextpage = 0;
static uint8_t  weakup_flag = false;   
static uint32_t  pstorage_count = 0;
static bool  scan_start = false;
static uint16_t batt_lvl_in_milli_volts;
				
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
				while(simple_uart_get_with_timeout(5,rx_data));
        simple_uart_put(*S++);
				}
}
bool send_string(char * S,char * Respond)
{
   bool ret = false;
	uint8_t Rcount=0;  
	 while (* (Respond+Rcount)) Rcount++;
//		Rcount--;
	
//		Respond-=2;
//	 char * SR; 
		
		uint8_t r=0,i=Rcount;
		while(simple_uart_get_with_timeout(5,rx_data));
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
		bHex &= 0x0f;
    if((bHex < 10))
        bHex += '0';
    else bHex += 'A'-10;
//    else bHex = 0xff;
    return bHex;
}

void gprs_gtm900()
{
//GTM900C 
//		simple_uart_config(NULL, 8, NULL, 9, false);
	    char s,subject[80]= "DTU:",sub_index=4;
			nrf_delay_ms(20000);
	    ble_gap_addr_t  	p_addr;
			sd_ble_gap_address_get	(	&p_addr	);
			for (uint8_t i = 0; i <6; i++) {
					subject[sub_index++]= char_hex(p_addr.addr[i]>>4);
					subject[sub_index++]= char_hex(p_addr.addr[i]);
			}
	    subject[sub_index++]= '@';
			send_string("AT\r\n","OK");
			send_string("AT\r\n","OK");
			send_string("AT\r\n","OK");
			send_string("ATE1\r\n","OK");
			if (!send_string("at+CLTS?\r\n","CLTS: 1")) {
					send_string("at+CLTS=1\r\n","OK");
					send_string("AT&W\r\n","OK");
					return;
					}
			if(!send_string("AT+CPIN?\r\n","READY")) return; //error
			send_string("AT+CREG=2\r\n","OK");
			if (!send_string("AT+CREG?\r\n","+CREG: 2,1")) send_string("AT+CREG?\r\n","+CREG: 2,1");
			for (uint8_t i = 32; i <36; i++) subject[sub_index++]= rx_data[i];
			for (uint8_t i = 39; i <43; i++) subject[sub_index++]= rx_data[i];
			send_string("AT+CREG=0\r\n","OK");
			send_string("at+csq\r\n","OK");  	//ÐÅºÅÖÊÁ¿
					for (uint8_t i = 11; i <21; i++) subject[sub_index++]= rx_data[i];
					subject[sub_index++]=',';
			send_string("at+cbc\r\n","OK");  	
					for (uint8_t i = 22; i <26; i++) subject[sub_index++]= rx_data[i];
			subject[sub_index++]=',';
			subject[sub_index++]= char_hex(batt_lvl_in_milli_volts >> 12);
			subject[sub_index++]= char_hex(batt_lvl_in_milli_volts >> 8);
			subject[sub_index++]= char_hex(batt_lvl_in_milli_volts >> 4);
			subject[sub_index++]= char_hex(batt_lvl_in_milli_volts );
			subject[sub_index++]=',';
			subject[sub_index++]=char_hex(pstorage_block_id>>12);
			subject[sub_index++]=char_hex((pstorage_block_id>>8));
			subject[sub_index++]=char_hex((pstorage_block_id>>4));
			subject[sub_index++]=char_hex(pstorage_block_id);

				if (send_string("AT+CCLK?\r\n","OK") && (rx_data[21]=='1' && rx_data[22]>='5')){
					for (uint8_t i = 20; i <42; i++) subject[sub_index++]= rx_data[i];
					rx_data[24] = (rx_data[24]-'0')*10 + (rx_data[25]-'0');  	//month
					rx_data[27] = (rx_data[27]-'0')*10 + (rx_data[28]-'0');		//day
					rx_data[30] = (rx_data[30]-'0')*10 + (rx_data[31]-'0');		//hour
					rx_data[33] = (rx_data[33]-'0')*10 + (rx_data[34]-'0');		//minus
					rx_data[36] = (rx_data[36]-'0')*10 + (rx_data[37]-'0');		//second
					timer_counter = ((((month_days[rx_data[24]-1]+(rx_data[27]) -1 )*24 + rx_data[30])*60 + rx_data[33])*60 + rx_data[36]);
					}
				else return;
			send_string("ATE0\r\n","OK");
////			send_string("at%ipclose=1\r\n","OK");
			if (!send_string("at+cgatt?\r\n","CGATT: 1")) 
							send_string("at+cgatt=1\r\n","OK");
//			send_string("AT+CIPCSGP=1,\"cmnet\",\"guest\",\"guest\"\r\n","OK");
			send_string("at+CSTT=\"CMNET\"\r\n","OK");
			send_string("AT+CIICR\r\n","OK");
			send_string("AT+CIFSR\r\n",".");
//			send_string("AT+CIPCLOSE=1\r\n","OK");
			if(!(send_string("AT+CIPSTART=\"TCP\",\"159.226.251.11\",25\r\n","CONNECT OK")||send_string("","CONNECT OK"))) return;

//send mail
			send_string("AT+CIPSPRT=1\r\n","OK");
			send_string("AT+CIPSEND\r\n",">");			
			send_string("ehlo helo\r\n\x1a","coremail");			//ehlo helo
			send_string("AT+CIPSEND\r\n",">");		
			send_string("auth login\r\n\x1a","334");		//auth login
			send_string("AT+CIPSEND\r\n",">");		
			send_string("Y2NkYw==\r\n\x1a","334");				//user name: ccdc
			send_string("AT+CIPSEND\r\n",">");		
			if(!send_string("Y2NkY2NzdG5ldA==\r\n\x1a","235"))	return;//passwore: ********

			send_string("AT+CIPSEND\r\n",">");		
			send_string("MAIL FROM:<lye@cstnet.cn>\r\n\x1a","250");//mail from lye@cstnet.cn
			send_string("AT+CIPSEND\r\n",">");		
//			send_string("RCPT to:<ccdc@cstnet.cn>\r\n\x1a","250");//RCPT to:<ccdc@cstnet.cn>
			send_string("RCPT to:<lye@cstnet.cn>\r\n\x1a","250");//RCPT to:<ccdc@cstnet.cn>
			send_string("AT+CIPSEND\r\n",">");		
			if(!send_string("DATA\r\n\x1a","354")) return;//DATA
			send_string("AT+CIPSEND\r\n",">");		
			send_string_no_answer("Subject: ");//Subject: 
			for (uint8_t i=0;i<72;i++) {
					simple_uart_put((subject[i]));//+CREG: 2,1,"11D4","2048"
					}

			send_string("\r\n\r\n\x1a","OK");
//			send_string("AT+CREG=0\r\n","OK");
			send_string("AT+CIPSEND\r\n",">");
//			send_string("AT+CIPSEND?\r\n","OK");
//		for (uint16_t block_id=0; block_id<(1024 / PSTORAGE_BLOCK_SIZE) * PSTORAGE_MAX_APPLICATIONS; block_id++){
		for (; pstorage_block_id  >0; pstorage_block_id--){
//send data area
				pstorage_handle_t 		flash_handle;
				pstorage_block_identifier_get(&flash_base_handle, pstorage_block_id-1, &flash_handle);
				uint16_t err_code = 0;
				while(simple_uart_get_with_timeout(1,rx_data));
				err_code = pstorage_load((uint8_t *)tx_data, &flash_handle,32,0);
				APP_ERROR_CHECK(err_code);
				for (uint8_t i=0; i < 32; i++){
						simple_uart_put(char_hex(tx_data[i]>>4));
						simple_uart_put(char_hex(tx_data[i]));
					}
				if (!((pstorage_block_id-1)%16)){
						if (!((pstorage_block_id-1)%32)) {
									err_code = pstorage_clear(&flash_handle, PSTORAGE_BLOCK_SIZE );
//									APP_ERROR_CHECK(err_code);
//									err_code = sd_app_evt_wait();
									APP_ERROR_CHECK(err_code);    
							}
						send_string("\r\n\x1a","OK");	
						if (!send_string("AT+CIPSEND\r\n",">")) if (!send_string("AT+CIPSEND\r\n",">")) return;	
				}
			}
			send_string("\r\n\x1a","OK");	
			if (!send_string("AT+CIPSEND\r\n",">")) if (!send_string("AT+CIPSEND\r\n",">")) return;	
			if (send_string("\r\n\x2e\r\n\x1a","250")) {
//							pstorage_block_id = 0;
//							pstorage_clear(&flash_base_handle,PSTORAGE_FLASH_PAGE_SIZE);
						send_string("AT+CIPCLOSE=1\r\n","OK");
					}
			else return;
}
//static void scan_stop_flash_write(void * p_context)
//{
//    UNUSED_PARAMETER(p_context);
//		uint32_t err_code;

//		err_code = sd_ble_gap_scan_stop();
////		if (tx_data[0] || tx_data[1] || tx_data[2] || tx_data[3] || tx_data[4]){
////		if (rx_count){
////				pstorage_handle_t 		flash_handle;
////				pstorage_block_identifier_get(&flash_base_handle, pstorage_block_id , &flash_handle);
////				err_code = pstorage_clear(&flash_handle,1024);
////				APP_ERROR_CHECK(err_code);
////				err_code = pstorage_store(&flash_handle, (uint8_t * )&tx_data, 1024, 0);
////				APP_ERROR_CHECK(err_code);
////				pstorage_block_id++ ;
////				}
////		if ((time_period_count > (3600*2)) && (pstorage_block_id > (PSTORAGE_MAX_APPLICATIONS/2)))	{
//		if ((time_period_count >= 300) || (pstorage_block_id > (PSTORAGE_MAX_APPLICATIONS/2)))	{
//				weakup_flag = true ;
//			}
//}

static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    uint32_t           err_code;
    const ble_gap_evt_t   * p_gap_evt = &p_ble_evt->evt.gap_evt;
//  	simple_uart_put( p_ble_evt->header.evt_id);
//  	simple_uart_put( 0xFF);

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_ADV_REPORT:
        {
				if ((p_ble_evt->evt.gap_evt.params.adv_report.data[2]) == 0x81 && 
						(p_ble_evt->evt.gap_evt.params.adv_report.data[3]) == 0x58 && 
				  	(p_ble_evt->evt.gap_evt.params.adv_report.scan_rsp )   &&							
						(p_gap_evt->params.adv_report.dlen>3) )
					  {
							tx_data[1]=0;
							for (uint8_t i=6;i>0;i--){ 						
	//							simple_uart_put( p_gap_evt->params.adv_report.peer_addr.addr[i-1] );
								tx_data[8-i]=p_gap_evt->params.adv_report.peer_addr.addr[i-1];
								}
							for (uint8_t i=4;i < 28 ;i++)     //27 is ADV data length
								{	
								tx_data[i+4]=p_ble_evt->evt.gap_evt.params.adv_report.data[i];
	//							simple_uart_put( p_ble_evt->evt.gap_evt.params.adv_report.data[i]);
								}
						  tx_data[0] = p_ble_evt->evt.gap_evt.params.adv_report.rssi;
							*(uint32_t *)(tx_data + 12) += timer_counter - *(uint32_t *)(tx_data + 8 );
//																												 	 +  *(uint32_t *)(tx_data + 12 );
							*(uint32_t *)(tx_data + 8) = timer_counter; //- *(uint32_t *)(tx_data + 8 )
																												   //			 +  *(uint32_t *)(tx_data + 12 );
//							for (uint8_t i = 0 ; i <32 ; i++) simple_uart_put( tx_data[i]);
									static pstorage_handle_t 		flash_handle;
									err_code = pstorage_block_identifier_get(&flash_base_handle, pstorage_block_id , &flash_handle);
									APP_ERROR_CHECK(err_code);
									err_code = pstorage_store(&flash_handle, tx_data, 32  , 0);
									APP_ERROR_CHECK(err_code);
									pstorage_block_id++;
									app_timer_stop(weakup_meantimer_id);
									app_timer_start(weakup_meantimer_id,  APP_TIMER_TICKS(2000, 0), NULL);
									if (pstorage_block_id >= (PSTORAGE_MAX_APPLICATIONS*32)) {
												sd_ble_gap_scan_stop();
												scan_start = false;
										    weakup_flag = true;
									}
								}
					break;
					}
        case BLE_GAP_EVT_TIMEOUT:
						{
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN) 
//											scan_stop_flash_write(NULL);
							break;
							}
				default:
						break;
			}
}

static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
//  	simple_uart_put( sys_evt);
//  	simple_uart_put( 0x33);

}

static void example_cb_handler(pstorage_handle_t  * handle,
															 uint8_t              op_code,
                               uint32_t             result,
                               uint8_t            * p_data,
                               uint32_t             data_len)
{
		if(handle->block_id == pstorage_wait_handle) { 
					pstorage_wait_flag = 0; 
				}  //If we are waiting for this callback, clear the wait flag.
}

static void sd_role_enable(uint8_t sd_role)
{
		uint32_t err_code;
		err_code = sd_softdevice_disable();
		APP_ERROR_CHECK(err_code);

		//softdevice enable
			SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, false);
//    SOFTDEVICE_HANDLER_APPSH_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, true);

// Enable BLE stack.
		ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));

    ble_enable_params.gatts_enable_params.service_changed = false;
    ble_enable_params.gap_enable_params.role              = sd_role;  //BLE_GAP_ROLE_CENTRAL;

    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

		err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
		APP_ERROR_CHECK(err_code);
		err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
		APP_ERROR_CHECK(err_code);
}

static void weakup_meantimeout_handler(void)
{
		sd_ble_gap_scan_stop();
		scan_start = false;
}
static void weakup_timeout_handler(void * p_context)
{		
    UNUSED_PARAMETER(p_context);
//			nrf_gpio_pin_set(LED_PIN);
//	  nrf_delay_ms(1);
//			nrf_gpio_pin_clear(LED_PIN);

		uint32_t err_code;
		timer_counter += TIME_PERIOD ;
		time_period_count += TIME_PERIOD;
			if ((time_period_count >= 3600) )	{
					weakup_flag = true ;
		}

		if (weakup_flag) return;

//		memset(tx_data,0,MAX_rx_count);
//		rx_count = 0;
		
//		battery_start(ADC_CONFIG_PSEL_AnalogInput5);
//		batt_lvl_in_milli_volts=(((NRF_ADC->RESULT) * 1200) / 255) * 3 ;
//    if (batt_lvl_in_milli_volts + 1600 < 3700 ){
//							return;
//						}
//		if (pstorage_block_id<(PSTORAGE_FLASH_PAGE_SIZE / PSTORAGE_BLOCK_SIZE) * PSTORAGE_MAX_APPLICATIONS) {
			if (pstorage_block_id < (PSTORAGE_MAX_APPLICATIONS*32 )){
						if (!(timer_counter % 15) && !scan_start){
	//						sd_role_enable(BLE_GAP_ROLE_CENTRAL);
							err_code = sd_ble_gap_scan_stop();
							m_scan_param.active       = 1;            // Active scanning set.
							m_scan_param.selective    = 0;            // Selective scanning not set.
							m_scan_param.interval     = 6;  //0x4000;   //10;     //0x10A0;     // Scan interval. in 0.625ms unit
							m_scan_param.window       = 5;   //0x3ffD;   //8;     //0x109E;   // Scan window.
							m_scan_param.p_whitelist  = NULL;         // No whitelist provided.
							m_scan_param.timeout      = 0;       // in seconds
							err_code = sd_ble_gap_scan_start(&m_scan_param);
							APP_ERROR_CHECK(err_code);
							scan_start = true;
							app_timer_stop(weakup_meantimer_id);
							app_timer_start(weakup_meantimer_id,  APP_TIMER_TICKS(1100, 0), NULL);
							}
					}		
				else if (time_period_count > 600) {
										weakup_flag = true;
										time_period_count = 0;
										}	
}

#define APP_TIMER_PRESCALER             0                                          /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            4                 /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                          /**< Size of timer operation queues. */

static void timers_init(void)
{
    uint32_t err_code;

    // Initialize timer module, making it use the scheduler.
//    APP_TIMER_APPSH_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, true);
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create battery timer.
			err_code = app_timer_create(&weakup_timer_id,
																	APP_TIMER_MODE_REPEATED,
																	weakup_timeout_handler);
			APP_ERROR_CHECK(err_code);
			err_code = app_timer_create(&weakup_meantimer_id,
																	APP_TIMER_MODE_SINGLE_SHOT,
																	weakup_meantimeout_handler);
			APP_ERROR_CHECK(err_code);

}
//void radio_notification_evt_handler(bool radio_evt){
//	static uint16_t radio_count=0;
//	
//  radio_count++;	
//}
int main(void)
{
		uint32_t err_code;
//	sd_power_dcdc_mode_set  ( NRF_POWER_DCDC_MODE_ON);
//	sd_power_dcdc_mode_set  ( NRF_POWER_DCDC_ENABLE);
		sd_role_enable(BLE_GAP_ROLE_CENTRAL);
    ble_gap_addr_t  	p_addr;
    sd_ble_gap_address_get	(	&p_addr	);
		p_addr.addr [0] = 0x58;
		p_addr.addr [1] = 0x81;
    err_code = sd_ble_gap_address_set	(	BLE_GAP_ADDR_CYCLE_MODE_NONE, &p_addr	);
		APP_ERROR_CHECK(err_code);
		timers_init();
		pstorage_module_param_t		 param;
		pstorage_init();
	
//		err_code = ble_radio_notification_init(1,NRF_RADIO_NOTIFICATION_DISTANCE_800US,radio_notification_evt_handler);
//    APP_ERROR_CHECK(err_code);
	
//		param.block_size  = PSTORAGE_BLOCK_SIZE;                   //Select block size of 16 bytes
//		param.block_count =(PSTORAGE_FLASH_PAGE_SIZE / PSTORAGE_BLOCK_SIZE) * PSTORAGE_MAX_APPLICATIONS;    //Select 10 blocks, total of 160 bytes
		param.block_size  = 32;                   //Select block size of 16 bytes
		param.block_count = PSTORAGE_MAX_APPLICATIONS * (1024/32) ;    //Select 10 blocks, total of 160 bytes
		param.cb          = example_cb_handler;  								//Set the pstorage callback handler
		err_code = pstorage_register(&param, &flash_base_handle);
		APP_ERROR_CHECK(err_code);
//							err_code = pstorage_store(&flash_base_handle, (uint8_t * )&tx_data, PSTORAGE_BLOCK_SIZE , 0);
//							APP_ERROR_CHECK(err_code);
		pstorage_handle_t 		flash_handle;
		for (pstorage_block_id = param.block_count  ; pstorage_block_id > 0 ; pstorage_block_id--){
					err_code = pstorage_block_identifier_get(&flash_base_handle, pstorage_block_id -1 , &flash_handle);
					APP_ERROR_CHECK(err_code);
					err_code = pstorage_load(tx_data, &flash_handle, 32 , 0);
					APP_ERROR_CHECK(err_code);
					if (*(uint64_t * )(tx_data) != 0xffffffffffffffff) break;
		}
		
		NRF_GPIO->PIN_CNF[GTM900_power_pin] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
																							| (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
																							| (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
																							| (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
																							| (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
//		NRF_GPIO->PIN_CNF[LED_PIN] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
//																							| (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
//																							| (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
//																							| (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
//																							| (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
//		nrf_gpio_pin_clear(LED_PIN);
		err_code = app_timer_start(weakup_timer_id,  APP_TIMER_TICKS(1000, 0), NULL) ;
		APP_ERROR_CHECK(err_code);
		weakup_flag=1;
		while ((timer_counter < 0x01200000) || (pstorage_block_id)) {
					nrf_gpio_pin_clear(GTM900_power_pin);
					nrf_delay_ms(10000);
					nrf_gpio_pin_set(GTM900_power_pin);
					NRF_UART0->POWER = (UART_POWER_POWER_Enabled << UART_POWER_POWER_Pos);
			////						simple_uart_config(NULL, 2, NULL, 3, false);  // for GPS box PCB
			//						simple_uart_config(NULL, 24, NULL, 23, false);  // for GPS box PCB
			//						simple_uart_config(NULL, 10, NULL, 11, false);  // for GPS box PCB
					battery_start(ADC_CONFIG_PSEL_AnalogInput5);
					batt_lvl_in_milli_volts=(((NRF_ADC->RESULT) * 1200) / 255) * 3 ;
					if (batt_lvl_in_milli_volts > 3500){
							simple_uart_config(NULL, 16, NULL, 8, false);  // for GPS box PCB
							gprs_gtm900();
							send_string("AT+CIPSHUT\r\n","OK");
							send_string("AT+CGATT=0\r\n","OK");
							send_string("AT+CPOWD=1\r\n","POWER");
					}
					nrf_gpio_pin_clear(GTM900_power_pin);
					NRF_UART0->POWER = (UART_POWER_POWER_Disabled << UART_POWER_POWER_Pos);
		} 
	weakup_flag = 0;
//	flash_handle = flash_base_handle;
//	for (uint8_t i = 0; i < PSTORAGE_MAX_APPLICATIONS; i ++)  {
//				err_code = pstorage_clear(&flash_handle, PSTORAGE_BLOCK_SIZE );
//				APP_ERROR_CHECK(err_code);
//				nrf_delay_ms(20);
////				err_code = sd_app_evt_wait();
////				APP_ERROR_CHECK(err_code);    
//				flash_handle.block_id += 1024;
//	}
		for (;;)
			{
				if (weakup_flag){
		        if (scan_start) {
								sd_ble_gap_scan_stop();
								scan_start = false;
						}
						time_period_count = 0;
						nrf_gpio_pin_clear(GTM900_power_pin);
						nrf_delay_ms(100);
						nrf_gpio_pin_set(GTM900_power_pin);

						NRF_UART0->POWER = (UART_POWER_POWER_Enabled << UART_POWER_POWER_Pos);
//						simple_uart_config(NULL, 8, NULL, 9, false);
//						simple_uart_config(NULL, 24, NULL, 23, false);  // for GPS box PCB
						battery_start(ADC_CONFIG_PSEL_AnalogInput5);
						batt_lvl_in_milli_volts=(((NRF_ADC->RESULT) * 1200) / 255) * 3 ;
						if (batt_lvl_in_milli_volts > 3500){
								simple_uart_config(NULL, 16, NULL, 8, false);  // for GPS box PCB
								gprs_gtm900();
								if ((pstorage_block_id) && 
										send_string("\r\n\x1a","OK")  &&
										send_string("AT+CIPSEND\r\n",">") &&
										send_string("\r\n\x2e\r\n\x1a","250") &&
										send_string("AT+CIPCLOSE=1\r\n","OK")) {};
								send_string("AT+CIPSHUT\r\n","OK");
								send_string("AT+CGATT=0\r\n","OK");
								send_string("AT+CPOWD=1\r\n","POWER");
						}
						nrf_gpio_pin_clear(GTM900_power_pin);
						NRF_UART0->POWER = (UART_POWER_POWER_Disabled << UART_POWER_POWER_Pos);
						weakup_flag = false ;
//						pstorage_block_id=0;     //test should be delete!
//						err_code = pstorage_clear(&flash_base_handle,PSTORAGE_FLASH_PAGE_SIZE);
						}
				err_code = sd_app_evt_wait();
				APP_ERROR_CHECK(err_code);    
				}
}
