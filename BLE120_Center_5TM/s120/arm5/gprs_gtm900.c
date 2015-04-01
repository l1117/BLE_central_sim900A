void send_string_no_answer(char * S)
{
   while(*S)
    {
        simple_uart_put(*S++);
				}
}
void send_string(char * S)
{
   while(*S)
    {
        simple_uart_put(*S++);
				}
		uint16_t i=2;
		memset(rx_data,0,522);
		while(simple_uart_get_with_timeout(5000,rx_data+i)){
				if (rx_data[i-1]=='O' && rx_data[i]=='K') break ;
				if (rx_data[i-3]=='C' && rx_data[i-2]=='O' && rx_data[i-1]=='N' && rx_data[i]=='N') break ;
				if (rx_data[i-3]=='E' && rx_data[i-2]=='R' && rx_data[i-1]=='R' && rx_data[i]=='O') break ;
				i++;
				}
}

 char char_hex( char bHex){
    if((bHex>=0)&&(bHex<=9))
        bHex += 0x30;
    else if((bHex>=10)&&(bHex<=15))//????
        bHex += 0x37;
    else bHex = 0xff;
    return bHex;
}
void gprs_gtm900()
{
//GTM900C 
//		simple_uart_config(NULL, 8, NULL, 9, false);
//			nrf_delay_ms(2);
			send_string("AT\r\nAT\r\nAT\r\n");
			send_string("ATE0\r\n");
			send_string("AT+CPIN?\r\n");
			send_string("AT%SLEEP=0\r\n");
			send_string("AT+CREG=2\r\n");
			send_string("AT+CGREG=1\r\n");
			send_string("at+cgatt=1\r\n");
//connect smtp server
			send_string("at+csq\r\n");
			send_string("at%ipclose=1\r\n");
			send_string("at+cgdcont=1,\"ip\",\"cmnet\"\r\n");
			send_string("at%etcpip=\"\",\"\"\r\n");
			send_string("AT%IPOPEN=\"TCP\",\"159.226.251.11\",25\r\n");
//send mail
			send_string("AT%IPSEND=\"65686C6F2068656C6F0D0A\"\r\n");			//ehlo helo
			send_string("AT%IPSEND=\"61757468206C6F67696E0D0A\"\r\n");		//auth login
			send_string("AT%IPSEND=\"59324E6B59773D3D0D0A\"\r\n");				//user name: ccdc
			send_string("AT%IPSEND=\"4D6A49304F484A766233513D0D0A\"\r\n");//passwore: ********
			send_string("AT%IPSEND=\"4D41494C2046524F4D3A3C6C7965406373746E65742E636E3E0D0A\"\r\n");//mail from lye@cstnet.cn
			send_string("AT%IPSEND=\"5243505420746F3A3C63636463406373746E65742E636E3E0D0A\"\r\n");//RCPT to:<ccdc@cstnet.cn>
			send_string("AT%IPSEND=\"444154410D0A\"\r\n");//DATA

			send_string("AT+CREG?\r\n");
			send_string_no_answer("AT%IPSEND=\"5375626A6563743A20");//Subject: 
			for (uint8_t i=6;i<30;i++) {
					simple_uart_put(char_hex(rx_data[i]>>4)); //+CREG: 2,1,"11D4","2048"
					simple_uart_put(char_hex(rx_data[i]&0x0f));
					}
			send_string("0D0A0D0A\"\r\n");
//send data area
			send_string_no_answer("AT%IPSEND=\"");
			for (uint16_t i=0;i<255;i++){
					simple_uart_put(char_hex(tx_data[i]>>4)); 
					simple_uart_put(char_hex(tx_data[i]&0x0f));
					}
			send_string("0D0A0D0A\"\r\n");

			send_string("AT%IPSEND=\"0D0A2E0D0A\"\r\n");//end

			send_string("at%ipclose=1\r\n");

}
