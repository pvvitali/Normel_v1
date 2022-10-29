#include "sim800.h"


//UART
#define MAX_DATA_UART 110
uint8_t data_uart[MAX_DATA_UART] = {0,};

void init_sim800_udp(UART_HandleTypeDef *huart){
	
	sprintf( (char *)data_uart, "AT\r\n");
	HAL_UART_Transmit(huart, data_uart, get_length(), 100);
	HAL_Delay(300);	
	
	sprintf( (char *)data_uart, "AT\r\n");
	HAL_UART_Transmit(huart, data_uart, get_length(), 100);
	HAL_Delay(300);
	
	sprintf( (char *)data_uart, "AT+CSTT=\"wap.orange.md\"\r\n");
	HAL_UART_Transmit(huart, data_uart, get_length(), 100);
	HAL_Delay(300);
	
	sprintf( (char *)data_uart, "AT+CIICR\r\n");
	HAL_UART_Transmit(huart, data_uart, get_length(), 100);
	HAL_Delay(1000);	
	
	sprintf( (char *)data_uart, "AT+CIFSR\r\n");
	HAL_UART_Transmit(huart, data_uart, get_length(), 100);
	HAL_Delay(1000);	

	sprintf( (char *)data_uart, "AT+CIPSTART=\"UDP\",\"195.133.145.188\",\"50003\"\r\n");
	HAL_UART_Transmit(huart, data_uart, get_length(), 100);
	HAL_Delay(1000);	

}

void sim800_send_udp_data(UART_HandleTypeDef *huart, uint16_t u1, uint16_t u2, uint16_t u3,
																											uint16_t u4, uint16_t u5, uint16_t u6,
																											float i1, float i2, float i3) {
	
//"00001-00220-00230-00240-00220-00220-00220-002.1-002.2-002.3-002.4-002.5-002.6-00000-00000-00000-00000#"
//"00001-00%03u-00%03u-00%03u-00%03u-00%03u-00%03u-0%02u.%01u-0%02u.%01u-0%02u.%01u-0%02u.%01u-0%02u.%01u-0%02u.%01u-00000-00000-00000-00000#%c"

	sprintf( (char *)data_uart, "AT+CIPSEND\r\n");
	HAL_UART_Transmit(huart, data_uart, get_length(), 100);
	HAL_Delay(100);
		
	sprintf( (char *)data_uart, "00001-00%03u-00%03u-00%03u-00%03u-00%03u-00%03u-%05.1f-%05.1f-%05.1f-000.0-000.0-000.0-00000-00000-00000-00000#%c",
		u1, u2, u3, u4, u5, u6, i1, i2, i3, 0x1A);
	HAL_UART_Transmit(huart, data_uart, get_length(), 1000);
	HAL_Delay(1);
	

}

uint8_t get_length(){
		uint8_t count = 0;
		for( uint8_t i =0; i < MAX_DATA_UART; i++){
				if( data_uart[i] == 0 ) return i;
		}
		return 0;
}