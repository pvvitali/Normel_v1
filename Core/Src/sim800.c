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

void sim800_send_udp_data(UART_HandleTypeDef *huart, uint8_t u1, uint8_t u2, uint8_t u3,
																											uint8_t u4, uint8_t u5, uint8_t u6,
																											uint8_t i1_d, uint8_t i1_f, uint8_t i2_d, uint8_t i2_f, uint8_t i3_d, uint8_t i3_f,
																											uint8_t i4_d, uint8_t i4_f, uint8_t i5_d, uint8_t i5_f, uint8_t i6_d, uint8_t i6_f){
	
//"00001-00220-00230-00240-00220-00220-00220-002.1-002.2-002.3-002.4-002.5-002.6-00000-00000-00000-00000#"
//"00001-00%03u-00%03u-00%03u-00%03u-00%03u-00%03u-0%02u.%01u-0%02u.%01u-0%02u.%01u-0%02u.%01u-0%02u.%01u-0%02u.%01u-00000-00000-00000-00000#%c"

	sprintf( (char *)data_uart, "AT+CIPSEND\r\n");
	HAL_UART_Transmit(huart, data_uart, get_length(), 100);
	HAL_Delay(100);
		
	sprintf( (char *)data_uart, "00001-00%03u-00%03u-00%03u-00%03u-00%03u-00%03u-0%02u.%01u-0%02u.%01u-0%02u.%01u-0%02u.%01u-0%02u.%01u-0%02u.%01u-00000-00000-00000-00000#%c",
		u1, u2, u3, u4, u5, u6, i1_d, i1_f, i2_d, i2_f, i3_d, i3_f, i4_d, i4_f, i5_d, i5_f, i6_d, i6_f, 0x1A);
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