#ifndef INC_SIM800_H_
#define INC_SIM800_H_

#include <stdio.h>
#include "stm32f0xx_hal.h"

uint8_t get_length();
void init_sim800_udp(UART_HandleTypeDef *huart);
void sim800_send_udp_data(UART_HandleTypeDef *huart, uint8_t u1, uint8_t u2, uint8_t u3,
																											uint8_t u4, uint8_t u5, uint8_t u6,
																											float i1, float i2, float i3);

#endif /* INC_SIM800_H_ */
