#ifndef INC_SIM800_H_
#define INC_SIM800_H_

#include <stdio.h>
#include "stm32f0xx_hal.h"

uint8_t get_length();
void init_sim800_udp(UART_HandleTypeDef *huart);
void sim800_send_udp_data(UART_HandleTypeDef *huart, uint8_t u1, uint8_t u2, uint8_t u3,
																											uint8_t u4, uint8_t u5, uint8_t u6,
																											uint8_t i1_d, uint8_t i1_f, uint8_t i2_d, uint8_t i2_f, uint8_t i3_d, uint8_t i3_f,
																											uint8_t i4_d, uint8_t i4_f, uint8_t i5_d, uint8_t i5_f, uint8_t i6_d, uint8_t i6_f);

#endif /* INC_SIM800_H_ */
