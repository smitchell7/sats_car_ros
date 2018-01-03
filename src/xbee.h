#ifndef XBEE_H
#define XBEE_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#define XBEE_UART_INTERRUPT INT_UART1
extern void xbee_uart_handler(void);

typedef void (* xbee_ControlDataHandler)(uint8_t controlid, const float * params, size_t nparams);
typedef void (* xbee_CarDataHandler)(uint8_t flags, const float * params, size_t nparams);
typedef void (* xbee_LogDataHandler)(uint8_t log_state); 
void xbee_log(float time_s, float * datas);
void xbee_init(xbee_ControlDataHandler on_control_data, xbee_CarDataHandler on_car_data);

#endif
