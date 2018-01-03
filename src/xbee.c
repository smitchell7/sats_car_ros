
#include "xbee.h"

#include "fifo.h"
#include <stdint.h>
#include <string.h>
#include <stddef.h>
#include <stdlib.h>

//Stellarisware header files
#include "inc/hw_gpio.h"
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/pwm.h"
#include "driverlib/i2c.h"
#include "inc/tm4c123gh6pm.h"

#define PACKET_CONTROLDATA   0x00
#define PACKET_CARDATA       0x01
#define PACKET_LOG           0x02
#define PACKET_ACK           0x03
#define PACKET_GETLOG        0x04

static const uint8_t server_addr[] = { 192, 168, 1, 200 };

xbee_ControlDataHandler on_control_data;
xbee_CarDataHandler on_car_data;

// Outgoing UDP frame for xBee
static size_t write_xbee_frame(const uint8_t addr[4], uint8_t * frame, size_t max_len, const uint8_t * msg, size_t msg_len)
{
	if(max_len < msg_len + 16)
		return 0;

	frame[ 0] = 0x7E;
	frame[ 1] = ((msg_len + 12) >> 8) & 0xFF;
	frame[ 2] =  (msg_len + 12)       & 0xFF;
	// API frame id
	frame[ 3] = 0x20;
	// value passed back in Tx status frame, 0x00 disables this
	frame[ 4] = 0x01;
	// destination address
	frame[ 5] = addr[0];
	frame[ 6] = addr[1];
	frame[ 7] = addr[2];
	frame[ 8] = addr[3];
	// destination / source ports
	frame[ 9] = 0x26;
	frame[10] = 0x16;
	frame[11] = 0x26;
	frame[12] = 0x16;
	// protocol (0x00 = UDP, 0x01 = TCP)
	frame[13] = 0x00;
	// terminate socket after tx complete (ignored for UDP)
	frame[14] = 0x00;

	memcpy((void *)(frame + 15), (void *)msg, msg_len);

	uint8_t sum = 0;
	for(int i = 3 ; i < msg_len + 15 ; i ++)
			sum += frame[i];

	frame[15 + msg_len] = 0xFF - sum;

	return msg_len + 16;
}

static uint8_t fifodata[200];
static fifo_t UART1_buffer = FIFO_INITIALIZER(fifodata, 200);

// Transmits until the UART tx buffer is full
static void transmit()
{
	while(UARTSpaceAvail(UART1_BASE))
	{
		uint8_t byte;
		// Pull a byte off the software fifo
		if(fifo_getbyte(&byte, &UART1_buffer))
		{
			UARTCharPutNonBlocking(UART1_BASE, byte);
		}
		else
		{
			break;
		}
	}
}

static bool send_udp(const uint8_t addr[4], const uint8_t * msg, size_t msg_len)
{
	uint8_t fdata[200];
	size_t len = write_xbee_frame(addr, fdata, 200, msg, msg_len);

	if(!len)
		return false;

	if(fifo_putbytes(&UART1_buffer, fdata, len))
	{
		// start sending
		transmit();

		return true;
	}

	return false;
}
static bool send_ack(uint8_t msgid)
{
	uint8_t data[2] = {
		PACKET_ACK,
		msgid
	};
	return send_udp(server_addr, data, 2);
}

#define min(a, b) ((a) < (b) ? (a) : (b))
static void handle_packet(const uint8_t * data, unsigned int datasize)
{
	uint8_t packet_type = data[1];
	
	// Call a different callback based on the packet type received
	if(packet_type == PACKET_CONTROLDATA)
	{
		uint8_t control_id = data[2];
		uint16_t param_num = *(uint16_t *)(data + 3);
		// Here lies the mighty alignment bug, slain July 23rd
		//float * params = *(float *)(data + 5);
		float params[10];
		memcpy(params, data+5, min(10, param_num)*sizeof(float));
		on_control_data(control_id, params, param_num);
	}
	else if(packet_type == PACKET_CARDATA)
	{
		uint8_t flags = data[2];
		uint16_t param_num = *(uint16_t *)(data + 3);
		float params[10];
		memcpy(params, data+5, min(10, param_num)*sizeof(float));
		on_car_data(flags, params, param_num);
	}

	uint8_t msgid = data[0];
	// 0xFF means do not acknowledge
	if(msgid != 0xFF)
	{
		// acknowledge this packet was received
		send_ack(msgid);
	}
}

static uint8_t recv_buffer[100];
static size_t recv_pos = 0;
static size_t expected_len = 0;
static void byte_in(uint8_t byte)
{
	// When we're at the beginning of the buffer,
	// we sure as hell expect to see the beginning
	// of an xbee packet. Anything else, and we'll
	// just ignore it.
	if(recv_pos == 0 && byte != 0x7E)
		return;

	// Add it to the buffer
	recv_buffer[recv_pos ++] = byte;

	// Wrap around, data effectively discarded
	if(recv_pos >= 100)
	{
		recv_pos = 0;
		return;
	}

	// The xbee has told us how much data will be sent
	if(recv_pos == 3)
		expected_len = ((uint32_t)recv_buffer[1] << 8) | recv_buffer[2];

	// Has the full packet been received? ( Header + length bytes + checksum = 4 )
	if(recv_pos == expected_len + 4)
	{
		// do a checksum on the packet
		uint8_t sum = 0;
		for(int i = 3 ; i < expected_len + 4 ; i ++)
			sum += recv_buffer[i];
		
		// 0xB0 is the code for an incoming udp packet
		// 14 is the byte offset where the udp data starts
		if(sum == 0xFF && recv_buffer[3] == 0xB0)
			handle_packet(recv_buffer + 14, expected_len);
		
		/* dummy breakpoint statements
		if(recv_buffer[3] == 0x89)
			recv_pos = 0;
		if(recv_buffer[3] == 0xFE)
			recv_pos = 0;
		
		*/
		// repeat
		recv_pos = 0;
	}
}

void xbee_uart_handler(void)
{
	uint32_t status = UARTIntStatus(UART1_BASE, true);
	
	if(status & (UART_INT_RX | UART_INT_RT))
	{
		UARTIntClear(UART1_BASE, UART_INT_RX | UART_INT_RT);
		
		while(UARTCharsAvail(UART1_BASE))
		{
			byte_in(UARTCharGetNonBlocking(UART1_BASE));
		}
	}
	
	if(status & UART_INT_TX)
	{
		UARTIntClear(UART1_BASE, UART_INT_TX);
		
		transmit();
	}
}

void xbee_log(float time_s, float * datas)
{
	const size_t msg_len = 1 + 4 + 40;
	uint8_t msg[msg_len];
	msg[0] = PACKET_LOG; // log packet id
	memcpy(msg + 1, &time_s, sizeof(float));
	//for(int i = 0 ; i < 10; i ++)
	memcpy(msg + 5, datas, 10*sizeof(float));
	send_udp(server_addr, msg, msg_len);
}
void xbee_init(xbee_ControlDataHandler _on_control_data, xbee_CarDataHandler _on_car_data)
{
	on_control_data = _on_control_data;
	on_car_data = _on_car_data;

	//****************************************************************************
	//Configure UART0: USB cable
	//****************************************************************************
	/*
	//Enable the GPIO Peripheral used by UART0.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	//Enable UART0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	//Configure GPIO Pins for UART mode.
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	//Use the internal 16MHz oscillator as the UART clock source.
	UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
	//Initialize the UART for console I/O.
	//UARTStdioConfig(0, 115200, 16000000);
	*/
	
	//****************************************************************************
	//Configure UART1: Xbee
	//****************************************************************************
	//Enable the GPIO Peripheral used by UART1.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	//Enable UART1
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
	//Configure GPIO Pins for UART mode.
	GPIOPinConfigure(GPIO_PB0_U1RX);
	GPIOPinConfigure(GPIO_PB1_U1TX);
	GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	//Use the internal 16MHz oscillator as the UART clock source.
	UARTClockSourceSet(UART1_BASE, UART_CLOCK_PIOSC);
	//Enable the FIFO. 
	UARTFIFOEnable(UART1_BASE);
	UARTFIFOLevelSet(UART1_BASE, UART_FIFO_TX1_8, UART_FIFO_RX7_8);
	//Initialize the UART for console I/O.
	//UARTStdioConfig(1, 115200, 16000000);
	UARTConfigSetExpClk(
		UART1_BASE, 
		16000000, 
		115200, 
		UART_CONFIG_WLEN_8 |
		UART_CONFIG_STOP_ONE |
		UART_CONFIG_PAR_NONE);
	
	//UARTEchoSet(false);
	
	// Setup the interrupt for the UART1 receive. 
	IntEnable(INT_UART1);
	UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_TX | UART_INT_RT);
}
