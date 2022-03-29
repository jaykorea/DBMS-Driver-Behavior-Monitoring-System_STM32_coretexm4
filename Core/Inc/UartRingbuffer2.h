/*
 * UartRingbuffer.h
 *
 *  Created on: 10-Jul-2019
 *      Author: Controllerstech
 */

#ifndef UARTRINGBUFFER2_H_
#define UARTRINGBUFFER2_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* change the size of the buffer */
#define UART_BUFFER_SIZE2 1024

typedef struct
{
  unsigned char buffer2[UART_BUFFER_SIZE2];
  volatile unsigned int head2;
  volatile unsigned int tail2;
} ring_buffer2;


/* Initialize the ring buffer */
void Ringbuf_init2(void);

/* reads the data in the rx_buffer and increment the tail count in rx_buffer */
int Uart_read2(void);

/* writes the data to the tx_buffer and increment the head count in tx_buffer */
int Uart_write2(int c);

/* function to send the string to the uart */
void Uart_sendstring2(const char *s);

/* Print a number with any base
 * base can be 10, 8 etc*/
void Uart_printbase2(long n, uint8_t base);



/* Checks if the data is available to read in the rx_buffer */
int IsDataAvailable2(void);


/* Look for a particular string in the given buffer
 * @return 1, if the string is found and -1 if not found
 * @USAGE:: if (Look_for ("some string", buffer)) do something
 */
int Look_for2 (char *str, char *buffertolookinto);

/* Copies the required data from a buffer
 * @startString: the string after which the data need to be copied
 * @endString: the string before which the data need to be copied
 * @USAGE:: GetDataFromBuffer ("name=", "&", buffertocopyfrom, buffertocopyinto);
 */
void GetDataFromBuffer2 (char *startString, char *endString, char *buffertocopyfrom, char *buffertocopyinto);


/* Resets the entire ring buffer, the new data will start from position 0 */
void Uart_flush2 (void);

/* Peek for the data in the Rx Bffer without incrementing the tail count 
* Returns the character
* USAGE: if (Uart_peek () == 'M') do something 
*/
int Uart_peek2();


/* Copy the data from the Rx buffer into the bufferr, Upto and including the entered string 
* This copying will take place in the blocking mode, so you won't be able to perform any other operations
* Returns 1 on success and -1 otherwise
* USAGE: while (!(Copy_Upto ("some string", buffer)));
*/
int Copy_upto2(char *string, char *buffertocopyinto);


/* Copies the entered number of characters (blocking mode) from the Rx buffer into the buffer, after some particular string is detected
* Returns 1 on success and -1 otherwise
* USAGE: while (!(Get_after ("some string", 6, buffer)));
*/
int Get_after2 (char *string, uint8_t numberofchars, char *buffertosave);


/* Wait until a paricular string is detected in the Rx Buffer
* Return 1 on success and -1 otherwise
* USAGE: while (!(Wait_for("some string")));
*/
int Wait_for2 (char *string);


/* the ISR for the uart. put it in the IRQ handler */
void Uart_isr2 (UART_HandleTypeDef *huart);



/*** Depreciated For now. This is not needed, try using other functions to meet the requirement ***/
/* get the position of the given string within the incoming data.
 * It returns the position, where the string ends 
 */
//uint16_t Get_position (char *string);

/* once you hit 'enter' (\r\n), it copies the entire string to the buffer*/
//void Get_string (char *buffer);
#ifdef __cplusplus
}
#endif


#endif /* UARTRINGBUFFER2_H_ */
