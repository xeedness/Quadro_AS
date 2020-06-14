/*
 * conf_usart.h
 *
 * Created: 12.10.2018 01:30:07
 *  Author: xeedn
 */ 


#ifndef CONF_USART_H_
#define CONF_USART_H_

#define CONF_UART            CONSOLE_UART
#define CONF_UART_CONTROL	 USART0
/** Baudrate setting */
#define CONF_UART_BAUDRATE   115200
/** Parity setting */
#define CONF_UART_PARITY     UART_MR_PAR_NO

/*#define USART_SERIAL                 USART0
#define USART_SERIAL_ID              ID_USART0  //USART0 for sam4l
#define USART_SERIAL_BAUDRATE        9600
#define USART_SERIAL_CHAR_LENGTH     US_MR_CHRL_8_BIT
#define USART_SERIAL_PARITY          US_MR_PAR_NO
#define USART_SERIAL_STOP_BIT        US_MR_NBSTOP_1_BIT*/

#endif /* CONF_USART_H_ */