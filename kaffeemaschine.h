/*
 * senseo.h
 *
 *  
 *
 */
#ifndef INC_SENSEO_H_
#define INC_SENSEO_H_

#define PRINT_TEXT HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY)

#define TIME_TO_FILL_ONE_CUP 30
#define TIME_TO_FILL_TWO_CUPS 60

#define OFF         0
#define ON			1
#define FASTBLINK	2
#define SLOWBLINK	3

#define PRESSED		1
#define RELEASED	0
#define NOCHANGE	0

#define FORCED		1
#define NOTFORCED	0


#endif /* INC_SENSEO_H_ */
