/**
 * Description: For anything that operates the board physcially.
 *
 * Author: Jordan, Matthew, Peter
 *
 * Note: None
 * */

#ifndef INC_OPRT_H_
#define INC_OPRT_H_

/* Includes ------------------------------------------------------------------*/

/* Variables ------------------------------------------------------------------*/
extern uint16_t ambient;
extern uint8_t UART1_rxBuffer[1];

/* Function prototypes ------------------------------------------------------------------*/
int veml3328_start(void);
void veml3328_oprt(void);
void gpio(void);
int debug_key(void);

#endif /* INC_OPRT_H_ */


