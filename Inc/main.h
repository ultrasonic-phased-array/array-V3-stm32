
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

#define LED_READY_Pin GPIO_PIN_0
#define LED_READY_GPIO_Port GPIOA
#define LED_ACT_Pin GPIO_PIN_1
#define LED_ACT_GPIO_Port GPIOA
#define LED_FAULT_Pin GPIO_PIN_3
#define LED_FAULT_GPIO_Port GPIOA
#define VOUT_Pin GPIO_PIN_0
#define VOUT_GPIO_Port GPIOB
#define VIN_Pin GPIO_PIN_1
#define VIN_GPIO_Port GPIOB
#define CUR_Pin GPIO_PIN_12
#define CUR_GPIO_Port GPIOB
#define US_PP_Pin GPIO_PIN_13
#define US_PP_GPIO_Port GPIOB
#define NTC1_Pin GPIO_PIN_14
#define NTC1_GPIO_Port GPIOB
#define NTC2_Pin GPIO_PIN_15
#define NTC2_GPIO_Port GPIOB

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */

/**
  * @}
*/

#endif /* __MAIN_H */
