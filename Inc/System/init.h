#include "main.h"

extern ADC_HandleTypeDef hadc;
extern COMP_HandleTypeDef hcomp1;
extern IWDG_HandleTypeDef hiwdg;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim14;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;
extern DMA_HandleTypeDef hdma_tim2_up;
extern DMA_HandleTypeDef hdma_tim2_ch3;
extern DMA_HandleTypeDef hdma_tim15_ch1_up_trig_com;
extern int timer_one_period;

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_ADC_Init(void);
void MX_COMP1_Init(void);
void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_TIM15_Init(void);
void MX_IWDG_Init(void);
void MX_TIM16_Init(void);
void MX_TIM14_Init(void);
void MX_TIM6_Init(void);
void MX_TIM17_Init(void);
void MX_TIM15_Init_PWM(void);
