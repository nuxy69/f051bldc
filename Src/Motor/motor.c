#include "motor/motor.h"
int step = 1;
int rising = 1;
int zcfound = 1;
int startcount = 0;
int advance = 0;   // set proportianal to commutation time. with advance divisor
int blanktime;
int degree_time;
int falsecount = 0;
int falsethreshold = 2;
int thiszctime = 0;
int waitTime = 0;
int threshold = 6;
int upthreshold = 6;
int compcount = 0;
int upcompcount = 0;
int stop_time = 0;

void commutate() {
//	TIM2->CNT = 0;
	if (forward == 1) {
		step++;
		if (step > 6) {
			step = 1;
		}
		if (step == 1 || step == 3 || step == 5) {
			rising = 1;                         // is back emf rising or falling
		}
		if (step == 2 || step == 4 || step == 6) {
			rising = 0;
		}
	}
	if (forward == 0) {
		step--;
		if (step < 1) {
			step = 6;
		}
		if (step == 1 || step == 3 || step == 5) {
			rising = 0;
		}
		if (step == 2 || step == 4 || step == 6) {
			rising = 1;
		}
	}

	if (input > 47) {
		comStep(step);
	}

	if ((bemf_counts > 50 && duty_cycle > 180)) {
		polling_mode = 0;

	} else {
		polling_mode = 1;
	}
	if (duty_cycle < 180 || commutation_interval > 5000) {
		polling_mode = 1;
	}

	if (!polling_mode) {
		changeCompInput();
		EXTI->IMR |= (1 << 21);
	} else {
		pollingChangeCompInput();
	}
	zcfound = 0;
	commutation_interval = ((2 * commutation_interval) + thiszctime) / 3;
	degree_time = commutation_interval >> 5; // about 1.85 degrees per unit divided by 32
	advance = degree_time * advance_multiplier; //  * 16 would be about 30 degrees
	waitTime = (commutation_interval >> 1) - advance;
	if (waitTime < 0) {
		waitTime = 0;
	}
	blanktime = commutation_interval >> 4;                       // divided by 8
	bemf_counts++;
}

void startMotor() {
	startcount++;
	char decaystate = comp_pwm;
	sensorless = 0;

	if (running == 0) {
		EXTI->IMR &= ~(1 << 21);
		EXTI->PR &= ~(1 << 21);
		comp_pwm = 1;
		commutate();
		commutation_interval = 10000;
		TIM3->CNT = 0;
		running = 1;
		if (!polling_mode) {
			EXTI->IMR |= (1 << 21);
		}
	} else {
		if (HAL_COMP_Start(&hcomp1) != HAL_OK) {
			/* Initialization Error */
			Error_Handler();
		}
	}
	comp_pwm = decaystate;    // return to normal
	sensorless = 1;
	bemf_counts = 0;

}

void forcedCommutation() {
	HAL_COMP_Stop_IT(&hcomp1);
	TIM3->CNT = commutation_interval / 2;
	commutate();
	while (TIM3->CNT - commutation_interval / 2 < blanktime) {
	}
	if (HAL_COMP_Start_IT(&hcomp1) != HAL_OK) {
		/* Initialization Error */
		Error_Handler();
	}

}

void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp) {
	if ((TIM3->CNT < commutation_interval >> 1) && bemf_counts > 3) {
		return;
	}

	if (commutation_interval > 500) {
		while (TIM3->CNT - thiszctime < filter_delay) {
		}
	}

	compit += 1;
	if (compit > 100) {
		EXTI->IMR &= ~(1 << 21);
		EXTI->PR &= ~(1 << 21);
		error = 1;
		return;
	}

	for (int i = 0; i < filter_level; i++) {
		if (rising == (COMP1->CSR & 1 << 14) >> 14) { // if the comparator output is not what is expected
			return;
		}
	}
	EXTI->IMR &= ~(1 << 21);        // turn off interrupts and pending requests.
	EXTI->PR &= ~(1 << 21);
	thiszctime = TIM3->CNT;
	TIM3->CNT = 0;
	zctimeout = 0;
	TIM6->CNT = 0;
	TIM6->ARR = waitTime;
	__HAL_TIM_ENABLE_IT(&htim6, TIM_IT_UPDATE);
}

void checkForZeroCross() {
	if (!zcfound) {
		if (rising == 0) {
			if (HAL_COMP_GetOutputLevel(&hcomp1) == COMP_OUTPUTLEVEL_LOW) {
				falsecount++;
				if (falsecount > falsethreshold) {
					compcount = 0;
					zcfound = 0;
					falsecount = 0;
				}
			}

			if (HAL_COMP_GetOutputLevel(&hcomp1) == COMP_OUTPUTLEVEL_HIGH) {
				compcount++;
			}
			if (compcount > threshold) {
				zcfound = 1;
				zctimeout = 0;
				compcount = 0;
				bemf_counts++;
				falsecount = 0;
				thiszctime = TIM3->CNT;
				TIM3->CNT = 0;

				commutation_interval = ((2 * commutation_interval) + thiszctime)
						/ 3;
				degree_time = commutation_interval >> 5; // about 1.85 degrees per unit
				advance = degree_time * advance_multiplier; //  * 16 would be about 30 degrees
				waitTime = (commutation_interval >> 1) - advance;

				if (sensorless) {
					while (TIM3->CNT < waitTime) {
					}
					commutate();

				}
				zcfound = 0;

			}
		}

		if (rising == 1) {
			if (HAL_COMP_GetOutputLevel(&hcomp1) == COMP_OUTPUTLEVEL_HIGH) {
				falsecount++;
				if (falsecount > falsethreshold) {
					upcompcount = 0;
					zcfound = 0;
					falsecount = 0;
				}

			}

			if (HAL_COMP_GetOutputLevel(&hcomp1) == COMP_OUTPUTLEVEL_LOW) {
				upcompcount++;
			}
			if (upcompcount > upthreshold) {
				zcfound = 1;
				zctimeout = 0;
				upcompcount = 0;
				falsecount = 0;
				bemf_counts++;
				thiszctime = TIM3->CNT;
				TIM3->CNT = 0;

				commutation_interval = ((2 * commutation_interval) + thiszctime)
						/ 3;
				degree_time = commutation_interval >> 5; // about 1.85 degrees per unit
				advance = degree_time * advance_multiplier;
				waitTime = (commutation_interval >> 1) - advance;
				if (sensorless) {
					while (TIM3->CNT < waitTime) {
					}
					commutate();
				}
				zcfound = 0;
			}
		}
	}
}

//////////////////////////////////PHASE 1//////////////////////
#ifdef MP6531
void phaseAPWM() {  // phaseB qfnf051 , phase A qfp32
#endif
#ifdef FD6288
void phaseBPWM() {
#endif

	if (!comp_pwm || prop_brake_active) {            // for future
		LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_0, LL_GPIO_MODE_OUTPUT);
		GPIOB->BRR = GPIO_PIN_0;
	} else {
		LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_0, LL_GPIO_MODE_ALTERNATE); // low
	}
	LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);  // high

}

#ifdef MP6531
void phaseAFLOAT() {  // phaseB qfnf051 , phase A qfp32
#endif
#ifdef FD6288
void phaseBFLOAT() {
#endif

	LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_0, LL_GPIO_MODE_OUTPUT);
	GPIOB->BRR = GPIO_PIN_0;
	LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_9, LL_GPIO_MODE_OUTPUT);
	GPIOA->BRR = GPIO_PIN_9;
}

#ifdef MP6531
void phaseALOW() {  // phaseB qfnf051 , phase A qfp32
#endif
#ifdef FD6288
void phaseBLOW() {
#endif

	// low mosfet on
	LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_0, LL_GPIO_MODE_OUTPUT);
	GPIOB->BSRR = GPIO_PIN_0;
	LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_9, LL_GPIO_MODE_OUTPUT);
	GPIOA->BRR = GPIO_PIN_9;
}

//////////////////////////////PHASE 2//////////////////////////////////////////////////

#ifdef MP6531
void phaseBPWM() {                                // phase c qfn , phase b qfp
#endif
#ifdef FD6288
void phaseCPWM() {
#endif

	if (!comp_pwm || prop_brake_active) {
		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_7, LL_GPIO_MODE_OUTPUT);
		GPIOA->BRR = GPIO_PIN_7;
	} else {
		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_7, LL_GPIO_MODE_ALTERNATE);
	}
	LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_8, LL_GPIO_MODE_ALTERNATE);

}

#ifdef MP6531
void phaseBFLOAT() {                                // phase c qfn , phase b qfp
#endif
#ifdef FD6288
void phaseCFLOAT() {
#endif

	// floating
	LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_7, LL_GPIO_MODE_OUTPUT);
	GPIOA->BRR = GPIO_PIN_7;
	LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_8, LL_GPIO_MODE_OUTPUT);
	GPIOA->BRR = GPIO_PIN_8;
}

#ifdef MP6531
void phaseBLOW() {                                // phase c qfn , phase b qfp
#endif
#ifdef FD6288
void phaseCLOW() {
#endif

	// lowside
	LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_7, LL_GPIO_MODE_OUTPUT);
	GPIOA->BSRR = GPIO_PIN_7;
	LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_8, LL_GPIO_MODE_OUTPUT);
	GPIOA->BRR = GPIO_PIN_8;
}

///////////////////////////////////////////////PHASE 3 /////////////////////////////////////////////////

#ifdef MP6531
void phaseCPWM() {                    // phaseA qfn , phase C qfp
#endif
#ifdef FD6288
void phaseAPWM() {
#endif

	if (!comp_pwm || prop_brake_active) {
		LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_1, LL_GPIO_MODE_OUTPUT);
		GPIOB->BRR = GPIO_PIN_1;
	} else {
		LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_1, LL_GPIO_MODE_ALTERNATE);
	}
	LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE);

}

#ifdef MP6531
void phaseCFLOAT() {                    // phaseA qfn , phase C qfp
#endif
#ifdef FD6288
void phaseAFLOAT() {
#endif

	LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_1, LL_GPIO_MODE_OUTPUT);
	GPIOB->BRR = GPIO_PIN_1;
	LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_10, LL_GPIO_MODE_OUTPUT);
	GPIOA->BRR = GPIO_PIN_10;
}

#ifdef MP6531
void phaseCLOW() {                    // phaseA qfn , phase C qfp
#endif
#ifdef FD6288
void phaseALOW() {
#endif

	LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_1, LL_GPIO_MODE_OUTPUT);
	GPIOB->BSRR = GPIO_PIN_1;
	LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_10, LL_GPIO_MODE_OUTPUT);
	GPIOA->BRR = GPIO_PIN_10;
}

void comStep(int newStep) {
	switch (newStep) {

	case 1:			//A-B
		phaseAPWM();
		phaseBLOW();
		phaseCFLOAT();
		break;

	case 2:		// C-B
		phaseAFLOAT();
		phaseBLOW();
		phaseCPWM();
		break;

	case 3:	// C-A
		phaseALOW();
		phaseBFLOAT();
		phaseCPWM();
		break;

	case 4:	// B-A
		phaseALOW();
		phaseBPWM();
		phaseCFLOAT();
		break;

	case 5:    // B-C
		phaseAFLOAT();
		phaseBPWM();
		phaseCLOW();
		break;

	case 6:      // A-C
		phaseAPWM();
		phaseBFLOAT();
		phaseCLOW();
		break;
	}
}

void allOff() {                   // coast
	phaseAFLOAT();
	phaseBFLOAT();
	phaseCFLOAT();
}

void fullBrake() {                     // full braking shorting all low sides
	phaseALOW();
	phaseBLOW();
	phaseCLOW();
}

void proBrake() {                    // duty cycle controls braking strength
	phaseAPWM();
	phaseBPWM();
	phaseCPWM();
}

void pollingChangeCompInput() {

	HAL_COMP_Stop(&hcomp1);

	if (step == 1 || step == 4) {   // c floating
		hcomp1.Init.InvertingInput = COMP_INVERTINGINPUT_IO1;
	}

	if (step == 2 || step == 5) {     // a floating
#ifdef MP6531
		hcomp1.Init.InvertingInput = COMP_INVERTINGINPUT_DAC1; /// if f051k6  step 2 , 5 is dac 1 ( swap comp input)
#endif
#ifdef FD6288
		hcomp1.Init.InvertingInput = COMP_INVERTINGINPUT_DAC2;
#endif
	}

	if (step == 3 || step == 6) {      // b floating
#ifdef MP6531
		hcomp1.Init.InvertingInput = COMP_INVERTINGINPUT_DAC2; /// if f051k6  step 2 , 5 is dac 1 ( swap comp input)
#endif
#ifdef FD6288
		hcomp1.Init.InvertingInput = COMP_INVERTINGINPUT_DAC1;
#endif
	}

	if (HAL_COMP_Init(&hcomp1) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_COMP_Start(&hcomp1) != HAL_OK) {
		/* Initialization Error */
		Error_Handler();
	}

}

void changeCompInput() {
	if (step == 1 || step == 4) {   // c floating
		COMP->CSR = 0b1100001;
	}

	if (step == 2 || step == 5) {     // a floating
#ifdef MP6531
		COMP->CSR = 0b1000001; /// if f051k6  step 2 , 5 is dac 1 ( swap comp input)
#endif
#ifdef FD6288
		COMP->CSR = 0b1010001;
#endif
	}

	if (step == 3 || step == 6) {      // b floating
#ifdef MP6531
		COMP->CSR = 0b1010001;
#endif
#ifdef FD6288
		COMP->CSR = 0b1000001;
#endif
	}
	if (rising) {
		EXTI->RTSR = 0x0;
		EXTI->FTSR = 0x200000;
	} else {                          // falling bemf
		EXTI->FTSR = 0x0;
		EXTI->RTSR = 0x200000;
	}
}

void playStartupTune() {
	TIM1->PSC = 75;
	TIM1->CCR1 = 5;
	TIM1->CCR2 = 5;
	TIM1->CCR3 = 5;
	comStep(3);
	HAL_Delay(100);
	TIM1->PSC = 50;
	HAL_Delay(100);
	TIM1->PSC = 25;
	HAL_Delay(100);
	allOff();
	TIM1->PSC = 0;
}

void playInputTune() {
	TIM1->PSC = 100;
	TIM1->CCR1 = 5;
	TIM1->CCR2 = 5;
	TIM1->CCR3 = 5;
	comStep(6);
	HAL_Delay(100);
	TIM1->PSC = 50;
	HAL_Delay(100);
	allOff();
	TIM1->PSC = 0;
}
