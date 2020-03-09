/* USER CODE BEGIN Header */
/*
 Firmware for the stm32f051 mcu for controlling brushless or brushed motors, current accepts servo signal input or dshot 300 ( bi-directional )
 other modes disabled for now.
*/
/* Includes ------------------------------------------------------------------*/
#include "main.h"


uint16_t VirtAddVarTab[NB_OF_VAR] = { 0x5555, 0x6666, 0x7777 };
uint16_t VarDataTab[NB_OF_VAR] = { 0, 0, 0 };
uint16_t VarValue, VarDataTmp;

uint32_t ee_status;

// variables to use for eeprom function,  ie: EE_WriteVariable(VirtAddVarTab[EEvehiclemode],  vehicle_mode ;

enum userVars {
	EEvehiclemode = 0, EEdirection = 1, EEbidirection = 2,
//	EEbrake_on_stop = 3
};
char bi_polar = 0;
char sine_mode_range = 52; // 0-52 at 0 there is no sine mode at 52 gives a input range 47-99 sine
int boost_level = 1; // for stall protection the amount that gets added to duty cycle.

char polling_mode = 0;  // for better low speed accuracy
int test_comp_output = 0;
int level = 0;
int dir_reversed = 0;   // global direction reversed set in eeprom
//uint32_t extiline;

int forcedcomcount = 0;
char bad_commutation = 0;

int bi_direction = 0;
char comp_pwm = 1;           // for complementary pwm , 0 for diode freewheeling
int brake = 1;                          // apply full motor brake on stop
int start_power = 200;
char prop_brake = 0;
int prop_brake_strength = 300;
int IC_buffer_size = 64;

char prop_brake_active = 0;
int adjusted_input;

int dshotcommand = 0;
uint8_t calcCRC;
uint8_t checkCRC;
int error = 0;

int count = 0;
int tempbrake = 0;

char advancedivisor = 12;               // increase divisor to decrease advance,
char advancedivisorup = 3;
char advancedivisordown = 3;
int advance_multiplier = 0;
int min_advance_multiplier = 4; // * 2.25 to get advance degrees to 10 would be 22.5 degrees advance.
int max_advance_multiplier = 10;

int dither_count;
int dither_amount = 10;


int lastzctime = 0;
int sensorless = 0;
int commutation_interval = 0;

int pwm_settle = 50;
int demagtime = 5;
char filter_level = 1;
char compit = 0;
int filter_delay = 2;

int filter_level_up = 8;
int filter_level_down = 8;
int forcedcount = 0;
int control_loop_count;
int zctimeout = 0;
int zc_timeout_threshold = 2400;   // depends on speed of main loop

int signaltimeout = 0;
int signal_timeout_threshold = 20000;

int temp_step;
int ROC = 1;

int tim2_start_arr = 9000;

int duty_cycle = 100;
int adjusted_duty_cycle = 0;

int pwm = 1;
int floating = 2;
int lowside = 3;
int bemf_counts;
int k_erpm;

int forward = 1;

int running = 0;
int started = 0;
uint8_t armed = 0;
int armedcount = 0;

int input_buffer_size = 64;
int smallestnumber = 20000;
uint32_t dma_buffer[64];

int input = 0;
int newinput = 0;

char dshot = 0;
char proshot = 0;
char multishot = 0;
char oneshot42 = 0;
char oneshot125 = 0;
char servoPwm = 0;

char brushed_direction_set = 0;

char inputSet = 0;

int phase_A_position;
int phase_B_position;
int phase_C_position;
int step_delay = 4000;
int gate_drive_offset = 100;
int sine_mode = 0;

int input_override;

uint32_t gcrtest[23] =  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint32_t gcr[23] =  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint16_t gcr_encode_table[16] = { 0b11001,
		0b11011,
		0b10010,
		0b10011,
		0b11101,
		0b10101,
		0b10110,
		0b10111,
		0b11010,
		0b01001,
		0b01010,
		0b01011,
		0b11110,
		0b01101,
		0b01110,
		0b01111
};
uint32_t gcrnumber;
int pwmSin[] = {128,130,132,134,136,139,141,143,
				145,147,150,152,154,156,158,160,
				163,165,167,169,171,173,175,177,
				179,181,183,185,187,189,191,193,
				195,197,199,201,202,204,206,208,
				209,211,213,214,216,218,219,221,
				222,224,225,227,228,229,231,232,
				233,234,236,237,238,239,240,241,
				242,243,244,245,246,247,247,248,
				249,249,250,251,251,252,252,253,
				253,253,254,254,254,255,255,255,
				255,255,255,255,255,255,255,255,
				254,254,254,253,253,253,252,252,
				251,251,250,249,249,248,247,247,
				246,245,244,243,242,241,240,239,
				238,237,236,234,233,232,231,229,
				228,227,225,224,222,221,219,218,
				216,214,213,211,209,208,206,204,
				202,201,199,197,195,193,191,189,
				187,185,183,181,179,177,175,173,
				171,169,167,165,163,160,158,156,
				154,152,150,147,145,143,141,139,
				136,134,132,130,128,125,123,121,
				119,116,114,112,110,108,105,103,
				101,99,97,95,92,90,88,86,
				84,82,80,78,76,74,72,70,
				68,66,64,62,60,58,56,54,
				53,51,49,47,46,44,42,41,
				39,37,36,34,33,31,30,28,
				27,26,24,23,22,21,19,18,
				17,16,15,14,13,12,11,10,
				9,8,8,7,6,6,5,4,
				4,3,3,2,2,2,1,1,
				1,0,0,0,0,0,0,0,
				0,0,0,0,1,1,1,2,
				2,2,3,3,4,4,5,6,
				6,7,8,8,9,10,11,12,
				13,14,15,16,17,18,19,21,
				22,23,24,26,27,28,30,31,
				33,34,36,37,39,41,42,44,
				46,47,49,51,53,54,56,58,
				60,62,64,66,68,70,72,74,
				76,78,80,82,84,86,88,90,
				92,95,97,99,101,103,105,108,
				110,112,114,116,119,121,123,125};
int tim1_arr = 0;
int timer_2_period = 55;
int timer_2_prescaler = 2;
int high_bit_length = 65;
int timer_15_prescaler = 0;
char is_output = 0;

int e_com_time = 65408;
int lastnumber = 0;
int shift_amount = 0;
char dshot_telemetry = 0;
char delay_before_output = 0;
int wait_after = 2500;
int wait_before = 700; // delay after getting dshot signal before changing over to output
//int dshot_full_number;
int timer_16_period = 6000;

char blanktime_int = 0;
int duration_in_microseconds;

int vehicle_mode = VEHICLE_MODE;
int commandcount = 0;
int tocheck = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
long map(long x, long in_min, long in_max, long out_min, long out_max) {
	if (x < in_min) {
		x = in_min;
	}
	if (x > in_max) {
		x = in_max;
	}
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void storeEEpromConfig() {

	EE_WriteVariable(VirtAddVarTab[EEvehiclemode], vehicle_mode);
	EE_WriteVariable(VirtAddVarTab[EEdirection], dir_reversed);
	EE_WriteVariable(VirtAddVarTab[EEbidirection], bi_direction);
	// EE_WriteVariable(VirtAddVarTab[EEbrake_on_stop], EEbrake_on_stop);

	// playEEpromSavedTune();
}

void loadEEpromConfig() {
	EE_ReadVariable(VirtAddVarTab[EEvehiclemode], &VarDataTab[EEvehiclemode]);
	EE_ReadVariable(VirtAddVarTab[EEdirection], &VarDataTab[EEdirection]);
	EE_ReadVariable(VirtAddVarTab[EEbidirection], &VarDataTab[EEbidirection]);
//	 EE_ReadVariable(VirtAddVarTab[EEbrake_on_stop], &VarDataTab[EEbrake_on_stop]);

	if (VarDataTab[EEvehiclemode] == 0) {             // nothing in the eeprom
		storeEEpromConfig();            // store default values
	} else {
		vehicle_mode = VarDataTab[EEvehiclemode];
		dir_reversed = VarDataTab[EEdirection];
		bi_direction = VarDataTab[EEbidirection];
//	 brake = VarDataTab[EEbrake_on_stop];
	}
}

int getAbsDif(int number1, int number2) {
	int result = number1 - number2;
	if (result < 0) {
		result = -result;
	}
	return result;
}

void detectInput() {
	smallestnumber = 20000;
	dshot = 0;
	proshot = 0;
	multishot = 0;
	oneshot42 = 0;
	oneshot125 = 0;
	servoPwm = 0;
//	int lastnumber = dma_buffer[0];
	for (int j = 1; j < input_buffer_size; j++) {

		if (dma_buffer[j] < smallestnumber) { // blank space
			smallestnumber = dma_buffer[j];
		}
	}

	if ((smallestnumber > 3) && (smallestnumber < 20)) {
		dshot = 1;
		timer_15_prescaler = 24;
		TIM15->PSC = timer_15_prescaler;
		IC_buffer_size = 32;
		TIM16->ARR = 8000;
	}
	if ((smallestnumber > 20) && (smallestnumber < 40)) {
		dshot = 1;
		timer_15_prescaler = 36;
		TIM15->PSC = timer_15_prescaler;
		TIM16->PSC = 1;
		TIM16->ARR = 8000;
		IC_buffer_size = 32;
	}

	if ((smallestnumber > 40) && (smallestnumber < 55)) {
		dshot = 1;
		timer_15_prescaler = 64;
		TIM2->PSC = 1;
		TIM15->PSC = timer_15_prescaler;
		TIM16->PSC = 1;
		TIM16->ARR = 8000;
		IC_buffer_size = 32;
	}

	if (smallestnumber > 100) {
		servoPwm = 1;
		TIM15->PSC = 47;
		HAL_TIM_Base_Stop(&htim16);
		IC_buffer_size = 6;

	}

	if (smallestnumber == 0) {
		inputSet = 0;
	} else {
		inputSet = 1;
		HAL_Delay(50);
	}
	HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, dma_buffer, IC_buffer_size);
}

void advanceincrement() { // called by timer interrupt when in forced sinusoidal mode.
	if (sine_mode) {
		if (!forward) {
			phase_A_position++;
			if (phase_A_position > 359) {
				phase_A_position = 0;
			}

			phase_B_position++;
			if (phase_B_position > 359) {
				phase_B_position = 0;
			}
			phase_C_position++;
			if (phase_C_position > 359) {
				phase_C_position = 0;
			}
		} else {
			phase_A_position--;
			if (phase_A_position < 0) {
				phase_A_position = 359;
			}

			phase_B_position--;
			if (phase_B_position < 0) {
				phase_B_position = 359;
			}
			phase_C_position--;
			if (phase_C_position < 0) {
				phase_C_position = 359;
			}
		}
		TIM1->CCR1 = (pwmSin[phase_A_position]) + gate_drive_offset;// set duty cycle to 50 out of 768 to start.
		TIM1->CCR2 = (pwmSin[phase_B_position]) + gate_drive_offset;
		TIM1->CCR3 = (pwmSin[phase_C_position]) + gate_drive_offset;
	}
}

void changeToOutput() {
	TIM16->ARR = 65535;
	__HAL_TIM_DISABLE_DMA(&htim15, TIM_DMA_CC1);
	__HAL_TIM_DISABLE_DMA(&htim15, TIM_DMA_UPDATE);
	GPIOA->AFR[0] = 0x200;
	is_output = 1;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM6)  // commutation timer
	{
		__HAL_TIM_DISABLE_IT(&htim6, TIM_IT_UPDATE); // disable update interrupt
		TIM17->CNT = 0;
		commutate();
		duration_in_microseconds = TIM17->CNT;
		return;
	}

	if (htim->Instance == TIM16)  // input timeout reset timer
	{
		if (!is_output) {
			if (inputSet == 1) {
				HAL_TIM_IC_Stop_DMA(&htim15, TIM_CHANNEL_1);
				TIM15->CNT = 0;
				HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, dma_buffer,
						IC_buffer_size);
			}
		} else {
			if (delay_before_output == 1) {
				changeToOutput();
				make_dshot_package();
				senddshotburst();
				TIM16->ARR = timer_16_period;
				delay_before_output = 0;
			}
		}
	}

	if (htim->Instance == TIM14) /// sinusoidal interval timer
	{
		advanceincrement();
	}
}

void setupInput() {
	TIM16->CNT = 0;
	TIM16->ARR = timer_16_period;
	GPIOA->AFR[0] = 0x00;                  // set alternate function to zero
	is_output = 0;
}

void transferComplete() {
	signaltimeout = 0;
	count++;
	if (is_output) { // if transfer complete happened on output the tansfer to dma is done and we can switch back lines and start input capture
		setupInput();
		HAL_TIM_IC_Stop_DMA(&htim15, TIM_CHANNEL_1);
		HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, dma_buffer, 33);
		return;
	}

	if (inputSet == 1) {
		if ((dshot_telemetry) && (!is_output)) {
			if ((dma_buffer[0] > 10) || (dma_buffer[1] > 10)) {
				TIM16->CNT = 0;
				TIM16->ARR = 65535;
				is_output = 1;
				TIM16->ARR = wait_before;
				computeDshotDMA();
				dma_buffer[0] = 0;
				dma_buffer[1] = 0;
				delay_before_output = 1;
			}
			//	return;
		}

		if (!dshot_telemetry) {

			if (dshot == 1) {
				computeDshotDMA();
				return;
			}
			if (proshot == 1) {
				computeProshotDMA();
				return;
			}

			if (servoPwm == 1) {
				computeServoInput();
				HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, dma_buffer, 3);
			}
//		if  (multishot){
//			computeMSInput();
//			HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, dma_buffer , 3);
//
//		}
//		if  (oneshot125){
//			computeOS125Input();
//			HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, dma_buffer , 3);
//
//		}
//		if  (oneshot42){
//			computeOS42Input();
//			HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, dma_buffer , 3);
//          }
		}

	}

}

int main(void) {
	/* MCU Configuration--------------------------------------------------------*/
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */
	HAL_FLASH_Unlock();
	EE_Init();
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC_Init();
	MX_COMP1_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM15_Init();
//  MX_IWDG_Init();
	MX_TIM16_Init();
	MX_TIM14_Init();
	MX_TIM6_Init();
	MX_TIM17_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_Base_Start(&htim6);           // commutation timer
	//HAL_TIM_Base_Start_IT(&htim14);
	HAL_TIM_Base_Start(&htim17);
	HAL_TIM_Base_Start(&htim3);
	//  HAL_Delay(500);
	for (int i = 0; i < vehicle_mode; i++) {
		playStartupTune();
		HAL_Delay(100);
	}
	MX_IWDG_Init();
	if (vehicle_mode == 1) {                    // quad single direction
		loadEEpromConfig();
	}
	if (vehicle_mode == 2) {                   // crawler or thruster
		bi_direction = 1;
		comp_pwm = 1;        // for complementary pwm , 0 for diode freewheeling
		brake = 1;                          // apply full motor brake on stop
	}
	if (vehicle_mode == 3) {              // rc car 50 percent brake on reverse.
		bi_direction = 1;
		comp_pwm = 0;        // for complementary pwm , 0 for diode freewheeling
		brake = 0;                          // apply full motor brake on stop
		prop_brake = 1;
		prop_brake_strength = 900;
	}
	if (vehicle_mode == 4) {              // rc car 50 percent brake on reverse.
		bi_direction = 1;
		comp_pwm = 0;        // for complementary pwm , 0 for diode freewheeling
		brake = 0;                          // apply full motor brake on stop
		prop_brake = 1;
		prop_brake_strength = 800;
	}

	if (vehicle_mode == 5) {                  // quad single direction no eeprom

	}

	if (HAL_COMP_Start_IT(&hcomp1) != HAL_OK) {
		/* Initialization Error */
		Error_Handler();
	}

	if (bi_direction) {
		newinput = 1001;
	}

	if (dir_reversed == 1) {
		forward = 0;
	} else {
		forward = 1;
	}

	//proBrake();
	TIM1->CCR1 = 1;
	TIM1->CCR2 = 1;
	TIM1->CCR3 = 1;

	TIM1->CCR4 = 800;

	if (!BRUSHED_MODE && bi_polar) { // sanity check, turn off bipolar pwm if brushed mode is not selected
		bi_polar = 0;
	}
	if (bi_polar) {
		comp_pwm = 1;
	}
	phase_A_position = 0;
	phase_B_position = 119;
	phase_C_position = 239;

	if (vehicle_mode != 2) {
		sine_mode_range = 0;
	}
	if (sine_mode_range > 52 || sine_mode_range < 0) {
		sine_mode_range = 0;
	}

	DMA1_Channel2->CCR = 0xa90; // for output compare
	DMA1_Channel1->CCR = 0xa91;     // for output compare
	DMA1_Channel5->CCR = 0x981;    // for input capture
	__HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_UPDATE);
	__HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC3);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		if (count > 100000) {
			count = 0;
		}
		compit = 0;
		if (HAL_IWDG_Refresh(&hiwdg) != HAL_OK)              // watchdog refresh
				{
			/* Refresh Error */
			Error_Handler();
		}
		control_loop_count++;
		if (control_loop_count > 2) {

			control_loop_count = 0;

			//	  		1-5: beep (1= low freq. 5 = high freq.)
			//	  		6: ESC info request (FW Version and SN sent over the tlm wire)
			//	  		7: rotate in one direction
			//	  		8: rotate in the other direction
			//	  		9: 3d mode off
			//	  		10: 3d mode on
			//	  		11: ESC settings request (saved settings over the TLM wire) (planed but not there yet)
			//	  		12: save Settings

			if (dshotcommand > 0) {
				if (dshotcommand == 2) {
					playInputTune();
				}
				if (dshotcommand == 21) {
					forward = dir_reversed;

				}
				if (dshotcommand == 20) {     // forward = 1 if dir_reversed = 0
					forward = 1 - dir_reversed;
				}
				if (dshotcommand == 7) {
					dir_reversed = 0;

				}
				if (dshotcommand == 8) {
					dir_reversed = 1;

				}
				if (dshotcommand == 9) {
					bi_direction = 0;
					armed = 0;

				}
				if (dshotcommand == 10) {
					bi_direction = 1;
					armed = 0;
				}
				if (dshotcommand == 12) {
					storeEEpromConfig();
					while (1) {   // resets esc as iwdg times out

					}
				}
				dshotcommand = 0;
			}

			if (bi_direction == 1 && (proshot == 0 && dshot == 0)) {
				if (newinput > 1100) {
					if (forward == dir_reversed) {
						adjusted_input = 0;
						prop_brake_active = 1;
						brushed_direction_set = 0;
						forward = 1 - dir_reversed;
					}

					if (prop_brake_active == 0) {

						adjusted_input = (newinput - 1099) * 3;
					}
				}
				if (newinput < 760) {
					if (forward == (1 - dir_reversed)) {
						prop_brake_active = 1;
						adjusted_input = 0;
						forward = dir_reversed;
						brushed_direction_set = 0;

					}
					if (prop_brake_active == 0) {
						adjusted_input = ((760 - newinput) * 3);

					}
				}
				if (vehicle_mode != 3) { // car mode requires throttle return to center before direction change
					prop_brake_active = 0;
				}

				if (newinput >= 760 && newinput < 1100) {
					adjusted_input = 0;
					prop_brake_active = 0;
				}

			} else if ((proshot || dshot) && bi_direction) {
				if (newinput > 1097) {

					if (forward == dir_reversed) {
						forward = 1 - dir_reversed;
						bemf_counts = 0;
						brushed_direction_set = 0;
					}
					adjusted_input = (newinput - 1100) * 2 + 100;
				}
				if (newinput <= 1047 && newinput > 0) {
					if (forward == (1 - dir_reversed)) {
						bemf_counts = 0;
						forward = dir_reversed;
						brushed_direction_set = 0;

					}
					adjusted_input = (newinput - 90) * 2;
				}
				if ((newinput > 1047 && newinput < 1098) || newinput <= 120) {
					adjusted_input = 0;
				}

			} else {
				adjusted_input = newinput;
			}

			if (adjusted_input > 2000) {
				adjusted_input = 2000;
			}
			if (input_override > 1) {
				input = input_override;
			} else {
				input = adjusted_input;
			}
		}

		if (BRUSHED_MODE) {
			dither_count++;
			if (dither_count > 2) {
				dither_count = 0;
			}
			if (input > 1990) { // keep slightly below 100 percent duty cycle for some drivers
				input = 1990;
			}
			bemf_counts = 200;

			if (!brushed_direction_set && !prop_brake_active) {

				if (!bi_polar) {
					if (forward) {
						comStep(6);
					} else {
						comStep(3);
					}
					brushed_direction_set = 1;
				} else {      // bipolar pwm  caution!!
					phaseAPWM();
					phaseCPWM();
					brushed_direction_set = 1;
				}
			}
		}

		advance_multiplier = map((commutation_interval), 150, 3000,
				max_advance_multiplier, min_advance_multiplier);
		if (inputSet == 0) {
			HAL_Delay(10);
			detectInput();

		}
		if (!armed) {
			if ((inputSet == 1) && (input == 0)) {
				armedcount++;
				HAL_Delay(1);
				if (armedcount > 2000) {
					armed = 1;
					playInputTune();
				}
			}
			if (input > 0) {
				armedcount = 0;
			}
		}

		if ((input >= (100 - (52-sine_mode_range)-(10*running))) && (armed == 1)) {
			if (sine_mode == 1) {
				sine_mode = 0;
				TIM1->ARR = timer_one_period;
				TIM1->CNT = 0;
				//	    allOff();
				running = 1;
				commutate();
			}
			prop_brake_active = 0;
			started = 1;
			start_power = map((input), 47, 1998, 150, 600);

			duty_cycle = (input - 20);

			if (bemf_counts < 20) {
				if (duty_cycle > 500) {
					duty_cycle = 500;
				}
			}

			if (bemf_counts < 5) {

				duty_cycle = start_power;

			}

			if (running) {
				if (duty_cycle > 1998) {                            // safety!!!
					duty_cycle = 1998;
				}
				if (duty_cycle < 60) {
					duty_cycle = 60;
				}

				if (STALL_PROTECTION && vehicle_mode == 2) {
					if (commutation_interval > 10000) {

						boost_level = map(commutation_interval, 10000, 20000,
								10, 80);
						duty_cycle = duty_cycle + boost_level;
					}
				}

				if (bi_polar) {
					if (dither_count == 0) {
						if (forward) {
							TIM1->CCR2 = (TIM1->ARR / 2) + (input / 2);
							TIM1->CCR3 = (TIM1->ARR / 2) - (input / 2);
						} else {
							TIM1->CCR2 = (TIM1->ARR / 2) - (input / 2);
							TIM1->CCR3 = (TIM1->ARR / 2) + (input / 2);
						}
					}
					if (dither_count == 1) {
						if (forward) {
							TIM1->CCR2 = (TIM1->ARR / 2) + (input / 2)
									+ dither_amount;
							TIM1->CCR3 = (TIM1->ARR / 2) - (input / 2)
									- dither_amount;
						} else {
							TIM1->CCR2 = (TIM1->ARR / 2) - (input / 2)
									- dither_amount;
							TIM1->CCR3 = (TIM1->ARR / 2) + (input / 2)
									+ dither_amount;
						}
					}
					if (dither_count == 2) {
						if (forward) {
							TIM1->CCR2 = (TIM1->ARR / 2) + (input / 2)
									- dither_amount;
							TIM1->CCR3 = (TIM1->ARR / 2) - (input / 2)
									+ dither_amount;
						} else {
							TIM1->CCR2 = (TIM1->ARR / 2) - (input / 2)
									+ dither_amount;
							TIM1->CCR3 = (TIM1->ARR / 2) + (input / 2)
									- dither_amount;
						}
					}
				} else {

					tim1_arr = map(commutation_interval, 84, 168, 1000, 2000);
					adjusted_duty_cycle = (duty_cycle * tim1_arr) / 2000 - 2;
					TIM1->ARR = tim1_arr;

					TIM1->CCR1 = adjusted_duty_cycle;
					TIM1->CCR2 = adjusted_duty_cycle;
					TIM1->CCR3 = adjusted_duty_cycle;

				}
			}

		}

		signaltimeout++;
		if (signaltimeout > signal_timeout_threshold) {
			input = 0;
			armed = 0;
			dshot_telemetry = 0;
			armedcount = 0;
			error = 1;
			inputSet = 0;
			TIM15->PSC = 0;
			TIM16->PSC = 0;
			dshot = 0;
			proshot = 0;
			servoPwm = 0;
			HAL_TIM_Base_Start_IT(&htim16);
			IC_buffer_size = 64;
			for (int i = 0; i < 64; i++) {
				dma_buffer[i] = 0;
			}
			HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, dma_buffer, 64);
		}

		if (input <= 47) {

			sine_mode = 0;
			if ((BRUSHED_MODE) && (brushed_direction_set)) {

				brushed_direction_set = 0;
			}
			if (brake == 1) {
				EXTI->IMR &= ~(1 << 21);
			}
			forcedcomcount = 0;
			started = 0;
			if (!brake && !prop_brake_active) {
				allOff();
			}
			duty_cycle = 0;
			if ((brake || tempbrake) && (!bi_polar)) {
				fullBrake();
				duty_cycle = 0;
				bemf_counts = 0;
			}

			if (prop_brake && prop_brake_active) {
				duty_cycle = prop_brake_strength;
				proBrake();
			}

			if (bi_polar) {
				TIM1->CCR2 = (TIM1->ARR) / 2;
				TIM1->CCR3 = (TIM1->ARR) / 2;
			} else {
				TIM1->CCR1 = duty_cycle;// set duty cycle to 50 out of 768 to start.
				TIM1->CCR2 = duty_cycle;
				TIM1->CCR3 = duty_cycle;
			}
		}

		if (vehicle_mode == 1) {
			if (bemf_counts < 40 || commutation_interval > 1000
					|| duty_cycle < 200) {
				filter_delay = 15;
				filter_level = 10;
			} else {
				filter_level = 5;

				filter_delay = 0;
			}
			if (duty_cycle > 600 && bemf_counts > 75) {
				filter_level = 2;
				//	filter_delay = 0;
			}

			if (commutation_interval < 100 && bemf_counts > 100) {
				filter_level = 2;
			}

		}

		if (vehicle_mode == 2 || vehicle_mode == 3) { // crawler much fewer poles, much more filtering time needed
			if (bemf_counts < 25 || commutation_interval > 4000
					|| duty_cycle < 200) {
				filter_delay = 15;
				filter_level = 15;
			} else {
				filter_level = 8;

				filter_delay = 0;
			}
		}
		if (vehicle_mode == 5) {

			if (bemf_counts < 15 || commutation_interval > 12000) {
				filter_level = 10;
			} else {
				filter_level = 2;
				wait_before = 700;
			}
			if (commutation_interval < 100) {
				filter_level = 1;
				wait_before = 500;
			}
		}

		if (started == 1) {
			if (running == 0) {
				if (BRUSHED_MODE) {
					running = 1;
				} else {

					zctimeout = 0;
					startMotor(); // safety on for input testing   ************************************************
				}
			}
		}

		if (polling_mode && running == 1) {
			checkForZeroCross();
		}

		if (!BRUSHED_MODE || !sine_mode) {
			zctimeout++;
			if (zctimeout > zc_timeout_threshold) {
				bemf_counts = 0;
				bad_commutation = 0;
				sensorless = 0;
				EXTI->IMR &= (0 << 21);
				running = 0;
			}

		}
		if ((input > 47
				&& input < (100 - (52 - sine_mode_range) - (10 * running)))
				&& (armed)) {
			if (running) {
				EXTI->IMR &= ~(1 << 21);
				running = 0;
				started = 0;
			}
			if (sine_mode == 0) {

				proBrake();
				sine_mode = 1;
			}

			step_delay = map(input, 48, 80, 2000, 500);
			TIM14->ARR = step_delay;
			if (TIM14->CNT >= TIM14->ARR) {
				TIM14->CNT = TIM14->ARR - 2;
			}
		}
		/* USER CODE END WHILE */
	}
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
