#include "input/input_dshot.h"

int dshot_full_number;
int dpulse[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };


void computeDshotDMA() {
	if (dma_buffer[0] > 10) {
		for (int i = 1; i < 32; i += 2) {
			dpulse[(i - 1) >> 1] = dma_buffer[i];
		}
	}
	if (dma_buffer[1] > 10) {
		for (int i = 2; i <= 32; i += 2) {
			dpulse[(i - 1) >> 1] = dma_buffer[i];
		}
	}

	calcCRC = ((dpulse[0] ^ dpulse[4] ^ dpulse[8]) << 3
			| (dpulse[1] ^ dpulse[5] ^ dpulse[9]) << 2
			| (dpulse[2] ^ dpulse[6] ^ dpulse[10]) << 1
			| (dpulse[3] ^ dpulse[7] ^ dpulse[11]));
	checkCRC =
			(dpulse[12] << 3 | dpulse[13] << 2 | dpulse[14] << 1 | dpulse[15]);
	if (!armed) {
		if (dshot_telemetry == 0) {
			if (calcCRC == ~checkCRC + 16) {
				dshot_telemetry = 1;
				is_output = 1; // so that setupinput() is called on next timer16 timeout
			}
		}
	}
	if (dshot_telemetry) {
		checkCRC = ~checkCRC + 16;
	}
	if (calcCRC == checkCRC) {
		tocheck = (dpulse[0] << 10 | dpulse[1] << 9 | dpulse[2] << 8
				| dpulse[3] << 7 | dpulse[4] << 6 | dpulse[5] << 5
				| dpulse[6] << 4 | dpulse[7] << 3 | dpulse[8] << 2
				| dpulse[9] << 1 | dpulse[10]);
		//				success++;
	} else {
		error++;
	}

	if (tocheck > 47 && tocheck < 2048) {
		newinput = tocheck;
		commandcount = 0;
	} else if (tocheck > 1 && tocheck < 48 && input == 0) {

		dshotcommand = tocheck;

	} else {
		commandcount++;
		if (commandcount > 1) {
			newinput = tocheck;
			commandcount = 0;
		}
	}

}

void make_dshot_package() {
	e_com_time = commutation_interval * 6 / 2;
	if (!running) {
		e_com_time = 65535;
	}
//	calculate shift amount for data in format eee mmm mmm mmm, first 1 found in first seven bits of data determines shift amount
// this allows for a range of up to 65408 microseconds which would be shifted 0b111 (eee) or 7 times.
	for (int i = 15; i >= 9; i--) {
		if (e_com_time >> i == 1) {
			shift_amount = i + 1 - 9;
			break;
		} else {
			shift_amount = 0;
		}
	}
// shift the commutation time to allow for expanded range and put shift amount in first three bits
	dshot_full_number = ((shift_amount << 9) | (e_com_time >> shift_amount));
//calculate checksum
	uint16_t csum = 0;
	uint16_t csum_data = dshot_full_number;
	for (int i = 0; i < 3; i++) {
		csum ^= csum_data;   // xor data by nibbles
		csum_data >>= 4;
	}
	csum = ~csum;       // invert it
	csum &= 0xf;

	dshot_full_number = (dshot_full_number << 4) | csum; // put checksum at the end of 12 bit dshot number

// GCR RLL encode 16 to 20 bit

	gcrnumber = gcr_encode_table[(dshot_full_number >> 12)] << 15 // first set of four digits
	| gcr_encode_table[(((1 << 4) - 1) & (dshot_full_number >> 8))] << 10 // 2nd set of 4 digits
	| gcr_encode_table[(((1 << 4) - 1) & (dshot_full_number >> 4))] << 5 //3rd set of four digits
	| gcr_encode_table[(((1 << 4) - 1) & (dshot_full_number >> 0))]; //last four digits
//GCR RLL encode 20 to 21bit output

	gcr[1] = 64;
	for (int i = 19; i >= 0; i--) {              // each digit in gcrnumber
		gcr[20 - i + 1] = ((((gcrnumber & 1 << i)) >> i) ^ (gcr[20 - i] >> 6))
				<< 6; // exclusive ored with number before it multiplied by 64 to match output timer.
	}
	gcr[0] = 0;

}

void senddshotburst() {
	if (HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_3, gcr, 23) != HAL_OK) {
		/* Starting Error */
		Error_Handler();
	}
	DMA1_Channel1->CCR = 0xa93;         // turn off half transfer interrupt.
	//GPIOA->BSRR = GPIO_PIN_15;
	count++;
}
