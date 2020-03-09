#include "input/input_proshot.h"
#include "settings.h"


void computeProshotDMA() {
	int total;
	int propulse[4] = { 0, 0, 0, 0 };

	total = dma_buffer[1] + dma_buffer[2] + dma_buffer[3] + dma_buffer[4]
			+ dma_buffer[5] + dma_buffer[6] + dma_buffer[7];

	if ((total < 118 && total > 98) && (dma_buffer[0] > 100)) {
		for (int i = 1; i < 8; i += 2) {
			propulse[(i - 1) / 2] = (dma_buffer[i] - 7);
		}
	} else {

		return;
	}
	calcCRC = ((propulse[0] ^ propulse[1] ^ propulse[2]) << 3
			| (propulse[0] ^ propulse[1] ^ propulse[2]) << 2
			| (propulse[0] ^ propulse[1] ^ propulse[2]) << 1
			| (propulse[0] ^ propulse[1] ^ propulse[2]));

	checkCRC = (propulse[3] << 3 | propulse[3] << 2 | propulse[3] << 1
			| propulse[3]);
	if (checkCRC == calcCRC) {
		tocheck = ((propulse[0] << 7 | propulse[1] << 3 | propulse[2] >> 1));
	} else {
		//   	error++;
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
