#include "input/input_oneshot.h"

void computeOS125Input(){

	int lastnumber = dma_buffer[0];
	for ( int j = 1 ; j < 2; j++){

		if(((dma_buffer[j] - lastnumber) < 12300) && ((dma_buffer[j] - lastnumber) > 0)){ // blank space

			newinput = map((dma_buffer[j] - lastnumber),6500,12000, 0, 2000);
			break;
		}
		lastnumber = dma_buffer[j];
	}
}

void computeOS42Input(){

	int lastnumber = dma_buffer[0];
	for ( int j = 1 ; j < 2; j++){

		if(((dma_buffer[j] - lastnumber) < 4500) && ((dma_buffer[j] - lastnumber) > 0)){ // blank space

			newinput = map((dma_buffer[j] - lastnumber),2020, 4032, 0, 2000);
			break;
		}
		lastnumber = dma_buffer[j];
	}
}
