#include "input/input_pwm.h"
#include "settings.h"

int servorawinput = 0;


void computeServoInput(){

	if ( dma_buffer[1] < 2000 && dma_buffer[1] > 1000){
		if(dma_buffer[2]< 1000 || dma_buffer[2] > 2500){

		servorawinput = map(dma_buffer[1], 1100,2000,0,2000);
		}
	}else if( dma_buffer[2] < 2000 && dma_buffer[2] > 1000) {
		if(dma_buffer[1]< 1000 || dma_buffer[1] > 2500){
		servorawinput = map(dma_buffer[2], 1100,2000,0,2000);
		}
	}


	if (servorawinput - newinput > MAX_SERVO_DEVIATION)
	{
		newinput += MAX_SERVO_DEVIATION;
	}
	else if(newinput - servorawinput > MAX_SERVO_DEVIATION)
	{
		newinput -= MAX_SERVO_DEVIATION;
	}
	else
	{
		newinput = servorawinput;
	}


}
