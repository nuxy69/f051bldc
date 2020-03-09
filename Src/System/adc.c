#include "system/adc.h"

void getADCs() {
	uint32_t ADC1ConvertedValues[2] = { 0, 0 };

	int voltageraw = 0;
	int currentraw = 0;
	int tempraw = 0;
	//voltageraw = ADC1ConvertedValues[0];
	currentraw = ADC1ConvertedValues[0];
	tempraw = ADC1ConvertedValues[1];

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	getADCs();
}
