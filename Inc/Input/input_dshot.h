#include <stdint.h>
#include "main.h"

extern char dshot_telemetry;
extern int dshotcommand;
extern char is_output;
extern int shift_amount;

void computeDshotDMA();
void make_dshot_package();
void senddshotburst();
