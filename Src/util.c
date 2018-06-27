
#include "stm32f3xx_hal.h"
#include "defines.h"

float r2temp(float r){
  r = r / 1000;
  const int step = 10;
  const int start = -10;
  //-10..160
  const float temp[] = {42.45, 27.28, 17.96, 12.09, 8.31, 5.83, 4.16, 3.02, 2.23, 1.67, 1.27, 0.975, 0.76, 0.599, 0.5, 0.4, 0.3};

  for(int i = 1; i < ARRAY_SIZE(temp); i++){
    if(temp[i] < r){
      float a = temp[i - 1];
      float b = temp[i];
      return(-(r - b) / (a - b) * step + i * step + start);
    }
  }
  return(temp[ARRAY_SIZE(temp)] + step);
}
