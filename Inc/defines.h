#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
#define TEMP_PULLUP 10000.0f
#define ADC_VREF 3.3f /* Vref+ input voltage in mV */
#define NTC_R(a) (-(a * TEMP_PULLUP) / (a - ADC_VREF))
#define VOUT_DIV ((float)15.24f)
#define ARES 4096.0f

#define BUCK_PWM_PERIOD ((uint16_t)8192) // 281.25kHz
#define DT_RISING       ((uint16_t)100)
#define DT_FALLING      ((uint16_t)100)

#define VTARGET 24.0
