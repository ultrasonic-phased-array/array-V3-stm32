#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
#define TEMP_PULLUP 10000.0f
#define ADC_VREF 3.3f /* Vref+ input voltage in mV */
#define NTC_R(a) (-(a * TEMP_PULLUP) / (a - ADC_VREF))
#define VOUT_DIV 15.24f
#define ARES    4096.0f
#define IOUT_DIV   1.5f
#define DC_RES     2.7f

#define BUCK_PWM_PERIOD ((uint16_t)4096) // 512kHz
#define DT_RISING       ((uint16_t)40)
#define DT_FALLING      ((uint16_t)40)

#define VTARGET 24.0

#define NO 0
#define YES 1
#define ABS(a) (((a) < 0.0) ? -(a) : (a))
#define LIMIT(x, lowhigh) (((x) > (lowhigh)) ? (lowhigh) : (((x) < (-lowhigh)) ? (-lowhigh) : (x)))
#define SAT(x, lowhigh) (((x) > (lowhigh)) ? (1.0) : (((x) < (-lowhigh)) ? (-1.0) : (0.0)))
#define SAT2(x, low, high) (((x) > (high)) ? (1.0) : (((x) < (low)) ? (-1.0) : (0.0)))
#define STEP(from, to, step) (((from) < (to)) ? (MIN((from) + (step), (to))) : (MAX((from) - (step), (to))))
#define DEG(a) ((a)*M_PI / 180.0)
#define RAD(a) ((a)*180.0 / M_PI)
#define SIGN(a) (((a) < 0.0) ? (-1.0) : (((a) > 0.0) ? (1.0) : (0.0)))
#define CLAMP(x, low, high) (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))
#define SCALE(value, high, max) MIN(MAX(((max) - (value)) / ((max) - (high)), 0.0), 1.0)
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN3(a, b, c) MIN(a, MIN(b, c))
#define MAX3(a, b, c) MAX(a, MAX(b, c))
