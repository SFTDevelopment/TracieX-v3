//Stepper Motor Driver TMC2209/08
#define TMC1_DIAG       42
#define TMC1_INDEX      41
#define TMC1_STEP       14
#define TMC1_DIR        20
#define TMC1_EN_N       16
#define TMC1_UART_ADDR  0

#define TMC2_DIAG       24
#define TMC2_INDEX      18
#define TMC2_STEP       17
#define TMC2_DIR        22
#define TMC2_EN_N       23
#define TMC2_UART_ADDR  1

#define TMC3_DIAG       25
#define TMC3_INDEX      19
#define TMC3_STEP       6
#define TMC3_DIR        5
#define TMC3_EN_N       3
#define TMC3_UART_ADDR  2

//TMC UART Connected to RX2/TX2, which is Serial1
// TMC VREF pins connectered to PWM pin OC4
#define TMCX_VREF       9   // PWM Duty =  100 -> 0.5V;  850 -> 2.5V

//RGB LED Strip
#define LEDPWR_EN       33
#define LEDPWR_OVRCUR_N 32  
#define LED_DATA        11  //MOSI

// SPT_PWR - Spectrometer Power
#define SPTPWR_EN       34
#define SPTPWR_OVRCUR_N 10  

// InterLock Switch
#define ITL1        2   //INT1
#define ITL2        7   //INT2

// PROXIMITY SENSOR 1 & 2
// Low when triggered
#define PROX1           A1  //15
#define PROX2           A7  //21
