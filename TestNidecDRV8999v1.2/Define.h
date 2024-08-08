
//Analog Adjustment VM
#define ADC_VM A0

//Input Internal
#define ExtX0 A1
#define ExtX1 A2
#define ExtX2 A3
#define ExtX3 A4
#define ExtX4 A5
#define ExtX5 A6
#define ExtX6 A7
#define ExtX7 A8
//Input External
#define ExtX8 A9
#define ExtX9 A10
#define ExtX10 A11
#define ExtX11 A12
#define ExtX12 A13
#define ExtX13 A14
#define ExtX14 A15
#define ExtX15 14
#define ExtX16 15
#define ExtX17 16
#define ExtX18 17
#define ExtX19 18
#define ExtX20 19
#define ExtX21 22
#define ExtX22 23
#define ExtX23 24
#define ExtX24 25
#define ExtX25 26
#define ExtX26 27
#define ExtX27 28

//Output Internal
#define ExtY7 29
#define ExtY6 30
#define ExtY5 31
#define ExtY4 32


//OutPut Driver
#define VREF 46
#define STEP 44
#define MOTOR_ON 42
#define DIR 43


//Sleep
#define SLEEP09 36 //DRIVER 10
#define SLEEP10 41 //DRIVER 10
//OutPut Sleep (8 Pin)
int latchPin = 33;
int clockPin = 35;
int dataPin = 34;

//Fault
#define Fault01 13
#define Fault02 11
#define Fault03 9
#define Fault04 7
#define Fault05 5
#define Fault06 3
#define Fault07 38
#define Fault08 40
#define Fault09 47
#define Fault10 49

//SPI Driver
#define SCS01 12
#define SCS02 10
#define SCS03 8
#define SCS04 6
#define SCS05 4
#define SCS06 2
#define SCS07 39
#define SCS08 45
#define SCS09 48
#define SCS10 53

#define SDO_PIN  50
#define SDI_PIN  51
#define SCK_PIN  52

const int ValVREF =65;

#define SPI_STATUS_CTRL1            (uint8_t)(0x00)  
#define SPI_REG_CTRL3                (uint8_t)(0x05)         /* CTRL3 register - Step, dir, microstep  */
#define SPI_REG_CTRL4                (uint8_t)(0x06)  

#define Tblank_CTRL1               (uint8_t)(0x03)
#define tblank_860           (uint8_t)(0x03)
#define decay_CTRL2              (uint8_t)(0x04)
#define smart_ripple           (uint8_t)(0x07)
//




#define MODE_FULL_STEP_100           (uint8_t)(0x00)
#define MODE_FULL_STEP_71          (uint8_t)(0x01)
#define MODE_HALF_STEP_1I2_NON         (uint8_t)(0x02)
#define MODE_HALF_STEP_1I2           (uint8_t)(0x03)
#define MODE_MICRO_STEP_1I4          (uint8_t)(0x04)
#define MODE_MICRO_STEP_1I8          (uint8_t)(0x05)
#define MODE_MICRO_STEP_1I16           (uint8_t)(0x06)
#define MODE_MICRO_STEP_1I32           (uint8_t)(0x07)
#define MODE_MICRO_STEP_1I64           (uint8_t)(0x08)
#define MODE_MICRO_STEP_1I128           (uint8_t)(0x09)
#define MODE_MICRO_STEP_1I256           (uint8_t)(0x0A)
#define n_fault_setting           (uint8_t)(0x38)
#define n_fault_clear           (uint8_t)(0xB8)

/////////////////////////////////
#define SPI_BUSY_FLAG      0x01         //<User define flag to 
#define SPI_ADDRESS_MASK   0x3F00       /**< Mask for SPI register address bit */
#define SPI_ADDRESS_POS    9           /**< Position for SPI register address bit */
#define SPI_DATA_MASK      0x00FF       /**< Mask for SPI register data bit */
#define SPI_DATA_POS       0x0          /**< Position for SPI register data bit */
#define SPI_RW_BIT_MASK    0x4000       /**< Mask for SPI register read write indication bit */
//**************************************************************//
char model,prog;
boolean setting,debug;
boolean lock, lock_para,lock_up,lock_down,lock_set;
long pul_set, pul_acc_set,change;
int freq;
char PTO,parametter;
volatile  long count_pulse;
unsigned long time_delay_bt,time_print,time_pause,time_blink,time_on_relay,time_off_relay,time_pulse,time_check,time_delay;
long frq_start,frq_max,pul_max,pul_acc,pul_stop;
double time_display;
int f,para,limit,delay_bt,new_freq,old_freq,cycle;
long pre_count_low,pre_count_high,pre_count_diffence,pre_count;
boolean pulse_enable = false;
boolean lock_end;
boolean setting_enable;

boolean led_blink, run_on, NG;
uint16_t error_code;
unsigned int        SPI_STEP_DIR_CMD;               // variable for STEP/DIR control over SPI
unsigned int        MICROSTEP_MODE_BITS;            // Microstep mode bits to store microstepping setting

typedef struct
{
  //bool MOTOR_CMD_ACTIVE1;     // Motor in Motion (Auto)   0 - No Motion   1 - In Motion
  //bool MOTOR_CMD_ACTIVE2;     // Motor in Motion (Man)    0 - No Motion   1 - In Motion
  //bool MOTOR_CMD_ACTIVE3;
  //bool RUN_STEPPER;           // Auto run enabled
  //bool MANUAL_ADVANCE;        // Manually advance motor
  //bool RECIPROCATION;
  //bool REVERSE_ON_STALL;      // Variable for "Reverse on Stall Fault" button feature
  bool SPI_STEP_DIR_EN;       // Variable for STEP/DIR control over SPI rather than GPIO

  volatile uint8_t gDIR;               // Direction                0 - Forward     1 - Reverse

  //uint16_t  STARTING_SPEED;    // initial speed in pulses per second when starting motor
  //uint16_t  TARGET_SPEED;      // speed of motor once running
  //uint16_t  ACCEL_RATE;        // acceleration rate from starting speed to target speed
  //uint16_t  STOPPING_SPEED;    // speed to stop motor once deceleration has reached this point
  //uint16_t  STEPS_2_STOP;      // number of steps used to stop the motor when manually stepping
  //uint16_t  NUM_STEPS;         // number of steps to move when manually stepping
  //uint16_t  AccelerateState;   // Accelerate state for state machine
  //uint16_t  StallState;        // Stall detection setup state

  //float  IFS;                  // Full scale current

} Stepper_Driving_Obj_t;

Stepper_Driving_Obj_t gStepper_Obj;
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display



#define RL_VM  37


byte dataRED;

struct error
{
  bool openload;
  bool overheat;
  bool overcurrent;
  bool undervol;
  bool spi_err;
  bool fault_bit;
  
}DRV8889;
const int chipSelect = 53;
int Steps=1354;
int _delay=250;
bool IsRun=true;
int num=0;
byte  dataOutPut [] {
  0x10000000b,
  0x01000000b,
  0x00100000b,
  0x00010000b,
  0x00001000b,
  0x00000100b,
  0x00000010b,
  0x00000001b,
  0x00000000b,
};
