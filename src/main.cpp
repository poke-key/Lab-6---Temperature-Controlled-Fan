#include <stdio.h>

#include "timerISR.h"
#include "helper.h"
#include "periph.h"
#include "LCD.h"

/* You have 5 tasks to implement for this lab */
#define NUM_TASKS 6
#define BEEPCOUNTER 100
#define START -1
#define TASK_LCD_PERIOD 500
#define TASK_JOYSTICK_PERIOD 1
#define TASK_BEEPER_PERIOD 1
#define TASK_MOTOR_PERIOD 1
#define TASK_THERMISTOR_PERIOD 1
#define TASK_REFRESH_PERIOD 1000
#define MOTOR_PWM 10

int motor_duty = 0;
int motor_duty_auto = 0;
int motor_duty_on = ( MOTOR_PWM * motor_duty ) / 100;
int motor_duty_off = MOTOR_PWM - motor_duty_on;

double tempF = 0;
double oldTempF = 0;
double tempTh = 75;

bool bDirty = true; //flag for checking if screen is showing gibberish

/*turn motor on and off*/
void MotorOn() {
  PORTB = SetBit(PORTB, 2, 1);
}
void MotorOff() {
  PORTB = SetBit(PORTB, 2, 0);
}

//Task struct for concurrent synchSMs implmentations
typedef struct _task{
	signed 	 char state; 		//Task's current state
	unsigned long period; 		//Task period
	unsigned long elapsedTime; 	//Time elapsed since last task tick
	int (*TickFct)(int); 		//Task tick function
} task;


//TODO: Define Periods for each task
// e.g. const unsined long TASK1_PERIOD = <PERIOD>
const unsigned long GCD_PERIOD = /* TODO: Calulate GCD of tasks */ 1;

task tasks[NUM_TASKS]; // declared task array with 5 tasks

int beep_counter =  0; //ms 0 means off.
void BeepOn()
{
    PORTC = SetBit(PORTC, 3, 1);
}
void BeepOff()
{
    PORTC = SetBit(PORTC, 3, 0);
}

//TODO: Define, for each task:
// (1) enums and
// (2) tick functions

void TimerISR() {
	for ( unsigned int i = 0; i < NUM_TASKS; i++ ) {                   // Iterate through each task in the task array
		if ( tasks[i].elapsedTime == tasks[i].period ) {           // Check if the task is ready to tick
			tasks[i].state = tasks[i].TickFct(tasks[i].state); // Tick and set the next state for this task
			tasks[i].elapsedTime = 0;                          // Reset the elapsed time for the next tick
		}
		tasks[i].elapsedTime += GCD_PERIOD;                        // Increment the elapsed time by GCD_PERIOD
	}
}

// /*LCD Screen*/
enum lcd_States{INIT, MANUAL, AUTO, REFRESH} lcd_state, n_lcd_state;
int Tick_LCD(int state) {
    //char str[100];
    /*state transitions*/
    switch(state) {
        case INIT:
            if(lcd_state != n_lcd_state ) {
                if (lcd_state == MANUAL)
                    state = AUTO ;
                else    
                    state = MANUAL ;
            } 
            break;
        case MANUAL:
        case AUTO:
        default:
            break;
    }
    /*state actions*/
    switch(state) {
      case MANUAL:
          lcd_state = MANUAL;
          n_lcd_state = MANUAL ;
          state = INIT ;
          break;
      case AUTO:
          lcd_state = AUTO;
          n_lcd_state = AUTO ;
          state = INIT ;
          break;
      default: 
        break;
    }
    return state;
}

enum joystick_states{INIT_1, IDLE, UP, DOWN, REMAIN_UP, REMAIN_DOWN, BUTTON_PRESS, BUTTON_RELEASE} joystick_state;
int Tick_JoyStick(int state) {
  /*state transitions*/
  switch(state)
  {
    case INIT_1:
      state = IDLE;
      break;
    case IDLE:
    // A5 A4 A3 A2 A1 A0
      if((PINC >> 1 & 0x01) == 0) state = BUTTON_PRESS;
      else if(ADC_read(0) > 950) state = UP;
      else if(ADC_read(0) < 100) state = DOWN;
      break;
    case UP:
      if(ADC_read(0) <= 950) 
        state = IDLE; 
      else 
        state = REMAIN_UP;
      break;
    case DOWN:
      if (ADC_read(0) >= 100) 
        state = IDLE;
      else 
        state = REMAIN_DOWN; 
      break;
    case REMAIN_UP:
      if(ADC_read(0) <= 950) 
        state = IDLE;
      break;
    case REMAIN_DOWN:
      if(ADC_read(0) >= 100) 
        state = IDLE;
      break;
    case BUTTON_PRESS:
        if((PINC >> 1 & 0x01) == 1) 
            state = BUTTON_RELEASE; 
        break ;
    case BUTTON_RELEASE:
      state = IDLE; 
      break;
    default: break;    
  }
  /*state actions*/
  switch(state) {
    case INIT_1:
    case IDLE:
    case REMAIN_UP:
    case REMAIN_DOWN:
    case BUTTON_PRESS:
      break ;
    case UP:
      bDirty = true;
      beep_counter = BEEPCOUNTER;
      if(lcd_state == MANUAL) {
        if(motor_duty != 100) 
          motor_duty += 10;
      }
      if(lcd_state == AUTO) 
        tempTh++;
      break ;
    case DOWN:
      bDirty = true;
      beep_counter = BEEPCOUNTER;
      if(lcd_state == MANUAL) {
        if(motor_duty != 0) 
          motor_duty -= 10;
      }
      if(lcd_state == AUTO) 
        tempTh--;
      break;
    case BUTTON_RELEASE:
        bDirty = true;
        beep_counter = BEEPCOUNTER;
        lcd_clear() ;
        if(n_lcd_state == MANUAL) 
            n_lcd_state = AUTO;
        else 
            n_lcd_state = MANUAL;
        break;  
    default: break;    
  }
  return state;
}

enum buzz_states {BEEP_IDLE, OFF, ON} buzz_state;
int Tick_Beeper(int state) {

  if (beep_counter) 
    beep_counter--;

  /*state transitions*/
  switch (state)
  {
    case BEEP_IDLE:
      state = OFF;
      break;
    case OFF:
      if(beep_counter > 0) state = ON;
      break;
    case ON:
      if(beep_counter == 0) state = OFF;
      break;
    default:
      break;
  }

  /*state actions*/
  switch(state) {
  case BEEP_IDLE:
    break;
  case OFF:
    BeepOff();
    break;
  case ON:
    BeepOn();
    break;
  default:
    break;
  }
  return state;
}

int motor_edge = 0 ;
/*motor control*/
enum motor_states{MOTOR_OFF, MOTOR_ON} motor_state;
int Tick_Motor(int state) {
  switch(state)
  {
    case MOTOR_OFF: 
      motor_duty_off-- ;
      if ( motor_duty_off  < 0 ) {
        state = MOTOR_ON ; // switch to ON state
        // recalculate counters
        if ( lcd_state == MANUAL )
          motor_duty_on = ( MOTOR_PWM * motor_duty ) / 100;
        else
          motor_duty_on = ( MOTOR_PWM * motor_duty_auto ) / 100;
        motor_duty_off = MOTOR_PWM - motor_duty_on;
        motor_edge = 1 ;
      }
      break ;
    case MOTOR_ON:
      motor_duty_on-- ;
      if ( motor_duty_on < 0 ) {
        state = MOTOR_OFF ; // switch to OFF state
        // recalculate counters
        if ( lcd_state == MANUAL )
          motor_duty_on = ( MOTOR_PWM * motor_duty ) / 100;
        else
          motor_duty_on = ( MOTOR_PWM * motor_duty_auto ) / 100;
        motor_duty_off = MOTOR_PWM - motor_duty_on;
        motor_edge = 1 ;
      }
      break;
    default:
      break ;
  }

  /*state actions*/
  switch (state)
  {
    case MOTOR_OFF:
      if ( motor_duty_off)
        MotorOff() ;
      motor_edge = 0 ;
      break;
    case MOTOR_ON:
      if ( motor_duty_on )
        MotorOn() ;
      motor_edge = 0 ;
      break;
    default:
      break;
  }
  return state;
}

/*auto mode, read temperature*/
enum temp_states {TEMP_INIT } temp_state;
int Tick_Thermistor(int state) {
  
  int tempReading = ADC_read(2);/* your code to fetch analog value of thermistor with ADC_read() */
  double tempK = log(10000.0 * ((1024.0 / tempReading - 1)));
  tempK = 1/ (0.001129148 + (0.000234125 + (0.0000000876741 * tempK * tempK)) * tempK);
  tempF = ((9.0 / 5) * (tempK - 273.15)) + 32;/* your code convert Kelvin into Fahrenheit */
  
  /*check if AUTO, then calculate duty cycle of motor*/
  if(lcd_state == AUTO) {
    if(tempF > tempTh)
      motor_duty_auto = (tempF - tempTh) * 10;
    else 
      motor_duty_auto = 0;
    if((int)oldTempF != (int)tempF) {
      bDirty = true;
      oldTempF = tempF;
    }
  }

  return state;
}

enum refresh_states {DIRTY, CLEAN} refresh_state;
int Tick_Refresh(int state ) {
  char str[100];

  if ( !bDirty)
    return state ;

  bDirty = false;
  //lcd_clear() ;
  lcd_goto_xy(0,0);
  if ( lcd_state == MANUAL) {
      sprintf(str, "Mode: MANUAL") ;
      lcd_write_str(str);
      sprintf(str, "FAN: %3d%%", motor_duty);
  }
  if ( lcd_state == AUTO) {
    lcd_write_str((char*)"Mode: AUTO");
    sprintf(str, "Curr:%dF Th:%dF ", (int)tempF, (int)tempTh);    
  }
  lcd_goto_xy(1,0);
  lcd_write_str(str);
  return state;
}


int main(void) 
{
    DDRC = 0xF8 ; PORTC = 0x07;
    DDRB = 0x06 ; PORTB = 0xF9;
    DDRD = 0xFF ; PORTD = 0x00 ;

    ADC_init();   // initializes ADC
    lcd_init() ;
    lcd_clear() ;

    lcd_state = MANUAL;
    n_lcd_state = MANUAL ;
    buzz_state = BEEP_IDLE;
    motor_state = MOTOR_OFF ;

    tasks[0].period = TASK_LCD_PERIOD ;
    tasks[0].state = INIT ; //initial state
    tasks[0].elapsedTime = 0;
    tasks[0].TickFct = &Tick_LCD;

    tasks[1].period = TASK_JOYSTICK_PERIOD ;
    tasks[1].state = INIT_1 ; //initial state
    tasks[1].elapsedTime = 0;
    tasks[1].TickFct = &Tick_JoyStick;

    tasks[2].period = TASK_BEEPER_PERIOD ;
    tasks[2].state = BEEP_IDLE ; //initial state
    tasks[2].elapsedTime = 0;
    tasks[2].TickFct = &Tick_Beeper;

    tasks[3].period = TASK_MOTOR_PERIOD ;
    tasks[3].state = MOTOR_OFF ; //initial state
    tasks[3].elapsedTime = 0;
    tasks[3].TickFct = &Tick_Motor;

    tasks[4].period = TASK_THERMISTOR_PERIOD ;
    tasks[4].state = TEMP_INIT ; //initial state
    tasks[4].elapsedTime = 0;
    tasks[4].TickFct = &Tick_Thermistor;

    tasks[5].period = TASK_REFRESH_PERIOD ;
    tasks[5].state =  DIRTY; //initial state
    tasks[5].elapsedTime = 0;
    tasks[5].TickFct = &Tick_Refresh;

    Tick_Refresh(DIRTY) ;
    TimerSet(GCD_PERIOD);
    TimerOn();

    while (1) {}

    return 0;
}