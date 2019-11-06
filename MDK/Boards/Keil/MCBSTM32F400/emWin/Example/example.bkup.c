/*----------------------------------------------------------------------------
 * Name:    Blinky.c
 * Purpose: LED Flasher
 *----------------------------------------------------------------------------
 * This file is part of the uVision/ARM development tools.
 * This software may only be used under the terms of a valid, current,
 * end user licence from KEIL for a compatible version of KEIL software
 * development tools. Nothing else gives you the right to use this software.
 *
 * This software is supplied "AS IS" without warranties of any kind.
 *
 * Copyright (c) 2018 Keil - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include <stdio.h>
//#include <stdbool.h>
#include <time.h>

#include "main.h"
#include "GUI.h"                         // Segger.MDK-Pro::Graphics:CORE
#include "Board_LED.h"                  /* ::Board Support:LED */
#include "Board_Buttons.h"              /* ::Board Support:Buttons */
#include "Board_ADC.h"                  /* ::Board Support:A/D Converter */

#include "RTE_Components.h"             /* Component selection */
#include "stm32f4xx_hal.h"              // Keil::Device:STM32Cube HAL:Common

#ifdef RTE_Compiler_EventRecorder
#include "EventRecorder.h"
#endif

// Main stack size must be multiple of 8 Bytes
#define APP_MAIN_STK_SZ (2048U)
uint64_t app_main_stk[APP_MAIN_STK_SZ / 8];
const osThreadAttr_t app_main_attr = {
  .stack_mem  = &app_main_stk[0],
  .stack_size = sizeof(app_main_stk)
};

static volatile uint32_t delay_val = 500U;

static osThreadId_t tid_thrADC;                /* Thread id of thread: ADC */
//static osThreadId_t tid_thrLED;                /* Thread id of thread: LED */
static osThreadId_t tid_thrBUZZ;                /* Thread id of thread: BUT */
static osThreadId_t tid_thrBUT_Left;                /* Thread id of thread: BUT */
static osThreadId_t tid_thrBUT_Middle;                /* Thread id of thread: BUT */
static osThreadId_t tid_thrBUT_Right;                /* Thread id of thread: BUT */

//static int BSR[10];
uint8_t BSR;
float temp = -200;
char* str = "C";


/*----------------------------------------------------------------------------
  Display: Set up display. Functions to show string/float.
 *----------------------------------------------------------------------------*/
int32_t xPos, yPos, xSize, ySize;

void setUpGUI(){
  GUI_Init();
	xSize = LCD_GetXSize();
  ySize = LCD_GetYSize();

  GUI_SetFont(&GUI_FontComic24B_1);
  GUI_SetColor(GUI_CYAN);
}

void showStringDisplay(char* str) {
  GUI_DispStringHCenterAt(str,   xSize / 2, (ySize / 4) * 3);
}

void showIntegerDisplay(float num) {    
  char result[10];
  sprintf(result, "%.2f", num);
  GUI_DispStringHCenterAt(result,   xSize / 2, ySize / 4);

}




//#define TEMP110_CAL_VALUE                                           ((uint16_t*)((uint32_t)0x1FFF7A2E))
//#define TEMP30_CAL_VALUE                                            ((uint16_t*)((uint32_t)0x1FFF7A2C))
#define TEMP_SENSOR_AVG_SLOPE_MV_PER_CELSIUS                        2.5f
#define TEMP_SENSOR_VOLTAGE_MV_AT_25                                760.0f
#define ADC_REFERENCE_VOLTAGE_MV                                    3300.0f
#define ADC_MAX_OUTPUT_VALUE                                        4095.0f
#define TEMP110_CAL_VALUE                                           ((uint16_t*)((uint32_t)0x1FFF7A2E))
#define TEMP30_CAL_VALUE                                            ((uint16_t*)((uint32_t)0x1FFF7A2C))
#define TEMP110                                                     110.0f
#define TEMP30                                                      30.0f


int32_t temperature;
float sensorValue;
float adcCalValue30;
float adcCalValue110;

// T3 : Convert Temp
int32_t convert_LM35(int32_t val){
	return (int32_t)((5 * val * 100)/ 1024);
}

// T3 : Convert Temp
int32_t convert_internal_temperatur(int32_t val){
	return (int32_t)((TEMP110 - TEMP30) / ((float)(*TEMP110_CAL_VALUE) - (float)(*TEMP30_CAL_VALUE)) * (val - (float)(*TEMP30_CAL_VALUE)) + TEMP30);
}


// T2: Read SDR
float get_ADC_Value() {
	ADC_StartConversion();                                    
	float val;
	
	// Loop until ADC is done processing signal. Wait 1ms before each check.
	while(1){
		if (ADC_ConversionDone() == 0) {
			val = ADC_GetValue();      			
			//val = (5*val*100)/1024;								/* Scale delay value */
			break;
		}		
	}    
	return val;
}

float Farenheit(float temp){
	return (float) (1.8 * temp + 32);
}

float Celcius(float temp){
	return (float) ((temp - 32)/ 1.8);
}

/*----------------------------------------------------------------------------
 * Bit Manipulation
 *---------------------------------------------------------------------------*/
int is_set(uint8_t a,int bit_no){
	int shift_count = 7 - bit_no;
	return  (a & (1 << shift_count)) ? 1 : 0;
}	

// Set nth bit
int set_bit(uint8_t num,int bit){
	int shift_count = 7 - bit;
	num |= 1UL << shift_count;
	return num;
}	
// Set nth bit
int clear_bit(uint8_t num,int bit){
	int shift_count = 7 - bit;
	num &= ~(1UL << shift_count);
	return num;
}	

// Toggle nth bit
int toggle_bit(uint8_t num,int bit){
	int shift_count = 7 - bit;
	num ^= 1UL << shift_count;
	return num;
}	

/* GPIO Pin identifier */
typedef struct PIN_GEN {
  GPIO_TypeDef *port;
  uint16_t      pin;
  uint16_t      reserved;
} PIN_GEN;


static const PIN_GEN GPIO_PINS[] = {
  { GPIOB, GPIO_PIN_0,  0U },
  { GPIOB, GPIO_PIN_1,  0U }
};


/*------------------------------------------------------------------------------
  Buzzer Setup: 
 *----------------------------------------------------------------------------*/
void buzzer_setup(){
  
  GPIO_InitTypeDef GPIO_InitStruct;
  
  // Initialize clock for GPIO B.
  __HAL_RCC_GPIOB_CLK_ENABLE();
  //RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  

  // COnfigure Pin 1 from PortC
  //GPIOC->MODER |= GPIOC_C
  
  /* ADC3 GPIO Configuration: PB1 -> ADC3_IN7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
}
/*------------------------------------------------------------------------------
  thrBUZZ: check buzzer state
 *----------------------------------------------------------------------------*/


__NO_RETURN static void thrBUZZ(void *argument) {
	
	HAL_GPIO_WritePin(GPIO_PINS[0].port, GPIO_PINS[0].pin, GPIO_PIN_SET);

	while(1){
		osDelay (500U);   
		if(is_set(BSR,4) != 0){
			// T5 : Turn on Buzzer
			HAL_GPIO_WritePin(GPIO_PINS[0].port, GPIO_PINS[0].pin, GPIO_PIN_RESET);
			LED_On(2);            
			
		}else{
			// T13 : Turn off Buzzer
			HAL_GPIO_WritePin(GPIO_PINS[0].port, GPIO_PINS[0].pin, GPIO_PIN_SET);
			LED_Off(2);
		}    
		osThreadFlagsWait (1U, osFlagsWaitAny, 0U);
		
	}
}

int set_bit(uint8_t num,int bit);
int is_set(uint8_t num,int bit);
int toggle_bit(uint8_t num,int bit);


void toggle_LED(){

	if(is_set(BSR, 7) == 1){			
		LED_On(7); 			
	}else{			
		LED_Off(7); 			
	}	

	if(is_set(BSR, 3) == 1){			
		LED_On(5); 			
	}else{			
		LED_Off(5); 			
	}	

	if(is_set(BSR, 0) == 1){			
		LED_On(1); 			
	}else{			
		LED_Off(1); 			
	}
}

/*----------------------------------------------------------------------------
  thrBUT_Left: check button state left
 *----------------------------------------------------------------------------*/
//osThreadFlagsSet(tid_thrBUZZ, 1U);
__NO_RETURN static void thrBUT_Right(void *argument) {
  //uint32_t button_msk = (1U << Buttons_GetCount()) - 1U;

	uint32_t last;
	uint32_t state;

	int32_t delay_val = 100U;
	
	for (;;) {
		state = Buttons_GetState ();           // Get pressed button state

		// If change in state
	    if (state != last){
			// Toggle GPIO for right button. Set r_flag, then toggle LED.
	      	if(is_set(state, 7) == 1){
						BSR = toggle_bit(BSR,7);
															
						LED_On(7); 		
							osDelay(100U);
						LED_Off(7); 		

					}			
	    }

	    last = state;
	    osDelay (delay_val);
	}

}
/*----------------------------------------------------------------------------
  thrBUT_Middle: check button state middle
 *----------------------------------------------------------------------------*/
//osThreadFlagsSet(tid_thrBUZZ, 1U);
__NO_RETURN static void thrBUT_Middle(void *argument) {
  //uint32_t button_msk = (1U << Buttons_GetCount()) - 1U;

	uint32_t last;
	uint32_t state;

	int32_t delay_val = 100U;
	
	for (;;) {
		state = Buttons_GetState ();           // Get pressed button state

		// If change in state
	    if (state != last){
					
					// Toggle GPIO for middle button. Set l_flag, then toggle LED.
					if(is_set(state, 6) == 1){
						BSR = toggle_bit(BSR,3);	
						osThreadFlagsSet (tid_thrBUZZ, 1U);        // Set flag to thrBUZZ
					}

	    }
			if(is_set(BSR, 3) == 1){			
				LED_On(5); 			
			}else{			
				LED_Off(5); 			
			}	

	    last = state;
	    osDelay (delay_val);
	}

}
/*----------------------------------------------------------------------------
  thrBUT_Right: check button state right
 *----------------------------------------------------------------------------*/
//osThreadFlagsSet(tid_thrBUZZ, 1U);
__NO_RETURN static void thrBUT_Left(void *argument) {
  //uint32_t button_msk = (1U << Buttons_GetCount()) - 1U;

	uint32_t last;
	uint32_t state;

	int32_t delay_val = 100U;
	
	for (;;) {
		state = Buttons_GetState ();           // Get pressed button state

		// If change in state
	    if (state != last){
					
					// Toggle GPIO for left button. Set m_flag, then toggle LED.
					if(is_set(state, 5) == 1){
						BSR = toggle_bit(BSR,0);
							
					}

			
	    }
			if(is_set(BSR, 0) == 1){			
				LED_On(1); 			
			}else{			
				LED_Off(1); 			
			}
	    last = state;
	    osDelay (delay_val);
	}

}


/*------------------------------------------------------------------------------
  thrADC: Get Value from ADC.
 *----------------------------------------------------------------------------*/

__NO_RETURN static void thrADC(void *argument) {
	// This function return stable temp in C.
	int initial = -200;

	int32_t val = get_ADC_Value();
	temp = val;
	initial = val;

	adcCalValue30 = (float)(*TEMP30_CAL_VALUE);
	adcCalValue110 = (float)(*TEMP110_CAL_VALUE);
	
	for(int i=0;i<7 ;){
		// Read The value from ADC
		ADC_StartConversion();          
		
		// Loop until ADC is done processing signal. Wait 1ms before each check.
		while(1){
			if (ADC_ConversionDone() == 0) {
				// T2: Read SDR
				val = ADC_GetValue();      			
				break;
			}		
		}    
		// Check if previous val is not equal to current
		if(initial != val){
		 
		 // Set new val as previous val
		 initial = val;		 
		 // start from begining
		 i=0;
		 }else{			 
			 // Increment counter
			 i++;			 
		 }
			 
		val = convert_internal_temperatur(val);
		//val = convert_LM35(val);
		temp = val;
	 
	}
	
	// Turn off sensor.
	BSR = set_bit(BSR, 4);
	osDelay(200U);
	osThreadFlagsSet(tid_thrBUZZ, 0x0001U);
	
	// T4 : Turn off sensor
	ADC_Uninitialize();																	// Turn off sensor.
	osThreadExit();
}


void setUp(){
		
	#ifdef RTE_Compiler_EventRecorder
		EventRecorderInitialize(0U, 1U);
		EventRecorderEnable (EventRecordError, 0xF0U, 0xF8U);     /* RTOS Events */
		EventRecorderEnable (EventRecordAll, 0xF2U, 0xF2U);       /* Thread Events */
	#endif
		
	Buttons_Initialize();                                     /* initalize Buttons */
	LED_Initialize();                                         /* initalize LEDs */
	BSR = 0;
	setUpGUI();
  buzzer_setup();
	
  showStringDisplay("Group R1.");
		
  tid_thrBUT_Left = osThreadNew (thrBUT_Left, NULL, NULL);            /* create BUT thread */
  if (tid_thrBUT_Left == NULL) { /* add error handling */ }
	
  tid_thrBUT_Middle = osThreadNew (thrBUT_Middle, NULL, NULL);            /* create BUT thread */
  if (tid_thrBUT_Middle == NULL) { /* add error handling */ }
	
  tid_thrBUT_Right = osThreadNew (thrBUT_Right, NULL, NULL);            /* create BUT thread */
  if (tid_thrBUT_Right == NULL) { /* add error handling */ }
	
  tid_thrBUZZ = osThreadNew (thrBUZZ, NULL, NULL);            /* create BUZZ thread */
  if (tid_thrBUZZ == NULL) { /* add error handling */ }
	
	
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	
}
#include <time.h>
  time_t rawtime;
  struct tm * timeinfo;

/*----------------------------------------------------------------------------
 * Tasks
 *---------------------------------------------------------------------------*/
void left_button(){
	// Left Button Pressed. check `F & 1000 0000`.
	// Check Left Button Pressed.
	int flag = is_set(BSR, 0);
	if(flag != 0){
			
		BSR = clear_bit(BSR, 0);

		// T8 :Toggle t_flag
		BSR = toggle_bit(BSR, 1);
	}		
}

int middle_button(int s_flag){
		// If Middle button pressed. check `F & 0001 0000`.
	if(is_set(BSR, 3) != 0){
		BSR = clear_bit(BSR, 3);
		
		// Toggle b_flag
		if(is_set(BSR, 4) != 0){
			// Set l_flag = 0. That is left button click detected.
			BSR = clear_bit(BSR, 4);
			//osDelay(100U);
		}else{
			// Clear state.
			BSR = 0;
			GUI_Clear();
			showStringDisplay("Taking new Reading ... ");
			
			osDelay(1000U);
			
			// T14 : turn on sensor 
			// Turn Sensor on.
			s_flag = ADC_Initialize();
			
		}
	}
	
	return s_flag;
}

int right_button(int s_flag){
			// If Right button pressed. Stop thermometer and exit loop.
		if(is_set(BSR, 7) != 0){
			GUI_Clear();
			for(int i=0; i<8;i++){
				LED_Off(i);
			}
			BSR &= 73U;
			
			showStringDisplay("Bye!");
			osThreadTerminate(tid_thrBUT_Left);
			osThreadTerminate(tid_thrBUT_Middle);
			//osThreadTerminate(tid_thrBUT_Right);
			osThreadTerminate(tid_thrBUZZ);
			HAL_GPIO_WritePin(GPIO_PINS[0].port, GPIO_PINS[0].pin, GPIO_PIN_SET);
			
			osDelay(1000U);
			GUI_Clear();
			while(1){
					
				 if(is_set(BSR, 7) == 0) {
					  tid_thrBUT_Left = osThreadNew (thrBUT_Left, NULL, NULL);            /* create BUT thread */
						if (tid_thrBUT_Left == NULL) { /* add error handling */ }

						tid_thrBUT_Middle = osThreadNew (thrBUT_Middle, NULL, NULL);            /* create BUT thread */
						if (tid_thrBUT_Middle == NULL) { /* add error handling */ }

						tid_thrBUZZ = osThreadNew (thrBUZZ, NULL, NULL);            /* create BUZZ thread */
						if (tid_thrBUZZ == NULL) { /* add error handling */ }
						
						BSR = 0;
						str = "C";
						s_flag = ADC_Initialize();		
						return s_flag;
				 }			
				 osDelay(500U);
				}
		}
		return s_flag;
}

void display(){
	GUI_Clear();
	showIntegerDisplay(temp);
	showStringDisplay(str);
	osDelay(500U);	
}

void mode(){
	if(is_set(BSR, 1) != 0){
		if(str[0] == 'C'){
			
			// T9: convert faren
			temp = Farenheit(temp);				
		}
		str = "F";
	}else{			
		if(str[0] == 'F'){
			temp = Celcius(temp);				
		}
		str = "C";
	}
}
	


/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/
__NO_RETURN void app_main (void *argument) {

	
  (void)argument;
	setUp();
	
	
	unsigned long t1 = DWT->CYCCNT;
	/* do something */
	//osDelay(1000U);
	unsigned long t2 = DWT->CYCCNT;
	unsigned long diff = t2 - t1;
	//printf("Print %04X\r\n", diff);
	osDelay(900U);
	GUI_Clear();
	
	// T1 : READ_SSR
	int s_flag = ADC_Initialize();														/* initialize ADC */
	
	
	while(1){
	
		// Check if ADC ready (s_flag value is 0/-1).
		if(s_flag == 1){
			
			// T1 : temp
			tid_thrADC = osThreadNew (thrADC, NULL, NULL);            /* create ADC thread. Read stable temperature and saves it in temp. */
			if (tid_thrADC == NULL) { /* add error handling */ }
	
			s_flag = 0;
		}
		
		
		// T2 : Left Button pressed.
		left_button();
		
		// T3 : Middle Button Pressed
		s_flag = middle_button(s_flag);
		
		// T4 : Display
		display();
		
		// T5 : Mode
		mode();
		
		// T6 : Right Button
		s_flag = right_button(s_flag);
		
	}
	
	
}
