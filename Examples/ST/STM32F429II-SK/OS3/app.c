/*
*********************************************************************************************************
*                                              EXAMPLE CODE
*
*                             (c) Copyright 2013; Micrium, Inc.; Weston, FL
*
*                   All rights reserved.  Protected by international copyright laws.
*                   Knowledge of the source code may not be used to write a similar
*                   product.  This file may only be used in accordance with a license
*                   and should not be redistributed in any way.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                            EXAMPLE CODE
*
*                                       IAR Development Kits
*                                              on the
*
*                                    STM32F429II-SK KICKSTART KIT
*
* Filename      : app.c
* Version       : V1.00
* Programmer(s) : YS
*                 DC
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#include  <includes.h>
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx.h"
#include "bsp.h"
/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/

#define  APP_TASK_EQ_0_ITERATION_NBR              16u
/*
*********************************************************************************************************
*                                            TYPES DEFINITIONS
*********************************************************************************************************
*/
typedef enum {
   TASK_500MS,
   TASK_1000MS,
   TASK_2000MS,

   TASK_N
}task_e;
typedef struct
{
   CPU_CHAR* name;
   OS_TASK_PTR func;
   OS_PRIO prio;
   CPU_STK* pStack;
   OS_TCB* pTcb;
}task_t;

/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/
static  void  AppTaskStart          (void     *p_arg);
static  void  AppTaskCreate         (void);
static  void  AppObjCreate          (void);

static void AppTask_500ms(void *p_arg);
static void AppTask_1000ms(void *p_arg);
static void AppTask_2000ms(void *p_arg);

static void Setup_Gpio(void);


/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/
/* ----------------- APPLICATION GLOBALS -------------- */
static  OS_TCB   AppTaskStartTCB;
static  CPU_STK  AppTaskStartStk[APP_CFG_TASK_START_STK_SIZE];

static  OS_TCB       Task_500ms_TCB;
static  OS_TCB       Task_1000ms_TCB;
static  OS_TCB       Task_2000ms_TCB;

static  CPU_STK  Task_500ms_Stack[APP_CFG_TASK_START_STK_SIZE];
static  CPU_STK  Task_1000ms_Stack[APP_CFG_TASK_START_STK_SIZE];
static  CPU_STK  Task_2000ms_Stack[APP_CFG_TASK_START_STK_SIZE];
int count=0;
task_t cyclic_tasks[TASK_N] = {
   {"Task_500ms" , AppTask_500ms,  0, &Task_500ms_Stack[0] , &Task_500ms_TCB},
   {"Task_1000ms", AppTask_1000ms, 0, &Task_1000ms_Stack[0], &Task_1000ms_TCB},
   {"Task_2000ms", AppTask_2000ms, 0, &Task_2000ms_Stack[0], &Task_2000ms_TCB},
};
/* ------------ FLOATING POINT TEST TASK -------------- */
/*
*********************************************************************************************************
*                                                main()
*
* Description : This is the standard entry point for C code.  It is assumed that your code will call
*               main() once you have performed all necessary initialization.
*
* Arguments   : none
*
* Returns     : none
*********************************************************************************************************
*/

#define DECODE_MODE_REG     0x09
#define INTENSITY_REG       0x0A
#define SCAN_LIMIT_REG      0x0B
#define SHUTDOWN_REG        0x0C
#define DISPLAY_TEST_REG    0x0F


void TIM2_Configuration(void)
{
	BSP_PeriphEn(BSP_PERIPH_ID_GPIOA);
	BSP_PeriphEn(BSP_PERIPH_ID_TIM2);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0,GPIO_AF_TIM2);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Prescaler = 1000 -1;
	TIM_TimeBaseStructure.TIM_Period = 16 -1 ;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM2, ENABLE);

	TIM_OCInitTypeDef TIM_OCInitStructure;
	/* TIM PWM1 Mode configuration */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 100;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;

	/* Enable TIM2 Preload register on ARR */
	TIM_ARRPreloadConfig(TIM2, ENABLE);
	/* Output Compare PWM1 Mode configuration: Channel1*/
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	//channel 1 - >TIM2->CCR1 를 통해서  듀티 사이클 결정 15 정도가 최대 크기가 되는 듯하다.

}
void init_TIM2(void){  // Output compare mode, PWM
	BSP_PeriphEn(BSP_PERIPH_ID_TIM3);

    TIM3->PSC = 16-1;   // 1us
    TIM3->EGR = (1<<0); // Bit 0 UG
    TIM3->CR1 = (1<<0); // Bit 0 CEN

    BSP_PeriphEn(BSP_PERIPH_ID_GPIOB);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource0,GPIO_AF_TIM3);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

}

uint16_t get_distance(void){
    uint16_t distance;  //Triger PB8, Echo PB0

	GPIO_WriteBit(GPIOB, GPIO_Pin_8, Bit_SET);
	TIM3->CNT=0; while(TIM3->CNT<12);  // 12us delay
	GPIO_WriteBit(GPIOB, GPIO_Pin_8, Bit_RESET);

	while(!(GPIOB->IDR&0x0001));
	TIM3->CNT=0; while(GPIOB->IDR&0x0001);
	distance=(TIM3->CNT+1)/58;  // cm

	return distance;
}

void ADC_configure(void){
	BSP_PeriphEn(BSP_PERIPH_ID_ADC1);
	BSP_PeriphEn(BSP_PERIPH_ID_GPIOA);

//PA3 -> ADC1
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	ADC_InitTypeDef ADC_InitStructure;
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_10b;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 1;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 1, ADC_SampleTime_15Cycles);

	ADC_Init(ADC1, &ADC_InitStructure);

	ADC_Cmd(ADC1, ENABLE);
	//    	u16 jodo = ADC1->DR; 를 통해서 값을 얻을 수 있다.
}

void GPIO_configure(void){
	BSP_PeriphEn(BSP_PERIPH_ID_GPIOA);
	BSP_PeriphEn(BSP_PERIPH_ID_GPIOB);

	GPIO_InitTypeDef gpio_init;
	//소리 센서 설정
	gpio_init.GPIO_Pin =  GPIO_Pin_2|GPIO_Pin_6;
	gpio_init.GPIO_Mode = GPIO_Mode_IN;
	gpio_init.GPIO_OType = GPIO_OType_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_25MHz;
	gpio_init.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOB, &gpio_init);

	//능동 수저 설정
	gpio_init.GPIO_Pin =  GPIO_Pin_5;
	gpio_init.GPIO_Mode = GPIO_Mode_OUT;
	gpio_init.GPIO_OType = GPIO_OType_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_25MHz;
	gpio_init.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &gpio_init);

	//
	gpio_init.GPIO_Pin =  GPIO_Pin_4;
	gpio_init.GPIO_Mode = GPIO_Mode_AN;
	gpio_init.GPIO_OType = GPIO_OType_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_25MHz;
	gpio_init.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &gpio_init);

}


//TIM4_CH3 -> PD14
// TIM3_CH3 -> PB0

int main(void)
{
    OS_ERR  err;

    /* Basic Init */
    RCC_DeInit();
//    SystemCoreClockUpdate();
    Setup_Gpio();

    /* BSP Init */
    BSP_IntDisAll();                                            /* Disable all interrupts.                              */

    CPU_Init();                                                 /* Initialize the uC/CPU Services                       */
    Mem_Init();                                                 /* Initialize Memory Management Module                  */
    Math_Init();                                                /* Initialize Mathematical Module                       */

   // spi_configure();

   // max7219_init();

    USART_Config();


    ADC_configure();
    TIM2_Configuration();
    init_TIM2();
    int i=0;
    while(1){
    	/* USER CODE END WHILE */
    	u8 sori = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_2);
    	u8 nock = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6);
    	char str[20];
    	ADC_SoftwareStartConv(ADC1);
    	u16 jodo = ADC1->DR;
    	sprintf(str," adc : %d, dist : %d \r\n",jodo,get_distance());
    	send_string(str);

    	i = (i+1)%15;
    	TIM2->CCR1 = i;
    	for(int i=0;i<1000000;i++);

    	if(sori || nock){
    		GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_SET);
    	}
    }

    /* OS Init */
    OSInit(&err);                                               /* Init uC/OS-III.                                      */

    OSTaskCreate((OS_TCB       *)&AppTaskStartTCB,              /* Create the start task                                */
                 (CPU_CHAR     *)"App Task Start",
                 (OS_TASK_PTR   )AppTaskStart,
                 (void         *)0u,
                 (OS_PRIO       )APP_CFG_TASK_START_PRIO,
                 (CPU_STK      *)&AppTaskStartStk[0u],
                 (CPU_STK_SIZE  )AppTaskStartStk[APP_CFG_TASK_START_STK_SIZE / 10u],
                 (CPU_STK_SIZE  )APP_CFG_TASK_START_STK_SIZE,
                 (OS_MSG_QTY    )0u,
                 (OS_TICK       )0u,
                 (void         *)0u,
                 (OS_OPT        )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR       *)&err);

   OSStart(&err);   /* Start multitasking (i.e. give control to uC/OS-III). */

   (void)&err;

   return (0u);
}
/*
*********************************************************************************************************
*                                          STARTUP TASK
*
* Description : This is an example of a startup task.  As mentioned in the book's text, you MUST
*               initialize the ticker only once multitasking has started.
*
* Arguments   : p_arg   is the argument passed to 'AppTaskStart()' by 'OSTaskCreate()'.
*
* Returns     : none
*
* Notes       : 1) The first line of code is used to prevent a compiler warning because 'p_arg' is not
*                  used.  The compiler should not generate any code for this statement.
*********************************************************************************************************
*/
static  void  AppTaskStart (void *p_arg)
{
    OS_ERR  err;

   (void)p_arg;

    BSP_Init();                                                 /* Initialize BSP functions                             */
    BSP_Tick_Init();                                            /* Initialize Tick Services.                            */

#if OS_CFG_STAT_TASK_EN > 0u
    OSStatTaskCPUUsageInit(&err);                               /* Compute CPU capacity with no task running            */
#endif

#ifdef CPU_CFG_INT_DIS_MEAS_EN
    CPU_IntDisMeasMaxCurReset();
#endif

   // BSP_LED_Off(0u);                                            /* Turn Off LEDs after initialization                   */

   APP_TRACE_DBG(("Creating Application Kernel Objects\n\r"));
   AppObjCreate();                                             /* Create Applicaiton kernel objects                    */

   APP_TRACE_DBG(("Creating Application Tasks\n\r"));
   AppTaskCreate();                                            /* Create Application tasks                             */
}

/*
*********************************************************************************************************
*                                          AppTask_500ms
*
* Description : Example of 500mS Task
*
* Arguments   : p_arg (unused)
*
* Returns     : none
*
* Note: Long period used to measure timing in person
*********************************************************************************************************
*/
static void AppTask_500ms(void *p_arg)
{
    OS_ERR  err;
    BSP_LED_On(1);
    while (DEF_TRUE) {                                          /* Task body, always written as an infinite loop.       */
       BSP_LED_Toggle(1);
        OSTimeDlyHMSM(0u, 0u, 0u, 500u,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);
       count++;

    }
}

/*
*********************************************************************************************************
*                                          AppTask_1000ms
*
* Description : Example of 1000mS Task
*
* Arguments   : p_arg (unused)
*
* Returns     : none
*
* Note: Long period used to measure timing in person
*********************************************************************************************************
*/
static void AppTask_1000ms(void *p_arg)
{
    OS_ERR  err;
    BSP_LED_On(2);
    while (DEF_TRUE) {                                          /* Task body, always written as an infinite loop.       */
       BSP_LED_Toggle(2);

        OSTimeDlyHMSM(0u, 0u, 1u, 0u,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);

    }
}

/*
*********************************************************************************************************
*                                          AppTask_2000ms
*
* Description : Example of 2000mS Task
*
* Arguments   : p_arg (unused)
*
* Returns     : none
*
* Note: Long period used to measure timing in person
*********************************************************************************************************
*/
static void AppTask_2000ms(void *p_arg)
{
    OS_ERR  err;
    BSP_LED_On(3);
    while (DEF_TRUE) {                                          /* Task body, always written as an infinite loop.       */
       BSP_LED_Toggle(3);

        OSTimeDlyHMSM(0u, 0u, 2u, 0u,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);

    }
}
/*
*********************************************************************************************************
*                                          AppTaskCreate()
*
* Description : Create application tasks.
*
* Argument(s) : none
*
* Return(s)   : none
*
* Caller(s)   : AppTaskStart()
*
* Note(s)     : none.
*********************************************************************************************************
*/

static  void  AppTaskCreate (void)
{
   OS_ERR  err;

   u8_t idx = 0;
   task_t* pTask_Cfg;
   for(idx = 0; idx < TASK_N; idx++)
   {
      pTask_Cfg = &cyclic_tasks[idx];

      OSTaskCreate(
            pTask_Cfg->pTcb,
            pTask_Cfg->name,
            pTask_Cfg->func,
            (void         *)0u,
            pTask_Cfg->prio,
            pTask_Cfg->pStack,
            pTask_Cfg->pStack[APP_CFG_TASK_START_STK_SIZE / 10u],
            APP_CFG_TASK_START_STK_SIZE,
            (OS_MSG_QTY    )0u,
            (OS_TICK       )0u,
            (void         *)0u,
            (OS_OPT        )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
            (OS_ERR       *)&err
      );
   }
}


/*
*********************************************************************************************************
*                                          AppObjCreate()
*
* Description : Create application kernel objects tasks.
*
* Argument(s) : none
*
* Return(s)   : none
*
* Caller(s)   : AppTaskStart()
*
* Note(s)     : none.
*********************************************************************************************************
*/

static  void  AppObjCreate (void)
{

}

/*
*********************************************************************************************************
*                                          Setup_Gpio()
*
* Description : Configure LED GPIOs directly
*
* Argument(s) : none
*
* Return(s)   : none
*
* Caller(s)   : AppTaskStart()
*
* Note(s)     :
*              LED1 PB0
*              LED2 PB7
*              LED3 PB14
*
*********************************************************************************************************
*/
static void Setup_Gpio(void)
{
   GPIO_InitTypeDef led_init = {0};

   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
   RCC_AHB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

   led_init.GPIO_Mode   = GPIO_Mode_OUT;
   led_init.GPIO_OType  = GPIO_OType_PP;
   led_init.GPIO_Speed  = GPIO_Speed_2MHz;
   led_init.GPIO_PuPd   = GPIO_PuPd_NOPULL;
   led_init.GPIO_Pin    = GPIO_Pin_0 | GPIO_Pin_7 | GPIO_Pin_14;

   GPIO_Init(GPIOB, &led_init);
}

