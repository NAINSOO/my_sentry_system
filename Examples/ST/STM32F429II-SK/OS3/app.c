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
   TASK_JODO,
   TASK_SORI,
   TASK_DIST,
   TASK_ALERT,
   TASK_LED,


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

static void AppTask_jodo(void *p_arg);
static void AppTask_sori(void *p_arg);
static void AppTask_dist(void *p_arg);
static void AppTask_alert(void *p_arg);
static void AppTask_led(void *p_arg);


static void Setup_Gpio(void);


/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/
/* ----------------- APPLICATION GLOBALS -------------- */
static  OS_TCB   AppTaskStartTCB;
static  CPU_STK  AppTaskStartStk[APP_CFG_TASK_START_STK_SIZE];

static  OS_TCB       Task_jodo_TCB;
static  OS_TCB       Task_sori_TCB;
static  OS_TCB       Task_dist_TCB;
static  OS_TCB       Task_led_TCB;
static  OS_TCB       Task_alert_TCB;


static  CPU_STK  Task_jodo_Stack[APP_CFG_TASK_START_STK_SIZE];
static  CPU_STK  Task_sori_Stack[APP_CFG_TASK_START_STK_SIZE];
static  CPU_STK  Task_dist_Stack[APP_CFG_TASK_START_STK_SIZE];
static  CPU_STK  Task_led_Stack[APP_CFG_TASK_START_STK_SIZE];
static  CPU_STK  Task_alert_Stack[APP_CFG_TASK_START_STK_SIZE];


OS_SEM emergency;
OS_SEM alertstart;
OS_Q led_Q;




task_t cyclic_tasks[TASK_N] = {
   {"Task_jodo",AppTask_jodo, 5,&Task_jodo_Stack[0], &Task_jodo_TCB },
   {"Task_sori",AppTask_sori, 5,&Task_sori_Stack[0], &Task_sori_TCB },
   {"Task_dist",AppTask_dist, 4,&Task_dist_Stack[0], &Task_dist_TCB },
   {"Task_led",AppTask_led, 2,&Task_led_Stack[0], &Task_led_TCB },
   {"Task_alert",AppTask_alert, 2,&Task_alert_Stack[0], &Task_alert_TCB },

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

int isNight=0; //1이면 night
u16 distance=0;
int isNomalState=1;
#define JODO_BOUNDARY 850

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
void init_TIM4(void){  // Output compare mode, PWM
	BSP_PeriphEn(BSP_PERIPH_ID_TIM4);

    TIM4->PSC = 16-1;   // 1us
    TIM4->EGR = (1<<0); // Bit 0 UG
    TIM4->CR1 = (1<<0); // Bit 0 CEN

    BSP_PeriphEn(BSP_PERIPH_ID_GPIOB);
    BSP_PeriphEn(BSP_PERIPH_ID_GPIOD);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource14,GPIO_AF_TIM4);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

}

uint32_t get_distance(void){
    uint32_t distance;  //Triger PB8, Echo PD14

	GPIO_WriteBit(GPIOB, GPIO_Pin_8, Bit_SET);
	TIM4->CNT=0; while(TIM4->CNT<12);  // 12us delay
	GPIO_WriteBit(GPIOB, GPIO_Pin_8, Bit_RESET);

	while(!(GPIOD->IDR&0x4000));
	TIM4->CNT=0; while(GPIOD->IDR&0x4000);
	distance=(TIM4->CNT+1)>>6;  // cm

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
	//ADC_SoftwareStartConv(ADC1);
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

CPU_SR_ALLOC();

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

   USART_Config();
   GPIO_configure();

   APP_TRACE_DBG(("Creating Application Kernel Objects\n\r"));
   AppObjCreate();                                             /* Create Applicaiton kernel objects                    */

   APP_TRACE_DBG(("Creating Application Tasks\n\r"));
   AppTaskCreate();                                            /* Create Application tasks                             */
}


/*
*********************************************************************************************************
*                                          AppTask_jodo
*
* Description : Example of jodo Task
*
* Arguments   : p_arg (unused)
*
* Returns     : none
*
* Note: 조도 센서를 읽고 thresholding을 통해서 global variable인 int isNight의 값을 write한다.
*********************************************************************************************************
*/

static void AppTask_jodo(void *p_arg)
{
    OS_ERR  err;
    ADC_configure();
	ADC_SoftwareStartConv(ADC1);

	send_string("jodo task start\r\n");
	char str[10];


    while (isNomalState) {                                          /* Task body, always written as an infinite loop.       */
    	u16 jodo = ADC1->DR;

    	sprintf(str,"%d %d\r\n",jodo,isNight);
    	send_string(str);
        if((jodo>=JODO_BOUNDARY) ){
        	OS_CRITICAL_ENTER();
        	isNight = 0;
        	OS_CRITICAL_EXIT();

    	}
        else{
        	OS_CRITICAL_ENTER();
        	isNight = 1;
        	OS_CRITICAL_EXIT();
        }
		OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_DLY,&err);

    }
}


/*
*********************************************************************************************************
*                                          AppTask_sori
*
* Description : Example of sori Task
*
* Arguments   : p_arg (unused)
*
* Returns     : none
*
* Note: 소리 센서 PB2가 high가 되면 semaphore를 release 해준다. aquire 하는 Task는 dist가 된다.
*********************************************************************************************************
*/

static void AppTask_sori(void *p_arg)
{
    OS_ERR  err;
	send_string("sori task start\r\n");

    while (isNomalState) {                                          /* Task body, always written as an infinite loop.       */
    	u8 sori = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_2);

    	if(sori){
    		OS_CRITICAL_ENTER();
    		isNomalState = 0;
    		OS_CRITICAL_EXIT();

    		OSSemPost(&emergency,
    					OS_OPT_POST_1,
						&err);
    	}
		OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_DLY,&err);

    }
}

/*
*********************************************************************************************************
*                                          AppTask_dist
*
* Description : Example of dist Task
*
* Arguments   : p_arg (unused)
*
* Returns     : none
*
* Note: 지속적으로 초음파 센서를 측정한다.alertTask가 timedly인 동안에 작동한다. ledTask와는 Q를 통해서 message passing을 한다. 원인 모를 오류가 발생 dist의 우선순위를 한 단계 내려본다.
*********************************************************************************************************
*/

static void AppTask_dist(void *p_arg)
{
    OS_ERR  err;
	CPU_TS ts;
    OSSemPend(&emergency,
    					0,
    					OS_OPT_PEND_BLOCKING,
    					&ts,
    					&err);

    send_string("dist start\r\n");


    OSSemPost(&alertstart,
        					OS_OPT_POST_1,
    						&err);
    init_TIM4();
    u32 dist = 400;
    while (DEF_TRUE) {
        send_string("dist around\r\n");
        OS_CRITICAL_ENTER();
        dist = get_distance();
        distance= dist;
		OS_CRITICAL_EXIT();

    	if(isNight){
    		OSQPost( (OS_Q *)&led_Q,
    	        	(void *)&dist,
    	        	(OS_MSG_SIZE)sizeof(void *),
    	        	(OS_OPT )OS_OPT_POST_FIFO,
    	        	(OS_ERR *)&err);
    	}
    }
}





/*
*********************************************************************************************************
*                                          AppTask_led
*
* Description : Example of led Task
*
* Arguments   : p_arg (unused)
*
* Returns     : none
*
* Note: distTask와 메시지 큐를 통해 통신한다. ledTask는 잠깐 동안만 ready 상태가 되고 곧 바로 pending된다.
*********************************************************************************************************
*/
static void AppTask_led(void *p_arg){
	 OS_ERR  err;
	 CPU_TS ts;
	 OS_MSG_SIZE size = 100;
	 u16 * dist_p;
	 u16 dist;

	 while (DEF_TRUE) {
		 dist_p = (u16 *)(CPU_INT32U)OSQPend((OS_Q *)&led_Q,
		 		(OS_TICK )0,
		 		(OS_OPT )OS_OPT_PEND_BLOCKING,
		 		(OS_MSG_SIZE *)&size,
		 		(CPU_TS *)&ts,
		 		(OS_ERR *)&err);
		 send_string("led \r\n");
		  TIM2_Configuration();


		 dist = *dist_p;

		 u16 bright = 20;

		 if(dist>300){
			 bright = 20;
		 }
		 else if(dist >200){
			 bright = 10;
		 }
		 else if(dist>100){
			 bright =7;
		 }
		 else if(dist>50){
			 bright = 4;
		 }
		 else{
			 bright = 2;
		 }


		 TIM2->CCR1 = bright ;
	 }

}

/*
*********************************************************************************************************
*                                          AppTask_alert
*
* Description : Example of alert Task
*
* Arguments   : p_arg (unused)
*
* Returns     : none
*
* Note: 전역변수 distance에 따라서 timedly의 시간을 줄인다.
*********************************************************************************************************
*/


static void AppTask_alert(void *p_arg)
{
    OS_ERR  err;
	CPU_TS ts;

    OSSemPend(&alertstart,
    					0,
    					OS_OPT_PEND_BLOCKING,
    					&ts,
    					&err);
    send_string("alert task start\r\n");
    // 30<= distance <= 400
    //   10 <=  delay <= 2000

    CPU_INT32U delay = 2000;
    int dist = distance;
    if(dist>300){
    	delay = 1900;
    		 }
    		 else if(dist >200){
    			 delay = 1500;
    		 }
    		 else if(dist>100){
    			 delay = 1000;
    		 }
    		 else if(dist>50){
    			 delay = 500;
    		 }
    		 else{
    			 delay = 100;
    		 }


    while (DEF_TRUE) {

        send_string("alert task around\r\n");
    	GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_SET);
		OSTimeDlyHMSM(0,0,0,delay,OS_OPT_TIME_DLY,&err);
		GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_RESET);
		OSTimeDlyHMSM(0,0,0,delay,OS_OPT_TIME_DLY,&err);

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
	OS_ERR err;

	OSSemCreate(&emergency,
	    		"emergency",
				(OS_SEM_CTR)0,
				&err);
	OSQCreate(&led_Q,
	    		"led queue",
				10,
				&err);
	OSSemCreate(&alertstart,
		    		"alertstart",
					(OS_SEM_CTR)0,
					&err);

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

