/**
  ******************************************************************************
  * @file    main.c
  * @author  
  * @version V1.0
  * @date    01-08-2019
  * @brief   PID with PWM and encoders
  ******************************************************************************
*/
#include "stm32f4xx.h"


	//available to the rest of the code
	//speeds
	volatile int16_t leftCount;
	volatile int16_t rightCount;
	volatile int16_t fwdCount;
	volatile int16_t rotCount;
	//distances
	volatile int32_t leftTotal;
	volatile int32_t rightTotal;
	volatile int32_t fwdTotal;
	volatile int32_t rotTotal;

	// local variables
	static volatile int16_t oldLeftEncoder;
	static volatile int16_t oldRightEncoder;
	static volatile int16_t leftEncoder;
	static volatile int16_t rightEncoder;
	static volatile int16_t encoderSum;
	static volatile int16_t encoderDiff;

	float dt = 2;
	float ep;
	float en;
	float U;
	float C;
	float Kp;
	float Ki;
	float Td;


 /*
   * definitions for the quadrature encoder pins
   */
// Left Motor Channels
#define ENCLA_PIN               GPIO_Pin_15
#define ENCLA_GPIO_PORT         GPIOA
#define ENCLA_GPIO_CLK          RCC_AHB1Periph_GPIOA
#define ENCLA_SOURCE            GPIO_PinSource15
#define ENCLA_AF                GPIO_AF_TIM2

#define ENCLB_PIN               GPIO_Pin_3
#define ENCLB_GPIO_PORT         GPIOB
#define ENCLB_GPIO_CLK          RCC_AHB1Periph_GPIOB
#define ENCLB_SOURCE            GPIO_PinSource3
#define ENCLB_AF                GPIO_AF_TIM2

// Right Motor Channels
#define ENCRA_PIN               GPIO_Pin_6
#define ENCRA_GPIO_PORT         GPIOB
#define ENCRA_GPIO_CLK          RCC_AHB1Periph_GPIOB
#define ENCRA_SOURCE            GPIO_PinSource6
#define ENCRA_AF                GPIO_AF_TIM4

#define ENCRB_PIN               GPIO_Pin_7
#define ENCRB_GPIO_PORT         GPIOB
#define ENCRB_GPIO_CLK          RCC_AHB1Periph_GPIOB
#define ENCRB_SOURCE            GPIO_PinSource7
#define ENCRB_AF                GPIO_AF_TIM4

// determine the timers to use
#define ENCL_TIMER              TIM2
#define ENCL_TIMER_CLK          RCC_APB1Periph_TIM2
#define ENCR_TIMER              TIM4
#define ENCR_TIMER_CLK          RCC_APB1Periph_TIM4

#define LEFT_COUNT()            ENCL_TIMER->CNT
#define RIGHT_COUNT()           ENCR_TIMER->CNT

/*
 * Inicjalizowanie pinów i kana³ów do sterowania silnikami
 */

// Piny generuj¹ce PWM

#define PWMR_PIN 				GPIO_Pin_8
#define PWMR_GPIO_CLK			RCC_AHB1Periph_GPIOC
#define PWMR_SOURCE				GPIO_PinSource8

#define PWML_PIN 				GPIO_Pin_6
#define PWML_GPIO_CLK			RCC_AHB1Periph_GPIOC
#define PWML_SOURCE				GPIO_PinSource6

#define PWM_GPIO_PORT			GPIOC
#define PWM_AF					GPIO_AF_TIM3

//	Piny okreœlaj¹ce kierunek ruchu
#define DIRR_PIN 				GPIO_Pin_12
#define DIRR_GPIO_PORT			GPIOC
#define DIRR_GPIO_CLK			RCC_AHB1Periph_GPIOC
#define DIRR_SOURCE				GPIO_PinSource12

#define DIRL_PIN 				GPIO_Pin_10
#define DIRL_GPIO_PORT			GPIOC
#define DIRL_GPIO_CLK			RCC_AHB1Periph_GPIOC
#define DIRL_SOURCE				GPIO_PinSource10


//  Ustawienia zegara
#define MOT_TIMER				TIM3
#define MOT_TIMER_CLOCK			RCC_APB1Periph_TIM3

// Ustawienia kana³ów i odpowiadaj¹cych im funkcji
//Silnik prawy na kanale 3, lewy na kanale 1
#define MOTR_CHANNEL_INIT(tim,channel_str)		TIM_OC3Init(tim,channel_str)
#define MOTL_CHANNEL_INIT(tim,channel_str)		TIM_OC1Init(tim,channel_str)

#define MOTR_SetCompare(Compare)				TIM_SetCompare3(TIM3, Compare)
#define MOTL_SetCompare(Compare)				TIM_SetCompare1(TIM3, Compare)

/*
 * Inicjalizacja pinów i timera do sterowania silnikami
 */
void motorContrInit(void)
{
	GPIO_InitTypeDef gpio;           //struktura przechowuj¹ca dane do konfiguracji GPIO
	TIM_TimeBaseInitTypeDef tim;     //struktura przechowuj¹ca dane do konfiguracji timera
	TIM_OCInitTypeDef  channel;      //struktura przechowuj¹ca dane do konfiguracji kana³ów timera

	RCC_AHB1PeriphClockCmd(DIRR_GPIO_CLK   , ENABLE);   //pod³¹czenie zegara do GPIO
	RCC_APB1PeriphClockCmd(MOT_TIMER_CLOCK , ENABLE);                                                  //pod³¹czenie zegara do timera TIM3

	TIM_TimeBaseStructInit(&tim);               //inicjalizacja timera wartoœciami domyœlnymi
	tim.TIM_CounterMode = TIM_CounterMode_Up;   //timer liczy w górê od 0 do Period
	tim.TIM_Prescaler = 100;                    //preskaler z zegara systemowego
	tim.TIM_Period = 1000 - 1;                  //Period - d³ugoœc impulsu
	TIM_TimeBaseInit(MOT_TIMER, &tim);               //inicjalizacja timera danymi ze struktury tim

	TIM_OCStructInit(&channel);                        //inicjalizacja kana³u timera wartoœciami domyœlnymi
	channel.TIM_OCMode = TIM_OCMode_PWM1;              //tryb PWM, gdzie wype³nienie jest stanem wysokim
	channel.TIM_OutputState = TIM_OutputState_Enable;  //w³¹cz dzia³anie
	//inicjalizacja kana³u danymi ze struktury channel
	MOTL_CHANNEL_INIT(MOT_TIMER,&channel);

	TIM_OCStructInit(&channel);                        //inicjalizacja kana³u timera wartoœciami domyœlnymi
	channel.TIM_OCMode = TIM_OCMode_PWM1;              //tryb PWM, gdzie wype³nienie jest stanem wysokim
	channel.TIM_OutputState = TIM_OutputState_Enable;  //w³¹cz dzia³anie
	//inicjalizacja kana³u danymi ze struktury channel
	MOTR_CHANNEL_INIT(MOT_TIMER,&channel);

	GPIO_PinAFConfig(PWM_GPIO_PORT, PWMR_SOURCE, PWM_AF);
	GPIO_PinAFConfig(PWM_GPIO_PORT, PWML_SOURCE, PWM_AF);

	GPIO_StructInit(&gpio);              //inicjalizacja GPIO wartoœciami domyœlnymi
	gpio.GPIO_Pin = PWMR_PIN|PWML_PIN;          //Piny
	gpio.GPIO_Mode = GPIO_Mode_AF;       //Pin jako "funkcyjny"
	gpio.GPIO_OType = GPIO_OType_PP;     //Pin w trybie push-pull
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;   //bez rezystora podci¹gaj¹cego
	gpio.GPIO_Speed = GPIO_Speed_100MHz; //czêstotliwoœc odœwie¿ania GPIO
	GPIO_Init(GPIOC, &gpio); //inicjalizacja GPIO dla PWM
 /*Inicjalizacja Pinów kierunku*/
	gpio.GPIO_Pin = DIRR_PIN;
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init (DIRR_GPIO_PORT, &gpio);
	gpio.GPIO_Pin = DIRL_PIN;
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init (DIRL_GPIO_PORT, &gpio);

	TIM_Cmd(MOT_TIMER, ENABLE);        //uruchomienie timera
}

/*
 *  Funkcje ustawiaj¹ce kierunek obrotu silnika i prêdkoœc
 *
 *  Prêdkosc ustawiana jest w 0.1% wypelnienia PWM
 *  Kierunek - naprzód: 1, wstecz: -1.
 */
void setMoveR(int8_t movementDir, uint32_t Comp)
{
	switch(movementDir)
	{
		case 1:
			GPIO_ResetBits(DIRR_GPIO_PORT,DIRR_PIN);
			MOTR_SetCompare(Comp);
			break;
		case -1:
			GPIO_SetBits(DIRR_GPIO_PORT,DIRR_PIN);
			MOTR_SetCompare(Comp);
			break;
		default:
			return;
	}
}

void setMoveL(int8_t movementDir, uint32_t Comp)
{
	switch(movementDir)
	{
		case 1:
			GPIO_ResetBits(DIRL_GPIO_PORT,DIRL_PIN);
			MOTL_SetCompare(Comp);
			break;
		case -1:
			GPIO_SetBits(DIRL_GPIO_PORT,DIRL_PIN);
			MOTL_SetCompare(Comp);
			break;
		default:
			return;
	}
}
/*
 * Configure two timers as quadrature encoder counters.
 * Details of which timers should be used are
 * in the project hardware header file.
 * Most timers can be used if channels 1 and 2 are available on pins.
 * The timers are mostly 16 bit. Timers can be set to 32 bit but they are
 * not very convenient for IO pins so the counters are simply set to to
 * 16 bit counting regardless.
 * A mouse needs 32 bits of positional data and, since it also needs the
 * current speed, distance is not maintained by the encoder code but will
 * be looked after by the motion control code.
 * The counters are set to X4 mode. The only alternative is X2 counting.
 */

void encodersRead (void)
{
  oldLeftEncoder = leftEncoder;
  leftEncoder = TIM_GetCounter (ENCL_TIMER) ;
  oldRightEncoder = rightEncoder;
  rightEncoder = -TIM_GetCounter (ENCR_TIMER) ;
  leftCount = leftEncoder - oldLeftEncoder;
  rightCount = rightEncoder - oldRightEncoder;
  fwdCount = leftCount + rightCount;
  rotCount = - (leftCount - rightCount);
  fwdTotal += fwdCount;
  rotTotal += rotCount;
  leftTotal += leftCount;
  rightTotal += rightCount;
}

void encodersReset (void)
{
  __disable_irq();
  oldLeftEncoder = 0;
  oldRightEncoder = 0;
  leftTotal = 0;
  rightTotal = 0;
  fwdTotal = 0;
  rotTotal = 0;
  TIM_SetCounter (ENCL_TIMER, 0);
  TIM_SetCounter (ENCR_TIMER, 0);
  encodersRead();
  __enable_irq();
}


void encodersInit (void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  // turn on the clocks for each of the ports needed
  RCC_AHB1PeriphClockCmd (ENCLA_GPIO_CLK, ENABLE);
  RCC_AHB1PeriphClockCmd (ENCLB_GPIO_CLK, ENABLE);
  RCC_AHB1PeriphClockCmd (ENCRA_GPIO_CLK, ENABLE);
  RCC_AHB1PeriphClockCmd (ENCRB_GPIO_CLK, ENABLE);

  // now configure the pins themselves
  // they are all going to be inputs with pullups
  GPIO_StructInit (&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Pin = ENCLA_PIN;
  GPIO_Init (ENCLA_GPIO_PORT, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = ENCLB_PIN;
  GPIO_Init (ENCLB_GPIO_PORT, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = ENCRA_PIN;
  GPIO_Init (ENCRA_GPIO_PORT, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = ENCRB_PIN;
  GPIO_Init (ENCRB_GPIO_PORT, &GPIO_InitStructure);

  // Connect the pins to their Alternate Functions
  GPIO_PinAFConfig (ENCLA_GPIO_PORT, ENCLA_SOURCE, ENCLA_AF);
  GPIO_PinAFConfig (ENCLB_GPIO_PORT, ENCLB_SOURCE, ENCLB_AF);
  GPIO_PinAFConfig (ENCRA_GPIO_PORT, ENCRA_SOURCE, ENCRA_AF);
  GPIO_PinAFConfig (ENCRB_GPIO_PORT, ENCRB_SOURCE, ENCRB_AF);

  // Timer peripheral clock enable
  RCC_APB1PeriphClockCmd (ENCL_TIMER_CLK, ENABLE);
  RCC_APB1PeriphClockCmd (ENCR_TIMER_CLK, ENABLE);

  // set them up as encoder inputs
  // set both inputs to rising polarity to let it use both edges
  TIM_EncoderInterfaceConfig (ENCL_TIMER, TIM_EncoderMode_TI12,
                              TIM_ICPolarity_Rising,
                              TIM_ICPolarity_Rising);
  TIM_SetAutoreload (ENCL_TIMER, 0xffff);
  TIM_EncoderInterfaceConfig (ENCR_TIMER, TIM_EncoderMode_TI12,
                              TIM_ICPolarity_Rising,
                              TIM_ICPolarity_Rising);
  TIM_SetAutoreload (ENCR_TIMER, 0xffff);

  // turn on the timer/counters
  TIM_Cmd (ENCL_TIMER, ENABLE);
  TIM_Cmd (ENCR_TIMER, ENABLE);
  encodersReset();
}




int main(void)
{
	GPIO_InitTypeDef gpio;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE); //uruchomienie zegarów do portów GPIO

	gpio.GPIO_Pin = GPIO_Pin_5;          //ustawienie numeru pinu GPIO
	gpio.GPIO_Mode = GPIO_Mode_OUT;      //ustawienie trubu na wyjœcie
	GPIO_Init(GPIOA, &gpio);             //inicjalizacja pinu gpio
	GPIO_SetBits(GPIOA,GPIO_Pin_5);      //ustawienie stanu wysokiego na pinie PA5


	encodersInit();

	motorContrInit();

	setMoveR(1,300);
	setMoveL(1,300);

	Kp=10;
	Ki=5e-6;
	Td=2;
	ep=0;

	while(1)
	{
		encodersRead();
		en = rightTotal - leftTotal;
		C+=((ep + en)/2)*dt;
		U=Kp*(en + Ki*C + Td*(en-ep)/dt);
		ep = en;
		//setMoveR(1,300.0f+U/2);
		//setMoveL(1,300.0f-U/2);
	}

}
