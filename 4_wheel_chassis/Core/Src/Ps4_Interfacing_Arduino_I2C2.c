/*
 * main.c
 *
 *  Created on: Dec 26, 2023
 *      Author: wardawg
 */
#include <string.h>
#include <stdio.h>
#include <main.h>
#include <math.h>
#include <stm32f407xx.h>
#include "i2c.h"
#include <tim.h>
#include "gpio.h"
#include "bno055_stm32.h"

/*
 * Macro Definitions
 */

#define MY_ADDR 0x61;

#define SLAVE_ADDR 0x68

/*
 * Handle Variables for the Peripherals
 */
GPIO_InitTypeDef led;

TIM_HandleTypeDef htimer4;

volatile uint32_t ccr_content;

I2C_Handle_t I2C2Handle;

//GPIO_Handle_t DirHandle;

uint8_t rcv_buf[7];

/*
 * Headers for the Functions
 */
extern void initialise_monitor_handles(void);

void System_Clock_Config(uint8_t CLOCK_FREQ);

void SystemClock_Config();

void Error_Handler();

void timer4_init();

void gpio_timer_Init();

void I2C2_GPIOInits(void);

void I2C2_Inits(void);

int x = 0;
int y = 0;
float w, v, v_x, v_y, v_w;
float w1, w2, w3, w4;
float targated_angle = 0, curr_angle = 0, error = 0, prev_error = 0,
		correction = 0, diff = 0;

float angle;
float KP = 0, KD = 0;

int map(int x, int in_min, int in_max, int out_min, int out_max);

void movement(float v, float v_w, float angle, float KP, float KD);

int main() {
	// Main Scope Variables and the Welcome Text!
	uint8_t commandcode;

	uint8_t len = 5;

	//	void SystemClock_Config();

	//	uint32_t brightness;
	initialise_monitor_handles();

	printf("Bravo-6 to Gold Eagle Actual--> Going Dark\n");

	// Calling All Initialization Functions

	// 1. The Main HAL_INIT
	HAL_Init();

	// BNO INITS
//	bno055_assignI2C(&hi2c1);
//	bno055_setup();
//	bno055_setOperationModeNDOF();

// We gotta run at 168, so here's the one for that
	SystemClock_Config(SYS_CLK_FREQ_168);

	// BNO_INITS_I2C
//	MX_I2C1_Init();

	// The GPIO-Acts-As-Timer Init
	gpio_timer_Init();

	// The actual Timer Init
	timer4_init();

	// The GPIO-Acts-As-I2C1 Init
	I2C2_GPIOInits();

	// The Actual I2C1 Init
	I2C2_Inits();

	// enable the i2c peripheral
	I2C_PeripheralControl(I2C2, ENABLE);

//	 ack bit is made 1 after PE=1
	I2C_ManageAcking(I2C2, I2C_ACK_ENABLE);

	/*******************************************************************************************************************
	 *Logic starts Here
	 *******************************************************************************************************************/

// Crank that Timer 4 on all the channels
	if (HAL_TIM_PWM_Start(&htimer4, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Start(&htimer4, TIM_CHANNEL_2) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Start(&htimer4, TIM_CHANNEL_3) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Start(&htimer4, TIM_CHANNEL_4) != HAL_OK) {
		Error_Handler();
	}

	while (1) {

		commandcode = 0x05;
		I2C_MasterSendData(&I2C2Handle, &commandcode, 1, SLAVE_ADDR,
		I2C_ENABLE_SR);

		I2C_MasterReceiveData(&I2C2Handle, rcv_buf, len, SLAVE_ADDR,
		I2C_DISABLE_SR);
		/*
		 * Post processing the data in order to convert it to a suitable range
		 */
		w = map(rcv_buf[2], 0, 255, -127, 127);
		if (rcv_buf[0] < 138 && rcv_buf[0] > 116) {
			x = 0;
		} else {
			//			x = map(rcv_buf[0], 0, 255, -127, 127);
			x = rcv_buf[0] - 127;
		}
		if (rcv_buf[1] < 138 && rcv_buf[1] > 116) {
			y = 0;
		} else {
			//			y = map(rcv_buf[0], 0, 255, -127, 127);
			y = 127 - rcv_buf[1];
		}
//		bno055_vector_t BNO_OP = bno055_getVectorEuler();
		//		printf(" x: %d  y: %d  w: %d b1: %d b2: %d BNO %.3f\n", x, y,
		//				rcv_buf[2], rcv_buf[3], rcv_buf[4], BNO_OP.x);
		v = sqrt((pow(x, 2) + pow(y, 2)));

		v = map(v, 0, 150, 0, 100);

		v_w = map(w, 0, 255, 0, 43);

		angle = atan2(y, x);

//		curr_angle = BNO_OP.x;
//		if (curr_angle) {
//			curr_angle = curr_angle - 180.0;
//		}
		movement(v, v_w, angle, KP, KD);
		HAL_Delay(1);
	}
	return 0;
}
void movement(float v, float v_w, float angle, float KP, float KD) {
	v_x = v * cos(angle);

	v_y = v * sin(angle);

	error = curr_angle - targated_angle;
	if (error < -180) {
		error = error + 360;
	}

	else if (error > 180) {
		error = error - 360;
	}

	diff = error - prev_error;

	correction = (error * KP) + (diff * KD);
//	correction = 0;

	//	prev_error = error;

	w1 = (0.70711 * (-v_x + v_y)) + w + correction;
	w2 = (0.70711 * (-v_x - v_y)) + w + correction;
	w3 = (0.70711 * (v_x - v_y)) + w + correction;
	w4 = (0.70711 * (v_x + v_y)) + w + correction;

	//	printf(" Correction %.2f\n", correction);
	//
	printf("Wheel_1: %f  Wheel_2: %f  Wheel_3: %f Wheel_4: %f \n", w1,
			w2, w3, w4);
	__HAL_TIM_SET_COMPARE(&htimer4, TIM_CHANNEL_1,
			(int )((w1 > 0) ? w1 : (w1 * -1))); // these make the wheels spin
//	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, 0); //these make them spin in CW/CCW dir

	__HAL_TIM_SET_COMPARE(&htimer4, TIM_CHANNEL_2,
			(int )((w2 > 0) ? w2 : (w2 * -1)));
//	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, 0);

	__HAL_TIM_SET_COMPARE(&htimer4, TIM_CHANNEL_3,
			(int )((w3 > 0) ? w3 : (w3 * -1)));
//	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, 0);

	__HAL_TIM_SET_COMPARE(&htimer4, TIM_CHANNEL_4,
			(int )((w4 > 0) ? w4 : (w4 * -1)));
//	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, 0);

}

int map(int x, int in_min, int in_max, int out_min, int out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void SystemClock_Config() {
}

void timer4_init() {
	TIM_OC_InitTypeDef tim4PWM_Config;
	htimer4.Instance = TIM4;
	htimer4.Init.Period = 255 - 1;
	htimer4.Init.Prescaler = 4999;
	if (HAL_TIM_PWM_Init(&htimer4) != HAL_OK) {
		Error_Handler();
	}

	memset(&tim4PWM_Config, 0, sizeof(tim4PWM_Config));
	tim4PWM_Config.OCMode = TIM_OCMODE_PWM1;
	tim4PWM_Config.OCNPolarity = TIM_OCPOLARITY_HIGH;

	tim4PWM_Config.Pulse = (htimer4.Init.Period * 25) / 100;
	if (HAL_TIM_PWM_ConfigChannel(&htimer4, &tim4PWM_Config, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	tim4PWM_Config.Pulse = (htimer4.Init.Period * 40) / 100;
	if (HAL_TIM_PWM_ConfigChannel(&htimer4, &tim4PWM_Config, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}

	tim4PWM_Config.Pulse = (htimer4.Init.Period * 75) / 100;
	if (HAL_TIM_PWM_ConfigChannel(&htimer4, &tim4PWM_Config, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}

	tim4PWM_Config.Pulse = (htimer4.Init.Period * 90) / 100;
	if (HAL_TIM_PWM_ConfigChannel(&htimer4, &tim4PWM_Config, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
}

void gpio_timer_Init() {
	GPIO_InitTypeDef timer4_gpio;
	timer4_gpio.Mode = GPIO_MODE_AF_PP;
	timer4_gpio.Alternate = GPIO_AF2_TIM4;
	timer4_gpio.Pull = GPIO_NOPULL;
	timer4_gpio.Speed = GPIO_SPEED_FAST;
	timer4_gpio.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;

	HAL_GPIO_Init(GPIOD, &timer4_gpio);

	//these are the direction pins for the motor drivers
	GPIO_InitTypeDef dir_gpio;
	dir_gpio.Mode = GPIO_MODE_OUTPUT_PP;
//		dir_gpio.Alternate = GPIO_AF2_TIM4;
	dir_gpio.Pull = GPIO_NOPULL;
	dir_gpio.Speed = GPIO_SPEED_FAST;
	dir_gpio.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;

	HAL_GPIO_Init(GPIOE, &dir_gpio);

	printf("Timer and Direction GPIOs Are Running\n");
}

void Error_Handler() {
	while (1)
		;
}

void System_Clock_Config(uint8_t CLOCK_FREQ) {
	RCC_OscInitTypeDef osc_init;
	RCC_ClkInitTypeDef clk_init;
	uint32_t FLatency = 0;

	osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	osc_init.HSEState = RCC_HSE_BYPASS;
	osc_init.PLL.PLLState = RCC_PLL_ON;
	osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	switch (CLOCK_FREQ) {
	case SYS_CLK_FREQ_50: {
		osc_init.PLL.PLLM = 8;
		osc_init.PLL.PLLN = 100;
		osc_init.PLL.PLLP = 2;
		osc_init.PLL.PLLQ = 2;

		clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
		RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
		clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
		clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
		clk_init.APB1CLKDivider = RCC_HCLK_DIV2;
		clk_init.APB2CLKDivider = RCC_HCLK_DIV2;

		FLatency = FLASH_ACR_LATENCY_1WS;

		break;
	}
	case SYS_CLK_FREQ_84: {
		osc_init.PLL.PLLM = 8;
		osc_init.PLL.PLLN = 168;
		osc_init.PLL.PLLP = 2;
		osc_init.PLL.PLLQ = 2;
		clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
		RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
		clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
		clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
		clk_init.APB1CLKDivider = RCC_HCLK_DIV2;
		clk_init.APB2CLKDivider = RCC_HCLK_DIV2;

		FLatency = FLASH_ACR_LATENCY_2WS;
		break;
	}
	case SYS_CLK_FREQ_120: {
		osc_init.PLL.PLLM = 8;
		osc_init.PLL.PLLN = 240;
		osc_init.PLL.PLLP = 2;
		osc_init.PLL.PLLQ = 2;

		clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
		RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
		clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
		clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
		clk_init.APB1CLKDivider = RCC_HCLK_DIV4;
		clk_init.APB2CLKDivider = RCC_HCLK_DIV2;

		FLatency = FLASH_ACR_LATENCY_3WS;
		break;
	}
	case SYS_CLK_FREQ_168: {

		// Enable the Clock for the Power Controller
		__HAL_RCC_PWR_CLK_ENABLE();

		// Set Voltage Scale As 1
		__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

		osc_init.PLL.PLLM = 8;
		osc_init.PLL.PLLN = 336;
		osc_init.PLL.PLLP = 2;
		osc_init.PLL.PLLQ = 2;

		clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
		RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
		clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
		clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
		clk_init.APB1CLKDivider = RCC_HCLK_DIV4;
		clk_init.APB2CLKDivider = RCC_HCLK_DIV2;

		FLatency = FLASH_ACR_LATENCY_5WS;
		break;
	}
	default:
		return;
	}

	if (HAL_RCC_OscConfig(&osc_init) != HAL_OK) {
		printf(
				"Gold Eagle Actual to Bravo 6-> We have a problem in the PLL OSC init\n");
		Error_Handler();
	}
	printf(
			"Gold Eagle Actual to Bravo 6-> The PLL OSC is confirmed. We are GO for clock Init\n");
	if (HAL_RCC_ClockConfig(&clk_init, FLatency) != HAL_OK) {
		printf(
				"Gold Eagle Actual to Bravo 6-> We have a problem in the PLL CLOCK init\n");
		Error_Handler();
	}
	printf(
			"Gold Eagle Actual to Bravo 6-> The PLL ClOCK is confirmed. Standby for further tasking\n");
	printf("Bravo 6 to Gold Eagle Actual-> Acknowledged. Standing By\n");

	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
}

void I2C2_GPIOInits(void) {
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// scl
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
	GPIO_Init(&I2CPins);

	// sda
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_11;
	GPIO_Init(&I2CPins);
}

void I2C2_Inits(void) {
	I2C2Handle.pI2Cx = I2C2;
	I2C2Handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	I2C2Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR
	;
	I2C2Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C2Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C2Handle);
}
