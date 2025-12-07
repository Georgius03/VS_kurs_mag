/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define X1_max 2
#define X1_min -2
#define Y2_max 450
#define Y2_min -450
#define Y4_max 200
#define Y4_min -200

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim8;

/* USER CODE BEGIN PV */
// Global variables
uint16_t cnt1 = 0, cnt2 = 0;

float angle_setpoint = 0.0f;
float angle_setpoint_saturated = 0.0f; // for debug
float err_angle = 0.0f;

float omega_setpoint = 0.0f;
float err_omega = 0.0f;

float X1 = 0.0f;
float X1_sat = 0.0f;
float X1_diff = 0.0f;  // For anti-windup compensation

float Y1 = 0.0f;
float X2 = 0.0f;

float Y2 = 0.0f;
float Y2_sat = 0.0f;

float Z = 0.0f;
float X4 = 0.0f;

float Y4 = 0.0f;
float Y4_sat = 0.0f;

float angle_setpoint_desired = 100.0f;

// System components
first_order sp_filter_fun = {0};
integrator sp_filter_int = {0};
first_order func_1 = {0};
first_order func_2 = {0};
integrator func_3 = {0};
pi_controller pi_angle = {0};
pi_controller_aw pi_omega = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void init_input_params(void) {
	float dt_plant = 0.00001f;  // 1/100000 = 0.00001s

	// ===== Initialize first-order input filter =====
	sp_filter_fun.dt = dt_plant;
	sp_filter_fun.K = 1.0f;      // K1
	sp_filter_fun.T = 0.1f;      // T1

	sp_filter_fun.k1 = (sp_filter_fun.K * sp_filter_fun.dt) / (sp_filter_fun.T + sp_filter_fun.dt);
	sp_filter_fun.k2 = sp_filter_fun.T / (sp_filter_fun.T + sp_filter_fun.dt);

	sp_filter_fun.y = 0.0f;
	sp_filter_fun.y_prev = 0.0f;

	// ===== Initialize integrator input =====
	sp_filter_int.dt = dt_plant;
	sp_filter_int.K = 3.0f;
	sp_filter_int.T = 1.0;      // T4

	sp_filter_int.k = (sp_filter_int.K * sp_filter_int.dt) / sp_filter_int.T;

	sp_filter_int.y = 0.0f;
	sp_filter_int.y_prev = 0.0f;
}

void init_controller_params(void) {
	float dt_plant = 0.00001f;  // 1/100000 = 0.00001s
	float dt_control = 0.00333f;  // 1/300 = 0.00333s
	float aw_comp_coef = 600.0f;



	// ===== Initialize first-order system W1 =====
	func_1.dt = dt_plant;
	func_1.K = 4.72f;      // K1
	func_1.T = 0.003f;     // T1

	func_1.k1 = (func_1.K * func_1.dt) / (func_1.T + func_1.dt);
	func_1.k2 = func_1.T / (func_1.T + func_1.dt);

	func_1.y = 0.0f;
	func_1.y_prev = 0.0f;



	// ===== Initialize first-order system W2 =====
	func_2.dt = dt_plant;
	func_2.K = 12.5f;      // K2
	func_2.T = 0.011f;     // T2

	func_2.k1 = (func_2.K * func_2.dt) / (func_2.T + func_2.dt);
	func_2.k2 = func_2.T / (func_2.T + func_2.dt);

	func_2.y = 0.0f;
	func_2.y_prev = 0.0f;



	// ===== Initialize integrator W4 =====
	func_3.dt = dt_plant;
	func_3.K = 1.0f;
	func_3.T = 0.188f;      // T4

	func_3.k = (func_3.K * func_3.dt) / func_3.T;

	func_3.y = 0.0f;
	func_3.y_prev = 0.0f;



	// ===== Initialize PI controller for SPEED loop (with anti-windup) =====
	pi_omega.dt = dt_control;

	pi_omega.T_mu = func_1.T;  // Use T1 from func_1
	pi_omega.Kp = func_2.T / (2.0f * func_1.K * func_2.K * pi_omega.T_mu);
	pi_omega.Ki = 1.0f / func_2.T;

	pi_omega.k = pi_omega.dt * pi_omega.Kp * pi_omega.Ki;

	pi_omega.k_comp = aw_comp_coef;
	pi_omega.k_sat = pi_omega.dt * pi_omega.k_comp ;

	pi_omega.P_out = 0.0f;
	pi_omega.I_out = 0.0f;
	pi_omega.C_out = 0.0f;
	pi_omega.I_prev = 0.0f;
	pi_omega.C_prev = 0.0f;
	pi_omega.err = 0.0f;
	pi_omega.sat_diff = 0.0f;
	pi_omega.y = 0.0f;



	// ===== Initialize PI controller for POSITION loop =====
	pi_angle.dt = dt_control;

	pi_angle.T_mu = 2.0f * pi_omega.T_mu;
	pi_angle.Ki = 1.0f / (4.0f * pi_angle.T_mu);
	pi_angle.Kp = func_3.T / (2.0f * pi_angle.T_mu);

	pi_angle.k = pi_angle.dt * pi_angle.Kp * pi_angle.Ki;

	pi_angle.P_out = 0.0f;
	pi_angle.I_out = 0.0f;
	pi_angle.I_prev = 0.0f;
	pi_angle.err = 0.0f;
	pi_angle.y = 0.0f;
}

float pi_reg_calc(float err, pi_controller* pi) {
	pi->err = err;

	pi->P_out = pi->err * pi->Kp;
	pi->I_out = pi->I_prev + pi->err * pi->k;

	pi->I_prev = pi->I_out;

	pi->y = pi->P_out + pi->I_out;

	return pi->y;
}

float pi_reg_calc_aw(float err, float sat_diff, pi_controller_aw* pi) {
	pi->err = err;
	pi->sat_diff = sat_diff;

	pi->P_out = pi->err * pi->Kp;

	pi->I_out = pi->I_prev + pi->err * pi->k;
	pi->C_out = pi->C_prev + pi->sat_diff * pi->k_sat;

	pi->I_prev = pi->I_out;
	pi->C_prev = pi->C_out;

	pi->y = pi->P_out + pi->I_out + pi->C_out;

	return pi->y;
}

float plant_calc_func(float u, first_order* sys) {
	sys->y = sys->k1 * u + sys->k2 * sys->y_prev;
	sys->y_prev = sys->y;

	return sys->y;
}

float plant_calc_int(float u, integrator* sys) {
	sys->y = sys->y_prev + u * sys->k;
	sys->y_prev = sys->y;

	return sys->y;
}

float saturation_calc(float u, float max_val, float min_val) {
	if (u > max_val) { u = max_val; }
	else if (u < min_val) { u = min_val; }

	return u;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)  {

	if (htim->Instance == TIM1) {
		// external loop
		err_angle = angle_setpoint - Y4_sat;
		omega_setpoint = pi_reg_calc(err_angle, &pi_angle);

		// internal loop
		err_omega = omega_setpoint - Y2_sat;
		X1_diff = X1_sat - X1;
		X1 = pi_reg_calc_aw(err_omega, X1_diff, &pi_omega);
		X1_sat = saturation_calc(X1, X1_max, X1_min);

		// counter for debug
		cnt1++;
	}

	if (htim->Instance == TIM8) {
		// input types
		// const
		angle_setpoint = 100.0f;
//		omega_setpoint = 100.0f;
		// filter
//		angle_setpoint = plant_calc_func(angle_setpoint_desired, &sp_filter_fun);
		// integrator
//		angle_setpoint = plant_calc_int(angle_setpoint_desired, &sp_filter_int);
//		angle_setpoint_saturated = saturation_calc(angle_setpoint, 200, -200);

		// internal loop
		Y1 = plant_calc_func(X1_sat, &func_1);
		X2 = Y1;
		Y2 = plant_calc_func(X2, &func_2);
		Y2_sat = saturation_calc(Y2, Y2_max, Y2_min);

		// external loop
		X4 = Y2_sat - Z;
		Y4 = plant_calc_int(X4, &func_3);
		Y4_sat = saturation_calc(Y4, Y4_max, Y4_min);


		// counter for debug
		cnt2++;
	}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */
	init_input_params();
	init_controller_params();
	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_TIM1_Init();
	MX_TIM8_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim8);
	HAL_TIM_Base_Start_IT(&htim1);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 150;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 5000-1;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 100-1;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_TIMING;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

}

/**
 * @brief TIM8 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM8_Init(void)
{

	/* USER CODE BEGIN TIM8_Init 0 */

	/* USER CODE END TIM8_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	/* USER CODE BEGIN TIM8_Init 1 */

	/* USER CODE END TIM8_Init 1 */
	htim8.Instance = TIM8;
	htim8.Init.Prescaler = 150-1;
	htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim8.Init.Period = 10-1;
	htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim8.Init.RepetitionCounter = 0;
	htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_OC_Init(&htim8) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_TIMING;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_OC_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM8_Init 2 */

	/* USER CODE END TIM8_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE();

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
