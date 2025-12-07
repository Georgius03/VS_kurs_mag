/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

typedef struct {
    float K;        // Gain of the integrator
    float T;        // Time constant
    float dt;       // Sampling time step

    float k;        // Precomputed coefficient: K * dt / T

    float y;        // Current output value
    float y_prev;   // Previous output value (for discrete update)

} integrator;

typedef struct {
    float K;        // Gain
    float T;        // Time constant
    float dt;       // Sampling time step

    float k1;       // Precomputed coefficient: T / (T + dt)
    float k2;       // Precomputed coefficient: K * dt / (T + dt)

    float y;        // Current output value
    float y_prev;   // Previous output value (for discrete update)

} first_order;

typedef struct {
    float dt;       // Sampling time step
    float T_mu;     // Smallest time constant

    float Kp;       // Proportional gain
    float Ki;       // Integral gain
    float k;        // Precomputed coefficient: Kp * Ki * dt

    float err;      // Current error (input to the controller)

    float P_out;    // Proportional part output
    float I_out;    // Integral part output

    float I_prev;   // Previous integral value (for discrete update)

    float y;        // Total controller output: y = P_out + I_out

} pi_controller;

typedef struct {
    float dt;       // Sampling time step
    float T_mu;     // Smallest time constant

    float Kp;       // Proportional gain
    float Ki;       // Integral gain
    float k;        // Precomputed coefficient: Kp * Ki * dt
    float k_comp;   // Anti-windup correction gain
    float k_sat;    // Anti-windup correction gain calc

    float err;      // Current error (input to the controller)
    float sat_diff;      // Current error (input to the controller)

    float P_out;    // Proportional part output
    float I_out;    // Integral part output
    float C_out;    // Anti-windup correction output

    float I_prev;   // Previous integral value
    float C_prev;   // Previous correction value

    float y;        // Total controller output: y = P_out + I_out + C_out

} pi_controller_aw;



// Function prototypes

void init_controller_params(void);
void init_input_params(void);

float pi_reg_calc(float u, pi_controller* pi);
float pi_reg_calc_aw(float err, float sat_diff, pi_controller_aw* pi);

float plant_calc_func(float u, first_order* sys);
float plant_calc_int(float u, integrator* sys);
float saturation_calc(float u, float max_val, float min_val);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
