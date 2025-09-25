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
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SECTOR_1 1
#define SECTOR_2 2
#define SECTOR_3 3
#define SECTOR_4 4
#define SECTOR_5 5
#define SECTOR_6 6
#define INVALID_SECTOR 0

// System Parameters
#define ADC_FREQUENCY 30000.0f					// ADC frequency
#define ADC_DT (1.0f / ADC_FREQUENCY)			// ADC period for current loop
#define V_BUS 24.0f      					    // DC bus voltage in Volts

// Encoder Parameters
#define ENCODER_PPR 4096  						// Encoder's Pulses Per Revolution
#define CONTROL_LOOP_DT 0.01f				    // Closed loop time step
#define ENCODER_TIMER_MAX_PERIOD 4294967295.0   // Maximum counter value for a 32-bit timer
#define CAPTURE_TIMER_CLK_FREQ 1000000.0f

// Current loop parameters
#define CALIBRATION_SAMPLES 2000
#define ADC_VREF 3.3f                   		// ADC reference voltage (V)
#define ADC_RESOLUTION 4096.0f          	 	// 12-bit ADC (0-4095)
#define SHUNT_RESISTOR 0.001f            		// Shunt resistor value in Ohms (1mΩ)
#define AD8417_GAIN 60.0f               		// AD8417BRMZ gain = 60 V/V

#define PWM_MAX_PULSE 1200.0f           		// (TIM1 Period - 1) = 1200 - 1
#define PWM_MIN_PULSE 0.0f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
const uint8_t hall_to_sector_map_CW[8] = {
    INVALID_SECTOR,
    SECTOR_6, // Hall 1
    SECTOR_4, // Hall 2
    SECTOR_5, // Hall 3
    SECTOR_2, // Hall 4
    SECTOR_1, // Hall 5
    SECTOR_3, // Hall 6
    INVALID_SECTOR
};

const uint8_t hall_to_sector_map_CCW[8] = {
    INVALID_SECTOR,
    SECTOR_3, // Hall 1
    SECTOR_1, // Hall 2
    SECTOR_2, // Hall 3
    SECTOR_5, // Hall 4
    SECTOR_4, // Hall 5
    SECTOR_6, // Hall 6
    INVALID_SECTOR
};

typedef enum {
    PHASE_A,
    PHASE_B,
    PHASE_C
} BldcPhase;

typedef enum {
	CW,
	CCW
} MotorDirection;
uint8_t sector = 0;
uint8_t motor_fault = 0;

// Encoder variables
volatile float angle_degrees = 0.0f;    // mechanical angle [0–360)
volatile float speed_rpm = 0.0f;        // filtered speed in RPM
volatile float latest_pulse_period_s = 0.0f;

volatile uint32_t ic_capture1 = 0;          // Stores the first capture timestamp
volatile uint32_t ic_capture2 = 0;          // Stores the second capture timestamp
volatile uint32_t capture_diff = 0;         // Stores the time difference between captures
volatile uint8_t is_first_capture = 1;      // Flag to handle the very first pulse
volatile uint8_t new_capture_event = 0;     // Flag set by ISR to signal new data

volatile uint32_t adc_cur_c, adc_cur_b, adc_cur_a;

volatile float current_a = 0.0f;
volatile float current_b = 0.0f;
volatile float current_c = 0.0f;

// Current control variables
volatile uint16_t current_a_offset_adc = 0;
volatile uint16_t current_b_offset_adc = 0;
volatile uint16_t current_c_offset_adc = 0;

volatile float current_setpoint = 0.0f;
volatile float current_magnitude = 0.0f;  // Actual current magnitude (A)
volatile float current_error = 0.0f;
volatile float current_integral = 0.0f;
volatile float pi_output = 0.0f;

float CURRENT_KP = 9.8f;
float CURRENT_KI = 0.144f;

// Velocity control variables
volatile float target_speed = 0.0f;
volatile float velocity_error = 0.0f;
volatile float previous_velocity_error = 0.0f;
volatile float velocity_integral = 0.0f;

//float VELOCITY_KP = 0.08f;
//float VELOCITY_KD = 0.0045f;
//float VELOCITY_KI = 0.06f;
float VELOCITY_KP = 0.1f;
float VELOCITY_KD = 0.002f;
float VELOCITY_KI = 0.2f;


// Position control variables
volatile float target_angle = 0.0f;
volatile float angle_error = 0.0f;
volatile float previous_angle_error = 0.0f;
volatile float angle_integral = 0.0f;

float POSITION_KP = 0.092f;
float POSITION_KD = 0.003f;
float POSITION_KI = 0.003f;

void convert_adc_to_current(void);
void getActivePhaseCurrent(void);
uint16_t current_pi_controller(void);
float velocity_pid_controller(void);
float position_pid_controller(void);
void calculate_speed_from_capture(void);
void update_angle_from_encoder(void);
void calibrate_current_sensors(void);

volatile uint16_t pwm_duty = 0.0f;
MotorDirection direction = CW;
uint8_t motor_stop = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t current_pi_controller(void)
{
    // Error
    current_error = fabs(current_setpoint) - current_magnitude;

    // Integral
    current_integral += current_error * ADC_DT;

    // Integral anti-windup
    if (current_integral > 50) {
    	current_integral = 50;
    } else if (current_integral < -50) {
    	current_integral = -50;
    }

    // Calculate PI output
    pi_output = current_error * CURRENT_KP + current_integral * CURRENT_KI;

    // Convert target voltage to PWM duty cycle
    float pid = (pi_output / V_BUS) * PWM_MAX_PULSE;

    // Saturate
    if (pid > 600) {
    	pid = 600;
    } else if (pid < 0.0f) {
    	pid = 0.0f;
    }

    return (uint16_t) pid;
}

float velocity_pid_controller(void)
{
	// Error
	velocity_error = target_speed - speed_rpm;

	// P term
	float p_term = VELOCITY_KP * velocity_error;

	// Integral
	velocity_integral += velocity_error * CONTROL_LOOP_DT;

	// Integral anti-windup
	if (velocity_integral > 100.0f) velocity_integral  = 100.0f;
	if (velocity_integral < -100.0f) velocity_integral = -100.0f;

	// I term
	float i_term = VELOCITY_KI * velocity_integral;

	float derivative = (velocity_error - previous_velocity_error) / CONTROL_LOOP_DT;

	// D term
	float d_term = VELOCITY_KD * derivative;
	previous_velocity_error = velocity_error;

	float pid_output = p_term + i_term + d_term;

	return pid_output;
}

float position_pid_controller(void) {

    float angle_error = target_angle - angle_degrees;
//    if (angle_error > 180.0f) {
//        angle_error -= 360.0f;
//    } else if (angle_error < -180.0f) {
//        angle_error += 360.0f;
//    }

    float p_term = POSITION_KP * angle_error;
    angle_integral += angle_error * CONTROL_LOOP_DT;
    float i_term = POSITION_KI * angle_integral;

    float derivative = (angle_error - previous_angle_error) / CONTROL_LOOP_DT;

    float d_term = POSITION_KD * derivative;
    previous_angle_error = angle_error;
    float pid_output = p_term + i_term + d_term;

    return pid_output;
}

void convert_adc_to_current(void)
{
    // Calculate the signed difference from the calibrated zero-current ADC value
    int32_t adc_delta_a = (int32_t)adc_cur_a - (int32_t)current_a_offset_adc;
    int32_t adc_delta_b = (int32_t)adc_cur_b - (int32_t)current_b_offset_adc;
    int32_t adc_delta_c = (int32_t)adc_cur_c - (int32_t)current_c_offset_adc;

    // Convert the ADC count difference directly to current
    // I = (ADC_delta / ADC_Resolution) * V_ref / (Gain * R_shunt)
    float volts_per_adc_count = ADC_VREF / ADC_RESOLUTION;
    float conversion_factor = volts_per_adc_count / (AD8417_GAIN * SHUNT_RESISTOR);

    current_a = (float)adc_delta_a * conversion_factor;
    current_b = (float)adc_delta_b * conversion_factor;
    current_c = (float)adc_delta_c * conversion_factor;
}

void calibrate_current_sensors(void)
{
    uint32_t offset_a_sum = 0;
    uint32_t offset_b_sum = 0;
    uint32_t offset_c_sum = 0;

    HAL_GPIO_WritePin(DRIVE_EN3_GPIO_Port, DRIVE_EN3_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);

    for (int i = 0; i < CALIBRATION_SAMPLES; i++)
    {
        // Read Phase C (ADC1_IN3)
        ADC_ChannelConfTypeDef sConfig = {0};
        sConfig.Channel = ADC_CHANNEL_3;
        sConfig.Rank = 1;
        HAL_ADC_ConfigChannel(&hadc1, &sConfig);
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 10);
        offset_c_sum += HAL_ADC_GetValue(&hadc1);

        // Read Phase B (ADC1_IN14)
        sConfig.Channel = ADC_CHANNEL_14;
        HAL_ADC_ConfigChannel(&hadc1, &sConfig);
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 10);
        offset_b_sum += HAL_ADC_GetValue(&hadc1);

        // Read Phase A (ADC1_IN15)
        sConfig.Channel = ADC_CHANNEL_15;
        HAL_ADC_ConfigChannel(&hadc1, &sConfig);
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 10);
        offset_a_sum += HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);

    // Calculate the average offset
    current_a_offset_adc = offset_a_sum / CALIBRATION_SAMPLES;
    current_b_offset_adc = offset_b_sum / CALIBRATION_SAMPLES;
    current_c_offset_adc = offset_c_sum / CALIBRATION_SAMPLES;
}


void calculate_speed_from_capture(void)
{
    static uint32_t last_valid_period_tick = 0;
    float pulse_period_s = latest_pulse_period_s; // Read the shared variable once

    // If the period is changing, a new pulse has arrived. Reset the timeout.
    if (pulse_period_s > 0.0f) {
        last_valid_period_tick = HAL_GetTick();
        latest_pulse_period_s = 0.0f; // Consume the value
    }

    // If no new pulse for 100ms, assume stopped
    if (HAL_GetTick() - last_valid_period_tick > 100) {
        speed_rpm = 0.0f;
        is_first_capture = 1;
        return; // Exit early
    }

    // Only calculate speed if we have a valid, new period
    if (pulse_period_s > 0.0f)
    {
        float speed_rps = (1.0f / (ENCODER_PPR * 4.0f)) / pulse_period_s;
        int8_t direction = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2) ? -1 : 1;
        float new_speed_rpm = speed_rps * 60.0f * (float)direction;

        const float alpha = 0.90f;
        speed_rpm = alpha * speed_rpm + (1.0f - alpha) * new_speed_rpm;
    }
}

void update_angle_from_encoder(void)
{
    static uint32_t last_encoder_count = 0;
    uint32_t current_encoder_count = __HAL_TIM_GET_COUNTER(&htim2);

    int32_t diff = (int32_t)(current_encoder_count - last_encoder_count);

    // Update angle in degrees
    angle_degrees += 360.0f * (float)diff / (ENCODER_PPR * 4.0f);

    // Wrap angle
//    angle_degrees = fmodf(angle_degrees, 360.0f);
//    if (angle_degrees < 0.0f) {
//        angle_degrees += 360.0f;
//    }

    last_encoder_count = current_encoder_count;
}


uint8_t read_hall_sensors(void)
{
	uint8_t hall_state = 0;
	hall_state = HAL_GPIO_ReadPin(HALL_A_GPIO_Port, HALL_A_Pin);	// Read Hall A (MSB)
	hall_state <<= 1;
	hall_state |= HAL_GPIO_ReadPin(HALL_B_GPIO_Port, HALL_B_Pin);	// Read Hall B
	hall_state <<= 1;
	hall_state |= HAL_GPIO_ReadPin(HALL_C_GPIO_Port, HALL_C_Pin);	// Read Hall C (LSB)

	return hall_state;	// A -> B -> C
}

void stopMotor(void)
{
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);

    HAL_GPIO_WritePin(INL1_EN1_GPIO_Port, INL1_EN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(INL2_EN2_GPIO_Port, INL2_EN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(INL3_EN3_GPIO_Port, INL3_EN3_Pin, GPIO_PIN_RESET);

    current_integral = 0;
    velocity_integral = 0;
    angle_integral = 0;

    velocity_error = 0.0f;
    previous_velocity_error = 0.0f;
    velocity_integral = 0.0f;

}

/**
 * @brief Turns the High-Side ON (with PWM) and the Low-Side OFF.
 * @param phase: The motor phase (PHASE_A, PHASE_B, or PHASE_C).
 * @param pwm_duty: The PWM duty cycle value.
 */
void SetPhase_PWM(BldcPhase phase, uint16_t pwm_duty)
{
    switch(phase)
    {
        case PHASE_A:
        	// High-side ON
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_duty);
            // Output enable
            HAL_GPIO_WritePin(INL1_EN1_GPIO_Port, INL1_EN1_Pin, GPIO_PIN_SET);
            break;
        case PHASE_B:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_duty);
            HAL_GPIO_WritePin(INL2_EN2_GPIO_Port, INL2_EN2_Pin, GPIO_PIN_SET);
            break;
        case PHASE_C:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm_duty);
            HAL_GPIO_WritePin(INL3_EN3_GPIO_Port, INL3_EN3_Pin, GPIO_PIN_SET);
            break;
    }
}

/**
 * @brief Turns the High-Side OFF and the Low-Side ON.
 * @param phase: The motor phase.
 */
void SetPhase_GND(BldcPhase phase)
{
    switch(phase)
    {
        case PHASE_A:
        	// High-side OFF (0% duty cycle)
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
            // Output enable
            HAL_GPIO_WritePin(INL1_EN1_GPIO_Port, INL1_EN1_Pin, GPIO_PIN_SET);
            break;
        case PHASE_B:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
            HAL_GPIO_WritePin(INL2_EN2_GPIO_Port, INL2_EN2_Pin, GPIO_PIN_SET);
            break;
        case PHASE_C:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
            HAL_GPIO_WritePin(INL3_EN3_GPIO_Port, INL3_EN3_Pin, GPIO_PIN_SET);
            break;
    }
}

/**
 * @brief Disconnects a phase (High-Z).
 * @note Turns BOTH the High-Side and Low-Side OFF.
 * @param phase: The motor phase (PHASE_A, PHASE_B, or PHASE_C).
 */
void SetPhase_FLOAT(BldcPhase phase)
{
    switch(phase)
    {
        case PHASE_A:
            // Output disable
            HAL_GPIO_WritePin(INL1_EN1_GPIO_Port, INL1_EN1_Pin, GPIO_PIN_RESET);
            break;
        case PHASE_B:
            HAL_GPIO_WritePin(INL2_EN2_GPIO_Port, INL2_EN2_Pin, GPIO_PIN_RESET);
            break;
        case PHASE_C:
            HAL_GPIO_WritePin(INL3_EN3_GPIO_Port, INL3_EN3_Pin, GPIO_PIN_RESET);
            break;
    }
}

void sixStepCommutation(uint8_t sector, uint16_t pwm_duty_cycle) {
	switch (sector) {
		case SECTOR_1: // Hall state: 5
			SetPhase_PWM(PHASE_B, pwm_duty_cycle);
			SetPhase_GND(PHASE_C);
			SetPhase_FLOAT(PHASE_A);
			break;

		case SECTOR_2: // Hall state: 4
			SetPhase_PWM(PHASE_B, pwm_duty_cycle);
			SetPhase_GND(PHASE_A);
			SetPhase_FLOAT(PHASE_C);
			break;

		case SECTOR_3: // Hall state: 6
			SetPhase_PWM(PHASE_C, pwm_duty_cycle);
			SetPhase_GND(PHASE_A);
			SetPhase_FLOAT(PHASE_B);
			break;

		case SECTOR_4: // Hall state: 2
			SetPhase_PWM(PHASE_C, pwm_duty_cycle);
			SetPhase_GND(PHASE_B);
			SetPhase_FLOAT(PHASE_A);
			break;

		case SECTOR_5: // Hall state: 3
			SetPhase_PWM(PHASE_A, pwm_duty_cycle);
			SetPhase_GND(PHASE_B);
			SetPhase_FLOAT(PHASE_C);
			break;

		case SECTOR_6: // Hall state: 1
			SetPhase_PWM(PHASE_A, pwm_duty_cycle);
			SetPhase_GND(PHASE_C);
			SetPhase_FLOAT(PHASE_B);
			break;

		default:
			SetPhase_FLOAT(PHASE_A);
			SetPhase_FLOAT(PHASE_B);
			SetPhase_FLOAT(PHASE_C);
			break;
	}
}

void getActivePhaseCurrent(void)
{
	switch (sector) {
			case SECTOR_2:
			case SECTOR_3:
				current_magnitude = fabs(current_a);
				break;

			case SECTOR_4:
			case SECTOR_5:
				current_magnitude = fabs(current_b);
				break;

			case SECTOR_1:
			case SECTOR_6:
				current_magnitude = fabs(current_c);
				break;

			default:
				current_magnitude = 0.0f;
				break;
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  calibrate_current_sensors();

  // Enable main driver.
  HAL_GPIO_WritePin(DRIVE_EN3_GPIO_Port, DRIVE_EN3_Pin, GPIO_PIN_SET);

  // Read initial hall state and set initial commutation
  uint8_t initial_hall_state = read_hall_sensors();
  sector = hall_to_sector_map_CW[initial_hall_state & 0x07];
  sixStepCommutation(sector, 0);

  // Start timer1
  HAL_TIM_Base_Start(&htim1);

  // Start three phase PWM
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  // Start ADC interrupt timer
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  // ADC injected mode with interrupt
  HAL_ADCEx_InjectedStart_IT(&hadc1);

  // Start Timer2 as Encoder mode
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

  // Start Timer3 for input capture timer
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);

  // Start Timer4 for velocity loop
  HAL_TIM_Base_Start_IT(&htim4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (HAL_GPIO_ReadPin(FAULT_GPIO_Port, FAULT_Pin) == GPIO_PIN_RESET) {
		  motor_fault = 1;
		  stopMotor();
	  }

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
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_3;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 3;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_28CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_CC4;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_14;
  sConfigInjected.InjectedRank = 2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_15;
  sConfigInjected.InjectedRank = 3;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 1200-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 600;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7200-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DRIVE_EN3_GPIO_Port, DRIVE_EN3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, INL1_EN1_Pin|INL2_EN2_Pin|INL3_EN3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DRIVE_EN3_Pin */
  GPIO_InitStruct.Pin = DRIVE_EN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DRIVE_EN3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FAULT_Pin */
  GPIO_InitStruct.Pin = FAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FAULT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INL1_EN1_Pin INL2_EN2_Pin INL3_EN3_Pin */
  GPIO_InitStruct.Pin = INL1_EN1_Pin|INL2_EN2_Pin|INL3_EN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : HALL_A_Pin HALL_B_Pin HALL_C_Pin */
  GPIO_InitStruct.Pin = HALL_A_Pin|HALL_B_Pin|HALL_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

float test_current = 0.0f;

// ADC call back
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if (motor_fault || motor_stop) {
		stopMotor();
		return;
	}

	uint8_t hall_state = read_hall_sensors();

	if (direction == CW)
	    sector = hall_to_sector_map_CW[hall_state & 0x07];
	else
	    sector = hall_to_sector_map_CCW[hall_state & 0x07];

	adc_cur_c = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1); // CH3 (PA3)
	adc_cur_b = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2); // CH14 (PC4)
	adc_cur_a = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3); // CH15 (PC5)
	convert_adc_to_current();

	getActivePhaseCurrent();

	pwm_duty = current_pi_controller();

	sixStepCommutation(sector, pwm_duty);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM4)
  {
    calculate_speed_from_capture();
    update_angle_from_encoder();

//    target_speed = position_pid_controller();

    float new_current_setpoint = velocity_pid_controller();

    if (new_current_setpoint > 0) {
        direction = CW;
    } else if (new_current_setpoint < 0) {
        direction = CCW;
    }

    __disable_irq();
    current_setpoint = new_current_setpoint;
    __enable_irq();
  }
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    ic_capture2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    capture_diff = ic_capture2 - ic_capture1;
    ic_capture1 = ic_capture2;

    if (!is_first_capture)
    {
      latest_pulse_period_s = (float)capture_diff / CAPTURE_TIMER_CLK_FREQ;
    }
    is_first_capture = 0;
  }
}


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
#ifdef USE_FULL_ASSERT
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
