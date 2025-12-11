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
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/float32.h>

#include <stdbool.h>
#include <string.h>

#include <pid_stm32.h>
#include "platooning.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* Definitions for uROSTask */
osThreadId_t uROSTaskHandle;
const osThreadAttr_t uROSTask_attributes = { .name = "uROSTask", .stack_size =
		3000 * 4, .priority = (osPriority_t) osPriorityNormal, };
/* Definitions for ctrlTask */
osThreadId_t ctrlTaskHandle;
const osThreadAttr_t ctrlTask_attributes = { .name = "ctrlTask", .stack_size =
		128 * 4, .priority = (osPriority_t) osPriorityBelowNormal7, };
/* USER CODE BEGIN PV */

// Platoon participation config
char *participant_name = "/veh_ego";
uint8_t participant_role = 1; // Follower

rcl_publisher_t throttle_pub;
rcl_publisher_t brake_pub;

rcl_subscription_t r_sub; // Speed setpoint
rcl_subscription_t pv_sub; // Current speed reading
rcl_subscription_t d_sub; // Distance to the car ahead

// Message typedef to store readings in subscription callbacks
volatile std_msgs__msg__Float32 r_msg;
volatile std_msgs__msg__Float32 pv_msg;
volatile std_msgs__msg__Float32 d_msg;

// Message typedef to store data before publishing
volatile std_msgs__msg__Float32 throttle_msg;
volatile std_msgs__msg__Float32 brake_msg;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

// Speed PID controller
pid_controller_t speed_pid;

// Platoon member
PLATOON_member_t platoon_member;

uint32_t rx_count = 0;
// TODO Remove button use, not useful in this project
char BspButtonState = 0;

volatile uint8_t pv_sample_rdy = 0;
volatile uint8_t d_sample_rdy = 0;

volatile uint8_t control_task_rdy = 0;
volatile uint8_t uros_task_rdy = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartUROSTask(void *argument);
void StartCrtlTask(void *argument);

/* USER CODE BEGIN PFP */
bool cubemx_transport_open(struct uxrCustomTransport *transport);
bool cubemx_transport_close(struct uxrCustomTransport *transport);
size_t cubemx_transport_write(struct uxrCustomTransport *transport,
		const uint8_t *buf, size_t len, uint8_t *err);
size_t cubemx_transport_read(struct uxrCustomTransport *transport, uint8_t *buf,
		size_t len, int timeout, uint8_t *err);

void* microros_allocate(size_t size, void *state);
void microros_deallocate(void *pointer, void *state);
void* microros_reallocate(void *pointer, size_t size, void *state);
void* microros_zero_allocate(size_t number_of_elements, size_t size_of_element,
		void *state);

float uros_get_speed(void);
float uros_get_dist(void);
float uros_get_controller_action(float speed, float setpoint);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// micro-ROS topic callback functions
// ======================================================================
void r_sub_cb(const void *msgin) {
	const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32*) msgin;
}

void pv_sub_cb(const void *msgin) {
	const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32*) msgin;
	pv_sample_rdy++;
}

void d_sub_cb(const void *msgin) {
	const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32*) msgin;
	d_sample_rdy++;
}

// =========================================================================
// END micro-ROS topic callbacks

// Platooning function implementations
// ========================================================================

float uros_get_speed(void) {
	return pv_msg.data;
}

float uros_get_dist() {
	return d_msg.data;
}

float uros_get_controller_action(float speed, float setpoint) {
	return pid_run(&speed_pid, setpoint, speed);
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of uROSTask */
	uROSTaskHandle = osThreadNew(StartUROSTask, NULL, &uROSTask_attributes);

	/* creation of ctrlTask */
	ctrlTaskHandle = osThreadNew(StartCrtlTask, NULL, &ctrlTask_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Supply configuration update enable
	 */
	HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48
			| RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1
			| RCC_CLOCKTYPE_D1PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LD1_Pin | LD3_Pin | LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);

	/*Configure GPIO pin : PC13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
	GPIO_InitStruct.Pin = LD1_Pin | LD3_Pin | LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : STLINK_RX_Pin STLINK_TX_Pin */
	GPIO_InitStruct.Pin = STLINK_RX_Pin | STLINK_TX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pin : PE1 */
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_13) {
		BspButtonState = 1;
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartUROSTask */
/**
 * @brief  Function implementing the uROSTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartUROSTask */
void StartUROSTask(void *argument) {
	/* init code for USB_DEVICE */
	MX_USB_DEVICE_Init();
	/* USER CODE BEGIN 5 */
	// micro-ROS configuration
	/*
	 * Turn off LEDs. Not needed but whatever
	 */
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

	rmw_uros_set_custom_transport(
	true, (void*) NULL, cubemx_transport_open, cubemx_transport_close,
			cubemx_transport_write, cubemx_transport_read);

	// Wait for micro-ROS Agent to start and be available
	while (rmw_uros_ping_agent(1000, 1) != RCL_RET_OK) {
		vTaskDelay(pdMS_TO_TICKS(100));
	}

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);

	// Micro-ROS Setup
	// ========================================================================================
	rcl_allocator_t freeRTOS_allocator =
			rcutils_get_zero_initialized_allocator();
	freeRTOS_allocator.allocate = microros_allocate;
	freeRTOS_allocator.deallocate = microros_deallocate;
	freeRTOS_allocator.reallocate = microros_reallocate;
	freeRTOS_allocator.zero_allocate = microros_zero_allocate;

	if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
		printf("Error on default allocators (line %d)\n", __LINE__);
	}

	// micro-ROS app

	allocator = rcl_get_default_allocator();

	//create init_options
#define CHECK(fn) do { rcl_ret_t rc = (fn); if (rc != RCL_RET_OK) { \
    printf("RCL ERROR %d at %s:%d -> %s\n", (int)rc, __FILE__, __LINE__, rcl_get_error_string().str); \
    rcl_reset_error(); } } while(0)

	CHECK(rclc_support_init(&support, 0, NULL, &allocator));
	CHECK(
			rclc_node_init_default(&node, strcat(participant_name, "_node"), "",
					&support));

	CHECK(
			rclc_publisher_init_best_effort( &throttle_msg, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), strcat(participant_name, "/command/throttle") ));

	CHECK(
			rclc_publisher_init_best_effort( &brake_msg, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), strcat(participant_name, "/command/brake") ));

	CHECK(
			rclc_subscription_init_best_effort( &pv_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), strcat(participant_name, "/state/speed") ));

	CHECK(
			rclc_subscription_init_default( &r_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), strcat(participant_name, "/state/setpoint") ));

	CHECK(
			rclc_subscription_init_default( &d_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), strcat(participant_name, "/state/dist_to_veh") ));

	executor = rclc_executor_get_zero_initialized_executor();
	CHECK(rclc_executor_init(&executor, &support.context, 7, &allocator));

	CHECK(
			rclc_executor_add_subscription(&executor, &pv_sub, &pv_msg,
					&pv_sub_cb, ON_NEW_DATA));
	CHECK(
			rclc_executor_add_subscription(&executor, &r_sub, &r_msg, &r_sub_cb,
					ON_NEW_DATA));
	CHECK(
			rclc_executor_add_subscription(&executor, &d_sub, &d_msg, &d_sub_cb,
					ON_NEW_DATA));

	// Ensure XRCE session synchronized after creating entities
	rmw_uros_sync_session(1000);

	// Initial spin to confirm creation
	rclc_executor_spin_some(&executor, 10);

	// ======================================================================================

	// Initialize messages explicitly
	pv_msg.data = 0.0f;
	r_msg.data = 0.0f;
	throttle_msg.data = 0.0f;
	brake_msg.data = 0.0f;

	// Start immediately at 1 ms for low latency. Must be faster than PV reading freq, if not will hang and
	// behaviour will not reflect tuning
	TickType_t xFreq = pdMS_TO_TICKS(1);

	uros_task_rdy = 1;

	// Wait until the controller and platoon setup is finished
	while (control_task_rdy != 1) {
		vTaskDelay(xFreq);
	}

	for (;;) {
		//vTaskDelayUntil(&xLastWakeTime, xFreq); // For real-time physical plant
		vTaskDelay(xFreq);
		rclc_executor_spin_some(&executor, 5);

		if (pv_sample_rdy) {
			pv_sample_rdy = 0;

			CHECK(rcl_publish(&throttle_pub, &throttle_msg, NULL));

			CHECK(rcl_publish(&brake_pub, &brake_msg, NULL));

		}
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartCrtlTask */
/**
 * @brief Function implementing the ctrlTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartCrtlTask */
void StartCrtlTask(void *argument) {
	/* USER CODE BEGIN StartCrtlTask */
	// Setup Speed PID controller
	// ===================================================================
	pid_init(&speed_pid,
	// Values from PSO optimization algorithm
	// Hardcoding values is a lot easier than passing with micro-ROS,
	// and frees up the ROS pub/sub ammount.
			2.59397071e-01f,// 	Kp
			1.27381733e-01, 	//	Ki
			6.21160744e-03, 	//	Kd
			5.0f,				//	N
			0.01f				//	Ts
			);
	// Integral anti windup
	pid_set_anti_windup(&speed_pid, _PID_ANTI_WINDUP_BACK_CALCULATION);
	pid_set_kb_aw(&speed_pid, 1.0f);
	// Controller clamping, positive should result in throttle
	// negative in brake, though brake should be carefully
	// calibrated, as PID wants to floor it.
	pid_set_clampling(&speed_pid, 1.0f, -1.0f);
	// Avoid huge jumps when setpoint changes suddenly
	pid_set_derivative_on_meas(&speed_pid, _PID_DERIVATIVE_ON_MEASURE_ON);
	// Set most appropriate discretization methods for each component
	pid_set_derivative_discretization_method(&speed_pid, _PID_DISCRETE_TUSTIN);
	pid_set_integral_discretization_method(&speed_pid,
			_PID_DISCRETE_BACKWARD_EULER);

	// ====================================================================

	// Setup Platoon member handler
	// ====================================================================
	platoon_member.name = participant_name;
	platoon_member.role = (PLATOON_member_type_TypeDef)participant_role;

	platoon_member.k_dist = 0.2f;
	platoon_member.min_spacing = 5.0f;
	platoon_member.time_headway = 0.5f;

	platoon_member.get_distance = uros_get_dist;
	platoon_member.get_speed = uros_get_speed;
	platoon_member.get_controller_action = uros_get_controller_action;

	// ====================================================================
	TickType_t xFreq = pdMS_TO_TICKS(1);

	control_task_rdy = 1;

	// Wait until uROS setup is done
	while(uros_task_rdy != 1) {
		vTaskDelay(xFreq);
	}

	for (;;) {

		PLATOON_command_t cmd = compute_control(&platoon_member, r_msg.data);

		throttle_msg.data = cmd.throttle_cmd;
		brake_msg.data = cmd.brake_cmd;

		vTaskDelay(xFreq);
	}
	/* USER CODE END StartCrtlTask */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM1) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
