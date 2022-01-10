/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Empty PSoC6 Application
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* (c) 2019-2021, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

#define STAGE_DELAY (500u)
#define MOTOR_PWM_FREQUENCY (100u)
#define PWM_CLOCK_FREQUENCY (1000000u)
#define GPIO_INTERRUPT_PRIORITY (6u)


#define MOTOR_SPEED (70)
#define MOTOR_OFF   (0.01)

#define SERVO_PERIOD (5000u)

#define MOTOR_LEFT_FWD_PIN	CYBSP_D11
#define MOTOR_LEFT_REV_PIN	CYBSP_D10
#define MOTOR_RIGHT_FWD_PIN	CYBSP_D9
#define MOTOR_RIGHT_REV_PIN	CYBSP_D8

#define SERVO_PIN CYBSP_A4

#define ULTRASONIC_TRIG_PIN CYBSP_D13

#define OPTICAL_LEFT_PIN 	CYBSP_D7
#define OPTICAL_RIGHT_PIN 	CYBSP_D2

/* Function Prototypes */
void updateMotor(uint8_t direction, uint8_t speed);
void updateServo(int8 angle);
static void optical_left_isr(void *handler_arg, cyhal_gpio_event_t event);
static void optical_right_isr(void *handler_arg, cyhal_gpio_event_t event);

/* Global Variables */

/* Use one clock divider for all PWMs - 1MHz clock will give us 1us resolution */
cyhal_clock_t clock_pwm;

/* PWM object for motors */
cyhal_pwm_t motor_left_fwd;
cyhal_pwm_t motor_left_rev;
cyhal_pwm_t motor_right_fwd;
cyhal_pwm_t motor_right_rev;

/* PWM object for servo motor */
cyhal_pwm_t servo_pwm;

/* Ultrasonic distance count */
uint32_t ultrasonic_count = 0;

/* Wheel distances */
uint32_t leftDistance =  0;
uint32_t rightDistance = 0;

/* ISR callback data structures for HAL v3.X */
cyhal_gpio_callback_data_t optical_left_cb_data =
{
	.callback     = optical_left_isr,
	.callback_arg = NULL
};

cyhal_gpio_callback_data_t optical_right_cb_data =
{
	.callback     = optical_right_isr,
	.callback_arg = NULL
};


int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

	/* Initialize retarget-io to use the debug UART port */
	cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);

	/* Initialize button */
	cyhal_gpio_init(CYBSP_USER_BTN2, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);
	cyhal_gpio_enable_event(CYBSP_USER_BTN2, CYHAL_GPIO_IRQ_FALL, GPIO_INTERRUPT_PRIORITY, true);

	/* Initialize clock divider for PWMs */
	cyhal_clock_allocate(&clock_pwm, CYHAL_CLOCK_BLOCK_PERIPHERAL_16BIT);
	cyhal_clock_set_frequency(&clock_pwm, PWM_CLOCK_FREQUENCY, NULL);
	if (!cyhal_clock_is_enabled(&clock_pwm))
	{
		cyhal_clock_set_enabled(&clock_pwm, true, true);
	}

	/* Initialize Motor PWMs */
    cyhal_pwm_init(&motor_left_fwd,  MOTOR_LEFT_FWD_PIN,  &clock_pwm);
    cyhal_pwm_init(&motor_left_rev,  MOTOR_LEFT_REV_PIN,  &clock_pwm);
    cyhal_pwm_init(&motor_right_fwd, MOTOR_RIGHT_FWD_PIN, &clock_pwm);
    cyhal_pwm_init(&motor_right_rev, MOTOR_RIGHT_REV_PIN, &clock_pwm);
    cyhal_pwm_set_duty_cycle(&motor_left_fwd,  MOTOR_OFF, MOTOR_PWM_FREQUENCY); /* Off */
    cyhal_pwm_set_duty_cycle(&motor_left_rev,  MOTOR_OFF, MOTOR_PWM_FREQUENCY); /* Off */
    cyhal_pwm_set_duty_cycle(&motor_right_fwd, MOTOR_OFF, MOTOR_PWM_FREQUENCY); /* Off */
    cyhal_pwm_set_duty_cycle(&motor_right_rev, MOTOR_OFF, MOTOR_PWM_FREQUENCY); /* Off */
    cyhal_pwm_start(&motor_left_fwd);
    cyhal_pwm_start(&motor_left_rev);
    cyhal_pwm_start(&motor_right_fwd);
    cyhal_pwm_start(&motor_right_rev);

    /* Initialize PWM for servo motor and set angle to 0 degrees */
    cyhal_pwm_init(&servo_pwm, SERVO_PIN, &clock_pwm);
    updateServo(0);
    cyhal_pwm_start(&servo_pwm);

    /* Initialize pins and timer for ultrasonic sensor */
    /* Trigger output */
	cyhal_gpio_init(ULTRASONIC_TRIG_PIN, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 0);
	/* Echo return */
	Cy_TCPWM_Counter_Init(ULTRASONIC_COUNT_HW, ULTRASONIC_COUNT_NUM, &ULTRASONIC_COUNT_config);
	Cy_TCPWM_Counter_Enable(ULTRASONIC_COUNT_HW, ULTRASONIC_COUNT_NUM);

	/* Initialize pins for optical wheel sensors */
	cyhal_gpio_init(OPTICAL_LEFT_PIN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, 0);
	//GJL HAL v1.X cyhal_gpio_register_callback(OPTICAL_LEFT_PIN, optical_left_isr, NULL);
	cyhal_gpio_register_callback(OPTICAL_LEFT_PIN, &optical_left_cb_data);
	cyhal_gpio_enable_event(OPTICAL_LEFT_PIN, CYHAL_GPIO_IRQ_RISE, GPIO_INTERRUPT_PRIORITY, true);

	cyhal_gpio_init(OPTICAL_RIGHT_PIN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, 0);
	//GJL HAL v1.X cyhal_gpio_register_callback(OPTICAL_RIGHT_PIN, optical_right_isr, NULL);
	cyhal_gpio_register_callback(OPTICAL_RIGHT_PIN, &optical_right_cb_data);
	cyhal_gpio_enable_event(OPTICAL_RIGHT_PIN, CYHAL_GPIO_IRQ_RISE, GPIO_INTERRUPT_PRIORITY, true);

    __enable_irq();

	printf("Press button 2 to run tests\n");
	cyhal_syspm_sleep(); /* Wait until button interrupt occurs */
	cyhal_system_delay_ms(STAGE_DELAY);

    /* Forward */
	printf("Forward 1/2 speed\n");
	/* Run the motors motors */
	updateMotor(2, MOTOR_SPEED);
	cyhal_system_delay_ms(STAGE_DELAY);
	updateMotor(0, 0);
	cyhal_system_delay_ms(STAGE_DELAY);

    /* Reverse */
	printf("Reverse 1/2 speed\n");
	/* Run the motors motors */
	updateMotor(7, MOTOR_SPEED);
	cyhal_system_delay_ms(STAGE_DELAY);
	updateMotor(0, 0);
	cyhal_system_delay_ms(STAGE_DELAY);

    /* Rotate Left */
	printf("Rotate Left 1/2 speed\n");
	/* Run the motors motors */
	updateMotor(4, MOTOR_SPEED);
	cyhal_system_delay_ms(STAGE_DELAY);
	updateMotor(0, 0);
	cyhal_system_delay_ms(STAGE_DELAY);

    /* Rotate Right */
	printf("Rotate Right 1/2 speed\n");
	/* Run the motors motors */
	updateMotor(5, MOTOR_SPEED);
	cyhal_system_delay_ms(STAGE_DELAY);
	updateMotor(0, 0);
	cyhal_system_delay_ms(STAGE_DELAY);

	/* Turn "head" left and then right */
	updateServo(-90);
	cyhal_system_delay_ms(STAGE_DELAY);
	updateServo(90);
	cyhal_system_delay_ms(STAGE_DELAY);
	updateServo(0);

	/* Wait for button  press to continue */
	printf("Press button 2 to measure distance\n");
	cyhal_syspm_sleep(); /* Wait until button interrupt occurs */

	/* Reset distance counters */
	leftDistance =  0;
	rightDistance = 0;

    for (;;)
    {
        /* Sense distance using ultrasonic sensor */
    	/* Reset and Start counter that will measure return signal pulse width */
    	Cy_TCPWM_Counter_SetCounter(ULTRASONIC_COUNT_HW, ULTRASONIC_COUNT_NUM, 0);
    	Cy_TCPWM_TriggerStart_Single(ULTRASONIC_COUNT_HW, ULTRASONIC_COUNT_NUM);

    	/* Send 10us pulse */
    	cyhal_gpio_write(ULTRASONIC_TRIG_PIN, 1);
		cyhal_system_delay_us(10);
		cyhal_gpio_write(ULTRASONIC_TRIG_PIN, 0);

		/* Wait for return pulse to complete before reading the count value
		 * Max count possible during this time is ~10000 */
		cyhal_system_delay_ms(10);

		/* Read count value of return signal  - shifting right by 7 (divide by 128) gives a distance result approximately in inches */
		ultrasonic_count = (Cy_TCPWM_Counter_GetCounter(ULTRASONIC_COUNT_HW, ULTRASONIC_COUNT_NUM)) >> 7;
		printf("Distance: %ld\t", ultrasonic_count);

		/* Print wheel distance measurements*/
		printf("Left: %ld\tRight: %ld\n",leftDistance, rightDistance);

		/* Wait to start next measurement */
		cyhal_system_delay_ms(200);
    }
}


void updateMotor(uint8_t direction, uint8_t speed)
{

	/* Motor Directions */
	enum motor_direction_e
	{
		MOTOR_STOP,
		MOTOR_FWD_LEFT,
		MOTOR_FWD,
		MOTOR_FWD_RIGHT,
		MOTOR_ROT_LEFT,
		MOTOR_ROT_RIGHT,
		MOTOR_REV_LEFT,
		MOTOR_REV,
		MOTOR_REV_RIGHT
	};

	/* Turn off all motors first before setting new values */
	cyhal_pwm_set_duty_cycle(&motor_left_fwd,  MOTOR_OFF, MOTOR_PWM_FREQUENCY);
	cyhal_pwm_set_duty_cycle(&motor_right_fwd, MOTOR_OFF, MOTOR_PWM_FREQUENCY);
	cyhal_pwm_set_duty_cycle(&motor_left_rev,  MOTOR_OFF, MOTOR_PWM_FREQUENCY);
	cyhal_pwm_set_duty_cycle(&motor_right_rev, MOTOR_OFF, MOTOR_PWM_FREQUENCY);

	switch (direction)
	{
		case MOTOR_FWD_LEFT:
			cyhal_pwm_set_duty_cycle(&motor_left_fwd,  (speed >> 1),	MOTOR_PWM_FREQUENCY);
			cyhal_pwm_set_duty_cycle(&motor_right_fwd, (speed),			MOTOR_PWM_FREQUENCY);
			break;

		case MOTOR_FWD:
			cyhal_pwm_set_duty_cycle(&motor_left_fwd,  (speed),			MOTOR_PWM_FREQUENCY);
			cyhal_pwm_set_duty_cycle(&motor_right_fwd, (speed),			MOTOR_PWM_FREQUENCY);
			break;

		case MOTOR_FWD_RIGHT:
			cyhal_pwm_set_duty_cycle(&motor_left_fwd,  (speed),			MOTOR_PWM_FREQUENCY);
			cyhal_pwm_set_duty_cycle(&motor_right_fwd, (speed >> 1),	MOTOR_PWM_FREQUENCY);
			break;

		case MOTOR_ROT_LEFT:
			cyhal_pwm_set_duty_cycle(&motor_left_rev,  (speed >> 1),	MOTOR_PWM_FREQUENCY);
			cyhal_pwm_set_duty_cycle(&motor_right_fwd, (speed >> 1),	MOTOR_PWM_FREQUENCY);
			break;

		case MOTOR_ROT_RIGHT:
			cyhal_pwm_set_duty_cycle(&motor_left_fwd,  (speed >> 1),	MOTOR_PWM_FREQUENCY);
			cyhal_pwm_set_duty_cycle(&motor_right_rev, (speed >> 1),	MOTOR_PWM_FREQUENCY);
			break;

		case MOTOR_REV_LEFT:
			cyhal_pwm_set_duty_cycle(&motor_left_rev,  (speed >> 1),	MOTOR_PWM_FREQUENCY);
			cyhal_pwm_set_duty_cycle(&motor_right_rev, (speed),			MOTOR_PWM_FREQUENCY);
			break;

		case MOTOR_REV:
			cyhal_pwm_set_duty_cycle(&motor_left_rev,  (speed),			MOTOR_PWM_FREQUENCY);
			cyhal_pwm_set_duty_cycle(&motor_right_rev, (speed),			MOTOR_PWM_FREQUENCY);
			break;

		case MOTOR_REV_RIGHT:
			cyhal_pwm_set_duty_cycle(&motor_left_rev,  (speed),			MOTOR_PWM_FREQUENCY);
			cyhal_pwm_set_duty_cycle(&motor_right_rev, (speed >> 1),	MOTOR_PWM_FREQUENCY);
			break;
	}
}

void updateServo(int8 angle)
{
	uint16_t pulse;

	/* Max range is -90 to +90 degrees */
	if(angle < -90.0)
	{
		angle = -90.0;
	}	if(angle > 90.0)
	{
		angle = 90.0;
	}

	/* Translate angle to pulse width */
	/* -90 degrees to +90 degrees <-> 2500us to 500us */
	pulse = (uint16_t)((float)angle * -11.1111 + 1500.0);

	/* Update pulse width */
	cyhal_pwm_set_period(&servo_pwm, SERVO_PERIOD, pulse);
}

static void optical_left_isr(void *handler_arg, cyhal_gpio_event_t event)
{
	leftDistance++;
}

static void optical_right_isr(void *handler_arg, cyhal_gpio_event_t event)
{
	rightDistance++;
}
/* [] END OF FILE */
