// Robot Template app
//
// Framework for creating applications that control the Kobuki robot

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_error.h"
#include "app_timer.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_spi.h"

#include "buckler.h"
#include "display.h"
#include "kobukiActuator.h"
#include "kobukiSensorPoll.h"
#include "kobukiSensorTypes.h"
#include "kobukiUtilities.h"
#include "mpu9250.h"

// I2C manager
NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);

typedef enum {
    OFF,
  DRIVING,
  TURNING,
  AVOIDING,
  SHUNNING,
  SHUNDRIVE,
  SHORTSHUN,
} robot_state_t;


static float measure_distance(uint16_t current_encoder, uint16_t previous_encoder) {
	const float CONVERSION = 0.00008529;
	int differential = current_encoder - previous_encoder;
	if (current_encoder < previous_encoder) {
		differential += (1 << 16);
	}
	return differential * CONVERSION;
}

static float measure_reverse(uint16_t current_encoder, uint16_t previous_encoder) {
	const float CONVERSION = 0.00008529;
	int differential = previous_encoder - current_encoder;
	if (current_encoder > previous_encoder) {
		differential += (1 << 16);
	}
	return differential * CONVERSION;
}
mpu9250_measurement_t mt;

static void read_tilt (void) {
	 mt = mpu9250_read_accelerometer();
	 printf("X: %f Y: %f Z: %f \n", asin(mt.x_axis) * 180/M_PI, asin(mt.y_axis) * 180/M_PI,
	 	asin(mt.z_axis)*180/M_PI);
}


int main(void) {
  ret_code_t error_code = NRF_SUCCESS;

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized!\n");

  // initialize LEDs
  nrf_gpio_pin_dir_set(23, NRF_GPIO_PIN_DIR_OUTPUT);
  nrf_gpio_pin_dir_set(24, NRF_GPIO_PIN_DIR_OUTPUT);
  nrf_gpio_pin_dir_set(25, NRF_GPIO_PIN_DIR_OUTPUT);

  // initialize display
  nrf_drv_spi_t spi_instance = NRF_DRV_SPI_INSTANCE(1);
  nrf_drv_spi_config_t spi_config = {
    .sck_pin = BUCKLER_LCD_SCLK,
    .mosi_pin = BUCKLER_LCD_MOSI,
    .miso_pin = BUCKLER_LCD_MISO,
    .ss_pin = BUCKLER_LCD_CS,
    .irq_priority = NRFX_SPI_DEFAULT_CONFIG_IRQ_PRIORITY,
    .orc = 0,
    .frequency = NRF_DRV_SPI_FREQ_4M,
    .mode = NRF_DRV_SPI_MODE_2,
    .bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
  };
  error_code = nrf_drv_spi_init(&spi_instance, &spi_config, NULL, NULL);
  APP_ERROR_CHECK(error_code);
  display_init(&spi_instance);
  display_write("Hello, Human!", DISPLAY_LINE_0);
  printf("Display initialized!\n");

  // initialize i2c master (two wire interface)
  nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;
  i2c_config.scl = BUCKLER_SENSORS_SCL;
  i2c_config.sda = BUCKLER_SENSORS_SDA;
  i2c_config.frequency = NRF_TWIM_FREQ_100K;
  error_code = nrf_twi_mngr_init(&twi_mngr_instance, &i2c_config);
  APP_ERROR_CHECK(error_code);
  mpu9250_init(&twi_mngr_instance);
  printf("IMU initialized!\n");

  // initialize Kobuki
  kobukiInit();
  printf("Kobuki initialized!\n");

  // configure initial state
  robot_state_t state = OFF;
  KobukiSensors_t sensors = {0};
  uint16_t rw; bool left; bool right; bool center;
  bool isLeft; bool isRight; bool isCenter;
  isLeft = false; isRight = false; isCenter = false;
  float rtnAngle = 0;

  // loop forever, running state machine
  while (1) {
    // read sensors from robot
    kobukiSensorPoll(&sensors);

    // delay before continuing
    // Note: removing this delay will make responses quicker, but will result
    //  in printf's in this loop breaking JTAG
    nrf_delay_ms(100);
    read_tilt();
    left = sensors.bumps_wheelDrops.bumpLeft;
    right = sensors.bumps_wheelDrops.bumpRight;
    center = sensors.bumps_wheelDrops.bumpCenter;

    // handle states
    switch(state) {

      case OFF: {
        // transition logic
        display_write("OFF", DISPLAY_LINE_0);
        if (is_button_pressed(&sensors)) {
          rw = sensors.rightWheelEncoder;
          state = DRIVING;
        } else {
          // perform state-specific actions here

          kobukiDriveDirect(0, 0);
          state = OFF;
        }
        break; // each case needs to end with break!
      }

      case DRIVING: {
        // transition logic
        display_write("DRIVING", DISPLAY_LINE_0);
        uint16_t rw_new = sensors.rightWheelEncoder;
        float dist = measure_distance(rw_new, rw);
        if (dist > 1) {
        	dist = 0;
        }
        char buf[16]; snprintf(buf, 16, "%f", dist);
        display_write(buf, DISPLAY_LINE_1);

        if (is_button_pressed(&sensors)) {
          state = OFF;
        } else if (left || right || center) {
        	rw = sensors.rightWheelEncoder;
        	isLeft = left; isRight = right; isCenter = center;
        	rtnAngle = 0;
        	state = AVOIDING;
        } else if (sensors.cliffLeft || sensors.cliffCenter || sensors.cliffRight){
        	state = OFF;
        } else {
        	display_write("DRIVING", DISPLAY_LINE_0);
          kobukiDriveDirect(100, 100);
          state = DRIVING;
        }
        break; // each case needs to end with break!
      }

      // case TURNING: {
      // 	display_write("TURNING", DISPLAY_LINE_0);
      // 	mpu9250_measurement_t angle = mpu9250_read_gyro_integration();
      // 	float z = abs(angle.z_axis);
      // 	char buf[16]; snprintf(buf, 16, "%f", z);
      //   display_write(buf, DISPLAY_LINE_1);
      // 	if (is_button_pressed(&sensors)) {
      // 		mpu9250_stop_gyro_integration();
      //     state = OFF;
      //   } else if (left || right || center) {
      //   	mpu9250_stop_gyro_integration();
      //   	rw = sensors.rightWheelEncoder;
      //   	isLeft = left; isRight = right; isCenter = center;
      //   	state = AVOIDING;
      //   } else if (z>=75.0) {
      //     rw = sensors.rightWheelEncoder;
      //  	  state = DRIVING;
      //  	  mpu9250_stop_gyro_integration();
      //   } else {
      //     // perform state-specific actions here
      //     kobukiDriveDirect(70, -70);
      //     state = TURNING;
      //   }
      //  	break;
      // }
      /**
        off -> button press -> orient
        orient -> rotate (iteratively) -> drive (150,150)
        drive -> left/right cliff sensing back up rotate slightly oposite -> adjust
        adjust -> drive*
      */

      case AVOIDING: {
      	display_write("AVOIDING", DISPLAY_LINE_0);
      	uint16_t rw_new = sensors.rightWheelEncoder;
        float dist = measure_reverse(rw_new, rw);
        if (dist > 1) {
        	dist = 0;
        }
        char buf[16]; snprintf(buf, 16, "%f", dist);
        display_write(buf, DISPLAY_LINE_1);

        if (is_button_pressed(&sensors)) {
          state = OFF;
        } else if (dist < 0.1) {
          kobukiDriveDirect(-100, -100);
          state = AVOIDING;
        } else if (isLeft || isRight || isCenter){
          // perform state-specific actions here
          mpu9250_start_gyro_integration();
        	dist = 0;
       	  state = SHUNNING;
        } else {
          state = ADJUST;
        }
        break;

      }

      case SHUNNING: {
      	display_write("SHUNNING", DISPLAY_LINE_0);
      	mpu9250_measurement_t angle = mpu9250_read_gyro_integration();
      	float z = abs(angle.z_axis);
      	char buf[16]; snprintf(buf, 16, "%f", z);
        display_write(buf, DISPLAY_LINE_1);
      	if (is_button_pressed(&sensors)) {
      		mpu9250_stop_gyro_integration();
          state = OFF;
        } else if (left || right || center) {
        	mpu9250_stop_gyro_integration();
        	rw = sensors.rightWheelEncoder;
        	if (isLeft || isCenter) {
          		rtnAngle += z;
          	} else if (isRight) {
          		rtnAngle -= z;
          	}
          	isLeft = left; isRight = right; isCenter = center;
        	state = AVOIDING;
        } else if (z >= 85) {
        	if (isLeft || isCenter) {
          		rtnAngle += 85;
          	} else if (isRight) {
          		rtnAngle -= 85;
          	}
          rw = sensors.rightWheelEncoder;
       	  state = SHUNDRIVE;
       	  mpu9250_stop_gyro_integration();
        } else {
          // perform state-specific actions here
          if (isLeft || isCenter) {
          	kobukiDriveDirect(70, -70);
          } else if (isRight) {
          	kobukiDriveDirect(-70, 70);
          }
          state = SHUNNING;
        }
       	break;
      }

      case SHUNDRIVE: {
      	display_write("SHUNDRIVE", DISPLAY_LINE_0);
        uint16_t rw_new = sensors.rightWheelEncoder;
        float dist = measure_distance(rw_new, rw);
        if (dist > 1) {
        	dist = 0;
        }
        char buf[16]; snprintf(buf, 16, "%f", dist);
        display_write(buf, DISPLAY_LINE_1);

        if (is_button_pressed(&sensors)) {
          state = OFF;
        } else if (left || right || center) {
        	rw = sensors.rightWheelEncoder;
        	isLeft = left; isRight = right; isCenter = center;
        	state = AVOIDING;
        } else if (sensors.cliffLeft || sensors.cliffCenter || sensors.cliffRight){
        	state = OFF;
        }else if (dist < 0.1) {
          kobukiDriveDirect(100, 100);
          state = SHUNDRIVE;
        } else {
          mpu9250_start_gyro_integration();
          state = SHORTSHUN;

        }
        break; // each ca

      }

      case SHORTSHUN: {
      	display_write("SHORTSHUN", DISPLAY_LINE_0);
      	mpu9250_measurement_t angle = mpu9250_read_gyro_integration();
      	float z = abs(angle.z_axis);
      	char buf[16]; snprintf(buf, 16, "%f", rtnAngle);
        display_write(buf, DISPLAY_LINE_1);
        if (rtnAngle < 0) {
        	// turning right
          	isRight = true;
          	isLeft = isCenter = false;
         } else {
         	// turning left
          	isLeft = true;
          	isRight = isCenter = false;
         }
      	if (is_button_pressed(&sensors)) {
      		mpu9250_stop_gyro_integration();
          state = OFF;
        } else if (left || right || center) {
        	mpu9250_stop_gyro_integration();
        	rw = sensors.rightWheelEncoder;
        	isLeft = left; isRight = right; isCenter = center;
        	state = AVOIDING;
        } else if (z >= abs(rtnAngle)) {
          rw = sensors.rightWheelEncoder;
       	  state = DRIVING;
       	  mpu9250_stop_gyro_integration();
        } else {
          // perform state-specific actions here
          if (isLeft || isCenter) {
          	kobukiDriveDirect(-70, 70);
          } else if (isRight) {
          	kobukiDriveDirect(70, -70);
          }
          state = SHORTSHUN;
        }
       	break;
      }

      // add other cases here

    }
  }
}

main.c
Displaying main.c.
