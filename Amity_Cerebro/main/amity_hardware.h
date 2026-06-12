#pragma once

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/gpio.h"

// ── MACROS DE PINES Y HARDWARE ───────────────────────────────────────────────
#define I2C_SDA_PIN 7
#define I2C_SCL_PIN 8
#define I2C_CLK_SPEED 400000

#define MPU_ADDR 0x68
#define ACCEL_SCALE 16384.0f
#define GYRO_SCALE 131.0f
#define MPU_REG_ACCEL_XOUT_H 0x3B

#define PIN_CADERA_IZQ 0
#define PIN_CADERA_DER 2
#define PIN_PIERNA_DER 1
#define PIN_PIERNA_IZQ 3

#define TOUCH_PIN_1 GPIO_NUM_2
#define TOUCH_PIN_2 GPIO_NUM_6
#define TOUCH_PIN_3 GPIO_NUM_43

#define LED_PIN GPIO_NUM_21

// ── VARIABLES COMPARTIDAS ────────────────────────────────────────────────────
extern SemaphoreHandle_t i2c_mutex;
extern SemaphoreHandle_t imu_mutex;

typedef struct {
  float roll;
  float pitch;
  float yaw;
  float gx;
  float gy;
  float gz;
} imu_data_t;
extern imu_data_t global_imu;

// ── FUNCIONES DE HARDWARE ────────────────────────────────────────────────────
void led_init(void);
void led_set_state(bool state);

void init_uart(void);
void send_uart_cmd(const char* cmd);

void touch_modules_init(void);
bool touch_modules_update(void);
bool touch_get_state(int index);

void init_i2c_bus(void);
void mpu_init(void);
esp_err_t mpu_read_regs(uint8_t reg_start, uint8_t *out, size_t len);

void pca9685_init(void);
void set_servo_angle(uint8_t channel, float angle);
void set_all_servos(float angle);
