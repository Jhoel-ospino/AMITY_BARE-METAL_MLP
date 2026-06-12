#include "amity_hardware.h"
#include <string.h>
#include "driver/uart.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

static const char *TAG_HW = "AMITY_HW";

// UART
#define UART_BRIDGE_NUM UART_NUM_1
#define UART_TX_PIN 44
#define UART_RX_PIN 1

// MPU
#define MPU_REG_PWR_MGMT_1 0x6B

// PCA
#define PCA9685_ADDR 0x40
#define PCA9685_MODE1 0x00
#define PCA9685_PRESCALE 0xFE
#define PCA9685_LED0_ON_L 0x06
#define SERVO_MIN_PULSE 150
#define SERVO_MAX_PULSE 600

// TOUCH
#define DEBOUNCE_MS 50

SemaphoreHandle_t i2c_mutex = NULL;
SemaphoreHandle_t imu_mutex = NULL;
imu_data_t global_imu = {};

i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t pca9685_handle;
i2c_master_dev_handle_t mpu_handle;

static bool touch_state[3] = {false, false, false};
static bool last_raw[3] = {false, false, false};
static TickType_t last_change[3] = {0, 0, 0};
static const gpio_num_t TOUCH_PINS[3] = {TOUCH_PIN_1, TOUCH_PIN_2, TOUCH_PIN_3};

// --- LED ---
void led_init(void) {
  gpio_reset_pin(LED_PIN);
  gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
  gpio_set_level(LED_PIN, 0); // Apagado por defecto
}

void led_set_state(bool state) {
  gpio_set_level(LED_PIN, state ? 1 : 0);
}

// --- UART ---
void init_uart(void) {
  uart_config_t uart_config = {};
  uart_config.baud_rate = 2000000;
  uart_config.data_bits = UART_DATA_8_BITS;
  uart_config.parity = UART_PARITY_DISABLE;
  uart_config.stop_bits = UART_STOP_BITS_1;
  uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  uart_config.source_clk = UART_SCLK_DEFAULT;

  ESP_ERROR_CHECK(uart_driver_install(UART_BRIDGE_NUM, 1024, 0, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_param_config(UART_BRIDGE_NUM, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(UART_BRIDGE_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

void send_uart_cmd(const char* cmd) {
  uart_write_bytes(UART_BRIDGE_NUM, cmd, strlen(cmd));
  uart_wait_tx_done(UART_BRIDGE_NUM, 100);
}

// --- TOUCH ---
void touch_modules_init(void) {
  gpio_config_t cfg = {
      .pin_bit_mask = (1ULL << TOUCH_PIN_1) | (1ULL << TOUCH_PIN_2) | (1ULL << TOUCH_PIN_3),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_ENABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&cfg);
}

bool touch_modules_update(void) {
  bool changed = false;
  TickType_t now = xTaskGetTickCount();
  for (int i = 0; i < 3; i++) {
    bool raw = (bool)gpio_get_level(TOUCH_PINS[i]);
    if (raw && !last_raw[i] && (now - last_change[i]) >= pdMS_TO_TICKS(DEBOUNCE_MS)) {
      touch_state[i] = !touch_state[i];
      last_change[i] = now;
      changed = true;
    }
    last_raw[i] = raw;
  }
  return changed;
}

bool touch_get_state(int index) {
  if (index < 0 || index > 2) return false;
  return touch_state[index];
}

// --- I2C ---
void init_i2c_bus(void) {
  ESP_LOGI(TAG_HW, "Inicializando I2C Maestro v6.0.0");
  i2c_master_bus_config_t bus_cfg = {};
  bus_cfg.i2c_port = I2C_NUM_0;
  bus_cfg.sda_io_num = (gpio_num_t)I2C_SDA_PIN;
  bus_cfg.scl_io_num = (gpio_num_t)I2C_SCL_PIN;
  bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
  bus_cfg.glitch_ignore_cnt = 7;
  bus_cfg.flags.enable_internal_pullup = true;

  ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));

  i2c_device_config_t pca_cfg = {};
  pca_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  pca_cfg.device_address = PCA9685_ADDR;
  pca_cfg.scl_speed_hz = I2C_CLK_SPEED;
  ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &pca_cfg, &pca9685_handle));

  i2c_device_config_t mpu_cfg = {};
  mpu_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  mpu_cfg.device_address = MPU_ADDR;
  mpu_cfg.scl_speed_hz = I2C_CLK_SPEED;
  ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &mpu_cfg, &mpu_handle));
}

esp_err_t mpu_write_reg(uint8_t reg, uint8_t data) {
  uint8_t buf[2] = {reg, data};
  return i2c_master_transmit(mpu_handle, buf, 2, -1);
}

esp_err_t mpu_read_regs(uint8_t reg_start, uint8_t *out, size_t len) {
  esp_err_t err = i2c_master_transmit(mpu_handle, &reg_start, 1, -1);
  if (err != ESP_OK) return err;
  return i2c_master_receive(mpu_handle, out, len, -1);
}

void mpu_init(void) {
  ESP_ERROR_CHECK(mpu_write_reg(MPU_REG_PWR_MGMT_1, 0x00));
  vTaskDelay(pdMS_TO_TICKS(100));
  ESP_LOGI(TAG_HW, "MPU6050 Inicializado");
}

esp_err_t pca9685_write_reg(uint8_t reg, uint8_t data) {
  uint8_t buffer[2] = {reg, data};
  return i2c_master_transmit(pca9685_handle, buffer, sizeof(buffer), -1);
}

void pca9685_init(void) {
  pca9685_write_reg(PCA9685_MODE1, 0x00);
  vTaskDelay(pdMS_TO_TICKS(10));
  uint8_t oldmode = 0x00;
  uint8_t newmode = (oldmode & 0x7F) | 0x10;
  pca9685_write_reg(PCA9685_MODE1, newmode);
  pca9685_write_reg(PCA9685_PRESCALE, 121);
  pca9685_write_reg(PCA9685_MODE1, oldmode);
  vTaskDelay(pdMS_TO_TICKS(5));
  pca9685_write_reg(PCA9685_MODE1, oldmode | 0xa0);
}

void set_servo_angle(uint8_t channel, float angle) {
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;
  uint16_t pulse = SERVO_MIN_PULSE + (uint16_t)(((SERVO_MAX_PULSE - SERVO_MIN_PULSE) * angle) / 180.0f);
  uint8_t reg_base = PCA9685_LED0_ON_L + (4 * channel);
  pca9685_write_reg(reg_base, 0);
  pca9685_write_reg(reg_base + 1, 0);
  pca9685_write_reg(reg_base + 2, pulse & 0xFF);
  pca9685_write_reg(reg_base + 3, pulse >> 8);
}

void set_all_servos(float angle) {
  if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
    set_servo_angle(PIN_CADERA_IZQ, angle);
    set_servo_angle(PIN_CADERA_DER, angle);
    set_servo_angle(PIN_PIERNA_IZQ, angle);
    set_servo_angle(PIN_PIERNA_DER, angle);
    xSemaphoreGive(i2c_mutex);
  }
}
