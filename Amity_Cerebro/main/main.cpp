#include <math.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"

// Hardware Abstraction
#include "amity_hardware.h"

// Red Neuronal Pre-entrenada
#include "amity_policy.h"

static const char *TAG = "AMITY_BRAIN";

static const float PI = 3.1415926535f;

// Fases de Dirección CPG
static const float PHASE_WALK[4] = {PI / 2.3f, 0.0f, PI / 2.3f, 0.0f};

// ── ESTADO HÍBRIDO ───────────────────────────────────────────────────────────
enum ControlMode { MODE_NN, MODE_PID };
volatile ControlMode current_mode = MODE_NN;
volatile bool is_walking = false;
volatile bool in_deep_sleep = false;

// ── VARIABLES GLOBALES DE TAREAS ─────────────────────────────────────────────
TaskHandle_t walk_task_handle = NULL;
TaskHandle_t imu_task_handle = NULL;

// Variables para observaciones IA
float last_joint_rad[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float last_actions[4] = {0.0f, 0.0f, 0.0f, 0.0f};

// ── CONTROL PID ──────────────────────────────────────────────────────────────
typedef struct {
  float kp, ki, kd;
  float integral;
  float prev_error;
  float integral_limit;
} pid_t;

static pid_t pid_roll = {.kp = 0.0f,
                         .ki = 0.008f,
                         .kd = 0.0f,
                         .integral = 0,
                         .prev_error = 0,
                         .integral_limit = 20.0f};
static pid_t pid_pitch = {.kp = 0.0f,
                          .ki = 0.008f,
                          .kd = 0.0f,
                          .integral = 0,
                          .prev_error = 0,
                          .integral_limit = 20.0f};
static pid_t pid_yaw = {.kp = 0.0f,
                        .ki = 0.005f,
                        .kd = 0.05f,
                        .integral = 0,
                        .prev_error = 0,
                        .integral_limit = 20.0f};

float pid_compute(pid_t *pid, float setpoint, float measured, float dt) {
  float error = setpoint - measured;
  pid->integral += error * dt;
  if (pid->integral > pid->integral_limit)
    pid->integral = pid->integral_limit;
  if (pid->integral < -pid->integral_limit)
    pid->integral = -pid->integral_limit;
  float derivative = (dt > 0) ? (error - pid->prev_error) / dt : 0.0f;
  pid->prev_error = error;
  return pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
}

void pid_reset(pid_t *pid) {
  pid->integral = 0.0f;
  pid->prev_error = 0.0f;
}

// ── RED NEURONAL (BARE-METAL) ────────────────────────────────────────────────
inline float elu(float x) { return (x > 0) ? x : (expf(x) - 1.0f); }

void forward_mlp(const float *obs, float *actions) {
  float obs_norm[20];
  for (int i = 0; i < 20; ++i)
    obs_norm[i] = (obs[i] - obs_mean[i]) / obs_std[i];

  float h1[64];
  for (int i = 0; i < 64; ++i) {
    float sum = b1[i];
    for (int j = 0; j < 20; ++j)
      sum += obs_norm[j] * w1[i * 20 + j];
    h1[i] = elu(sum);
  }

  float h2[64];
  for (int i = 0; i < 64; ++i) {
    float sum = b2[i];
    for (int j = 0; j < 64; ++j)
      sum += h1[j] * w2[i * 64 + j];
    h2[i] = elu(sum);
  }

  for (int i = 0; i < 4; ++i) {
    float sum = b3[i];
    for (int j = 0; j < 64; ++j)
      sum += h2[j] * w3[i * 64 + j];
    actions[i] = sum;
  }
}

// ── TAREA DE IMU (50Hz) ──────────────────────────────────────────────────────
void imu_task(void *pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(20);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  float local_roll = 0.0f, local_pitch = 0.0f, local_yaw = 0.0f;
  const float dt = 0.02f;
  const float ALPHA = 0.98f;

  int calib_count = 0;
  float gx_offset = 0.0f;
  float gy_offset = 0.0f;
  float gz_offset = 0.0f;

  while (1) {
    if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
      uint8_t raw[14];
      if (mpu_read_regs(MPU_REG_ACCEL_XOUT_H, raw, 14) == ESP_OK) {
        int16_t ax_raw = (int16_t)(raw[0] << 8 | raw[1]);
        int16_t ay_raw = (int16_t)(raw[2] << 8 | raw[3]);
        int16_t az_raw = (int16_t)(raw[4] << 8 | raw[5]);
        int16_t gx_raw = (int16_t)(raw[8] << 8 | raw[9]);
        int16_t gy_raw = (int16_t)(raw[10] << 8 | raw[11]);
        int16_t gz_raw = (int16_t)(raw[12] << 8 | raw[13]);

        float ax = (float)ax_raw / ACCEL_SCALE;
        float ay = (float)ay_raw / ACCEL_SCALE;
        float az = -((float)az_raw / ACCEL_SCALE);

        float gx = ((float)gx_raw / GYRO_SCALE) * (PI / 180.0f);
        float gy = -((float)gy_raw / GYRO_SCALE) * (PI / 180.0f);
        float gz = ((float)gz_raw / GYRO_SCALE) * (PI / 180.0f);

        if (calib_count < 100) {
          gx_offset += gx;
          gy_offset += gy;
          gz_offset += gz;
          calib_count++;
          if (calib_count == 100) {
            gx_offset /= 100.0f;
            gy_offset /= 100.0f;
            gz_offset /= 100.0f;
            ESP_LOGI(TAG, "Calibración IMU terminada");
          }
        } else {
          gx -= gx_offset;
          gy -= gy_offset;
          gz -= gz_offset;

          // Zona muerta (deadband) para evitar que el ruido micro-integre el Yaw
          if (fabsf(gz) < 0.005f) gz = 0.0f;

        float roll_acc = atan2f(ay, az);
        float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az));

        local_roll = ALPHA * (local_roll + gx * dt) + (1.0f - ALPHA) * roll_acc;
        local_pitch =
            ALPHA * (local_pitch + gy * dt) + (1.0f - ALPHA) * pitch_acc;
        local_yaw += gz * dt;

        if (xSemaphoreTake(imu_mutex, portMAX_DELAY) == pdTRUE) {
          global_imu.roll = local_roll;
          global_imu.pitch = local_pitch;
          global_imu.yaw = local_yaw;
          global_imu.gx = gx;
          global_imu.gy = gy;
          global_imu.gz = gz;
          xSemaphoreGive(imu_mutex);
        }

        // Telemetría para la extensión Teleplot
        printf(">roll:%.4f\n>pitch:%.4f\n>yaw:%.4f\n",
               local_roll * (180.0f / PI), local_pitch * (180.0f / PI),
               local_yaw * (180.0f / PI));
        }
      }
      xSemaphoreGive(i2c_mutex);
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ── TAREA DE CAMINATA / INFERENCIA (30Hz) ────────────────────────────────────
void walk_task(void *pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(33);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  const float T_NN = 1.2f;
  const float A_rad = 0.43f;

  const float A_base_pid = 25.0f;
  const float O_i = 90.0f;
  const float T_PID = 1200.0f;
  const float MAX_CORRECTION = 15.0f;

  while (1) {
    if (!is_walking) {
      vTaskDelay(pdMS_TO_TICKS(50));
      xLastWakeTime = xTaskGetTickCount();
      continue;
    }

    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;

    if (xSemaphoreTake(imu_mutex, portMAX_DELAY) == pdTRUE) {
      roll = global_imu.roll;
      pitch = global_imu.pitch;
      yaw = global_imu.yaw;
      gx = global_imu.gx;
      gy = global_imu.gy;
      gz = global_imu.gz;
      xSemaphoreGive(imu_mutex);
    }

    if (current_mode == MODE_NN) {
      // -- RED NEURONAL --
      float t = (float)esp_timer_get_time() / 1000000.0f;
      float obs[20] = {0};

      obs[3] = roll;
      obs[4] = pitch;
      obs[5] = gx;
      obs[6] = gy;
      obs[7] = gz;

      for (int i = 0; i < 4; ++i)
        obs[8 + i] = last_joint_rad[i];
      for (int i = 0; i < 4; ++i)
        obs[12 + i] = last_actions[i];

      float cpg_waves[4];
      for (int i = 0; i < 4; ++i) {
        cpg_waves[i] = sinf((2.0f * PI / T_NN) * t + PHASE_WALK[i]);
        obs[16 + i] = cpg_waves[i];
      }

      float actions[4] = {0};
      forward_mlp(obs, actions);
      for (int i = 0; i < 4; ++i)
        last_actions[i] = actions[i];

      if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
        for (int i = 0; i < 4; ++i) {
          float target_rad = A_rad * cpg_waves[i] + (actions[i] * 0.28f);
          last_joint_rad[i] = target_rad;
          float target_deg = (target_rad * (180.0f / PI)) + 90.0f;

          uint8_t pin = 0;
          switch (i) {
          case 0:
            pin = PIN_CADERA_IZQ;
            break;
          case 1:
            pin = PIN_CADERA_DER;
            break;
          case 2:
            pin = PIN_PIERNA_DER;
            break;
          case 3:
            pin = PIN_PIERNA_IZQ;
            break;
          }
          set_servo_angle(pin, target_deg);
        }
        xSemaphoreGive(i2c_mutex);
      }

    } else {
      // -- CONTROL PID CLÁSICO --
      static float filtered_yaw = 0.0f;
      float roll_deg = roll * (180.0f / PI);
      float pitch_deg = pitch * (180.0f / PI);
      float yaw_deg = yaw * (180.0f / PI);

      if (filtered_yaw == 0.0f && yaw_deg != 0.0f)
        filtered_yaw = yaw_deg;
      filtered_yaw = filtered_yaw * 0.95f + yaw_deg * 0.05f;

      float corr_roll = pid_compute(&pid_roll, 0.0f, roll_deg, 0.033f);
      float corr_pitch = pid_compute(&pid_pitch, 0.0f, pitch_deg, 0.033f);
      float corr_yaw = pid_compute(&pid_yaw, 0.0f, filtered_yaw, 0.033f);

      if (corr_roll > MAX_CORRECTION)
        corr_roll = MAX_CORRECTION;
      if (corr_roll < -MAX_CORRECTION)
        corr_roll = -MAX_CORRECTION;
      if (corr_pitch > MAX_CORRECTION)
        corr_pitch = MAX_CORRECTION;
      if (corr_pitch < -MAX_CORRECTION)
        corr_pitch = -MAX_CORRECTION;
      if (corr_yaw > MAX_CORRECTION)
        corr_yaw = MAX_CORRECTION;
      if (corr_yaw < -MAX_CORRECTION)
        corr_yaw = -MAX_CORRECTION;

      float A_cadera = A_base_pid + corr_roll;
      float A_pierna_izq = A_base_pid + corr_pitch - corr_yaw;
      float A_pierna_der = A_base_pid + corr_pitch + corr_yaw;

      if (A_cadera < 5.0f)
        A_cadera = 5.0f;
      if (A_pierna_izq < 5.0f)
        A_pierna_izq = 5.0f;
      if (A_pierna_der < 5.0f)
        A_pierna_der = 5.0f;

      float t = (float)(esp_timer_get_time() / 1000 % (int64_t)T_PID);

      if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
        set_servo_angle(PIN_CADERA_IZQ,
                        A_cadera * sinf((2 * PI / T_PID) * t + PHASE_WALK[0]) +
                            O_i);
        set_servo_angle(PIN_CADERA_DER,
                        A_cadera * sinf((2 * PI / T_PID) * t + PHASE_WALK[1]) +
                            O_i);
        set_servo_angle(
            PIN_PIERNA_DER,
            A_pierna_der * sinf((2 * PI / T_PID) * t + PHASE_WALK[2]) + O_i);
        set_servo_angle(
            PIN_PIERNA_IZQ,
            A_pierna_izq * sinf((2 * PI / T_PID) * t + PHASE_WALK[3]) + O_i);
        xSemaphoreGive(i2c_mutex);
      }
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ── TAREA TOUCH Y ENERGÍA ────────────────────────────────────────────────────
void sleep_logic() {
  ESP_LOGW(TAG, "DURMIENDO...");
  send_uart_cmd("<CMD:SLEEP>\n");

  set_all_servos(90.0f);
  vTaskDelay(pdMS_TO_TICKS(200));

  // Suspender tareas en background para que sus timers no despierten al ESP32
  if (imu_task_handle != NULL)
    vTaskSuspend(imu_task_handle);
  if (walk_task_handle != NULL)
    vTaskSuspend(walk_task_handle);

  // Esperar a que el usuario retire el dedo del sensor antes de dormir
  while (gpio_get_level(TOUCH_PIN_3) == 1) {
    vTaskDelay(pdMS_TO_TICKS(50));
  }

  gpio_wakeup_enable(TOUCH_PIN_3, GPIO_INTR_HIGH_LEVEL);
  esp_sleep_enable_gpio_wakeup();

  // Entrar en Light Sleep
  esp_light_sleep_start();

  // Al Despertar
  ESP_LOGI(TAG, "DESPERTANDO...");
  send_uart_cmd("<CMD:WAKE>\n");

  gpio_wakeup_disable(TOUCH_PIN_3);

  // Esperar a que el usuario retire el dedo del sensor antes de reanudar todo
  while (gpio_get_level(TOUCH_PIN_3) == 1) {
    vTaskDelay(pdMS_TO_TICKS(50));
  }

  // Reanudar tareas
  if (imu_task_handle != NULL)
    vTaskResume(imu_task_handle);
  if (walk_task_handle != NULL)
    vTaskResume(walk_task_handle);

  in_deep_sleep = false;
  vTaskDelay(pdMS_TO_TICKS(100)); // Pequeño delay de seguridad
}

void control_task(void *pvParameters) {
  uint32_t press_time = 0;

  while (1) {
    if (touch_modules_update()) {
      if (touch_get_state(0)) {
        is_walking = !is_walking;
        ESP_LOGI(TAG, "%s",
                 is_walking ? "Caminata: REANUDADA" : "Caminata: PAUSADA");
        if (!is_walking) {
          set_all_servos(90.0f);
        } else {
          pid_reset(&pid_roll);
          pid_reset(&pid_pitch);
          pid_reset(&pid_yaw);
          if (xSemaphoreTake(imu_mutex, portMAX_DELAY) == pdTRUE) {
            global_imu.yaw = 0.0f;
            xSemaphoreGive(imu_mutex);
          }
        }
      }

      if (touch_get_state(1)) {
        current_mode = (current_mode == MODE_NN) ? MODE_PID : MODE_NN;
        ESP_LOGI(TAG, "Modo cambiado a: %s",
                 current_mode == MODE_NN ? "RED NEURONAL" : "PID CLASICO");
      }
    }

    if (gpio_get_level(TOUCH_PIN_3) == 1) {
      press_time += 50;
      if (press_time >= 2000 && !in_deep_sleep) {
        in_deep_sleep = true;
        is_walking = false;
        led_set_state(false);
        press_time = 0;
        sleep_logic();
      }
    } else {
      press_time = 0;
    }

    // Actualizar estado del LED indicador
    led_set_state(is_walking && current_mode == MODE_NN);

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ── PUNTO DE ENTRADA PRINCIPAL ───────────────────────────────────────────────
extern "C" void app_main(void) {
  ESP_LOGI(TAG, "Iniciando Sistema de Control Híbrido...");

  i2c_mutex = xSemaphoreCreateMutex();
  imu_mutex = xSemaphoreCreateMutex();

  init_uart();
  touch_modules_init();
  led_init();
  init_i2c_bus();

  if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
    pca9685_init();
    mpu_init();
    xSemaphoreGive(i2c_mutex);
  }
  set_all_servos(90.0f);

  xTaskCreatePinnedToCore(imu_task, "IMU_Task", 4096, NULL, 6, &imu_task_handle,
                          0);
  xTaskCreatePinnedToCore(walk_task, "Walk_Task", 8192, NULL, 5,
                          &walk_task_handle, 1);
  xTaskCreatePinnedToCore(control_task, "Control_Task", 4096, NULL, 4, NULL, 1);
}
