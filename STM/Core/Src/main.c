/* ======================  main.c  (USART3 cmds + OLED US & LastCmd)  ====================== */
#include "main.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "oled.h"
#include "cmsis_os2.h"   // CMSIS-RTOS v2 API

/* ---------------- HAL handles ---------------- */
ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c2;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim12;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* ---------------- RTOS objects ---------------- */
typedef struct { char text[16]; } CmdMsg_t;
osMessageQueueId_t qCmd;         // UART->motion command queue
osEventFlagsId_t   evFlags;      // STOP / obstacle flags
#define EVF_OBS_NEAR  (1u << 0)
#define EVF_STOP      (1u << 1)
osMutexId_t oledMutex;

/* ---------------- Thread handles & attributes ---------------- */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,  //// was Normal
};
osThreadId_t obstacle_detectHandle;
const osThreadAttr_t obstacle_detect_attributes = {
  .name = "obstacle_detect",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
osThreadId_t message_receiveHandle;
const osThreadAttr_t message_receive_attributes = {
  .name = "message_receive",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal, ///// was Low
};
osThreadId_t ir_detectHandle;
const osThreadAttr_t ir_detect_attributes = {
  .name = "ir_detect",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
osThreadId_t move_controlHandle;
const osThreadAttr_t move_control_attributes = {
  .name = "move_control",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,//// was Low
};

// --- Mission planner / navigator ---
osThreadId_t missionHandle;
const osThreadAttr_t mission_attributes = {
  .name = "mission",
  .stack_size = 512 * 4,
  .priority = (osPriority_t)osPriorityAboveNormal, // a notch above Move/Msg
};

/* Display task handle & attributes */
osThreadId_t displayHandle;
const osThreadAttr_t display_attributes = {
  .name = "display",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow
};


/* ---------------- UART command buffer (legacy parser retained) ---------------- */
#define CMD_BUF_LEN 64
char cmd_buf[CMD_BUF_LEN];
int cmd_index = 0;

/* ---------------- Debug / UI ---------------- */
static char buf[128];

/* ---------------- System tick mirrors & motor vars ---------------- */
volatile uint32_t no_of_tick = 0;
volatile float position;
int16_t speed = 0, rpm = 0;
int start = 0;
int32_t pwmVal = 0, pwmVal_raw = 0;
int16_t pwmMax = (7200 - 200);
int16_t pwmMin = 250;
int16_t angle = 0, target_angle = 0;
int16_t error; int32_t error_area = 0, error_change, error_rate;
int32_t millisOld, millisNow, dt;
int distobs2 = 90;

/* ---------------- Encoders (TIM2 left, TIM5 right) ---------------- */
static uint16_t L0 = 0, R0 = 0;
static inline uint32_t tim_arr(TIM_TypeDef *t) { return (t->ARR ? t->ARR : 0xFFFFu); }
static inline int32_t left_ticks(void) {
  uint32_t arrp1 = tim_arr(TIM2) + 1u;
  int32_t d = (int32_t)((uint32_t)TIM2->CNT - (uint32_t)L0);
  if (d < 0) d += (int32_t)arrp1; return d;
}
static inline int32_t right_ticks(void) {
  uint32_t arrp1 = tim_arr(TIM5) + 1u;
  int32_t d = (int32_t)((int32_t)R0 - (int32_t)TIM5->CNT);
  if (d < 0) d += (int32_t)arrp1; return d;
}
static inline void reset_encoders(void) {
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  __HAL_TIM_SET_COUNTER(&htim5, 0);
  HAL_Delay(2);
  L0 = (uint16_t)TIM2->CNT;
  R0 = (uint16_t)TIM5->CNT;
}
static float cm_travelled(void) {
  static float COUNTS_PER_CM_L = 76.70f;
  static float COUNTS_PER_CM_R = 80.87f;
  float cmL = (float)left_ticks()  / COUNTS_PER_CM_L;
  float cmR = (float)right_ticks() / COUNTS_PER_CM_R;
  return 0.5f * (cmL + cmR);
}

//ir sensors

/* Choose the ADC1 channels your IR sensors use */
#define IR_LEFT_CH   ADC_CHANNEL_4   // PA4 by default in your MX_ADC1_Init()
#define IR_RIGHT_CH  ADC_CHANNEL_5   // change if wired elsewhere

/* Published readings (atomic enough for your usage) */
volatile uint32_t g_us_cm = 0;     // from StartTaskObstacle
volatile uint16_t ir_left_adc  = 0, ir_right_adc = 0;
volatile uint16_t ir_left_mm, ir_right_mm;
static char g_last_cmd[16] = "—";


/* IIR filter taps (float, but we store mm as uint16_t for consumers) */
static float irL_f = 0.f, irR_f = 0.f;

/* ---------------- ICM-20948 minimal ---------------- */
volatile float ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps;
#define WHO_AM_I 0x00
#define WHO_AM_I_VAL 0xEA
#define REG_BANK_SEL 0x7F
#define ICM_ADDR_68 (0x68 << 1)
#define ICM_ADDR_69 (0x69 << 1)
static uint16_t ICM_ADDR = ICM_ADDR_69;

static void ICM20948_SelectBank(uint8_t bank)
{ uint8_t d[2] = {REG_BANK_SEL, (uint8_t)(bank << 4)}; HAL_I2C_Master_Transmit(&hi2c2, ICM_ADDR, d, 2, HAL_MAX_DELAY); }
static void ICM20948_WriteReg(uint8_t bank, uint8_t reg, uint8_t val)
{ ICM20948_SelectBank(bank); uint8_t d[2] = {reg, val}; HAL_I2C_Master_Transmit(&hi2c2, ICM_ADDR, d, 2, HAL_MAX_DELAY); }
static void ICM20948_ReadRegs(uint8_t bank, uint8_t reg, uint8_t *data, uint8_t len)
{ ICM20948_SelectBank(bank); HAL_I2C_Master_Transmit(&hi2c2, ICM_ADDR, &reg, 1, HAL_MAX_DELAY); HAL_I2C_Master_Receive(&hi2c2, ICM_ADDR, data, len, HAL_MAX_DELAY); }
static HAL_StatusTypeDef icm_read_raw(uint16_t addr, uint8_t reg, uint8_t *data, uint8_t len)
{ HAL_StatusTypeDef s; s = HAL_I2C_Master_Transmit(&hi2c2, addr, &reg, 1, 100); if (s != HAL_OK) return s; return HAL_I2C_Master_Receive(&hi2c2, addr, data, len, 100); }
int ICM20948_Detect(void)
{
  uint8_t who = 0;
  if (icm_read_raw(ICM_ADDR_68, WHO_AM_I, &who, 1) == HAL_OK && who == WHO_AM_I_VAL) { ICM_ADDR = ICM_ADDR_68; return 0; }
  if (icm_read_raw(ICM_ADDR_69, WHO_AM_I, &who, 1) == HAL_OK && who == WHO_AM_I_VAL) { ICM_ADDR = ICM_ADDR_69; return 0; }
  return -1;
}
int ICM20948_Init(void)
{
  uint8_t whoami; ICM20948_ReadRegs(0, WHO_AM_I, &whoami, 1);
  if (whoami != 0xEA) return -1;
  ICM20948_WriteReg(0, 0x06, 0x80); HAL_Delay(100);
  ICM20948_WriteReg(0, 0x06, 0x01);
  ICM20948_WriteReg(0, 0x07, 0x00); ICM20948_WriteReg(0, 0x05, 0x00);
  ICM20948_WriteReg(2, 0x14, 0x00); ICM20948_WriteReg(2, 0x01, 0x00);
  return 0;
}
void ICM20948_ReadRaw(int16_t *ax, int16_t *ay, int16_t *az,
                      int16_t *gx, int16_t *gy, int16_t *gz)
{
  uint8_t d[12]; ICM20948_ReadRegs(0, 0x2D, d, 12);
  *ax = (d[0] << 8) | d[1]; *ay = (d[2] << 8) | d[3]; *az = (d[4] << 8) | d[5];
  *gx = (d[6] << 8) | d[7]; *gy = (d[8] << 8) | d[9]; *gz = (d[10] << 8) | d[11];
}

/* ---------------- Servo helpers ---------------- */
static inline void Servo_WriteUS(uint16_t us)
{
  if (us < 500)
    us = 500;
  if (us > 2500)
    us = 2500;
  __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, us);
}
uint16_t Steering_ToUS(int16_t steer_angle)
{
    if (steer_angle < -45) steer_angle = -45;
    if (steer_angle >  45) steer_angle =  45;
    int32_t us = 1150 + (int32_t)steer_angle * ((2400 - 500) / 90);
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, (uint16_t)us);
    return (uint16_t)us;
}
uint16_t Servo_SetAngle_Safe(int16_t angle_deg, uint8_t gradual)
{
    static int16_t current_angle = 0;
    if (angle_deg < -45) angle_deg = -45;
    if (angle_deg > 45)  angle_deg = 45;
    if (gradual) {
        int16_t step = (angle_deg > current_angle) ? 1 : -1;
        while (current_angle != angle_deg) {
            current_angle += step;
            Steering_ToUS(current_angle);
            HAL_Delay(20);
        }
    } else { current_angle = angle_deg; Steering_ToUS(angle_deg); }
    return Steering_ToUS(current_angle);
}

/* ---------------- Motor drive primitives ---------------- */
void MotorDrive_enable(void)
{
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}
void Motor_stop(void)
{
  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 0);
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 0);
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 0);
}
void Motor_forward(int pwmVal) {
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwmVal);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 0);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 0);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, pwmVal);
}
void Motor_reverse(int pwmVal)
{
  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, pwmVal);
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, pwmVal);
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 0);
}

/* ---------------- Legacy command processor ---------------- */
void process_command(char *cmd) {
    if (strncmp(cmd, "motor_forward(", 14) == 0) {
        int d = atoi(cmd + 14); Motor_forward(d);
    } else if (strncmp(cmd, "motor_reverse(", 14) == 0) {
        int d = atoi(cmd + 14); Motor_reverse(d);
    } else if (strncmp(cmd, "servo_us(", 9) == 0) {
        int us = atoi(cmd + 9); Servo_WriteUS((uint16_t)us);
    } else if (strncmp(cmd, "servo_deg(", 10) == 0) {
        int deg = atoi(cmd + 10); Steering_ToUS(deg);
    } else if (strncmp(cmd, "stop", 4) == 0) {
        Motor_stop();
    }
    // ACK over USART3 (now the command port)
    char msg[64]; snprintf(msg, sizeof(msg), "Executed: %s\r\n", cmd);
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

/* ---------------- HCSR04 (Ultrasonic) ---------------- */
uint32_t HCSR04_Read(void)
{
    uint32_t start_tick, stop_tick, pulse_length;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    for (volatile int i = 0; i < 300; i++);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    uint32_t t0 = HAL_GetTick();
    while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == GPIO_PIN_RESET) {
      if (HAL_GetTick() - t0 > 20) return 0;
    }
    start_tick = DWT->CYCCNT;
    while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == GPIO_PIN_SET) {
      if (HAL_GetTick() - t0 > 50) break;
    }
    stop_tick = DWT->CYCCNT;
    pulse_length = stop_tick - start_tick;
    uint32_t time_us = pulse_length / (SystemCoreClock / 1000000);
    return (uint32_t)((time_us * 343) / 20000);
}

/* ---------------- Heading integration + Rotate_Angle ---------------- */
float yaw_angle = 0;   uint32_t last_time_ms = 0;
void Update_Yaw(void)
{
    static float gz_filtered = 0.0f; const float alpha = 0.8f;
    uint32_t now = HAL_GetTick();
    float dt = (now - last_time_ms) / 1000.0f; if (dt <= 0) dt = 0.001f; last_time_ms = now;
    int16_t ax, ay, az, gx, gy, gz; ICM20948_ReadRaw(&ax, &ay, &az, &gx, &gy, &gz);
    float gz_dps = gz / 131.0f;
    gz_filtered = alpha * gz_filtered + (1.0f - alpha) * gz_dps;
    yaw_angle += gz_filtered * dt;
}
void Rotate_Angle(float target_deg, int pwmVal, int steer_angle)
{
    if (steer_angle < -45) steer_angle = -45; if (steer_angle > 45) steer_angle = 45;
    yaw_angle = 0; last_time_ms = HAL_GetTick();
    float gyro_bias = 0;
    for(int i = 0; i < 10; i++) { int16_t ax, ay, az, gx, gy, gz;
        ICM20948_ReadRaw(&ax,&ay,&az,&gx,&gy,&gz); gyro_bias += gz / 131.0f; HAL_Delay(10); }
    gyro_bias /= 10.0f;
    Servo_SetAngle_Safe(steer_angle, 1); HAL_Delay(100);
    uint32_t t0 = HAL_GetTick(); const uint32_t timeout_ms = 5000;
    while (fabsf(yaw_angle) < fabsf(target_deg)) {
        if (HAL_GetTick() - t0 > timeout_ms) break;
        Update_Yaw();
        yaw_angle -= gyro_bias * (HAL_GetTick() - last_time_ms) / 1000.0f;
        float prog = fabsf(yaw_angle) / fabsf(target_deg);
        int current_pwm = pwmVal;
        if (prog > 0.7f) current_pwm = (int)(pwmVal * 0.5f);
        if (prog > 0.9f) current_pwm = (int)(pwmVal * 0.3f);
        Motor_forward(current_pwm);
        if (HCSR04_Read() <= 15) { Motor_stop(); break; }
        HAL_Delay(10);
    }
    Motor_stop();
    Servo_WriteUS(1250);
}

/* ---------------- Straight drive ---------------- */
void Drive_Straight_ToCM(float target_cm, int base_pwm) {
  reset_encoders();
  Servo_WriteUS(1250);
  const float STOP_TOL_CM = fmaxf(2.0f, fabsf(target_cm) * 0.06f);
  while (1) {
    float cm_now  = cm_travelled();
    float cm_left = fabsf(target_cm) - fabsf(cm_now);
    if (cm_left <= STOP_TOL_CM) break;
    int pwm = base_pwm;
    if (cm_left <= 3.0f)  pwm = (int)(base_pwm * 0.25f);
    else if (cm_left <= 10.0f) pwm = (int)(base_pwm * 0.35f);
    else if (cm_left <= 30.0f) pwm = (int)(base_pwm * 0.60f);
    if (pwm < pwmMin) pwm = pwmMin;
    Motor_forward(pwm);
    HAL_Delay(10);
  }
  Motor_stop();
}

void Drive_Forward_Until_Obstacle(int base_pwm, uint32_t obstacle_threshold_cm)
{
    // Safety check on parameters
    if (base_pwm < pwmMin) base_pwm = pwmMin;
    if (base_pwm > pwmMax) base_pwm = pwmMax;
    if (obstacle_threshold_cm < 5) obstacle_threshold_cm = 5;   // Minimum 5cm safety
    if (obstacle_threshold_cm > 50) obstacle_threshold_cm = 50; // Maximum 50cm range

    reset_encoders();

    // Display initial status
    sprintf(buf, "Moving forward...");
    OLED_ShowString(0, 10, (uint8_t*)buf);
    OLED_Refresh_Gram();

    while (1) {
        // Read distance from ultrasonic sensor
        uint32_t distance = HCSR04_Read();

        // Display current distance
        sprintf(buf, "Dist: %lu cm", distance);
        OLED_ShowString(0, 20, (uint8_t*)buf);
        OLED_Refresh_Gram();

        // Check if obstacle is detected
        if (distance <= obstacle_threshold_cm) {
            // Stop immediately when obstacle detected
            Motor_stop();

            // Provide feedback
            sprintf(buf, "Obstacle at %lu cm!", distance);
            OLED_ShowString(0, 30, (uint8_t*)buf);
            HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, GPIO_PIN_SET); // Alert buzzer
            OLED_Refresh_Gram();

            HAL_Delay(500); // Brief pause
            HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, GPIO_PIN_RESET); // Turn off buzzer
            break;
        }

        // Continue moving forward
        Motor_forward(base_pwm);

        // Small delay for sensor reading stability
        HAL_Delay(50);
    }
}



/* ---------------- RTOS flag helpers ---------------- */
static inline void Request_Stop(void){ osEventFlagsSet(evFlags, EVF_STOP); }
static inline void Clear_Stop(void){   osEventFlagsClear(evFlags, EVF_STOP); }
static inline bool Stop_Requested(void){ return (osEventFlagsGet(evFlags) & EVF_STOP) != 0; }
static bool US_Near(uint32_t cm_threshold) {
    if (cm_threshold == 0) return false;
    uint32_t d = HCSR04_Read();
    return (d > 0 && d <= cm_threshold);
}

/* ---------------- Abort-aware wrappers ---------------- */
static void Straight_Abortable(float target_cm, int base_pwm, uint32_t near_cm) {
    Clear_Stop();
    Servo_WriteUS(1250);
    const float step = (target_cm >= 0) ? +5.0f : -5.0f;
    float done = 0.0f;
    while (fabsf(target_cm - done) > 0.5f) {
        if (Stop_Requested() || US_Near(near_cm)) { Motor_stop(); break; }
        float remain = target_cm - done;
        float this_leg = fabsf(remain) < fabsf(step) ? remain : step;
        Drive_Straight_ToCM(this_leg, base_pwm);
        done += this_leg;
    }
}
static void Turn_Abortable(float deg, int pwm, int steer) {
    Clear_Stop();
    if (Stop_Requested()) return;
    Rotate_Angle(deg, pwm, steer);
}

//turn helper functions

// --- Fixed 90° turns using your arc-turn primitive ---
static inline void TurnLeft90(void)  { Turn_Abortable(90.0f, 2500, -40); }
static inline void TurnRight90(void) { Turn_Abortable(90.0f, 2500, +40); }

/* ---------------- Go-around sequences (Week 9) ---------------- */
static void Approach_Stop(uint32_t stop_cm, int base_pwm) {
  while (!Stop_Requested() && !US_Near(stop_cm)) {
    Drive_Straight_ToCM(+5.0f, base_pwm);
    if (US_Near(stop_cm)) break;
  }
  Motor_stop();
}
// Wrap a ~10×10 cm cube and come back to the original heading/centerline.
// Sequence: detect -> STOP -> reverse a bit -> Corner A -> leg1 -> Corner B -> leg2 (alongside) ->
//           Corner C -> leg3 (behind) -> Corner D -> back on lane
static void GoAround_Small(bool left)
{
  const int PWM = 2300;
  const uint32_t STOP_NEAR_CM = 22;    // when we “see” the block (front US)
  const float CLEAR_FRONT_CM   = 12.0f; // clear the front edge before aligning
  const float SIDE_RUN_CM      = 20.0f; // run alongside (10 cm + margin)
  const float BACK_RUN_CM      = 18.0f; // pass behind (10 cm + margin)
  const float EXIT_CM          = 10.0f; // small straight to settle back on lane

  // 0) Approach until the obstacle is near, then hard stop.
  Drive_Forward_Until_Obstacle(3000, 25);
  Motor_stop();
  Motor_reverse(1200);
  HAL_Delay(1000);
  Motor_stop();


//  // 1) Create some clearance: back up a little so the first turn won’t clip.
//  Straight_Abortable(-10.0f, PWM, 0);

  // 2) Four-corner “box” around the object.
  if (left) {
//    // Corner A: turn left 90°, head outward
//    //TurnLeft90();
//             Straight_Abortable(CLEAR_FRONT_CM, PWM, 14);
//
//    // Corner B: turn right 90°, align to run alongside the object’s side
//    TurnRight90();
//    Straight_Abortable(SIDE_RUN_CM, PWM, 14);
//
//    // Corner C: turn right 90°, go behind the object
//    TurnRight90();
//    Straight_Abortable(BACK_RUN_CM, PWM, 14);
//
//    // Corner D: turn left 90°, we’re back to original heading
//    TurnLeft90();

	          //Motor_stop();
	         // HAL_Delay(200);
	          Rotate_Angle(-90.0f, PWM, -29); // 90° right turn - 29 before
	          HAL_Delay(1000);
	          Motor_stop();
	          Motor_reverse(1200);
	          HAL_Delay(1000);
	          Motor_stop();
	          Rotate_Angle(180.0f, (PWM+150) , 34);
	          HAL_Delay(1000);
	          Motor_stop();
	          Motor_reverse(1200);
	          HAL_Delay(1000);
	          Rotate_Angle(90.0f, (PWM) , -29);
	          HAL_Delay(1000);
	          Motor_stop();



  } else {
    // Mirrored path around the right side
	  Rotate_Angle(90.0f, PWM, 29); // 90° right turn - 29 before
	  	          HAL_Delay(1000);
	  	          Motor_stop();
	  	          Motor_reverse(1200);
	  	          HAL_Delay(1000);
	  	          Motor_stop();
	  	          Rotate_Angle(-180.0f, (PWM+150) , -34);
	  	          HAL_Delay(1000);
	  	          Motor_stop();
	  	          Motor_reverse(600);
	  	          HAL_Delay(1000);
	  	          Rotate_Angle(90.0f, (PWM) , 29);
	  	          HAL_Delay(1000);
	  	          Motor_stop();
  }

  // 3) Settle back onto the original center axis.
  //Straight_Abortable(EXIT_CM, PWM, 0);
  Servo_WriteUS(1250);
  distobs2 += HCSR04_Read();
  char msg[10];
  snprintf(msg, sizeof(msg), "distance: %s\r\n", distobs2);
  HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

  Motor_stop();

}

// ===== Hardcoded big-obstacle go-around (70 cm wide, no IR) =====
// Conventions:
//  - left==true  → pass via LEFT of obstacle (counter-clockwise), keep box on your RIGHT
//  - left==false → pass via RIGHT (clockwise), keep box on your LEFT
static void GoAround_Large(bool left)
{
  // Keep these identical to your small-obstacle "good" values
  const int   PWM             = 2350;     // base straight/turn power
  const int   STEER_L         = -29;      // your calibrated left steer
  const int   STEER_R         = +34;      // your calibrated right steer

  // Distances (tune on floor if needed)
  const float BACKUP_CM       = 14.0f;    // create clearance before first corner
  const float CLEAR_FRONT_CM  = 18.0f;    // move outward a bit after 1st turn
  const float WIDTH_CM        = 70.0f;    // given: obstacle width
  const float SIDE_MARGIN_CM  = 5.0f;    // extra to clear corners
  const float SIDE_RUN_CM     = WIDTH_CM + SIDE_MARGIN_CM;  // ~80 cm
  const float BACK_DEPTH_CM   = 60.0f;    // assumed obstacle depth (tune 30–50)
  const float EXIT_SETTLE_CM  = 10.0f;    // tiny move to settle after last corner

  // 0) Approach until you see the big obstacle, then stop & back off a touch
  uint32_t distance = HCSR04_Read();
  if(distance>30){
	  Drive_Forward_Until_Obstacle(3000, 28);   // or Approach_Stop(28, PWM);
	    Motor_stop();
  }else{
	  Motor_reverse(1200);
	  HAL_Delay(500);
  }
//  Drive_Forward_Until_Obstacle(3000, 28);   // or Approach_Stop(28, PWM);
//  Motor_stop();

  Motor_reverse(1200); HAL_Delay(800); Motor_stop();

  if (left) {
    // CCW around: obstacle stays on your RIGHT on long legs
    // Corner A: turn LEFT 90°, then clear the front face
    Rotate_Angle(+90.0f, PWM, STEER_L);  HAL_Delay(200); Motor_stop();
    Straight_Abortable(CLEAR_FRONT_CM, PWM, 14); Motor_stop();

    // Corner B: turn RIGHT 90°, align to left side; run along the side (width)
    Rotate_Angle(+90.0f, PWM, STEER_R);  HAL_Delay(200); Motor_stop();
    Straight_Abortable(SIDE_RUN_CM, PWM, 14);   Motor_stop();

    // Corner C: turn RIGHT 90°, go behind the obstacle (depth)
    Rotate_Angle(+90.0f, PWM, STEER_R);  HAL_Delay(200); Motor_stop();
    Straight_Abortable(BACK_DEPTH_CM, PWM, 14); Motor_stop();

    // Corner D: turn RIGHT 90°, run along opposite side (width)
    Rotate_Angle(+90.0f, PWM, STEER_R);  HAL_Delay(200); Motor_stop();
    Straight_Abortable(SIDE_RUN_CM, PWM, 14);   Motor_stop();

    // Now we’re at the far front edge; turn around to face “home”
    Rotate_Angle(+180.0f, PWM, STEER_R); HAL_Delay(250); Motor_stop();

  } else {
    // CW around: obstacle stays on your LEFT on long legs
    // Corner A: turn RIGHT 90°, then clear the front face
    Rotate_Angle(+90.0f, PWM, STEER_R);  HAL_Delay(200); Motor_stop();
    Straight_Abortable(CLEAR_FRONT_CM, PWM, 14); Motor_stop();

    // Corner B: turn LEFT 90°, align to right side; run along the side (width)
    Rotate_Angle(+90.0f, PWM, STEER_L);  HAL_Delay(200); Motor_stop();
    Straight_Abortable(SIDE_RUN_CM, PWM, 14);   Motor_stop();

    // Corner C: turn LEFT 90°, go behind the obstacle (depth)
    Rotate_Angle(+90.0f, PWM, STEER_L);  HAL_Delay(200); Motor_stop();
    Straight_Abortable(BACK_DEPTH_CM, PWM, 14); Motor_stop();

    // Corner D: turn LEFT 90°, run along opposite side (width)
    Rotate_Angle(+90.0f, PWM, STEER_L);  HAL_Delay(200); Motor_stop();
    Straight_Abortable(SIDE_RUN_CM, PWM, 14);   Motor_stop();

    // Turn around to face “home”
    Rotate_Angle(+180.0f, PWM, STEER_L); HAL_Delay(250); Motor_stop();
  }

  // Small settle move (optional), then centre steering and stop
  Straight_Abortable(EXIT_SETTLE_CM, PWM, 0);
  Servo_WriteUS(1250);
  Motor_stop();
}


//DISPLAY FUNCT

/* Single owner of the OLED: renders the whole HUD periodically */
void StartTaskDisplay(void *argument)
{
  // Initial clear once RTOS is running
  OLED_Clear();
  for(;;) {
    char line[32];

    // Line 0: Ultrasonic
    OLED_ShowString(0, 0,  (uint8_t*)"                ");
    snprintf(line, sizeof(line), "US: %3lu cm", (unsigned long)g_us_cm);
    OLED_ShowString(0, 0, (uint8_t*)line);

    // Line 10: Last command
    OLED_ShowString(0, 10, (uint8_t*)"                ");
    snprintf(line, sizeof(line), "CMD: %s", g_last_cmd);
    OLED_ShowString(0, 10, (uint8_t*)line);

    // Line 20: IR Left (mm)
    OLED_ShowString(0, 20, (uint8_t*)"                ");
    snprintf(line, sizeof(line), "IRL:%3u mm", (unsigned)ir_left_mm);
    OLED_ShowString(0, 20, (uint8_t*)line);

    // Line 30: IR Right (mm)
    OLED_ShowString(0, 30, (uint8_t*)"                ");
    snprintf(line, sizeof(line), "IRR:%3u mm", (unsigned)ir_right_mm);
    OLED_ShowString(0, 30, (uint8_t*)line);

    OLED_Refresh_Gram();
    osDelay(100); // ~10 Hz HUD refresh
  }
}

  //mission function (the whole run hardcoded)

static void Mission_Run(void *argument)
{
  // 0) Prep: wheels on, steer straightp
  Steering_ToUS(0);  // center
  Motor_stop();

  // 1) Exit parking: creep forward until we "see" free space, then go
  Straight_Abortable(+15.0f, 2200, 0);   // roll out a bit (no US stop)
  // 2) Move forward until ultrasonic sees the first obstacle
  //    (use a conservative threshold, e.g., 28 cm)
  while (!US_Near(32)) { Drive_Straight_ToCM(+5.0f, 2300); }

  // 3) Go around the ~10x10 cm obstacle and come back to center line
  //    (choose left or right based on what’s safer in your arena)
  GoAround_Small(true);   // left; use false for right

  // 4) Resume forward along center axis until the unknown obstacle
  while (!US_Near(28)) { Drive_Straight_ToCM(+5.0f, 2350); }

//  // 5) Decide to pass left or right using the two side IRs
//  //    (pseudo: read your filtered left/right ranges; pick the larger clearance)
//  bool goLeft = true; // TODO: replace with (ir_right_clearance > ir_left_clearance)
//  if (goLeft)  GoAround_Large(true);
//  else         GoAround_Large(false);
//
//  // 6) Head back near the initial parking “square bracket” and park inside
//  //    (straight approach then short S to align, tweak distances for your bay)
//  Straight_Abortable(+60.0f, 2300, 0);
//  Turn_Abortable(  90.0f, 2500, -40);   // face into bay (left example)
//  Straight_Abortable(+25.0f, 2000, 14); // creep in, stop if US < 14 cm
//  Motor_stop();
//  Steering_ToUS(0);

  // End of mission: you may stop the task or loop again
  for(;;) { osDelay(1000); }  // idle here
}

//IR FUNCTIONS
static uint16_t ADC1_ReadChannel(uint32_t channel)
{
  ADC_ChannelConfTypeDef s = {0};
  s.Channel = channel;
  s.Rank = 1;
  s.SamplingTime = ADC_SAMPLETIME_15CYCLES;  // a bit longer for accuracy
  HAL_ADC_ConfigChannel(&hadc1, &s);

  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 2);
  uint16_t v = (uint16_t)HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);
  return v;
}

/* Power-law fit: distance_mm ≈ A * ADC^B  (tune A,B per sensor) */
static inline float ir_adc_to_mm_left(uint16_t adc)
{
  const float A = 2.85e8f, B = -1.92f;      // placeholder fit
  float mm = A * powf((float)adc, B);
  if (mm < 40.f) mm = 40.f;                 // clamp to plausible range
  if (mm > 800.f) mm = 800.f;
  return mm;
}
static inline float ir_adc_to_mm_right(uint16_t adc)
{
  const float A = 1.10e9f, B = -2.06f;      // placeholder fit (your right curve)
  float mm = A * powf((float)adc, B);
  if (mm < 40.f) mm = 40.f;
  if (mm > 800.f) mm = 800.f;
  return mm;
}

/* Small moving average for robustness */
static inline uint16_t avg_u16(const uint16_t *a, int n)
{
  uint32_t s = 0; for (int i=0;i<n;i++) s += a[i];
  return (uint16_t)(s / (uint32_t)n);
}


/* ---------------- Prototypes for MX_* ---------------- */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM12_Init(void);
static void MX_ADC1_Init(void);

/* ---------------- Tasks ---------------- */
void StartDefaultTask(void *argument)
{
  for(;;) { osDelay(1); }
}
void StartTaskObstacle(void *argument)
{
  const uint32_t THRESH_CM = 18;
  for(;;) {
    uint32_t d = HCSR04_Read();
    // Update event flag
    if (d > 0 && d <= THRESH_CM) osEventFlagsSet(evFlags, EVF_OBS_NEAR);
    else                         osEventFlagsClear(evFlags, EVF_OBS_NEAR);
    // --- OLED: show ultrasonic distance on line 0 ---
    OLED_ShowString(0, 0, (uint8_t*)"                "); // clear line
    snprintf(buf, sizeof(buf), "US: %3lu cm", (unsigned long)d);
    OLED_ShowString(0, 0, (uint8_t*)buf);
    // keep last command on line 10 (updated by move task)
    OLED_Refresh_Gram();
    osDelay(100);
  }
}
//void StartTaskMsg(void *argument)
//{
//  // NOTE: USART3 is now the command port (BT/host).
//  // To avoid TX/RX contention with printf, _write() below is redirected to USART2.
//  char line[16]; int idx = 0;
//  for(;;) {
//    uint8_t ch;
//    if (HAL_UART_Receive(&huart3, &ch, 1, 20) == HAL_OK) {   // <-- USART3 here
//      if (ch == '\r' || ch == '\n') {
//        if (idx > 0) { line[idx] = '\0';
//          CmdMsg_t m = {0}; strncpy(m.text, line, sizeof(m.text)-1);
//          osMessageQueuePut(qCmd, &m, 0, 0);
//          idx = 0;
//        }
//      } else if (idx < (int)sizeof(line)-1) {
//        line[idx++] = (char)ch;
//      } else { idx = 0; }
//    } else {
//      osDelay(1);
//    }
//  }
//}
void StartTaskMsg(void *argument)
{
  // Small ring buffer for a single line; accepts CR, LF, CRLF; supports backspace.
  char line[32]; int idx = 0;

  for (;;)
  {
    uint8_t ch;
    // Poll one byte; short timeout so we can yield
    if (HAL_UART_Receive(&huart3, &ch, 1, 10) == HAL_OK)
    {
      // Visual heartbeat: toggle LED on every byte (optional)
      HAL_GPIO_TogglePin(GPIOA, LED_Pin);

      // Echo back so you see what arrives (diagnostic; keep during bring-up)
      HAL_UART_Transmit(&huart3, &ch, 1, 10);

      if (ch == '\r' || ch == '\n')
      {
        if (idx > 0)
        {
          line[idx] = '\0';

          // push to queue
          CmdMsg_t m = {0};
          strncpy(m.text, line, sizeof(m.text) - 1);
          osStatus_t st = osMessageQueuePut(qCmd, &m, 0, 0);

          // report status
          const char *ok = (st == osOK) ? "\r\nACK:QUEUED\r\n" : "\r\nERR:QUEUE\r\n";
          HAL_UART_Transmit(&huart3, (uint8_t*)ok, strlen(ok), HAL_MAX_DELAY);

          idx = 0;
        }
        // swallow extra LF after CR (CRLF) automatically; no else
      }
      else if (ch == 0x7F || ch == 0x08) // backspace
      {
        if (idx > 0) idx--;
      }
      else
      {
        // Accept only visible ASCII to keep things simple
        if (ch >= 32 && ch <= 126)
        {
          if (idx < (int)sizeof(line) - 1)
          {
            line[idx++] = (char)ch;
          }
          else
          {
            // overflow: reset line, tell the user
            idx = 0;
            const char *ovf = "\r\nERR:LINE_TOO_LONG\r\n";
            HAL_UART_Transmit(&huart3, (uint8_t*)ovf, strlen(ovf), HAL_MAX_DELAY);
          }
        }
      }
    }
    else
    {
      // time slice so we don't busy-wait
      osDelay(1);
    }
  }
}

void StartTaskIR(void *argument)
{
  /* Oversample a bit, IIR smooth, publish mm + draw on OLED */
  const float alpha = 0.25f;   // IIR smoothing (0..1), lower = smoother
  uint16_t bufL[8], bufR[8];

  for (;;)
  {
    // --- sample left ---
    for (int i=0;i<8;i++) bufL[i] = ADC1_ReadChannel(IR_LEFT_CH);
    uint16_t l_adc = avg_u16(bufL, 8);
    float l_mm_f = ir_adc_to_mm_left(l_adc);
    irL_f = alpha * l_mm_f + (1.f - alpha) * irL_f;
    ir_left_adc = l_adc;
    ir_left_mm  = (uint16_t)(irL_f + 0.5f);

    // --- sample right ---
    for (int i=0;i<8;i++) bufR[i] = ADC1_ReadChannel(IR_RIGHT_CH);
    uint16_t r_adc = avg_u16(bufR, 8);
    float r_mm_f = ir_adc_to_mm_right(r_adc);
    irR_f = alpha * r_mm_f + (1.f - alpha) * irR_f;
    ir_right_adc = r_adc;
    ir_right_mm  = (uint16_t)(irR_f + 0.5f);

    // --- HUD on OLED (put it under your US/CMD lines) ---
    // Lines used in your build: 0 = US, 10 = CMD. Use line 20/30 for IR.
    OLED_ShowString(0, 20, (uint8_t*)"                ");
    snprintf(buf, sizeof(buf), "IRL:%3umm", (unsigned)ir_left_mm);
    OLED_ShowString(0, 20, (uint8_t*)buf);

    OLED_ShowString(0, 30, (uint8_t*)"                ");
    snprintf(buf, sizeof(buf), "IRR:%3umm", (unsigned)ir_right_mm);
    OLED_ShowString(0, 30, (uint8_t*)buf);

    OLED_Refresh_Gram();

    osDelay(25); // ~40 Hz update
  }
}
void StartTaskMove(void *argument)
{
  CmdMsg_t m;
  for(;;) {
    if (osMessageQueueGet(qCmd, &m, NULL, osWaitForever) == osOK) {
      // normalize to uppercase
      for (char *p=m.text; *p; ++p) if (*p>='a'&&*p<='z') *p = (char)(*p - 32);
      // store & show last command on OLED line 10
      char ack[32];
      snprintf(ack, sizeof(ack), "RCV:%s\r\n", m.text);
      HAL_UART_Transmit(&huart3, (uint8_t*)ack, strlen(ack), HAL_MAX_DELAY);
      strncpy(g_last_cmd, m.text, sizeof(g_last_cmd)-1);
//      OLED_ShowString(0, 10, (uint8_t*)"                "); // clear line
//      snprintf(buf, sizeof(buf), "CMD: %s", g_last_cmd);
//      OLED_ShowString(0, 10, (uint8_t*)buf);
//      OLED_Refresh_Gram();

      if (strcmp(m.text, "STOP") == 0) {
        Request_Stop(); Motor_stop();
        HAL_UART_Transmit(&huart3,(uint8_t*)"ACK:STOP\r\n",10,HAL_MAX_DELAY); // USART3 ACK
        continue;
      }
      Clear_Stop();

      if (strncmp(m.text, "FW", 2) == 0) {
        int cm = atoi(m.text+2); Straight_Abortable((float)cm, 2300, 14);
        HAL_UART_Transmit(&huart3,(uint8_t*)"DONE:FW\r\n",9,HAL_MAX_DELAY);
        continue;
      }
      if (strncmp(m.text, "BW", 2) == 0) {
        int cm = atoi(m.text+2); Straight_Abortable(-(float)cm, 2300, 14);
        HAL_UART_Transmit(&huart3,(uint8_t*)"DONE:BW\r\n",9,HAL_MAX_DELAY);
        continue;
      }

      if (strcmp(m.text, "OB01") == 0) { Approach_Stop(22, 2300);
        HAL_UART_Transmit(&huart3,(uint8_t*)"DONE:OB01\r\n",11,HAL_MAX_DELAY); continue; }
      if (strcmp(m.text, "UL00") == 0) { GoAround_Small(true);
        HAL_UART_Transmit(&huart3,(uint8_t*)"DONE:UL00\r\n",11,HAL_MAX_DELAY); continue; }
      if (strcmp(m.text, "UR00") == 0) { GoAround_Small(false);
        HAL_UART_Transmit(&huart3,(uint8_t*)"DONE:UR00\r\n",11,HAL_MAX_DELAY); continue; }
      if (strcmp(m.text, "PL01") == 0) { GoAround_Large(true);
        HAL_UART_Transmit(&huart3,(uint8_t*)"DONE:PL01\r\n",11,HAL_MAX_DELAY); continue; }
      if (strcmp(m.text, "PR01") == 0) { GoAround_Large(false);
        HAL_UART_Transmit(&huart3,(uint8_t*)"DONE:PR01\r\n",11,HAL_MAX_DELAY); continue; }

      // Fallback to legacy command parser for ad-hoc cmds
      process_command(m.text);
    }
  }
}

/* ---------------- HAL callbacks ---------------- */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  uint32_t counter = __HAL_TIM_GET_COUNTER(htim);
  int16_t count = (int16_t)counter;
  position = count / 2.0f;
  angle = count / 2;
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == USER_PB_Pin) {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
    if (start == 0) {
      start = 1;
      TIM2->CNT = 0; speed = 0; position = 0; angle = 0; pwmVal = 0;
    } else start = 0;
  }
}

/* ---------------- printf redirection ---------------- */
/* NOTE: USART3 is the command port now. To avoid contention with command I/O,
   printf is redirected to USART2 here. If you prefer no prints, comment _write(). */
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
  for (int i = 0; i < len; i++) ITM_SendChar(ptr[i]);
  return len;
}

/* ---------------- Prototypes for MX_* ---------------- */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM12_Init(void);
static void MX_ADC1_Init(void);

/* ---------------- Main ---------------- */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();   // prints
  MX_TIM1_Init();
  MX_USART3_UART_Init();   // commands
  MX_I2C2_Init();
  MX_TIM5_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_TIM11_Init();
  MX_TIM12_Init();
  MX_ADC1_Init();

  MotorDrive_enable();
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

  OLED_Init();
  OLED_Clear();
  OLED_ShowString(0, 0, (uint8_t*)"US: --- cm");
  OLED_ShowString(0, 10,(uint8_t*)"CMD: —");
  OLED_Refresh_Gram();

  if (ICM20948_Detect() == 0) { sprintf(buf, "ICM @0x%02X", (unsigned)(ICM_ADDR >> 1)); }
  else                         { sprintf(buf, "ICM NOT FOUND"); }
  OLED_ShowString(0, 20, (uint8_t*)buf); OLED_Refresh_Gram(); HAL_Delay(300);
  if (ICM20948_Init() == 0) sprintf(buf, "ICM OK"); else sprintf(buf, "ICM FAIL");
  OLED_ShowString(0, 30, (uint8_t*)buf); OLED_Refresh_Gram(); HAL_Delay(300);

  // Enable DWT for cycle counter (HCSR04 timing)
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0; DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  // RTOS kernel
  Servo_WriteUS(1250);
  osKernelInitialize();
  // after osKernelInitialize()
  //oledMutex = osMutexNew(NULL);
  evFlags = osEventFlagsNew(NULL);
  qCmd    = osMessageQueueNew(8, sizeof(CmdMsg_t), NULL);

  //GoAround_Small(true);
  //to do the missionnnn
  //missionHandle = osThreadNew(Mission_Run, NULL, &mission_attributes);

  defaultTaskHandle     = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  obstacle_detectHandle = osThreadNew(StartTaskObstacle, NULL, &obstacle_detect_attributes);
  message_receiveHandle = osThreadNew(StartTaskMsg, NULL, &message_receive_attributes);
  ir_detectHandle       = osThreadNew(StartTaskIR, NULL, &ir_detect_attributes);
  move_controlHandle    = osThreadNew(StartTaskMove, NULL, &move_control_attributes);
  //missionHandle = osThreadNew(Mission_Run, NULL, &mission_attributes);
  //displayHandle = osThreadNew(StartTaskDisplay, NULL, &display_attributes);



  osKernelStart();

  while (1) { }
}

/* ---------------- CubeMX init fns (same as your project) ---------------- */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
}
static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) { Error_Handler(); }
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }
}
static void MX_I2C2_Init(void)
{
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK) { Error_Handler(); }
}
static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 7199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK) { Error_Handler(); }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) { Error_Handler(); }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) { Error_Handler(); }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) { Error_Handler(); }
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) { Error_Handler(); }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) { Error_Handler(); }
  HAL_TIM_MspPostInit(&htim1);
}
static void MX_TIM2_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
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
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) { Error_Handler(); }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) { Error_Handler(); }
}
static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 720;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK) { Error_Handler(); }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) { Error_Handler(); }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) { Error_Handler(); }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) { Error_Handler(); }
  HAL_TIM_MspPostInit(&htim3);
}
static void MX_TIM4_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 7199;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK) { Error_Handler(); }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) { Error_Handler(); }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) { Error_Handler(); }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) { Error_Handler(); }
  HAL_TIM_MspPostInit(&htim4);
}
static void MX_TIM5_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65535;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK) { Error_Handler(); }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK) { Error_Handler(); }
}
static void MX_TIM8_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 7199;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK) { Error_Handler(); }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK) { Error_Handler(); }
}
static void MX_TIM11_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 0;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 7199;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_Init(&htim11) != HAL_OK) { Error_Handler(); }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
}
static void MX_TIM12_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 83;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 19999;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK) { Error_Handler(); }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) { Error_Handler(); }
  HAL_TIM_MspPostInit(&htim12);
}
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;  // for prints
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) { Error_Handler(); }
}
static void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;    // command link
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK) { Error_Handler(); }
}
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOD, OLED4_Pin|OLED3_Pin|OLED2_Pin|OLED1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, Buzzer_Pin|LED_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = OLED4_Pin|OLED3_Pin|OLED2_Pin|OLED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = Buzzer_Pin|LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = USER_PB_Pin|IMU_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  // Ultrasonic: TRIG PB14
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  // Ultrasonic: ECHO PC9
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}
void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { (void)file; (void)line; }
#endif
