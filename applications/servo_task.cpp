#include "cmsis_os.h"
#include "io/servo/servo.hpp"

sp::Servo servo(&htim1, TIM_CHANNEL_1, 168e6f, 180.0f);

extern "C" void servo_task(void const * argument)
{
  // --- 1. 参数定义 ---
  const float START_ANGLE = 0.0f;
  const float END_ANGLE = 180.0f;
  const float SPEED_DEG_PER_SEC = 45.0f;

  const uint32_t UPDATE_INTERVAL_MS = 1;

  // --- 2. 运动时间计算 ---
  // 总角度 = 180.0 度
  // 总时长 = 总角度 / 速度 = 180.0 / 45.0 = 4.0 秒
  const uint32_t TOTAL_DURATION_MS =
    (uint32_t)((END_ANGLE - START_ANGLE) / SPEED_DEG_PER_SEC * 1000.0f);  // 4000 ms

  // 总步数 = 4000 ms / 5 ms = 800 步
  const uint32_t TOTAL_STEPS = TOTAL_DURATION_MS / UPDATE_INTERVAL_MS;

  // 每步增加的角度 = 180.0 度 / 800 步 = 0.225 度/步
  const float ANGLE_INCREMENT_PER_STEP = (END_ANGLE - START_ANGLE) / (float)TOTAL_STEPS;

  float current_angle = START_ANGLE;  // 初始角度

  // --- 3. 启动 PWM 输出 ---
  servo.start();

  servo.set(START_ANGLE);
  osDelay(100);

  // --- 4. 匀速转动序列 (0° -> 180°) ---
  for (uint32_t step = 1; step <= TOTAL_STEPS; step++) {
    current_angle = START_ANGLE + (ANGLE_INCREMENT_PER_STEP * (float)step);

    if (current_angle > END_ANGLE) {
      current_angle = END_ANGLE;
    }

    servo.set(current_angle);

    osDelay(UPDATE_INTERVAL_MS);

    // 确保最终停在 180.0 度
    servo.set(END_ANGLE);
  }

  while (true) {
    osDelay(100);
  }
}