#include "cmsis_os.h"
#include "io/led/led.hpp"

sp::LED led(&htim5);

extern "C" void led_task()
{
  led.start();

  // --- 流水灯参数定义 ---
  const uint8_t STEPS = 50;
  const float INCREMENT = 1.0f / (float)STEPS;
  const uint32_t DELAY_MS = 10;

  // Initial Color: Red (100% R, 0% G, 0% B)
  float r = 1.0f;
  float g = 0.0f;
  float b = 0.0f;

  while (true) {
    // 1. Red -> Yellow (Add Green)
    for (uint8_t i = 0; i < STEPS; i++) {
      g += INCREMENT;
      led.set(r, g, b);
      osDelay(DELAY_MS);
    }

    // 2. Yellow -> Green (Subtract Red)
    for (uint8_t i = 0; i < STEPS; i++) {
      r -= INCREMENT;
      led.set(r, g, b);
      osDelay(DELAY_MS);
    }

    // 3. Green -> Cyan (Add Blue)
    for (uint8_t i = 0; i < STEPS; i++) {
      b += INCREMENT;
      led.set(r, g, b);
      osDelay(DELAY_MS);
    }

    // 4. Cyan -> Blue (Subtract Green)
    for (uint8_t i = 0; i < STEPS; i++) {
      g -= INCREMENT;
      led.set(r, g, b);
      osDelay(DELAY_MS);
    }

    // 5. Blue -> Magenta (Add Red)
    for (uint8_t i = 0; i < STEPS; i++) {
      r += INCREMENT;
      led.set(r, g, b);
      osDelay(DELAY_MS);
    }

    // 6. Magenta -> Red (Subtract Blue)
    for (uint8_t i = 0; i < STEPS; i++) {
      b -= INCREMENT;
      led.set(r, g, b);
      osDelay(DELAY_MS);
    }
  }
}