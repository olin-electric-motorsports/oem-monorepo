#include "throttle.h"
#include "can_api.h"

static void GpioInit(void);

int main(void) {
  HAL_Init();
  SystemClockConfig();
  GpioInit();
  if (can_init_throttle() != 0) {
        
        while (1) {}
    }

    uint8_t dummy_data = 0;

  while (1) {
        throttle.test_counter = dummy_data; 



        if (bspd.brake_gate == 1) {
          if (can_send_throttle() == 0) {
            dummy_data++; 
          }
        } 


        HAL_Delay(100);
    }
  return 0;
}

