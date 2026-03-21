#include "throttle.h"
#include "can_api.h"

static void GpioInit(void);

int main(void) {
    HAL_Init();
    SystemClockConfig();
    GpioInit(); // Ensure this is still called 

    // Turn LED OFF initially 
    HAL_GPIO_WritePin(PA1_GPIO_Port, PA1_Pin, GPIO_PIN_RESET);

    // Check Initialization
    if (can_init_throttle() != 0) {
        // If initialization fails, turn the LED ON SOLID.
        HAL_GPIO_WritePin(PA1_GPIO_Port, PA1_Pin, GPIO_PIN_SET);
        while (1) {}
    }

    uint8_t dummy_data = 0;
    uint8_t bspd_rx_count = 0;

    while (1) {
        can_poll_receive_all(); 

        if (bspd.brake_gate == BRAKE_GATE_BRAKE_LIGHT_ON) { 
            bspd_rx_count++; 
        }

        throttle.test_counter = dummy_data; 
        throttle.test_counter_bspd = bspd_rx_count; 
        
        Check Transmission
        if (can_send_throttle() == 0) {
            dummy_data++; 
      
        }
        HAL_GPIO_TogglePin(PA1_GPIO_Port, PA1_Pin);
        HAL_Delay(100);
    }
    
    return 0;
}

