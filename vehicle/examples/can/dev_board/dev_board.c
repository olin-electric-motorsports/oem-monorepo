#include "dev_board.h"
#include "can_api.h"

static void GpioInit(void);

int main(void) {
    HAL_Init();
    SystemClockConfig();
    GpioInit(); 

    HAL_GPIO_WritePin(PA1_GPIO_Port, PA1_Pin, GPIO_PIN_RESET);

    // Check Initialization
    if (can_init_dev_board() != 0) {
        // If initialization fails, turn the LED ON SOLID.
        HAL_GPIO_WritePin(PA1_GPIO_Port, PA1_Pin, GPIO_PIN_SET);
        while (1) {}
    }

    uint8_t dummy_data = 0;
    // uint8_t bspd_rx_count = 0;

    while (1) {
        can_poll_receive_all(); 

        // everything related to the BSPD does work its just in the example code there is not YAML file that works for it
        // if (bspd.brake_gate == BRAKE_GATE_BRAKE_LIGHT_ON) { 
        //     bspd_rx_count++; 
        // }

        dev_board.test_counter = dummy_data; // Updates the signal in the struct that will be sent via CAN
        // dev_board.test_counter_bspd = bspd_rx_count; 
        
        // Check Transmission
        if (can_send_dev_board() == 0) { // This makes the function run which sends out new CAN messages to the network
            dummy_data++; 
      
        }
        // You can also just call the function to send data too. 
        // can_send_dev_board() // this is completely valid and will send messages
        HAL_GPIO_TogglePin(PA1_GPIO_Port, PA1_Pin);
        HAL_Delay(100);
    }
    
    return 0;
}

