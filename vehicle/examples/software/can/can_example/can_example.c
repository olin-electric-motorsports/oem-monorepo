#include "can_example.h"
#include "can_api.h"

// static void GpioInit(void);
void SystemClockConfig(void);

int main(void) {
    HAL_Init();
    SystemClockConfig();
    // GpioInit(); 


    // Check Initialization
    if (can_init_can_example() != 0) {
        // If initialization fails, trap CPU
        while (1) {}
    }

    uint8_t recieved_data_from_DEV_board = 0;
    while (1) {
        can_poll_receive_all(); // this checks for new CAN messages and updates the structs

        recieved_data_from_DEV_board = dev_board.test_counter; // This struct is from CAN

        can_example.dummy_signal = recieved_data_from_DEV_board; // This struct is what we will send out via CAN, we are just setting it to the value we got from the dev board for testing
        
        if (can_send_can_example() == 0) { // this sends out the CAN message with the updated data in can_example struct
            // You can also just call the function to send data too. 
            // can_send_can_example() // this is completely valid and will send messages
      
        }

        HAL_Delay(100);
    }
    
    return 0;
}

