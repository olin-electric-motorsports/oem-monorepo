#include "dev_board.h"
#include "can_api.h"

static void GpioInit(void);
void SystemClockConfig(void);

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

    //uint8_t dummy_data = 0;
    // uint8_t bspd_rx_count = 0;
    // You can read these natively without doing any manual parsing!

    double current_msg_counter=0;
    double previous_current_msg_counter=15;
    double current_msg_missing_count=0;
    double current_mA=0;
    double temp_C=0;
    
    while (1) {
        can_poll_receive_all(); //receive all message

        //software check if current and temperature message id is in range
        if(can_tools_ivt_msg_result_i_ivt_id_result_i_is_in_range(ivt_msg_result_i.ivt_id_result_i)
            &&can_tools_ivt_msg_result_t_ivt_id_result_t_is_in_range(ivt_msg_result_t.ivt_id_result_t))
        {
                
            //decoding raw data in stucts with helper methods from can_tool
            current_mA=can_tools_ivt_msg_result_i_ivt_result_i_decode(ivt_msg_result_i.ivt_result_i);
            temp_C=can_tools_ivt_msg_result_t_ivt_result_t_decode(ivt_msg_result_t.ivt_result_t)*10.;
            current_msg_counter=can_tools_ivt_msg_result_i_ivt_msg_count_result_i_decode(ivt_msg_result_i.ivt_msg_count_result_i);

            
            if (current_mA > 5000&&temp_C>100) {
                //do something
            }

            //check for any messages not received
            if(previous_current_msg_counter==15&&current_msg_counter!=0){
                current_msg_missing_count=current_msg_counter;
            }
            else if(current_msg_counter-previous_current_msg_counter>1){
                current_msg_missing_count=current_msg_counter-previous_current_msg_counter-1;
            }

            if(current_msg_missing_count>0){

            }
        }   

        HAL_Delay(100);
    }
    
    return 0;
}

