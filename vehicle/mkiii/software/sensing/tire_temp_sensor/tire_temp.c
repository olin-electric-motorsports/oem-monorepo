#include "tire_temp.h"

static void GpioInit(void);
/*
int[[]] pixel_map(){
    returns pixel_map;
}
int[[]] filter pixels(pixel_map){
    faulty_pixels
    for(int i = 0; i< display_width;i++){
    for (int j = 0; j<display_length;j++){
    if(!(75>pixels_temp>120)){
        final_pixel_map= pixel_map[i][j]
            }
        }
    }
    return final_pixel_map;
}
int average_temp(filtered_pixel_map){
    return average filtered_pixel_map
}
*/
void Heartbeat_Update(void)
{
    static uint32_t count = 0;

    count++;

    if (count >= 8000000)   // adjust for blink rate
    {
        HAL_GPIO_TogglePin(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN);
        count = 0;
    }
}

static void GpioInit(void);
void SystemClockConfig(void);
int main(void)
{
    HAL_Init();
    SystemClockConfig();    
    GpioInit();

    while (1)
    {
        HAL_GPIO_TogglePin(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN);
        HAL_Delay(500);
        //Heartbeat_Update(); 
        /*
        sensor_map = IR_output
        average_temp = IR_average_temp
        HB_LED_function
        Debug_LED function
        CAN_SEND(sensor_map, average_temp)
        */
    }
}

