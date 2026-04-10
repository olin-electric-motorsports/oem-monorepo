#include "tire_temp.h"
//#include "i2c.h"
#include "MLX90640_API.h"
I2C_HandleTypeDef hi2c1;
#define ROWS 32// resolution of sensor is 32x24
#define COLS 24
#define RES 768
void I2C_setup(void){

    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x1060679A;                   // I2C clock timing
    hi2c1.Init.OwnAddress1 = 0;                       // primary device does not need address
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED; // primary device does not need address
    hi2c1.Init.OwnAddress2 = 0;                       // primary device does not need address
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED; // only communicating with one device
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;    // primary device does not need address
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED; // disabling no stretch means stretch is on

    HAL_I2C_Init(&hi2c1);
    HAL_I2CEx_AnalogFilter_Config(&hi2c1, I2C_ANALOGFILTER_ENABLED); // filters sudden spikes
}

void readSensor(I2C_HandleTypeDef *handle, uint8_t address, uint16_t s_register, uint16_t bits, uint8_t *buffer, 
    uint16_t bytes, uint32_t wait){
    //bits is # of bits for the message
    //bytes is # of bytes of buffer that is read
    //buffer is where data is stored
    //handle is i2c handle
    //address is sensor address
    //wait time is time waits for message
    HAL_I2C_Mem_Read(handle, address, s_register, bits, buffer, bytes, wait);
}
int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nMemAddressRead,uint16_t *data)
{   

    //sensor sends 2 bytes of information per address
    int wait_time = 100;
    uint8_t mlx_buffer[2 * nMemAddressRead];  
    // eaxh adress is a 16-bit value -> 2 bytes each

    readSensor(&hi2c1, slaveAddr << 1,startAddress, I2C_MEMADD_SIZE_16BIT,
               mlx_buffer, 2 * nMemAddressRead, wait_time);                   

    // Convert byte stream -> uint16_t array
    for (int i = 0; i < nMemAddressRead; i++)
    {
        data[i] = (mlx_buffer[2*i] << 8) | mlx_buffer[2*i + 1];
    }
    return 0;
}
int MLX90640_I2CGeneralReset(void)
{
    // Sends a general reset command on the I2C bus
    HAL_I2C_Master_Transmit(&hi2c1, 0x00, NULL, 0, 1000);
    return 0;
}
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
float average_temp(float *data){
    float average_t = 0;
    for (int i = 0; i < 768; i++) {
    
            average_t = average_t + data[i];
        
    }
    return average_t/768.0;    
}

void map_to_2D(float *flat, float map[24][32])
{
    for (int i = 0; i < 24; i++)
    {
        for (int j = 0; j < 32; j++)
        {
            map[i][j] = flat[i * 32 + j];
        }
    }
}

int main(void)
{
    HAL_Init();
    SystemClockConfig();    
    GpioInit();
    I2C_setup();//initialize I2C

    uint8_t mlx_address = 0x33;        // 7-bit I2C address
    uint8_t mlx_buffer[2];
    int sensor_bytes = 2;
    int wait_time = 100;

    int refresh_rate = 0x07; //64 Hz refresh rate

    uint16_t ee_data[832];     // EEPROM data. Handles data from adress 0x2400 to 0x273F which is 
    //832 different adresses total(fifference between 2400 and 273F hexedecimal).
    float temp_data[768];//32x24 pixels   
    uint16_t frame_data[834];//same size as EEPROM data but also has control register and subpage register
    float temp_map[24][32];
    uint16_t overheat_temp = 95; //temp for overheated tires in celsius

    paramsMLX90640 params;//paramsMLX90640 is defined in the mlx90640 api
    MLX90640_DumpEE(mlx_address, ee_data);  // Read calibration data from sensor EEPROM
    MLX90640_ExtractParameters(ee_data, &params);
    MLX90640_SetRefreshRate(mlx_address, refresh_rate); //set refresh rate
    MLX90640_SetChessMode(mlx_address); //set sensor to chess mode
  
    while (1)
    {
        


        if (MLX90640_GetFrameData(mlx_address, frame_data) >= 0)
        {
            float avg = 0;
            float max = 0;
            uint16_t hotspot_index = 0;
            uint8_t overheat = 0;
            // Convert to temperature
            MLX90640_CalculateTo(frame_data, &params, 0.95, 25.0, temp_data);
            
            //map_to_2D(temp_data, temp_map); <- CAN messages can't send that much data

            avg = average_temp(temp_data);

            // max temp 
            max = temp_data[0];

            for (int i = 1; i < 768; i++)
            {
                if (temp_data[i] > max)
                {
                    max = temp_data[i];
                    hotspot_index = i;
                }
            }
            uint8_t can_tx[8] = {0};
            if (max >= overheat_temp){
                overheat = 1;
            }
            can_tx[0] = (uint8_t)max;
            can_tx[1] = (uint8_t)avg;
            can_tx[2] = (uint8_t)(max - avg); // thermal gradient
            can_tx[3] = overheat; // status flags
            can_tx[4] = (hotspot_index >> 8) & 0xFF;
            can_tx[5] = hotspot_index & 0xFF;
        }

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