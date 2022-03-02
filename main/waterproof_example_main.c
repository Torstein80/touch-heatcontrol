/*
On/off button:
-When power is applied, on/off state is remembered, hence if button is off before previous power off, system is off when power is applied again.

Mode button:
-Toggles between Auto, Manual and Dewpoint mode

Grip/throttle button:
-Press: Grips power level
-Long press: Thumb power level

Other buttons:
-Set power levels on press

Auto mode:
- inputs: Temp and Relative humidity.
-- Temp array give input to power level
-- Relative humidity > 90% set 100% power level for 30 minutes, then off or whatever temp array demands

Manual mode:
-All settings are set manual, settings are remembered between power cycles

Dewpoint mode:
-input: Relative humidity
-- Relative humidity > 90% set 100% power level for 30 minutes, then off
*/


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "touch_element/touch_button.h"
#include "driver/gpio.h"
#include "esp_idf_version.h"
#include "max7219.h"
#include "stdio.h"
#include "esp_littlefs.h"
// #include "esp_system.h"
// #include "nvs_flash.h"
// #include "nvs.h"
#include <dht.h>
#include "driver/ledc.h"
#include "esp_err.h"



static const dht_sensor_type_t sensor_type = DHT_TYPE_AM2301; //AM2301 is for DHT22
static const gpio_num_t dht_gpio = 38;

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

#define SCROLL_DELAY 50
#define CASCADE_SIZE 1

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4, 0, 0)
#define HOST    HSPI_HOST
#else
#define HOST    SPI2_HOST
#endif

#define PIN_NUM_MOSI 1  //max 7219
#define PIN_NUM_CLK  3  //max 7219
#define PIN_NUM_CS   2  //LOAD pin12 on max 7219



u_char symbols[] = { // Array for max7219 LED matrix
    0b00000000, // backrest leds
    0b00000000, // passenger seat leds
    0b00000000, // driver seat leds
    0b00000000, // grips/throttle leds
    0b00000000, // not in use for current project
    0b00000000, // not in use for current project
    0b00000000, // not in use for current project
    0b00000000  // not in use for current project
};

const static size_t symbols_size = sizeof(symbols) - sizeof(u_char) * CASCADE_SIZE;

static const u_char led_states[] = { // Possible led states that can be assigned to symbols array for current project. leds are lit from left to right
    0b00000000,     // 0 led
    0b10000000,     // 1 led
    0b11000000,     // 2 led
    0b11100000,     // 3 led
    0b11110000,     // 4 led 
    0b11111000      // 5 led
};


static const char *TAG = "Touch Element: ";
static const char *TAG02 = "max7219 task";
static const char *TAG03 = "DHT22: ";

#define TOUCH_BUTTON_NUM     6

/*< Touch buttons handle */
static touch_button_handle_t button_handle[TOUCH_BUTTON_NUM]; //Button handler

/* Touch buttons channel array */
static const touch_pad_t channel_array[TOUCH_BUTTON_NUM] = {
    TOUCH_PAD_NUM4,     //button_backrest
    TOUCH_PAD_NUM5,     //button_Passenger_seat
    TOUCH_PAD_NUM6,     //button_driver_seat
    TOUCH_PAD_NUM7,     //button_on_off
    TOUCH_PAD_NUM10,    //button_mode button
    TOUCH_PAD_NUM11,    //button_grips
};

/* Touch buttons channel sensitivity array */
static const float channel_sens_array[TOUCH_BUTTON_NUM] = {
    0.15F,
    0.15F,
    0.15F,
    0.15F,
    0.15F,
    0.15F,
};

bool press = false;         // for button logic
bool long_press = false;    // for button logic
bool release = false;       // for button logic

#define led_auto 35         //gpio for mode LED
#define led_manual 36       //gpio for mode LED
#define led_dewpoint 37     //gpio for mode LED

#define LEDC_LS_CH0_GPIO       (35)
#define LEDC_LS_CH0_CHANNEL    LEDC_CHANNEL_0

#define LEDC_LS_CH1_GPIO       (36)
#define LEDC_LS_CH1_CHANNEL    LEDC_CHANNEL_1

#define LEDC_LS_CH2_GPIO       (37)
#define LEDC_LS_CH2_CHANNEL    LEDC_CHANNEL_2

#define LEDC_LS_CH3_GPIO       (16)     //tmb PWM
#define LEDC_LS_CH3_CHANNEL    LEDC_CHANNEL_3

#define LEDC_LS_CH4_GPIO       (17)     //grip PWM
#define LEDC_LS_CH4_CHANNEL    LEDC_CHANNEL_4

#define LEDC_LS_CH5_GPIO       (21)     // driver seat PWM
#define LEDC_LS_CH5_CHANNEL    LEDC_CHANNEL_5

#define LEDC_LS_CH6_GPIO       (33)     // Passenger seat PWM
#define LEDC_LS_CH6_CHANNEL    LEDC_CHANNEL_6

#define LEDC_LS_CH7_GPIO       (34)     // Back rest PWM
#define LEDC_LS_CH7_CHANNEL    LEDC_CHANNEL_7

#define LEDC_LS_TIMER          LEDC_TIMER_1
#define LEDC_LS_MODE           LEDC_LOW_SPEED_MODE

#define LEDC_CH_NUM       (8)
#define LEDC_DUTY        (1000) //4000
#define LEDC_FADE_TIME    (3000)




#define button_mode_button 2    // #of modes (0,1,2)
int mode_b_state= 0;            // Current button state

#define button_on_off 1
int on_off_b_state = 0;

#define button_grips 5
int grips_b_state = 0;
int grips_b_long = 0;

#define button_driver_seat 5
int driver_b_state = 0;

#define button_Passenger_seat 5
int pass_b_state = 0;

#define button_backrest 5
uint16_t back_b_state = 0;

int pl_0;               // input for power level backrest       @ array index 0
int pl_1;               // input for power level passenger seat @ array index 1
int pl_2;               // input for power level driver seat    @ array index 2
int pl_3;               // input for power level grips          @ array index 3
int pl_4;               // input for power level thumb throttle

int *LED_levels[4];   // array with current power levels used for LED array, input may come from different modes/user input

// LEDc channels 3-7 use power_states as input for the duty cycle
int power_states[5]; // array with current power levels for the PWM outputs.

int duty_cycles[6]={  // Set duty to e.g. 50%: ((2 ** 13) - 1) * 50% = 4095. This controls the PWM duty cycle for the 5 Mosfets
//  0%, 20%,  40%,  60%,  80%, 100%    
    0, 1638, 3276, 4914, 6552, 8190
};


// LEDc channels 0-2 use mode_states as input for the duty cycle
int mode_states[3][3]={ // LEDS_DUTY controls the mode LEDs light intensity
    {LEDC_DUTY, 0, 0},
    {0, LEDC_DUTY, 0},
    {0, 0, LEDC_DUTY}
};
// const static size_t mode_states_size = sizeof(mode_states);


void dht22(void *pvParameters)
{
    int16_t temperature = 0;
    int16_t humidity = 0;

    // DHT sensors that come mounted on a PCB generally have
    // pull-up resistors on the data pin.  It is recommended
    // to provide an external pull-up resistor otherwise...

    gpio_set_pull_mode(dht_gpio, GPIO_PULLUP_ONLY);

    while (1)
    {
        // dht_read_data(sensor_type, dht_gpio, &humidity, &temperature);
        // ESP_LOGI(TAG03, "Humidity: %d%% Temp: %dC\n", humidity / 10, temperature / 10);
        if (dht_read_data(sensor_type, dht_gpio, &humidity, &temperature) == ESP_OK)
            ESP_LOGI(TAG03, "Humidity: %d%% Temp: %dC\n", humidity / 10, temperature / 10); 
            // printf("Humidity: %d%% Temp: %dC\n", humidity / 10, temperature / 10);
        else
            ESP_LOGI(TAG03, "Could not read data from sensor\n"); 
            // printf("Could not read data from sensor\n");
            

        // If you read the sensor data too often, it will heat up
        // http://www.kandrsmith.org/RJS/Misc/Hygrometers/dht_sht_how_fast.html
        
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void task(void *pvParameter)
{
    esp_err_t res;
    
    // Configure SPI bus
    spi_bus_config_t cfg = {
       .mosi_io_num = PIN_NUM_MOSI,
       .miso_io_num = -1,
       .sclk_io_num = PIN_NUM_CLK,
       .quadwp_io_num = -1,
       .quadhd_io_num = -1,
       .max_transfer_sz = 0,
       .flags = 0
    };
    ESP_ERROR_CHECK(spi_bus_initialize(HOST, &cfg, 1));

    // Configure device
    max7219_t dev = {
       .cascade_size = CASCADE_SIZE,
       .digits = 0,
       .mirrored = false
    };
    ESP_ERROR_CHECK(max7219_init_desc(&dev, HOST, PIN_NUM_CS)); //Initialize device descriptor.
    ESP_ERROR_CHECK(max7219_init(&dev)); //initialize display
    ESP_ERROR_CHECK(max7219_clear(&dev)); //clear display
    size_t offs = 0;

    while (1)
    {     
        for(uint8_t j = 0; j < CASCADE_SIZE; j++){
            // max7219_draw_image_8x8(&dev, i * 8, (uint8_t *)symbols + i * 8); // + offs               
            max7219_draw_image_8x8(&dev, j * 8, (uint8_t *)symbols + j * 8); //
            vTaskDelay(pdMS_TO_TICKS(SCROLL_DELAY)); 
        }             
        if (++offs == symbols_size)
            offs = 0;
         
    }
}


void mode_auto(void *pvParameter){
    //Auto mode:
    // - inputs: Temp and Relative humidity.
    // -- Temp array give input to power level
    // -- Relative humidity > 90% set 100% power level for 30 minutes, then off or whatever temp array demands
    while(1){

    }
}


void mode_manual(void *pvParameter){
    // Manual mode:
    // -All settings are set manual, settings are remembered between power cycles
    // teste om grips long press er true og erstatte [3] med tommel varme mens den er trykket + 3 sekunder, deretter tilbake igjen
    int grips_b_long_temp = 0;
    while(1){

        if (long_press == true){ 
            pl_0 = back_b_state;
            pl_1 = pass_b_state;
            pl_2 = driver_b_state;
            pl_3 = grips_b_long; // temporarely display power level for thumb throttle, this will also change PWM for grips, but only for 2 seconds, hence it is ok.
            pl_4 = grips_b_long;
            vTaskDelay(pdMS_TO_TICKS(2000)); 
            long_press = false;                                                 
        } 
        else if(long_press == false) {
            pl_0 = back_b_state;
            pl_1 = pass_b_state;
            pl_2 = driver_b_state;
            pl_3 = grips_b_state;
            pl_4 = grips_b_long;
        } 
        vTaskDelay(pdMS_TO_TICKS(20)); 
    }
}


void mode_dewpoint(void *pvParameter){

    // Dewpoint mode:
    // -input: Relative humidity
    // -- Relative humidity > 90% set 100% power level for 30 minutes, then off
    while(1){
        pl_0 = 1;
        pl_1 = 2;
        pl_2 = 3;
        pl_3 = 4;
    }
}


void buttons_modes(void *pvParameter)
{
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
        .freq_hz = 5000,                      // frequency of PWM signal
        .speed_mode = LEDC_LS_MODE,           // timer mode
        .timer_num = LEDC_LS_TIMER,            // timer index
        .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
    };
    
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel[LEDC_CH_NUM] = {
        {
            .channel    = LEDC_LS_CH0_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_LS_CH0_GPIO,
            .speed_mode = LEDC_LS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_LS_TIMER,
            .flags.output_invert = 0
        },
        {
            .channel    = LEDC_LS_CH1_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_LS_CH1_GPIO,
            .speed_mode = LEDC_LS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_LS_TIMER,
            .flags.output_invert = 0
        },
        {
            .channel    = LEDC_LS_CH2_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_LS_CH2_GPIO,
            .speed_mode = LEDC_LS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_LS_TIMER,
            .flags.output_invert = 0
        },
        {
            .channel    = LEDC_LS_CH3_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_LS_CH3_GPIO,
            .speed_mode = LEDC_LS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_LS_TIMER,
            .flags.output_invert = 0
        },
        {
            .channel    = LEDC_LS_CH4_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_LS_CH4_GPIO,
            .speed_mode = LEDC_LS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_LS_TIMER,
            .flags.output_invert = 0
        },
        {
            .channel    = LEDC_LS_CH5_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_LS_CH5_GPIO,
            .speed_mode = LEDC_LS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_LS_TIMER,
            .flags.output_invert = 0
        },
        {
            .channel    = LEDC_LS_CH6_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_LS_CH6_GPIO,
            .speed_mode = LEDC_LS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_LS_TIMER,
            .flags.output_invert = 0
        },
        {
            .channel    = LEDC_LS_CH7_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_LS_CH7_GPIO,
            .speed_mode = LEDC_LS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_LS_TIMER,
            .flags.output_invert = 0
        },
    };

    // Set LED Controller with previously prepared configuration
    for (int ch = 0; ch < LEDC_CH_NUM; ch++) {
        ledc_channel_config(&ledc_channel[ch]);
    }

    TaskHandle_t xMode_auto;
    TaskHandle_t xMode_manual;
    TaskHandle_t xMode_dewpoint;

    xTaskCreate(&mode_auto, "mode_auto", 4* 1024, NULL, 4, &xMode_auto);
    vTaskSuspend(xMode_auto);
    xTaskCreate(&mode_manual, "mode_manual", 4* 1024, NULL, 4, &xMode_manual);
    vTaskSuspend(xMode_manual);
    xTaskCreate(&mode_dewpoint, "mode_dewpoint", 4* 1024, NULL, 4, &xMode_dewpoint);
    vTaskSuspend(xMode_dewpoint);
    

    while(1){ 
        // write power levels to LED levels aarray
            LED_levels[0] = &pl_0;
            LED_levels[1] = &pl_1;
            LED_levels[2] = &pl_2;
            LED_levels[3] = &pl_3;
        // write power levels for the PWM outputs
            power_states[0] = &pl_0;
            power_states[1] = &pl_1;
            power_states[2] = &pl_2;
            power_states[3] = &pl_3;
            power_states[4] = &pl_4;
            
          
        // Write button state to led matrix
        for(uint8_t i = 0; i < 4; i++ ){ 
            symbols[i] = led_states[*LED_levels[i]];
        } 
        if(on_off_b_state == 0){                // OFF state
            
            vTaskSuspend(xMode_auto);
            vTaskSuspend(xMode_manual);
            vTaskSuspend(xMode_dewpoint);

            pl_0 = 0;
            pl_1 = 0;
            pl_2 = 0;
            pl_3 = 0;
            

            for (int i=0; i<3;i++){
                ledc_set_duty(ledc_channel[i].speed_mode, ledc_channel[i].channel, 0);
                ledc_update_duty(ledc_channel[i].speed_mode, ledc_channel[i].channel);
            }
            // back_b_state = 0;
            // pass_b_state = 0;
            // driver_b_state = 0;
            // grips_b_state = 0;
            // grips_b_long = 0;

        }
        else if (on_off_b_state == 1){          // ON state
            for(int i=0; i<3; i++){  
                ledc_set_duty(ledc_channel[i].speed_mode, ledc_channel[i].channel, mode_states[mode_b_state][i]);
                ledc_update_duty(ledc_channel[i].speed_mode, ledc_channel[i].channel);   
            }
            // set duty cycle for PWM outputs. 0-5 = 0-100%
            for(int i=3; i<8; i++){
                for(int j=0; j<6;j++){
                    ledc_set_duty(ledc_channel[i].speed_mode, ledc_channel[i].channel, duty_cycles[power_states[j]]);
                    ledc_update_duty(ledc_channel[i].speed_mode, ledc_channel[i].channel);  
                }
            }
            if(mode_b_state == 0){              // Auto mode
                vTaskResume(xMode_auto);
                vTaskSuspend(xMode_manual);
                vTaskSuspend(xMode_dewpoint);
            }
            else if(mode_b_state == 1){         // Manual mode
                vTaskSuspend(xMode_auto);
                vTaskResume(xMode_manual);
                vTaskSuspend(xMode_dewpoint);
            } 
            else if(mode_b_state == 2){         // Dewpoint mode
                vTaskSuspend(xMode_auto);
                vTaskSuspend(xMode_manual);
                vTaskResume(xMode_dewpoint);
            }
        }
    }
}        
            

static void button_handler_task(void *arg)
{

    touch_elem_message_t element_message;
    while (1) {
        touch_element_message_receive(&element_message, portMAX_DELAY); //Block take
        const touch_button_message_t *button_message = touch_button_get_message(&element_message);
        
        if (button_message->event == TOUCH_BUTTON_EVT_ON_PRESS) {
            ESP_LOGI(TAG, "Button[%d] Press", (uint32_t)element_message.arg); 
            press = true;
            long_press = false;          
        }
        else if (button_message->event == TOUCH_BUTTON_EVT_ON_LONGPRESS) {
            ESP_LOGI(TAG, "Button[%d] LongPress", (uint32_t)element_message.arg);
            press = false;
            long_press = true; 
        }       
        else if (button_message->event == TOUCH_BUTTON_EVT_ON_RELEASE) {
            ESP_LOGI(TAG, "Button[%d] Release", (uint32_t)element_message.arg);

            if((uint32_t)element_message.arg == 4){
                ESP_LOGI(TAG, "button_backrest is pressed"); 
                if(back_b_state >= button_backrest){       // increment states for each button press
                    back_b_state = 0;
                    }
                else if (back_b_state < 6){
                    back_b_state++;                    
                    }                          
            ESP_LOGI(TAG, "button_backrest mode[%d]", (int)back_b_state);
            }
            else if((uint32_t)element_message.arg == 5){
                ESP_LOGI(TAG, "button_Passenger_seat is pressed"); 
                if(pass_b_state >= button_Passenger_seat){
                    pass_b_state = 0;
                    }
                else if (pass_b_state < 6){
                    pass_b_state++;                    
                    }
            ESP_LOGI(TAG, "button_Passenger_seat mode[%d]", (int)pass_b_state); 
            }

            else if((uint32_t)element_message.arg == 6){
                ESP_LOGI(TAG, "button_driver_seat is pressed"); 
                if(driver_b_state >= button_driver_seat){
                    driver_b_state = 0;
                    }
                else if (driver_b_state < 6){
                    driver_b_state++;                    
                    }
            ESP_LOGI(TAG, "button_driver_seat mode[%d]", (int)driver_b_state); 
            }

            else if((uint32_t)element_message.arg == 7){
                ESP_LOGI(TAG, "button_on_off is pressed"); 
                if(on_off_b_state >= button_on_off){
                    on_off_b_state = 0;
                    }
                else if (on_off_b_state < 2){
                    on_off_b_state++;                    
                    }
            ESP_LOGI(TAG, "button_on_off mode[%d]", (int)on_off_b_state); 
            }

            else if ((uint32_t)element_message.arg == 10){
                ESP_LOGI(TAG, "Mode button is pressed");                  
                if(mode_b_state>= button_mode_button){
                    mode_b_state= 0;
                    }
                else if (mode_b_state< 3){
                    mode_b_state++;                    
                    }
                    ESP_LOGI(TAG, "mode_button mode[%d]", (int)mode_b_state);                    
            }
            else if((uint32_t)element_message.arg == 11){
                if(press == true && long_press == false){
                    ESP_LOGI(TAG, "button_grips is pressed");                
                    if(grips_b_state >= button_grips){
                        grips_b_state = 0;
                    }
                    else if (grips_b_state < 6){
                        grips_b_state++;                    
                    }
                    // nvs_set_i16(my_handle, "back_b_state", back_b_state);
                    ESP_LOGI(TAG, "button_grips mode[%d]", (int)grips_b_state); 
                }
                else if(press == false && long_press == true){
                    ESP_LOGI(TAG, "button_grips is pressed");                
                    if(grips_b_long >= button_grips){
                        grips_b_long = 0;
                    }
                    else if (grips_b_long < 6){
                        grips_b_long++;                    
                    }
                    ESP_LOGI(TAG, "button_grips Thumb longpress[%d]", (int)grips_b_long); 
                }
            }
        }
    }
}





void app_main(void)
{
    /*< Initialize Touch Element library */
    touch_elem_global_config_t element_global_config = TOUCH_ELEM_GLOBAL_DEFAULT_CONFIG();
    ESP_ERROR_CHECK(touch_element_install(&element_global_config));
    ESP_LOGI(TAG, "Touch Element library install");
    /*< Create and configure touch element waterproof */
    touch_elem_waterproof_config_t waterproof_config = {
#ifdef CONFIG_TOUCH_WATERPROOF_GUARD_ENABLE
        .guard_channel = TOUCH_PAD_NUM12,
#else
        .guard_channel = TOUCH_WATERPROOF_GUARD_NOUSE,
#endif
        .guard_sensitivity = 0.05F  //The guard sensor sensitivity has to be explored in experiments
    };
    ESP_ERROR_CHECK(touch_element_waterproof_install(&waterproof_config));
    ESP_LOGI(TAG, "Touch Element waterproof install");

    touch_button_global_config_t button_global_config = TOUCH_BUTTON_GLOBAL_DEFAULT_CONFIG();
    ESP_ERROR_CHECK(touch_button_install(&button_global_config));
    ESP_LOGI(TAG, "Touch button install");
    for (int i = 0; i < TOUCH_BUTTON_NUM; i++) {
        touch_button_config_t button_config = {
            .channel_num = channel_array[i],
            .channel_sens = channel_sens_array[i]
        };
        /* Create touch button */
        ESP_ERROR_CHECK(touch_button_create(&button_config, &button_handle[i]));
        /* Subscribe touch button event(Press, Release, LongPress) */
        ESP_ERROR_CHECK(touch_button_subscribe_event(button_handle[i], TOUCH_ELEM_EVENT_ON_PRESS | TOUCH_ELEM_EVENT_ON_RELEASE | TOUCH_ELEM_EVENT_ON_LONGPRESS,
                                                     (void *)channel_array[i]));
        /* Button set dispatch method */
        ESP_ERROR_CHECK(touch_button_set_dispatch_method(button_handle[i], TOUCH_ELEM_DISP_EVENT));
#ifdef CONFIG_TOUCH_WATERPROOF_GUARD_ENABLE
        /* Add button element into waterproof guard sensor's protection */
        ESP_ERROR_CHECK(touch_element_waterproof_add(button_handle[i]));
#endif
    }

    xTaskCreate(dht22, "dht22", 4 * configMINIMAL_STACK_SIZE, NULL, 5, NULL);
    xTaskCreate(&task, "task", 4 * configMINIMAL_STACK_SIZE , NULL, 5, NULL);
    
    ESP_LOGI(TAG, "Touch buttons create");
    /*< Create a monitor task to take Touch Button event */
    xTaskCreate(&button_handler_task, "button_handler_task", 4 * configMINIMAL_STACK_SIZE, NULL, 4, NULL);
    touch_element_start();

    xTaskCreate(buttons_modes, "buttons_modes", 4 * configMINIMAL_STACK_SIZE, NULL, 4, NULL);
}



// Demo ESP LittleFS Example

//    This example code is in the Public Domain (or CC0 licensed, at your option.)

//    Unless required by applicable law or agreed to in writing, this
//    software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
//    CONDITIONS OF ANY KIND, either express or implied.

// #include <stdio.h>
// #include "sdkconfig.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_system.h"
// #include "esp_spi_flash.h"
// #include "esp_err.h"
// #include "esp_log.h"

// #include "esp_littlefs.h"

// static const char *TAG = "demo_esp_littlefs";

// void app_main(void)
// {
//         printf("Demo LittleFs implementation by esp_littlefs!\n");
//         printf("   https://github.com/joltwallet/esp_littlefs\n");

//         /* Print chip information */
//         esp_chip_info_t chip_info;
//         esp_chip_info(&chip_info);
//         printf("This is %s chip with %d CPU cores, WiFi%s%s, ",
//                CONFIG_IDF_TARGET,
//                chip_info.cores,
//                (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
//                (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

//         printf("silicon revision %d, ", chip_info.revision);

//         printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
//                (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

//         printf("Free heap: %d\n", esp_get_free_heap_size());

//         printf("Now we are starting the LittleFs Demo ...\n");

//         ESP_LOGI(TAG, "Initializing LittelFS");

//         esp_vfs_littlefs_conf_t conf = {
//             .base_path = "/littlefs",
//             .partition_label = "littlefs",
//             .format_if_mount_failed = true,
//             .dont_mount = false,
//         };

//         // Use settings defined above to initialize and mount LittleFS filesystem.
//         // Note: esp_vfs_littlefs_register is an all-in-one convenience function.
//         esp_err_t ret = esp_vfs_littlefs_register(&conf);

//         if (ret != ESP_OK)
//         {
//                 if (ret == ESP_FAIL)
//                 {
//                         ESP_LOGE(TAG, "Failed to mount or format filesystem");
//                 }
//                 else if (ret == ESP_ERR_NOT_FOUND)
//                 {
//                         ESP_LOGE(TAG, "Failed to find LittleFS partition");
//                 }
//                 else
//                 {
//                         ESP_LOGE(TAG, "Failed to initialize LittleFS (%s)", esp_err_to_name(ret));
//                 }
//                 return;
//         }

//         size_t total = 0, used = 0;
//         ret = esp_littlefs_info(conf.partition_label, &total, &used);
//         if (ret != ESP_OK)
//         {
//                 ESP_LOGE(TAG, "Failed to get LittleFS partition information (%s)", esp_err_to_name(ret));
//         }
//         else
//         {
//                 ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
//         }

//         // Use POSIX and C standard library functions to work with files.
//         // First create a file.
//         ESP_LOGI(TAG, "Opening file");
//         FILE *f = fopen("/littlefs/hello.txt", "w");
//         if (f == NULL)
//         {
//                 ESP_LOGE(TAG, "Failed to open file for writing");
//                 return;
//         }
//         fprintf(f, "LittleFS Rocks!\n");
//         fclose(f);
//         ESP_LOGI(TAG, "File written");

//         // Check if destination file exists before renaming
//         struct stat st;
//         if (stat("/littlefs/foo.txt", &st) == 0)
//         {
//                 // Delete it if it exists
//                 unlink("/littlefs/foo.txt");
//         }

//         // Rename original file
//         ESP_LOGI(TAG, "Renaming file");
//         if (rename("/littlefs/hello.txt", "/littlefs/foo.txt") != 0)
//         {
//                 ESP_LOGE(TAG, "Rename failed");
//                 return;
//         }

//         // Open renamed file for reading
//         ESP_LOGI(TAG, "Reading file");
//         f = fopen("/littlefs/foo.txt", "r");
//         if (f == NULL)
//         {
//                 ESP_LOGE(TAG, "Failed to open file for reading");
//                 return;
//         }
//         char line[64];
//         fgets(line, sizeof(line), f);
//         fclose(f);
//         // strip newline
//         char *pos = strchr(line, '\n');
//         if (pos)
//         {
//                 *pos = '\0';
//         }
//         ESP_LOGI(TAG, "Read from file: '%s'", line);

//         // All done, unmount partition and disable LittleFS
//         esp_vfs_littlefs_unregister(conf.partition_label);
//         ESP_LOGI(TAG, "LittleFS unmounted");
// }

// // # Special partition table for unit test app
// // #
// // # Name,     Type, SubType, Offset,   Size, Flags
// // # Note: if you change the phy_init or app partition offset, make sure to change the offset in Kconfig.projbuild
// // nvs,        data, nvs,     0x9000,  0x4000
// // otadata,    data, ota,     0xd000,  0x2000
// // phy_init,   data, phy,     0xf000,  0x1000
// // factory,    0,    0,       0x10000, 2M
// // # these OTA partitions are used for tests, but can't fit real OTA apps in them
// // # (done this way so tests can run in 2MB of flash.)
// // ota_0,      0,    ota_0,   ,        64K
// // ota_1,      0,    ota_1,   ,        64K
// // # flash_test partition used for SPI flash tests, WL FAT tests, and SPIFFS tests
// // fat_store, data, fat,     ,        528K
// // spiffs_store,  data, spiffs,    ,        512K
// // flash_test,  data, spiffs,    ,        512K 
// */