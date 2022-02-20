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
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"



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

#define PIN_NUM_MOSI 1
#define PIN_NUM_CLK  3
#define PIN_NUM_CS   2 //LOAD pin12 on max 7219



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

static const char *TAG02 = "max7219 task";




//---------------------------------------------------------------------------------------------------
static const char *TAG = "Touch Element: ";
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

bool press = false;
bool long_press = false;
bool release = false;

#define led_auto 35
#define led_manual 36
#define led_dewpoint 37

#define button_mode_button 2
int mode_b_state= 0;

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
int back_b_state = 0;

int *power_levels[4];   // array for current power levels, input may come from different modes/user input
int pl_0;               // input for power level @ array index 0
int pl_1;               // input for power level @ array index 1
int pl_2;               // input for power level @ array index 2
int pl_3;               // input for power level @ array index 3


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
    ESP_ERROR_CHECK(max7219_init_desc(&dev, HOST, PIN_NUM_CS));
    ESP_ERROR_CHECK(max7219_init(&dev));
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
    while(1){

        pl_0 = back_b_state;
        pl_1 = pass_b_state;
        pl_2 = driver_b_state;
        pl_3 = grips_b_state;
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
    TaskHandle_t xMode_auto;
    TaskHandle_t xMode_manual;
    TaskHandle_t xMode_dewpoint;

    xTaskCreate(&mode_auto, "mode_auto", 4* 1024, NULL, 4, &xMode_auto);
    vTaskSuspend(xMode_auto);
    xTaskCreate(&mode_manual, "mode_manual", 4* 1024, NULL, 4, &xMode_manual);
    vTaskSuspend(xMode_manual);
    xTaskCreate(&mode_dewpoint, "mode_dewpoint", 4* 1024, NULL, 4, &xMode_dewpoint);
    vTaskSuspend(xMode_dewpoint);

    gpio_pad_select_gpio(led_auto);
    gpio_set_direction(led_auto, GPIO_MODE_OUTPUT);

    gpio_pad_select_gpio(led_manual);
    gpio_set_direction(led_manual, GPIO_MODE_OUTPUT);

    gpio_pad_select_gpio(led_dewpoint);
    gpio_set_direction(led_dewpoint, GPIO_MODE_OUTPUT);

    while(1){ 
        // Write current power level to power_levels array.
            power_levels[0] = &pl_0;
            power_levels[1] = &pl_1;
            power_levels[2] = &pl_2;
            power_levels[3] = &pl_3;

            // power_levels[0] = &back_b_state;
            // power_levels[1] = &pass_b_state;
            // power_levels[2] = &driver_b_state;
            // power_levels[3] = &grips_b_state;           
        // Write button state to led matrix
        for(uint8_t i = 0; i < 4; i++ ){
            // int x = *power_levels[i];  
            symbols[i] = led_states[*power_levels[i]];
        } 

        if(on_off_b_state == 0){                // OFF state
            vTaskSuspend(xMode_auto);
            vTaskSuspend(xMode_manual);
            vTaskSuspend(xMode_dewpoint);
            gpio_set_level(led_auto, 0);
            gpio_set_level(led_manual, 0); 
            gpio_set_level(led_dewpoint, 0); 
            pl_0 = 0;
            pl_1 = 0;
            pl_2 = 0;
            pl_3 = 0;
            // back_b_state = 0;
            // pass_b_state = 0;
            // driver_b_state = 0;
            // grips_b_state = 0;
            // grips_b_long = 0;
        }
        else if (on_off_b_state == 1){          // ON state

            if(mode_b_state == 0){              // Auto mode
                vTaskResume(xMode_auto);
                vTaskSuspend(xMode_manual);
                vTaskSuspend(xMode_dewpoint);
                gpio_set_level(led_auto, 0);
                gpio_set_level(led_manual, 0); 
                gpio_set_level(led_dewpoint, 0); 
            }
            else if(mode_b_state == 1){         // Manual mode
                vTaskSuspend(xMode_auto);
                vTaskResume(xMode_manual);
                vTaskSuspend(xMode_dewpoint);
                gpio_set_level(led_auto, 0);
                gpio_set_level(led_manual, 1); 
                gpio_set_level(led_dewpoint, 0); 
            } 
            else if(mode_b_state == 2){         // Dewpoint mode
                vTaskSuspend(xMode_auto);
                vTaskSuspend(xMode_manual);
                vTaskResume(xMode_dewpoint);
                gpio_set_level(led_auto, 0);
                gpio_set_level(led_manual, 0); 
                gpio_set_level(led_dewpoint, 1); 
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
   // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    // Open memory
    nvs_handle_t my_handle;
    err = nvs_open("storage", NVS_READWRITE, &my_handle);

    // Read
    int32_t restart_counter = 0; // value will default to 0, if not set yet in NVS
    err = nvs_get_i32(my_handle, "restart_counter", &restart_counter);
 

    // Write
    restart_counter++;
    err = nvs_set_i32(my_handle, "restart_counter", restart_counter);

    // Commit written value.
    err = nvs_commit(my_handle);

    // Close
    nvs_close(my_handle);
    // printf(restart_counter);
    ESP_LOGI(TAG, "Restart counter %d ", restart_counter);


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
    xTaskCreate(&task, "task", 4 * configMINIMAL_STACK_SIZE , NULL, 5, NULL);
    
    ESP_LOGI(TAG, "Touch buttons create");
    /*< Create a monitor task to take Touch Button event */
    xTaskCreate(&button_handler_task, "button_handler_task", 4 * configMINIMAL_STACK_SIZE, NULL, 4, NULL);
    touch_element_start();

    xTaskCreate(buttons_modes, "buttons_modes", 4 * configMINIMAL_STACK_SIZE, NULL, 4, NULL);
}


/*
// xTaskCreatePinnedToCore(task, "task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL, 0); //configMINIMAL_STACK_SIZE   APP_CPU_NUM
// xTaskCreate(&mode_select, "mode_select", 4* 1024, NULL, 4, NULL); 

// TaskHandle_t xMode_auto = NULL;
// TaskHandle_t xMode_manual = NULL;
// TaskHandle_t xMode_dewpoint = NULL;
// void mode_select(void){

//     while(1)
//     { 
//         mode_states[0] = &mode_b_state; 
//         for(uint8_t i = 0; i < 2; i++ ){
//             int x = *mode_states[i];  
//             ESP_LOGI(TAG, "mode_select_b_state == %d", x); 
//             ESP_LOGI(TAG, "mode button [%d]", (int)mode_b_state);  
//             // vTaskDelay(pdMS_TO_TICKS(500));

//             // if(x == 0){

//             // // vTaskResume(mode_auto);
//             // // vTaskSuspend(mode_manual);
//             // // vTaskSuspend(mode_dewpoint);

//             // }
//             // else if(x == 1){
//             //     // vTaskSuspend(mode_auto);
//             //     // vTaskResume(mode_manual);
//             //     // vTaskSuspend(mode_dewpoint);

//             // } 
//             // else if(x == 2){
//             //     // vTaskSuspend(mode_auto);
//             //     // vTaskSuspend(mode_manual);
//             //     // vTaskResume(mode_dewpoint);
//             // }
//         }

//     }
// }

*/