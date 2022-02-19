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



#define led_auto 35
#define led_manual 36
#define led_dewpoint 37

#define button_mode_button 2
int mode_button_state = 0;

#define button_on_off 1
int button_on_off_state = 0;

#define button_grips 5
int button_grips_state = 0;

#define button_driver_seat 5
int button_driver_seat_state = 0;

#define button_Passenger_seat 5
int button_Passenger_seat_state = 0;

#define button_backrest 5
int button_backrest_state = 0;

int *button_states[4];




void buttons_modes(void)
{
    gpio_pad_select_gpio(led_auto);
    gpio_set_direction(led_auto, GPIO_MODE_OUTPUT);

    gpio_pad_select_gpio(led_manual);
    gpio_set_direction(led_manual, GPIO_MODE_OUTPUT);

    gpio_pad_select_gpio(led_dewpoint);
    gpio_set_direction(led_dewpoint, GPIO_MODE_OUTPUT);

    while(1){  
        // button_states[0] = &button_on_off_state;
        // button_states[1] = &mode_button_state;       
        button_states[0] = &button_backrest_state;
        button_states[1] = &button_Passenger_seat_state;
        button_states[2] = &button_driver_seat_state;
        button_states[3] = &button_grips_state;
        
        

        // Write button state to led matrix
        for(uint8_t i = 0; i < 5; i++ ){
            int x = *button_states[i];  
            symbols[i] = led_states[x];
        }

        if(mode_button_state == 0){
            gpio_set_level(led_auto, 0);
            gpio_set_level(led_manual, 0); 
            gpio_set_level(led_dewpoint, 0); 
        }
        else if(mode_button_state == 1){
            gpio_set_level(led_auto, 0);
            gpio_set_level(led_manual, 1); 
            gpio_set_level(led_dewpoint, 0); 
        } 
        else if(mode_button_state == 2){
            gpio_set_level(led_auto, 0);
            gpio_set_level(led_manual, 0); 
            gpio_set_level(led_dewpoint, 1); 
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
            
 
            if((uint32_t)element_message.arg == 4){
                ESP_LOGI(TAG, "button_backrest is pressed"); 
                if(button_backrest_state >= button_backrest){       // increment states for each button press
                    button_backrest_state = 0;
                    }
                else if (button_backrest_state < 6){
                    button_backrest_state++;                    
                    }
            // symbols[2] = led_states[3];                          // test OK, kan hente verdi fra et array og oppdatere array til max7219                      
            ESP_LOGI(TAG02, "button_backrest array test [%d]", led_states[0]);  
            
            // ESP_LOGI(TAG02, "symbols %d ", (int)symbols[1]);                    
            ESP_LOGI(TAG, "button_backrest mode[%d]", (int)button_backrest_state);
            ESP_LOGI(TAG, "button_backrest mode from button_state array[%d]", *button_states[3]);   
            }

            else if((uint32_t)element_message.arg == 5){
                ESP_LOGI(TAG, "button_Passenger_seat is pressed"); 
                if(button_Passenger_seat_state >= button_Passenger_seat){
                    button_Passenger_seat_state = 0;
                    }
                else if (button_Passenger_seat_state < 6){
                    button_Passenger_seat_state++;                    
                    }
            ESP_LOGI(TAG, "button_Passenger_seat mode[%d]", (int)button_Passenger_seat_state); 
            }

            else if((uint32_t)element_message.arg == 6){
                ESP_LOGI(TAG, "button_driver_seat is pressed"); 
                if(button_driver_seat_state >= button_driver_seat){
                    button_driver_seat_state = 0;
                    }
                else if (button_driver_seat_state < 6){
                    button_driver_seat_state++;                    
                    }
            ESP_LOGI(TAG, "button_driver_seat mode[%d]", (int)button_driver_seat_state); 
            }

            else if((uint32_t)element_message.arg == 7){
                ESP_LOGI(TAG, "button_on_off is pressed"); 
                if(button_on_off_state >= button_on_off){
                    button_on_off_state = 0;
                    }
                else if (button_on_off_state < 2){
                    button_on_off_state++;                    
                    }
            ESP_LOGI(TAG, "button_on_off mode[%d]", (int)button_on_off_state); 
            }

            else if ((uint32_t)element_message.arg == 10){
                ESP_LOGI(TAG, "Mode button is pressed");                  
                if(mode_button_state >= button_mode_button){
                    mode_button_state = 0;
                    }
                else if (mode_button_state < 3){
                    mode_button_state++;                    
                    }
            ESP_LOGI(TAG, "mode button mode [%d]", (int)mode_button_state);                     
            }

            else if((uint32_t)element_message.arg == 11){
                ESP_LOGI(TAG, "button_grips is pressed");                
                if(button_grips_state >= button_grips){
                    button_grips_state = 0;
                    }
                else if (button_grips_state < 6){
                    button_grips_state++;                    
                    }
            ESP_LOGI(TAG, "button_grips mode[%d]", (int)button_grips_state); 
            }
        } else if (button_message->event == TOUCH_BUTTON_EVT_ON_RELEASE) {
            ESP_LOGI(TAG, "Button[%d] Release", (uint32_t)element_message.arg);
        } else if (button_message->event == TOUCH_BUTTON_EVT_ON_LONGPRESS) {
            ESP_LOGI(TAG, "Button[%d] LongPress", (uint32_t)element_message.arg);
            if ((uint32_t)element_message.arg == 10){
                ESP_LOGI(TAG, "Mode button is long_pressed");
                gpio_set_level(led_auto, 0);
                gpio_set_level(led_manual, 0); 
                gpio_set_level(led_dewpoint, 0); 
                // mode_button();
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

    xTaskCreate(&task, "task", 4 * configMINIMAL_STACK_SIZE , NULL, 5, NULL);
    // xTaskCreatePinnedToCore(task, "task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL, 0); //configMINIMAL_STACK_SIZE   APP_CPU_NUM
    
    ESP_LOGI(TAG, "Touch buttons create");
    /*< Create a monitor task to take Touch Button event */
    xTaskCreate(&button_handler_task, "button_handler_task", 4 * configMINIMAL_STACK_SIZE, NULL, 4, NULL);
    touch_element_start();

    xTaskCreate(&buttons_modes, "buttons_modes", 4* 1024 , NULL, 4, NULL);
}
