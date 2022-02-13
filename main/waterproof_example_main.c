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



static const uint64_t symbols[] = {
    0x383838fe7c381000, // arrows
    0x10387cfe38383800,
    0x10307efe7e301000,
    0x1018fcfefc181000,
    0x10387cfefeee4400, // heart
    0x105438ee38541000, // sun

    0x7e1818181c181800, // digits
    0x7e060c3060663c00,
    0x3c66603860663c00,
    0x30307e3234383000,
    0x3c6660603e067e00,
    0x3c66663e06663c00,
    0x1818183030667e00,
    0x3c66663c66663c00,
    0x3c66607c66663c00,
    0x3c66666e76663c00
};
const static size_t symbols_size = sizeof(symbols) - sizeof(uint64_t) * CASCADE_SIZE;

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
       .mirrored = true
    };
    ESP_ERROR_CHECK(max7219_init_desc(&dev, HOST, PIN_NUM_CS));
    ESP_ERROR_CHECK(max7219_init(&dev))
    ;
    size_t offs = 0;
    while (1)
    {
        printf("---------- draw\n");
        

        for (uint8_t c = 0; c < CASCADE_SIZE; c ++)
            max7219_draw_image_8x8(&dev, c * 8, (uint8_t *)symbols + c * 8 + offs);
        vTaskDelay(pdMS_TO_TICKS(SCROLL_DELAY));

        if (++offs == symbols_size)
            offs = 0;
    }
}


//---------------------------------------------------------------------------------------------------
static const char *TAG = "Touch Element Waterproof Example";
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

// typedef struct
// {
//     int press;
//     int release;
//     int longpress;
// } buttons_t;

// typedef enum {
//     MODE_AUTO,         //!< 
//     MODE_MANUAL,       //!< 
//     MODE_DEWPOINT,     //!< 
// } mode_button_event_t;

// typedef struct {
//     mode_button_event_t event;        //!< mode_Button event
// } mode_button_message_t;



#define led_auto 35
#define led_manual 36
#define led_dewpoint 37

#define button_mode_button 3
int mode_button_state = 1;

#define button_on_off 2
int button_on_off_state = 1;

#define button_grips 5
int button_grips_state = 0;

#define button_driver_seat 5
int button_driver_seat_state = 0;

#define button_Passenger_seat 5
int button_Passenger_seat_state = 0;

#define button_backrest 5
int button_backrest_state = 0;



void mode_button(void)
{
    gpio_pad_select_gpio(led_auto);
    gpio_set_direction(led_auto, GPIO_MODE_OUTPUT);

    gpio_pad_select_gpio(led_manual);
    gpio_set_direction(led_manual, GPIO_MODE_OUTPUT);

    gpio_pad_select_gpio(led_dewpoint);
    gpio_set_direction(led_dewpoint, GPIO_MODE_OUTPUT);

    while(1){  
        if(mode_button_state == 1){
            gpio_set_level(led_auto, 1);
            gpio_set_level(led_manual, 0); 
            gpio_set_level(led_dewpoint, 0); 
        }
        else if(mode_button_state == 2){
            gpio_set_level(led_auto, 0);
            gpio_set_level(led_manual, 1); 
            gpio_set_level(led_dewpoint, 0); 
        } 
        else if(mode_button_state == 3){
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
                if(button_backrest_state >= button_backrest){
                    button_backrest_state = 0;
                    }
                else if (button_backrest_state < 6){
                    button_backrest_state++;                    
                    }
            ESP_LOGI(TAG, "button_backrest mode[%d]", (int)button_backrest_state);  
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
                    button_on_off_state = 1;
                    }
                else if (button_on_off_state < 3){
                    button_on_off_state++;                    
                    }
            ESP_LOGI(TAG, "button_on_off mode[%d]", (int)button_on_off_state); 
            }

            else if ((uint32_t)element_message.arg == 10){
                ESP_LOGI(TAG, "Mode button is pressed");                  
                if(mode_button_state >= button_mode_button){
                    mode_button_state = 1;
                    }
                else if (mode_button_state < 4){
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
    ESP_LOGI(TAG, "Touch buttons create");
    /*< Create a monitor task to take Touch Button event */
    xTaskCreate(&button_handler_task, "button_handler_task", 4 * 1024, NULL, 4, NULL);
    touch_element_start();

    xTaskCreate(&mode_button, "mode_button", 4 * 1024, NULL, 3, NULL);
    xTaskCreate(&task, "task", 4 * 1024, NULL, 5, NULL);

    // xTaskCreatePinnedToCore(task, "task", 4096 * 3, NULL, 5, NULL, APP_CPU_NUM); //configMINIMAL_STACK_SIZE
}
