 
#include "ui.h"
#include "ili9341_display_driver.h"

#include "esp_log.h"


 
static const char * TAG = "main";
 
 

void app_main(void)
{
    ESP_LOGI(TAG, "start display test app");

    display_init();
    
}
