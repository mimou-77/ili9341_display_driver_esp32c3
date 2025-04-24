#include "ui.h"


#include "esp_log.h"
#include "lvgl.h"


/*******************************************************
 *                change according to your app
 *******************************************************/


#define LCD_H_RES 	 (CONFIG_LCD_H_RES)//default (240)
#define LCD_V_RES 	 (CONFIG_LCD_V_RES)//default (320)

//ui

void ui() 
{
    lv_obj_set_style_bg_color(lv_screen_active(), lv_color_hex(0x003300), LV_PART_MAIN);
    lv_disp_set_rotation(lv_disp_get_default(), LV_DISP_ROTATION_90);

    //label_ssid
    lv_obj_t * label_hello = lv_label_create(lv_screen_active());

    lv_label_set_text(label_hello, "hello");
    lv_obj_set_style_text_color(label_hello, lv_color_hex(0xffffff), LV_PART_MAIN);
    lv_obj_align(label_hello, LV_ALIGN_CENTER, 0, 80);
    lv_obj_set_width(label_hello, LCD_H_RES);
    lv_obj_set_style_text_align(label_hello, LV_TEXT_ALIGN_CENTER, 0);
}
    
