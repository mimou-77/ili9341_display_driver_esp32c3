#include "ili9341_display_driver.h"
#include "ui.h"


#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <sys/lock.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"
#include "esp_lcd_ili9341.h"



/*******************************************************
 *                macros
 *******************************************************/
//add later to menuconfig
#define LCD_H_RES 	 (CONFIG_LCD_H_RES)//default (240)
#define LCD_V_RES 	 (CONFIG_LCD_V_RES)//default (320)

#define LCD_PIXEL_CLK_HZ 	 (20 * 1000 * 1000)
#define LCD_CMD_BITS 	 8
#define LCD_PARAM_BITS 	 8
#define LCD_COLOR_SPACE 	 (ESP_LCD_COLOR_SPACE_BGR)
#define LCD_BITS_PER_PIXEL 	 (16)
#define LCD_DRAW_BUFF_DOUBLE 	 (1)
#define LCD_DRAW_BUFF_HEIGHT 	 20 //= draw buffer lines = nbr of display lines in each draw buffer
#define LCD_BL_ON_LEVEL 	 1
#define LCD_BL_OFF_LEVEL 	 !LCD_BL_ON_LEVEL

#if  CONFIG_SPI_PERIPH_SPI2
#define LCD_SPI_NUM 	 SPI2_HOST //default SPI2_HOST
#endif

#define LCD_GPIO_SCLK 	 CONFIG_LCD_GPIO_SCLK //default 18
#define LCD_GPIO_MOSI 	 CONFIG_LCD_GPIO_MOSI//default 19
#define LCD_GPIO_MISO    CONFIG_LCD_GPIO_MISO //default -1
#define LCD_GPIO_RST 	 CONFIG_LCD_GPIO_RST //default 3
#define LCD_GPIO_DC 	 CONFIG_LCD_GPIO_DC //default 5
#define LCD_GPIO_CS 	 CONFIG_LCD_GPIO_CS //default 4
#define LCD_GPIO_BL 	 CONFIG_LCD_GPIO_BL //default 2


#define LVGL_DRAW_BUF_HEIGHT     20	 
#define LVGL_TICK_PERIOD_MS 	 2    //lvgl timer period?
#define LVGL_TASK_MAX_DELAY_MS 	 500 //max delay inside the lvgl task before the watchdog timer wakes it up
#define LVGL_TASK_MIN_DELAY_MS 	 1
#define LVGL_TASK_STACK_SIZE 	 (4 * 1024)
#define LVGL_TASK_PRIORITY 	 2


/*******************************************************
 *                global variables
 *******************************************************/
static const char *TAG = "display_driver";

static _lock_t lvgl_api_lock; //lvgl APIs will br called from different tasks => mutex ; before every lvgl API call: aquire lock 

// /*******************************************************
//  *                functions implementations
//  *******************************************************/
/**
 * @brief - callback that executes when flushing the lv draw buffers content to the panel is finished (lv display is flush-ready i.e: can be flushed again)
 *        - informs lvgl that the lv display is flush-ready
 * @param panel_io 
 * @param data 
 * @param user_ctx 
 * @return 
 */
static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *data, void * user_ctx)
{
    lv_display_t * disp = (lv_display_t *)user_ctx;
    
    lv_display_flush_ready(disp); //informs lvgl that display can be re-flushed
    return false;
}


/**
 * @brief called inside the flush cb ; checks the panel rotation and applies it
 * @param disp 
 */
static void lvgl_port_update_callback(lv_display_t *disp)
{
    esp_lcd_panel_handle_t panel_handle = lv_display_get_user_data(disp);
    lv_display_rotation_t rotation = lv_display_get_rotation(disp);

    switch (rotation) {
        case LV_DISPLAY_ROTATION_0:
            // Rotate LCD display
            esp_lcd_panel_swap_xy(panel_handle, false);
            esp_lcd_panel_mirror(panel_handle, true, false);
            break;
        case LV_DISPLAY_ROTATION_90:
            // Rotate LCD display
            esp_lcd_panel_swap_xy(panel_handle, true);
            esp_lcd_panel_mirror(panel_handle, true, true);
            break;
        case LV_DISPLAY_ROTATION_180:
            // Rotate LCD display
            esp_lcd_panel_swap_xy(panel_handle, false);
            esp_lcd_panel_mirror(panel_handle, false, true);
            break;
        case LV_DISPLAY_ROTATION_270:
            // Rotate LCD display
            esp_lcd_panel_swap_xy(panel_handle, true);
            esp_lcd_panel_mirror(panel_handle, false, false);
            break;
        }
}


/**
 * @brief cb that executes when lv_display is flush ready ; does the flushing
 * @param disp 
 * @param area 
 * @param px_map 
 */
static void lvgl_flush_cb(lv_display_t * disp, const lv_area_t * area, uint8_t * px_map)
{
    lvgl_port_update_callback(disp); //updates the display rotation
    esp_lcd_panel_handle_t panel_handle = lv_display_get_user_data(disp);
    
    //does the flushing :  copies lv draw buffer content to the panel area (via spi)
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    // because SPI LCD is big-endian, we need to swap the RGB bytes order
    lv_draw_sw_rgb565_swap(px_map, (offsetx2 + 1 - offsetx1) * (offsety2 + 1 - offsety1));
    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, px_map);
}


/**
 * @brief callbak ; executed when the periodic esp timer expires each 2 ms ; lvgl_time+=lv_period
 * @param arg 
 */
static void increase_lvgl_tick(void * arg)
{
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
} 


/**
 * @brief handles lv timers each 1000 * next timer expiration time ; (updates interface and other lv things)
 *        continuously parses lv timers, excutes cbs of expired lv timers, returns time until next timer expires 
 * @param arg 
 */
static void lvgl_port_task(void * arg)
{
    ESP_LOGI(TAG, "starting lvgl task");
    uint32_t time_till_next_ms = 0;
    uint32_t time_threashold_ms = 1000 / CONFIG_FREERTOS_HZ; // = 1000 * rtos_period ; 
    while(1)
    {
        _lock_acquire(&lvgl_api_lock);
        time_till_next_ms = lv_timer_handler();  //parses lv timers, executes the callbacks of the expired ones, and returns next time until
        _lock_release(&lvgl_api_lock);

        time_till_next_ms = MAX(time_till_next_ms, time_threashold_ms); //next timer expiration time must be below a threashold to avoid watchdog problems 
        usleep(1000 * time_till_next_ms); //delay
    } 
}


void display_init()
{
    ESP_LOGI(TAG, "config bl pin");
    gpio_config_t bl_config =
    {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << LCD_GPIO_BL,
    };
    ESP_ERROR_CHECK(gpio_config(&bl_config));

    ESP_LOGI(TAG, "init spi bus");
    spi_bus_config_t buscfg =
    {
        .sclk_io_num = LCD_GPIO_SCLK,
        .mosi_io_num = LCD_GPIO_MOSI,
        .miso_io_num = LCD_GPIO_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * 80 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_SPI_NUM, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "init lcd panel io");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = 
    {
        .dc_gpio_num = LCD_GPIO_DC,
        .cs_gpio_num = LCD_GPIO_CS,
        .pclk_hz = LCD_PIXEL_CLK_HZ,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_SPI_NUM,&io_config, &io_handle));

    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config =
    {
        .reset_gpio_num = LCD_GPIO_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };

    ESP_LOGI(TAG, "init ili9341 driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));
    
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true)); //lcd ON
    gpio_set_level(LCD_GPIO_BL, LCD_BL_ON_LEVEL); //lcd bl ON

    
    //lvgl part 

    ESP_LOGI(TAG, "init lvgl");
    lv_init();

    lv_display_t *display = lv_display_create(LCD_H_RES, LCD_V_RES);

    size_t draw_buffer_sz = LCD_H_RES * LVGL_DRAW_BUF_HEIGHT * sizeof(lv_color16_t);
    void * buf1 = spi_bus_dma_memory_alloc(LCD_SPI_NUM, draw_buffer_sz, 0);
    assert(buf1);
    void * buf2 = spi_bus_dma_memory_alloc(LCD_SPI_NUM, draw_buffer_sz, 0);
    assert(buf2);
    lv_display_set_buffers(display, buf1, buf2, draw_buffer_sz, LV_DISPLAY_RENDER_MODE_PARTIAL);
    
    lv_display_set_user_data(display, panel_handle); //attach lv_display to panel_handle
    lv_display_set_color_format(display, LV_COLOR_FORMAT_RGB565);
    lv_display_set_flush_cb(display, lvgl_flush_cb); //when lv_display is flush ready (and lvgl notified) => does the flushing the the panel (via spi)  

    ESP_LOGI(TAG, "init lv tick_timer"); //one of the timers is the tick timer
    const esp_timer_create_args_t lvgl_tick_timer_args =
    {
        .callback = &increase_lvgl_tick,
        .name = "lvgl tick_timer",
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));

    ESP_LOGI(TAG, "attach callback to lv_display event: \"display is flush ready\" ie: transfer from lv buffers is finished; cb = notify lvgl");
    const esp_lcd_panel_io_callbacks_t cbs = 
    {
        .on_color_trans_done = notify_lvgl_flush_ready,
    };
    ESP_ERROR_CHECK(esp_lcd_panel_io_register_event_callbacks(io_handle, &cbs, display));

    ESP_LOGI(TAG, "create lvgl task");
    xTaskCreate(lvgl_port_task, "lvgl_task", LVGL_TASK_STACK_SIZE, NULL, LVGL_TASK_PRIORITY, NULL);



    //ui
    ESP_LOGI(TAG, "start ui ");

    _lock_acquire(&lvgl_api_lock); //start ui

    ui();

    _lock_release(&lvgl_api_lock); //end ui
    
}
