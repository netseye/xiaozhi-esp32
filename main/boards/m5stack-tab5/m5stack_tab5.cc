#include "wifi_board.h"
#include "tab5_audio_codec.h"
#include "display/lcd_display.h"
#include "esp_lcd_ili9881c.h"
#include "font_awesome_symbols.h"
#include "font_emoji.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "iot/thing_manager.h"

#include <esp_log.h>
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_ops.h"
#include "esp_ldo_regulator.h"
#include <esp_lcd_panel_vendor.h>
#include <driver/i2c_master.h>
#include <driver/spi_common.h>
#include <wifi_station.h>
#include <esp_lvgl_port.h>
#include "i2c_device.h"
#include "esp_lcd_touch_gt911.h"
#include <cstring>


#define TAG "M5StackTab5Board"

LV_FONT_DECLARE(font_puhui_30_4);
LV_FONT_DECLARE(font_awesome_30_4);


#define AUDIO_CODEC_ES8388_ADDR ES8388_CODEC_DEFAULT_ADDR
#define LCD_MIPI_DSI_PHY_PWR_LDO_CHAN       3  // LDO_VO3 is connected to VDD_MIPI_DPHY
#define LCD_MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV 2500
#define I2C_MASTER_TIMEOUT_MS 50

// PI4IO registers
#define PI4IO_REG_CHIP_RESET 0x01
#define PI4IO_REG_IO_DIR     0x03
#define PI4IO_REG_OUT_SET    0x05
#define PI4IO_REG_OUT_H_IM   0x07
#define PI4IO_REG_IN_DEF_STA 0x09
#define PI4IO_REG_PULL_EN    0x0B
#define PI4IO_REG_PULL_SEL   0x0D
#define PI4IO_REG_IN_STA     0x0F
#define PI4IO_REG_INT_MASK   0x11
#define PI4IO_REG_IRQ_STA    0x13

#define setbit(x, y) x |= (0x01 << y)
#define clrbit(x, y) x &= ~(0x01 << y)

class Pi4ioe1 : public I2cDevice {
public:
    Pi4ioe1(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr)
    {
        WriteReg(PI4IO_REG_CHIP_RESET, 0xFF);
        uint8_t data = ReadReg(PI4IO_REG_CHIP_RESET);
        WriteReg(PI4IO_REG_IO_DIR, 0b01111111);      // 0: input 1: output
        WriteReg(PI4IO_REG_OUT_H_IM, 0b00000000);    // 使用到的引脚关闭 High-Impedance
        WriteReg(PI4IO_REG_PULL_SEL, 0b01111111);    // pull up/down select, 0 down, 1 up
        WriteReg(PI4IO_REG_PULL_EN, 0b01111111);     // pull up/down enable, 0 disable, 1 enable
        WriteReg(PI4IO_REG_IN_DEF_STA, 0b10000000);  // P1, P7 默认高电平
        WriteReg(PI4IO_REG_INT_MASK, 0b01111111);    // P7 中断使能 0 enable, 1 disable
        WriteReg(PI4IO_REG_OUT_SET, 0b01110110);     // Output Port Register P1(SPK_EN), P2(EXT5V_EN), P4(LCD_RST), P5(TP_RST), P6(CAM)RST 输出高电平
    }
    void ResetTouchPad()
    {
        ESP_LOGI(TAG, "reset tp");

        // Reset INT pin settings and set it to OUTPUT mode
        gpio_reset_pin(GPIO_NUM_23);
        gpio_set_direction(GPIO_NUM_23, GPIO_MODE_OUTPUT);

        // Read current output state
        uint8_t write_buf[2] = {0};
        uint8_t read_buf[1]  = {0};
        write_buf[0] = PI4IO_REG_OUT_SET;
        i2c_master_transmit_receive(i2c_device_, write_buf, 1, read_buf, 1, I2C_MASTER_TIMEOUT_MS);

        // Set P4(LCD_RST) and P5(TP_RST) to HIGH, set Touch INT to LOW
        write_buf[0] = PI4IO_REG_OUT_SET;
        write_buf[1] = read_buf[0];
        setbit(write_buf[1], 4);
        setbit(write_buf[1], 5);
        i2c_master_transmit(i2c_device_, write_buf, 2, I2C_MASTER_TIMEOUT_MS);
        gpio_set_level(GPIO_NUM_23, 0);
        vTaskDelay(50 / portTICK_PERIOD_MS);

        // Set P4(LCD_RST) and P5(TP_RST) to HIGH, set Touch INT to HIGH
        write_buf[0] = PI4IO_REG_OUT_SET;
        write_buf[1] = read_buf[0];
        setbit(write_buf[1], 4);
        setbit(write_buf[1], 5);
        i2c_master_transmit(i2c_device_, write_buf, 2, I2C_MASTER_TIMEOUT_MS);
        gpio_set_level(GPIO_NUM_23, 1);
        vTaskDelay(50 / portTICK_PERIOD_MS);

        // Reset INT pin settings
        gpio_reset_pin(GPIO_NUM_23);
    }
};

class Pi4ioe2 : public I2cDevice {
public:
    Pi4ioe2(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr)
    {
        WriteReg(PI4IO_REG_CHIP_RESET, 0xFF);
        uint8_t data = ReadReg(PI4IO_REG_CHIP_RESET);
        WriteReg(PI4IO_REG_IO_DIR, 0b10111001);      // 0: input 1: output
        WriteReg(PI4IO_REG_OUT_H_IM, 0b00000110);    // 使用到的引脚关闭 High-Impedance
        WriteReg(PI4IO_REG_PULL_SEL, 0b10111001);    // pull up/down select, 0 down, 1 up
        WriteReg(PI4IO_REG_PULL_EN, 0b11111001);     // pull up/down enable, 0 disable, 1 enable
        WriteReg(PI4IO_REG_IN_DEF_STA, 0b01000000);  // P6 默认高电平
        WriteReg(PI4IO_REG_INT_MASK, 0b10111111);    // P6 中断使能 0 enable, 1 disable
        WriteReg(PI4IO_REG_OUT_SET, 0b10001001);     // Output Port Register P0(WLAN_PWR_EN), P3(USB5V_EN), P7(CHG_EN) 输出高电平
    }
};

class CustomLcdDisplay : public MipiLcdDisplay {
public:
    CustomLcdDisplay(esp_lcd_panel_io_handle_t io_handle,
                    esp_lcd_panel_handle_t panel_handle,
                    int width,
                    int height,
                    int offset_x,
                    int offset_y,
                    bool mirror_x,
                    bool mirror_y,
                    bool swap_xy)
        : MipiLcdDisplay(io_handle, panel_handle,
                    width, height, offset_x, offset_y, mirror_x, mirror_y, swap_xy,
                    {
                        .text_font = &font_puhui_30_4,
                        .icon_font = &font_awesome_30_4,
#if CONFIG_USE_WECHAT_MESSAGE_STYLE
                        .emoji_font = font_emoji_32_init(),
#else
                        .emoji_font = font_emoji_64_init(),
#endif
                    }) {
        DisplayLockGuard lock(this);
        lv_obj_set_style_pad_left(status_bar_, LV_HOR_RES * 0.1, 0);
        lv_obj_set_style_pad_right(status_bar_, LV_HOR_RES * 0.1, 0);
        
        lv_obj_t *btn = lv_btn_create(content_);
        lv_obj_set_size(btn, 200, 80);
        lv_obj_align(btn, LV_ALIGN_CENTER, 0, 0);
        // 添加按钮点击事件
        lv_obj_add_event_cb(btn, [](lv_event_t *e) {
            printf("Button clicked!\n");
        }, LV_EVENT_CLICKED, NULL);
        // 设置文字
        lv_obj_t *label = lv_label_create(btn);
        lv_label_set_text(label, "Click Me");
        lv_obj_center(label);
    }
};
 
class M5StackTab5Board : public WifiBoard {
private:
    i2c_master_bus_handle_t i2c_bus_;
    Button boot_button_;
    LcdDisplay* display_;
    Pi4ioe1* pi4ioe1_;
    Pi4ioe2* pi4ioe2_;
    esp_lcd_touch_handle_t touch_ = nullptr;

    void InitializeI2c()
    {
        // Initialize I2C peripheral
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port          = (i2c_port_t)1,
            .sda_io_num        = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num        = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source        = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority     = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));
    }

    void I2cDetect()
    {
        uint8_t address;
        printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
        for (int i = 0; i < 128; i += 16) {
            printf("%02x: ", i);
            for (int j = 0; j < 16; j++) {
                fflush(stdout);
                address       = i + j;
                esp_err_t ret = i2c_master_probe(i2c_bus_, address, pdMS_TO_TICKS(200));
                if (ret == ESP_OK) {
                    printf("%02x ", address);
                } else if (ret == ESP_ERR_TIMEOUT) {
                    printf("UU ");
                } else {
                    printf("-- ");
                }
            }
            printf("\r\n");
        }
    }

    void InitializePi4ioe()
    {
        ESP_LOGI(TAG, "Init I/O Exapander PI4IOE");
        pi4ioe1_ = new Pi4ioe1(i2c_bus_, 0x43);
        pi4ioe2_ = new Pi4ioe2(i2c_bus_, 0x44);
    }

    void InitializeButtons()
    {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
                // ResetWifiConfiguration();
            }
            app.ToggleChatState();
        });
    }

    void InitializeGt911TouchPad() {
        ESP_LOGI(TAG, "Init GT911");
        if (pi4ioe1_) {
            pi4ioe1_->ResetTouchPad();
        } else {
            ESP_LOGE(TAG, "pi4ioe2_ is null, cannot reset touch pad");
        }
        /* Initialize Touch Panel */
        ESP_LOGI(TAG, "Initialize touch IO (I2C)");
        const esp_lcd_touch_config_t tp_cfg = {
            .x_max = DISPLAY_WIDTH,
            .y_max = DISPLAY_HEIGHT,
            .rst_gpio_num = GPIO_NUM_NC, 
            .int_gpio_num = TOUCH_INT_GPIO, 
            .levels = {
                .reset = 0,
                .interrupt = 0,
            },
            .flags = {
                .swap_xy = 0,
                .mirror_x = 0,
                .mirror_y = 0,
            },
        };
        esp_lcd_panel_io_handle_t tp_io_handle = NULL;
        esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
        tp_io_config.dev_addr = ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS_BACKUP; // 更改 GT911 地址 
        tp_io_config.scl_speed_hz = 100000;
        esp_lcd_new_panel_io_i2c(i2c_bus_, &tp_io_config, &tp_io_handle);
        esp_lcd_touch_new_i2c_gt911(tp_io_handle, &tp_cfg, &touch_);
        esp_lcd_touch_exit_sleep(touch_); 

        /* Add touch input (for selected screen) */
        const lvgl_port_touch_cfg_t touch_cfg = {
            .disp   = lv_display_get_default(),
            .handle = touch_,
        };

        lvgl_port_add_touch(&touch_cfg);
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = GPIO_NUM_37;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = GPIO_NUM_36;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeIli9881cDisplay()
    {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel       = nullptr;

        ESP_LOGI(TAG, "Turn on the power for MIPI DSI PHY");
        esp_ldo_channel_handle_t ldo_mipi_phy        = NULL;
        esp_ldo_channel_config_t ldo_mipi_phy_config = {
            .chan_id    = LCD_MIPI_DSI_PHY_PWR_LDO_CHAN,
            .voltage_mv = LCD_MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV,
        };
        ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_mipi_phy_config, &ldo_mipi_phy));

        ESP_LOGI(TAG, "Install MIPI DSI LCD control panel");
        esp_lcd_dsi_bus_handle_t mipi_dsi_bus;
        esp_lcd_dsi_bus_config_t bus_config = {
            .bus_id             = 0,
            .num_data_lanes     = 2,
            .phy_clk_src        = MIPI_DSI_PHY_CLK_SRC_DEFAULT,
            .lane_bit_rate_mbps = 900,  // 900MHz
        };
        ESP_ERROR_CHECK(esp_lcd_new_dsi_bus(&bus_config, &mipi_dsi_bus));

        ESP_LOGI(TAG, "Install panel IO");
        esp_lcd_dbi_io_config_t dbi_config = {
            .virtual_channel = 0,
            .lcd_cmd_bits    = 8,
            .lcd_param_bits  = 8,
        };
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_dbi(mipi_dsi_bus, &dbi_config, &panel_io));

        ESP_LOGI(TAG, "Install LCD driver of ili9881c");
      esp_lcd_dpi_panel_config_t dpi_config = {.virtual_channel    = 0,
                                                 .dpi_clk_src        = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
                                                 .dpi_clock_freq_mhz = 60,
                                                 .pixel_format       = LCD_COLOR_PIXEL_FORMAT_RGB565,
                                                 .num_fbs            = 2,
                                                 .video_timing =
                                                     {
                                                         .h_size            = DISPLAY_WIDTH,
                                                         .v_size            = DISPLAY_HEIGHT,
                                                         .hsync_pulse_width = 40,
                                                         .hsync_back_porch  = 140,
                                                         .hsync_front_porch = 40,
                                                         .vsync_pulse_width = 4,
                                                         .vsync_back_porch  = 20,
                                                         .vsync_front_porch = 20,
                                                     },
                                                 .flags = {
                                                     .use_dma2d = false,
                                                 }};

        ili9881c_vendor_config_t vendor_config = {
            .init_cmds      = tab5_lcd_ili9881c_specific_init_code_default,
            .init_cmds_size = sizeof(tab5_lcd_ili9881c_specific_init_code_default) /
                              sizeof(tab5_lcd_ili9881c_specific_init_code_default[0]),
            .mipi_config =
                {
                    .dsi_bus    = mipi_dsi_bus,
                    .dpi_config = &dpi_config,
                    .lane_num   = 2,
                },
        };

        esp_lcd_panel_dev_config_t lcd_dev_config = {};
        lcd_dev_config.rgb_ele_order              = LCD_RGB_ELEMENT_ORDER_RGB;
        lcd_dev_config.reset_gpio_num             = -1;
        lcd_dev_config.bits_per_pixel             = 16;
        lcd_dev_config.vendor_config              = &vendor_config;

        ESP_ERROR_CHECK(esp_lcd_new_panel_ili9881c(panel_io, &lcd_dev_config, &panel));
        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
        ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
        // ESP_ERROR_CHECK(esp_lcd_panel_mirror(disp_panel, false, true));
        ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));

        display_ = new CustomLcdDisplay(panel_io, panel, DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X,
                                      DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY
                                    //   ,
                                    //   {
                                    //       .text_font  = &font_puhui_30_4,
                                    //       .icon_font  = &font_awesome_30_4,
                                    //       .emoji_font = font_emoji_64_init(),
                                    //   }
                                    );
    }

    // 物联网初始化，添加对 AI 可见设备
    void InitializeIot() {
        auto& thing_manager = iot::ThingManager::GetInstance();
        thing_manager.AddThing(iot::CreateThing("Speaker"));
        thing_manager.AddThing(iot::CreateThing("Screen"));
    }

public:
    M5StackTab5Board() : boot_button_(BOOT_BUTTON_GPIO)
    {
        InitializeI2c();
        I2cDetect();
        InitializePi4ioe();
        InitializeIli9881cDisplay();
        InitializeGt911TouchPad();
        InitializeButtons();
        InitializeIot();
        GetBacklight()->RestoreBrightness();
    }

    virtual AudioCodec* GetAudioCodec() override
    {
        static Tab5AudioCodec audio_codec(i2c_bus_, 
                                        AUDIO_INPUT_SAMPLE_RATE, 
                                        AUDIO_OUTPUT_SAMPLE_RATE,
                                        AUDIO_I2S_GPIO_MCLK, 
                                        AUDIO_I2S_GPIO_BCLK, 
                                        AUDIO_I2S_GPIO_WS,
                                        AUDIO_I2S_GPIO_DOUT, 
                                        AUDIO_I2S_GPIO_DIN, 
                                        AUDIO_CODEC_PA_PIN,
                                        AUDIO_CODEC_ES8388_ADDR, 
                                        AUDIO_CODEC_ES7210_ADDR, 
                                        AUDIO_INPUT_REFERENCE);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override
    {
        return display_;
    }

    virtual Backlight* GetBacklight() override
    {
        static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        return &backlight;
    }
};

DECLARE_BOARD(M5StackTab5Board);
