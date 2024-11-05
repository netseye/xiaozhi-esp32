#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include <driver/gpio.h>

#define AUDIO_INPUT_SAMPLE_RATE  24000
#define AUDIO_OUTPUT_SAMPLE_RATE 24000
#define AUDIO_DEFAULT_OUTPUT_VOLUME 70

#define AUDIO_INPUT_REFERENCE    true

#define AUDIO_I2S_GPIO_MCLK GPIO_NUM_0
#define AUDIO_I2S_GPIO_LRCK GPIO_NUM_47
#define AUDIO_I2S_GPIO_BCLK GPIO_NUM_48
#define AUDIO_I2S_GPIO_DIN  GPIO_NUM_45
#define AUDIO_I2S_GPIO_DOUT GPIO_NUM_21

#define AUDIO_CODEC_PA_PIN       GPIO_NUM_40
#define AUDIO_CODEC_I2C_SDA_PIN  GPIO_NUM_39
#define AUDIO_CODEC_I2C_SCL_PIN  GPIO_NUM_38
#define AUDIO_CODEC_ES8311_ADDR  ES8311_CODEC_DEFAULT_ADDR
#define AUDIO_CODEC_ES7210_ADDR  ES7210_CODEC_DEFAULT_ADDR

#define BUILTIN_LED_GPIO        GPIO_NUM_8
#define BOOT_BUTTON_GPIO        GPIO_NUM_0
#define VOLUME_UP_BUTTON_GPIO   GPIO_NUM_6
#define VOLUME_DOWN_BUTTON_GPIO GPIO_NUM_7

#define DISPLAY_SDA_PIN GPIO_NUM_4
#define DISPLAY_SCL_PIN GPIO_NUM_5
#define DISPLAY_WIDTH   128
#define DISPLAY_HEIGHT  64
#define DISPLAY_MIRROR_X true
#define DISPLAY_MIRROR_Y true

#define ML307_RX_PIN GPIO_NUM_17
#define ML307_TX_PIN GPIO_NUM_16


#endif // _BOARD_CONFIG_H_
