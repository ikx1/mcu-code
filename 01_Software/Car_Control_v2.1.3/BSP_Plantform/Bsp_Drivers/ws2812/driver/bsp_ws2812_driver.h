

#ifndef __BSP_WS2812_DRIVER_H__
#define __BSP_WS2812_DRIVER_H__

/********************************** Includes *********************************/
#include "spi.h"

/********************************** Defines **********************************/
#define WS2811_LED_NUM          8	// number of rgb
#define WS2811_BITS_PER_LED     24	// a rgb have 24bit
#define WS2811_BUF_LEN         (WS2811_LED_NUM * WS2811_BITS_PER_LED)

#define TIM3_ARR_VALUE          111	// 180MHz / (224 + 1) = 800KHz, a bit 1.25us
#define DUTY_0                 (uint16_t)(TIM3_ARR_VALUE * 0.2f) //the duty of ws2811 code0
#define DUTY_1                 (uint16_t)(TIM3_ARR_VALUE * 0.5f) //the duty of ws2811 code1

#define WS2811_RING_DIR_S_TO_B 0
#define WS2811_RING_DIR_B_TO_S 1

#define RGB_NONE      0x000000
#define RGB_WHITE     0xFFFFFF
#define RGB_RED       0xFF0000
#define RGB_GREEN     0x006400
#define RGB_BLUE      0x0000FF
#define RGB_YELLOW    0x646400	//0xFFFF00
#define RGB_PINK      0xFF69B4
#define RGB_CYAN      0x00FFFF

/********************************** Variables ********************************/
typedef enum {
	Red,
	Green,
	Blue,
	Yellow,
	Purple,
	Orange,
	Indigo,
	White,
} ColorName;

typedef struct RGB_Color
{
	uint8_t R;
	uint8_t G;
	uint8_t B;
}RGB_Color;

extern uint16_t ws2811_buf[WS2811_BUF_LEN];

/********************************** Functions ********************************/
void ws2811_show(void);
void ws2811_set_pixel(const uint8_t index, const uint32_t rgb);
void ws2811_all_same_color(const uint32_t rgb);
void ws2811_twinkle(const uint8_t start_index, const uint8_t end_index, \
											 const uint32_t rgb, \
											 const uint8_t freq_scaler);
void ws2811_fade(const uint8_t start_index, const uint8_t end_index, 
													const uint32_t rgb);
void ws2811_ring(const uint8_t start_index, const uint8_t end_index, \
                        			const uint8_t dir, const uint32_t rgb);

#endif /* __BSP_WS2812_DRIVER_H__ */
