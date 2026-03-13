

#ifndef __BSP_WS2812_DRIVER_H__
#define __BSP_WS2812_DRIVER_H__

/********************************** Includes *********************************/
#include <stdint.h>
#include "system_cfg.h"

/********************************** Defines **********************************/
/* The business layer keeps the historical ws2811/ws2812 API, but the current
 * F103 hardware baseline drives an SM16703P/TM16703 single-wire RGB strip.
 * The strip still uses an 800kHz single-wire return-to-zero code, so only the
 * bit high-time and reset low window need to change. */
#define WS2811_BITS_PER_LED     24	// a rgb have 24bit
#define WS2811_LED_NUM          SYSTEM_CFG_RGB_STRIP_LED_NUM
#define WS2811_RESET_SLOTS     192
#define WS2811_DATA_BUF_LEN    (WS2811_LED_NUM * WS2811_BITS_PER_LED)
#define WS2811_BUF_LEN         (WS2811_DATA_BUF_LEN + WS2811_RESET_SLOTS)

#define TIM3_ARR_VALUE          89	// 72MHz / (89 + 1) = 800KHz, a bit 1.25us
/* SM16703P/TM16703 timing window:
 * T0H ~= 0.2~0.4us, T1H ~= 0.8~1.0us, Trst >= 200us. */
#define DUTY_0                 (uint16_t)((((uint32_t)(TIM3_ARR_VALUE + 1u)) * 24u + 50u) / 100u)
#define DUTY_1                 (uint16_t)((((uint32_t)(TIM3_ARR_VALUE + 1u)) * 72u + 50u) / 100u)

#define WS2811_RING_DIR_S_TO_B 0
#define WS2811_RING_DIR_B_TO_S 1

#define WS2811_COLOR_ORDER_RGB 0u
#define WS2811_COLOR_ORDER_RBG 1u
#define WS2811_COLOR_ORDER_GRB 2u
#define WS2811_COLOR_ORDER_GBR 3u
#define WS2811_COLOR_ORDER_BRG 4u
#define WS2811_COLOR_ORDER_BGR 5u

/* Current SM16703 strip maps the second byte to blue and the third byte to
 * green, so the on-wire order is RBG rather than RGB. */
#define WS2811_COLOR_ORDER     WS2811_COLOR_ORDER_RBG

#define RGB_NONE      0x000000
#define RGB_WHITE     0xFFFFFF
#define RGB_RED       0xFF0000
#define RGB_GREEN     0x006400
#define RGB_BLUE      0x40e0D0	//0x0000FF
#define RGB_YELLOW    0xFFFF00	//0x646400	
#define RGB_PURPLE    0xFF00FF
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
