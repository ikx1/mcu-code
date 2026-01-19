

/********************************** Includes *********************************/
#include "bsp_ws2812_driver.h"
#include <string.h>
#include <math.h>

/********************************** Defines **********************************/


/********************************** Variables ********************************/
extern TIM_HandleTypeDef htim3;
uint8_t ws2811_colors[WS2811_LED_NUM][3] = {0};
uint16_t ws2811_buf[WS2811_BUF_LEN];
//static const uint16_t ws2811_reset_buf[240] = {0};


/********************************** Functions ********************************/
static void ws2811_encode(const uint8_t *rgb, uint32_t len)
{
    uint32_t i, j, k = 0;
    for (i = 0; i < len; ++i)
    {
        uint8_t byte = rgb[i];
        for (j = 0; j < 8; ++j)
        {
            ws2811_buf[k++] = (byte & 0x80) ? DUTY_1 : DUTY_0;
            byte <<= 1;
        }
    }
}

//static void ws2811_reset(void)
//{
//	HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_3, (uint32_t *)ws2811_reset_buf, 240);
//}

void ws2811_show(void)
{

	//ws2811_reset();
	
	ws2811_encode((uint8_t*)ws2811_colors, WS2811_LED_NUM * 3);
	
	HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_3, (uint32_t *)ws2811_buf, sizeof(ws2811_buf) / 2);
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM3)
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
		HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_3);
	}
}

void ws2811_set_pixel(const uint8_t index, const uint32_t rgb)
{
	  if (index >= WS2811_LED_NUM)
		{
			return;
		}
		
	  uint8_t r = (rgb >> 16) & 0xFF;
    uint8_t g = (rgb >> 8)  & 0xFF;
    uint8_t b = (rgb)       & 0xFF;
		
    ws2811_colors[index][0] = r;
    ws2811_colors[index][1] = g;
    ws2811_colors[index][2] = b;
}

void ws2811_all_same_color(const uint32_t rgb)
{
    uint8_t r = (rgb >> 16) & 0xFF;
    uint8_t g = (rgb >> 8)  & 0xFF;
    uint8_t b = (rgb)       & 0xFF;
	  for (uint8_t i = 0; i < WS2811_LED_NUM; i++)
	  {
		  ws2811_colors[i][0] = r;
		  ws2811_colors[i][1] = g;
		  ws2811_colors[i][2] = b;
	  }
}

/* twinkle interval = freq_scaler * task time */
void ws2811_twinkle(const uint8_t start_index, const uint8_t end_index, \
										const uint32_t rgb, const uint8_t freq_scaler)
{	
	  if ((start_index > end_index) || (end_index > WS2811_LED_NUM))
	  {
		  return;
	  }
	
	  memset(ws2811_colors, 0, sizeof(ws2811_colors));
	
		static uint8_t twinkle_count = 0;
	  static uint8_t twinkle_state = 1;
  
		if (twinkle_count >= freq_scaler)
		{	
			twinkle_state = !twinkle_state;
			twinkle_count = 0;
		}
	  
		for (uint8_t i = start_index; i <= end_index; ++i)
		{
			if (twinkle_state)
			{
				ws2811_set_pixel(i, rgb);
			}
			else 
			{
				ws2811_set_pixel(i, RGB_NONE);
			}
			
		}
		
		twinkle_count += 1;	
}

void ws2811_fade(const uint8_t start_index, const uint8_t end_index, const uint32_t rgb)
{
    if ((start_index > end_index) || (end_index > WS2811_LED_NUM))
	  {
		  return;
	  }
	
    uint8_t r = (rgb >> 16) & 0xFF;
    uint8_t g = (rgb >> 8)  & 0xFF;
    uint8_t b = (rgb)       & 0xFF;
	 
	  static float t = 0.0f;
	  float brightness = 0.5f * (1.0f + sinf(t));
    if (brightness < 0.0f)
	  {
		  brightness = 0.0f;
	  }		 
    if (brightness > 1.0f)
	  {
		  brightness = 1.0f;
	  }
  
    memset(ws2811_colors, 0, sizeof(ws2811_colors));
	
    uint8_t r_out = (uint8_t)(r * brightness);
    uint8_t g_out = (uint8_t)(g * brightness);
    uint8_t b_out = (uint8_t)(b * brightness);
	 
	  for (uint16_t i = start_index; i <= end_index; ++i)
    {
      ws2811_colors[i][0] = r_out;
      ws2811_colors[i][1] = g_out;
      ws2811_colors[i][2] = b_out;
    }
	
    t += 0.08f; // change this value if want to change the speed of breath
    if (t > 6.28f)
	  {
		  t = 0.0f;
	  }   
}

/*
ring mode:
000...000 -> 100...000 -> 010...000 -> 001...000 -> ... -> 000...100 -> 000...010 -> 000...001
*/
void ws2811_ring(const uint8_t start_index, const uint8_t end_index, \
                 const uint8_t dir, const uint32_t rgb)
{
	  if ((start_index > end_index) || (end_index > WS2811_LED_NUM) ||
			  ((dir != WS2811_RING_DIR_S_TO_B) && (dir != WS2811_RING_DIR_B_TO_S)))
	  {
		  return;
	  }
	
    uint8_t r = (rgb >> 16) & 0xFF;
    uint8_t g = (rgb >> 8)  & 0xFF;
    uint8_t b = (rgb)       & 0xFF;
		
		static uint16_t cur_index    = 0;
		
		if ((cur_index < start_index) || (cur_index > end_index))
		{
			cur_index = (dir == WS2811_RING_DIR_S_TO_B) ? start_index : end_index;
		}
		
		//memset(&ws2811_colors[start_index][0], 0, (end_index - start_index + 1) * 3);
		memset(ws2811_colors, 0, sizeof(ws2811_colors));
		
		ws2811_colors[cur_index][0] = r;
    ws2811_colors[cur_index][1] = g;
    ws2811_colors[cur_index][2] = b;
		
		if (dir == WS2811_RING_DIR_S_TO_B)
    {
      cur_index = (cur_index >= end_index) ? start_index : (cur_index + 1);
    }
    else
    {
      cur_index = (cur_index <= start_index) ? end_index : (cur_index - 1);
    }
}
