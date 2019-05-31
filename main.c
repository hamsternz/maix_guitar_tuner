/* Copyright 2018 Canaan Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <stdio.h>
#include <math.h>
#include "dmac.h"
#include "bsp.h"
#include "fpioa.h"
#include "lcd.h"
#include "sysctl.h"
#include "nt35310.h"
#include "board_config.h"
#include "fft.h"
#include "i2s.h"
#include "sysctl.h"
#include "uarths.h"
#include "gpiohs.h"

uint16_t g_lcd_gram[LCD_X_MAX * LCD_Y_MAX] __attribute__((aligned(128)));

#define FRAME_LEN 128
#define AUDIO_SAMPLES 8192
int32_t rx_buf[AUDIO_SAMPLES];
uint32_t g_index;
uint32_t g_tx_len; 
#define PIN_WIFI  8
#define GPIO_WIFI 3

#define N_STRING (6)
#define SAMPLE_RATE (44100)
#define PI (3.1415928)

static const float freqs[N_STRING]            = { 164.814, 220.000, 293.665, 391.995, 493.883, 659.255};
static float phase_per_sample[N_STRING];
static float phase_from_tuned[N_STRING];
static float display_phase[N_STRING];
static float levels[N_STRING];
static float kernel_sin[N_STRING][2048];
static float kernel_cos[N_STRING][2048];

static void io_set_power(void)
{
#if BOARD_LICHEEDAN
    sysctl_set_power_mode(SYSCTL_POWER_BANK6, SYSCTL_POWER_V18);
    sysctl_set_power_mode(SYSCTL_POWER_BANK7, SYSCTL_POWER_V18);
#else
    sysctl_set_power_mode(SYSCTL_POWER_BANK1, SYSCTL_POWER_V18);
#endif
}
static void io_mux_init(void)
{
#if BOARD_LICHEEDAN
    /**
    *lcd_cs	36
    *lcd_rst  	37
    *lcd_dc	38
    *lcd_wr 	39
    * */
    fpioa_set_function(37, FUNC_GPIOHS0 + RST_GPIONUM);
    fpioa_set_function(38, FUNC_GPIOHS0 + DCX_GPIONUM);
    fpioa_set_function(36, FUNC_SPI0_SS3);
    fpioa_set_function(39, FUNC_SPI0_SCLK);
    sysctl_set_spi0_dvp_data(1);
#else
    fpioa_set_function(8, FUNC_GPIOHS0 + DCX_GPIONUM);
    fpioa_set_function(6, FUNC_SPI0_SS3);
    fpioa_set_function(7, FUNC_SPI0_SCLK);
    sysctl_set_spi0_dvp_data(1);
#endif
    //mic
    fpioa_set_function(30, FUNC_I2S0_WS);
    fpioa_set_function(20, FUNC_I2S0_IN_D0);
    fpioa_set_function(32, FUNC_I2S0_SCLK);
}

void update_display(void) {
    int x, y, i, max_l = 0;
    float max = levels[0];
    for(i = 1; i < N_STRING; i++) { 
      if(levels[i] > max) {
         max = levels[i];
         max_l = i;
      }
    }
    if(max < (float)SAMPLE_RATE*SAMPLE_RATE/50) {
      max_l = -1;
    } 

    for(i = 0; i < N_STRING; i++) {

       for(y = 0; y < LCD_Y_MAX; y++) {
          for(x = i*30+30; x < i*30+30+17; x++) {
            g_lcd_gram[y*LCD_X_MAX+x]  = 0x0000;
          }
       }
       y = display_phase[i] * LCD_Y_MAX/2;
       y = phase_from_tuned[i] * LCD_Y_MAX+LCD_Y_MAX/2;
       if(y>=0 && y <= LCD_Y_MAX) {
         for(x = i*30+30; x < i*30+30+17; x++) {
           if(max_l == i) {
             if(y > 0) g_lcd_gram[(y-1)*LCD_X_MAX+x] = 0xFFFF;
             g_lcd_gram[y*LCD_X_MAX+x] = 0xFFFF;
             if(y < LCD_Y_MAX-1) g_lcd_gram[(y+1)*LCD_X_MAX+x] = 0xFFFF;
           }
         }
       }
       
    }
    for(i = 0; i < N_STRING; i++) {
      display_phase[i] += phase_from_tuned[i]/5;
      /* Clamp between 1.0 and zero */
      if(display_phase[i] >= 1.0)
         display_phase[i] -= 1.0;
      if(display_phase[i] < 0.0)
         display_phase[i] += 1.0;
      
    }

    lcd_ram_draw_char( 36, 10, 'E', RED, (uint32_t *)g_lcd_gram);
    lcd_ram_draw_char( 66, 10, 'A', RED, (uint32_t *)g_lcd_gram);
    lcd_ram_draw_char( 96, 10, 'D', RED, (uint32_t *)g_lcd_gram);
    lcd_ram_draw_char(126, 10, 'G', RED, (uint32_t *)g_lcd_gram);
    lcd_ram_draw_char(156, 10, 'B', RED, (uint32_t *)g_lcd_gram);
    lcd_ram_draw_char(186, 10, 'E', RED, (uint32_t *)g_lcd_gram);
    lcd_draw_picture(0, 0, LCD_X_MAX, LCD_Y_MAX, (uint32_t *)g_lcd_gram);

}

void i2s_setup(void) {
    sysctl_pll_set_freq(SYSCTL_PLL0, 320000000UL);
    sysctl_pll_set_freq(SYSCTL_PLL1, 160000000UL);
    sysctl_pll_set_freq(SYSCTL_PLL2, 45158400UL);
    
    i2s_init(I2S_DEVICE_0, I2S_RECEIVER, 0x3);
    i2s_init(I2S_DEVICE_2, I2S_TRANSMITTER, 0xC);
    i2s_rx_channel_config(I2S_DEVICE_0, I2S_CHANNEL_0,
                          RESOLUTION_16_BIT, SCLK_CYCLES_32,
                          TRIGGER_LEVEL_4, STANDARD_MODE);

    i2s_tx_channel_config(I2S_DEVICE_2, I2S_CHANNEL_1,
                          RESOLUTION_16_BIT, SCLK_CYCLES_32,
                          TRIGGER_LEVEL_4,
                          RIGHT_JUSTIFYING_MODE);

}

void receive_audio(void) {
    g_index = 0;

    // Throw away the first buffer of data
    i2s_receive_data_dma(I2S_DEVICE_0, (uint32_t *)(&rx_buf[g_index]), FRAME_LEN * 2, DMAC_CHANNEL1);
    while (g_index < AUDIO_SAMPLES)
    {
        i2s_receive_data_dma(I2S_DEVICE_0, (uint32_t *)(&rx_buf[g_index]), FRAME_LEN * 2, DMAC_CHANNEL1);
        g_index += (FRAME_LEN * 2);
    }
}

void do_dsp(int offset, int length);

int main(void)
{
    uint64_t core_id = current_coreid();
    int x,y;

    for(y = 0; y < LCD_Y_MAX; y++) {
       for(x = 0; x <  LCD_X_MAX; x++) {
          g_lcd_gram[y*LCD_X_MAX+x] = 0x000F;
       }
    }

    for(x = 0; x <  LCD_X_MAX; x++) {
       g_lcd_gram[(LCD_Y_MAX/2+0)*LCD_X_MAX+x] = 0xFFFF;
       g_lcd_gram[(LCD_Y_MAX/2-1)*LCD_X_MAX+x] = 0xFFFF;
    }

    if (core_id == 0)
    {
        io_mux_init();
        io_set_power();
        i2s_setup();
        uarths_init();
        lcd_init();
#if BOARD_LICHEEDAN
        lcd_set_direction(DIR_XY_LRDU);
#endif
    }

    update_display();
    while (1) {
        receive_audio();
        do_dsp(2000, 2000);
        update_display();
    }
}

#define MAX_REF_LENGTH 2048

void do_dsp(int offset, int length) {
   static int first = 1;
   int i,s;

   if(first) {
      /***********************************
      * Build reference sin/cos waves
      ***********************************/
      for(i = 0; i < N_STRING; i++) {
         int j;
         phase_per_sample[i] = freqs[i]/SAMPLE_RATE;       
         phase_from_tuned[i] = 0.0;
         display_phase[i]    = 0; 
         for(j = 0; j < MAX_REF_LENGTH; j++) {
            kernel_cos[i][j] += cos(j * phase_per_sample[i]*PI*2);
            kernel_sin[i][j] += sin(j * phase_per_sample[i]*PI*2);
         }
      }
      first = 0;
   }

   for(s = 0; s < N_STRING; s++) {
      int i;
      float x1 = 0.0, y1 = 0.0;
      float x2 = 0.0, y2 = 0.0;
      float a1, a2, diff;


      /* Find the phase angle at the start of the buffer */
      for(i = 0; i < length; i++) {
         x1 += kernel_cos[s][i] * rx_buf[2*i];
         y1 += kernel_sin[s][i] * rx_buf[2*i];
      }
      a1 = atan2f(y1,x1)/(2*PI);
      levels[s] = x1*x1+y1*y1;
 
      /* Find the phase angle at an offset through the buffer */
      x2 = 0; y2 = 0;
      for(i = 0; i < length; i++) {
        x2 += kernel_cos[s][i] * rx_buf[2*(i+offset)];
        y2 += kernel_sin[s][i] * rx_buf[2*(i+offset)];
      }
      a2 = atan2f(y2,x2)/(2*PI);
 
      /* adjust the phase angle of the early signal by 'offset' samples */
      a1 -= phase_per_sample[s]*offset-floor(phase_per_sample[s]*offset);
      if(a2 < 0.0) a2 += 1.0;
      if(a2 < 0.0) a2 += 1.0;
      if(a1 < 0.0) a1 += 1.0;

      /* Work out the phase difference */ 
      diff = a2-a1;

      /* Clamp the phase angle to +/- 0.5 */ 
      if(diff >= 0.5) diff -= 1.0;
      if(diff < -0.5) diff += 1.0;
      phase_from_tuned[s] = diff; 
   }  
}
