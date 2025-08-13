#include "../SSD1322_OLED_lib/SSD1322.h"
#include "../SSD1322_OLED_lib/Fonts/FreeMono12pt7b.h"
#include "../SSD1322_OLED_lib/Fonts/FreeSansOblique9pt7b.h"
#include "../SSD1322_OLED_lib/Tests/tom_and_jerry.h"
#include "../SSD1322_OLED_lib/Tests/creeper.h"
#include "../SSD1322_OLED_lib/Tests/krecik.h"
#include "../SSD1322_OLED_lib/Tests/pat_i_mat.h"
#include "../SSD1322_OLED_lib/Tests/stars_4bpp.h"
#include "../SSD1322_OLED_lib/Tests/WSC_SSE.h"
#include "../SSD1322_OLED_lib/Tests/Layout_2.h"
#include "../SSD1322_OLED_lib/Tests/Signal_Lights.h"

#include <stdio.h>
#include <string.h>

uint8_t OLED_Bright = 0xf;

void SSD1322_Layout_init(uint8_t *tx_buf){
  draw_string_tiny(tx_buf, "CC:", 26, 1); 
  draw_string_tiny(tx_buf, "DM:", 72, 1);
  draw_string_tiny(tx_buf, "RG:", 72, 13);
  draw_string_tiny(tx_buf, "MT:", 102, 1);
  draw_string_tiny(tx_buf, "DT:", 102, 13);
  draw_string_tiny(tx_buf, "V:", 160, 1);
  draw_string_tiny(tx_buf, "A:", 160, 13);
  draw_string_tiny(tx_buf, "SOC", 209, 1);
  draw_string_tiny(tx_buf, "km/h", 100, 49);

  char buf_per[1];
  sprintf(buf_per, "%%");
  draw_string_tiny(tx_buf, buf_per, 225, 13);

  draw_AA_line(tx_buf, 0, 25, 256, 25, 15);
  draw_AA_line(tx_buf, 207, 0, 207, 25, 15);
}

void SSD1322_Layout(uint8_t *tx_buf, float* data_buf){
  fill_buffer(tx_buf, 0);
  SSD1322_Layout_init(tx_buf);

  update_brightness((uint8_t)data_buf[OLED_Brightness]);

  char buf[20][7];
  for(int i = 3; i <20; i++){
    switch(i){
      case OLED_Target_Speed: case OLED_Motor_T: case OLED_D_T: 
      case OLED_Voltage: case OLED_Current: case OLED_Velocity: //0.1 float
        if(data_buf[i]<1000)
          sprintf(buf[i], "%0.1f", data_buf[i]);
        else
          sprintf(buf[i], "999.9");
        break;

      case OLED_Regen_Lv: case OLED_Cruise_Control: //int < 10;
        if(data_buf[i]<10)
          sprintf(buf[i], "%0.0f", data_buf[i]);
        else
          sprintf(buf[i], "9");
        break;
      
        

      case OLED_SOC:
        if(data_buf[i]<100)
          sprintf(buf[i], "%0.0f", data_buf[i]);
        else
          sprintf(buf[i], "99");
        break;
    }
  }

  if(data_buf[OLED_Cruise_Control] == 1){
    draw_string_tiny(tx_buf, "ON", 44, 1);  
    draw_string_tiny(tx_buf, buf[OLED_Target_Speed], 26, 13);
  }
  else{
    draw_string_tiny(tx_buf, "OFF", 44, 1);
    
  }
  
  
  if(data_buf[OLED_Drive_Mode]== 0){
    draw_string_tiny(tx_buf, "R", 90, 1);
  }
  else if(data_buf[OLED_Drive_Mode]== 1){
    draw_string_tiny(tx_buf, "D", 90, 1);
  }
  else{
    draw_string_tiny(tx_buf, "N", 90, 1);
  }
  draw_string_tiny(tx_buf, buf[OLED_Regen_Lv], 90, 13);
  draw_string_tiny(tx_buf, buf[OLED_Motor_T], 122, 1);
  draw_string_tiny(tx_buf, buf[OLED_D_T], 122, 13);
  draw_string_tiny(tx_buf, buf[OLED_Voltage], 172, 1);
  draw_string_tiny(tx_buf, buf[OLED_Current], 172, 13);
  draw_string_tiny(tx_buf, buf[OLED_SOC], 209, 13);
  draw_string_tiny_3x(tx_buf, buf[OLED_Velocity], 5, 30);

  if(data_buf[OLED_L_TS])
    draw_string_tiny_2x(tx_buf, "<", 1, 1);
  if(data_buf[OLED_R_TS])
    draw_string_tiny_2x(tx_buf, ">", 232, 1);
  if(data_buf[OLED_H_TS]){
    draw_string_tiny_2x(tx_buf, "<", 1, 1);
    draw_string_tiny_2x(tx_buf, ">", 232, 1);
  }
  
  //draw_bitmap_4bpp(tx_buf, Left_signal, 1, 1, 21, 23);
  //draw_bitmap_4bpp(tx_buf, Right_signal, 234, 1, 21, 23);

  send_buffer_to_OLED(tx_buf, 0, 0);
  
}


void SSD1322_MPPT_energy(uint8_t *tx_buf,
                         float *mppt_display[3][4]){
    char buffer[8];                       // "9999.9W\0" 까지 여유
    draw_string_tiny(tx_buf, "MPPT:", 1, 1);
    draw_string_tiny(tx_buf, "IN V:", 1, 12);
    draw_string_tiny(tx_buf, "IN I:", 1, 22);
    draw_string_tiny(tx_buf, "OU V:", 1, 34);
    draw_string_tiny(tx_buf, "OU I:", 1, 44);
    draw_string_tiny(tx_buf, "POW :", 1, 54);

    for (int i = 0; i < 3; ++i)
    {
      /* 각 채널 머리글 */
      snprintf(buffer, sizeof buffer, "[%d]", i);
      draw_string_tiny(tx_buf, buffer, 45 + 65 * i, 1);

      /* IN V */
      snprintf(buffer, sizeof buffer, "%.1fV", *mppt_display[i][0]);
      draw_string_tiny(tx_buf, buffer, 45 + 65 * i, 12);

      snprintf(buffer, sizeof buffer, "%.1fA", *mppt_display[i][1]);
      draw_string_tiny(tx_buf, buffer, 45+65*i, 22);

      snprintf(buffer, sizeof buffer, "%.1fV", *mppt_display[i][2]);
      draw_string_tiny(tx_buf, buffer, 45+65*i, 34);

      snprintf(buffer, sizeof buffer, "%.1fA", *mppt_display[i][3]);
      draw_string_tiny(tx_buf, buffer, 45+65*i, 44);

      float power = (*mppt_display[i][2])*(*mppt_display[i][3]);
      snprintf(buffer, sizeof buffer, "%.1fW", power);
      draw_string_tiny(tx_buf, buffer, 45+65*i, 54);
  }
  send_buffer_to_OLED(tx_buf, 0, 0);
}

void SSD1322(uint8_t *tx_buf, bool cc){
    set_buffer_size(256, 64);
    if(cc)
      fill_buffer(tx_buf, 0xf);
    else
      fill_buffer(tx_buf, 0);
    send_buffer_to_OLED(tx_buf, 0, 0);
}

void SSD1322_test(uint8_t * tx_buf){
    set_buffer_size(256, 64);
    // Fill buffer with zeros to clear any garbage values
    fill_buffer(tx_buf, 0);

    // send a frame buffer to the display
    send_buffer_to_OLED(tx_buf, 0, 0);
    HAL_Delay(2000);

    // Let's try some features of this OLED display

    // First, draw some pixels on frame buffer
    // draw_pixel(frame_buffer, x, y, brightness);
    draw_pixel(tx_buf, 10, 10, 1);
    draw_pixel(tx_buf, 115, 15, 5);
    draw_pixel(tx_buf, 220, 20, 9);
    draw_pixel(tx_buf, 225, 25, 15);
    draw_pixel(tx_buf, 230, 30, 9);
    draw_pixel(tx_buf, 235, 35, 5);
    draw_pixel(tx_buf, 240, 40, 1);
    draw_pixel(tx_buf, 245, 45, 5);
    draw_pixel(tx_buf, 250, 50, 9);
    draw_pixel(tx_buf, 255, 55, 15);
    draw_pixel(tx_buf, 260, 60, 9);
    draw_pixel(tx_buf, 263, 63, 5);
    draw_pixel(tx_buf, 264, 64, 1);


    // send a frame buffer to the display
    send_buffer_to_OLED(tx_buf, 0, 0);
    HAL_Delay(10000);

    // draw vertical and horizontal lines
    draw_hline(tx_buf, 31, 20, 50, 10);
    draw_vline(tx_buf, 31, 0, 31, 10);

    // send a frame buffer to the display
    send_buffer_to_OLED(tx_buf, 0, 0);
    HAL_Delay(2000);

    // draw simple oblique line
    draw_line(tx_buf, 40, 0, 80, 31, 12);
    // send a frame buffer to the display
    send_buffer_to_OLED(tx_buf, 0, 0);
    HAL_Delay(2000);

    // draw antialiased oblique line. It should appear softer and nicer than a simple one
    draw_AA_line(tx_buf, 50, 0, 90, 31, 12);
    // send a frame buffer to the display
    send_buffer_to_OLED(tx_buf, 0, 0);
    HAL_Delay(2000);

    //draw circle, empty rectangle and filled rectangle
    draw_circle(tx_buf, 180, 20, 20, 15);
    draw_rect(tx_buf, 100, 5, 120, 25, 15);
    draw_rect_filled(tx_buf, 124, 5, 144, 25, 8);
    // send a frame buffer to the display
    send_buffer_to_OLED(tx_buf, 0, 0);
    HAL_Delay(2000);

    //clean buffer
    fill_buffer(tx_buf, 0);

    //display 8-bit grayscale bitmap (ony first 4 bits are actually written to memory)
    draw_bitmap_8bpp(tx_buf, pat_i_mat, 0, 0, 64, 64);
    draw_bitmap_8bpp(tx_buf, krecik, 128, 0, 64, 64);
    // send a frame buffer to the display
    send_buffer_to_OLED(tx_buf, 0, 0);
    HAL_Delay(5000);

    //display 4-bit grayscale bitmap (one byte in bitmap array corresponds to two pixels)
    draw_bitmap_4bpp(tx_buf, stars_4bpp, 0, 0, 256, 64);
    send_buffer_to_OLED(tx_buf, 0, 0);
    HAL_Delay(3000);

    //you can invert screen colors using API function
    SSD1322_API_set_display_mode(SSD1322_MODE_INVERTED);
    HAL_Delay(2000);
    //pixels can be also turned on or off
    SSD1322_API_set_display_mode(SSD1322_MODE_ON);
    HAL_Delay(1000);
    SSD1322_API_set_display_mode(SSD1322_MODE_OFF);
    HAL_Delay(1000);
    //ok, go back to normal
    SSD1322_API_set_display_mode(SSD1322_MODE_NORMAL);
    HAL_Delay(500);

    //exact grayscale values can be set individually for each level from 0 to 15 - always send 16 byte array of values 0-180
    uint8_t grayscale_tab[16] = {0, 5, 10, 15, 20, 25, 30, 35, 145, 150, 155, 160, 165, 170, 175, 180};
    SSD1322_API_custom_grayscale(grayscale_tab);
    HAL_Delay(2000);
    //New grayscale values should be close to black in darker areas and close to white in brighter

    //reset grayscale to default linear values
    SSD1322_API_default_grayscale();
    HAL_Delay(2000);

    //display can be set to sleep mode and then woken up
    SSD1322_API_sleep_on();
    HAL_Delay(1000);
    SSD1322_API_sleep_off();

    //clean buffer
    fill_buffer(tx_buf, 0);

    // now let's try to write some text with a font
    // first thing to do is font selection
    select_font(&FreeMono12pt7b);
    // now text will we written with that font
    draw_text(tx_buf, "aaaaaaaaaaaa", 10, 20, 15);
    // send a frame buffer to the display
    send_buffer_to_OLED(tx_buf, 0, 0);
    HAL_Delay(2000);

    //change font to a differen one
    select_font(&FreeSansOblique9pt7b);
    draw_text(tx_buf, "dolor sit amet", 10, 45, 15);
    // send a frame buffer to the display
    send_buffer_to_OLED(tx_buf, 0, 0);
    HAL_Delay(2000);

    //you can use frame buffer that is bigger than default 256x64 pixels.
    //Remember to divide size by two, because one byte stores two pixels.

    uint8_t tx_buf2[256*256 / 2];
    set_buffer_size(256, 256);

    //now print a huge bitmap into frame buffer
    draw_bitmap_8bpp(tx_buf2, creeper, 0, 0, 256, 256);
    send_buffer_to_OLED(tx_buf2, 0, 0);
    HAL_Delay(2000);

    //only 1/4 of image is seen, so let's scroll the frame buffer down
    for(int i = 0; i < 192; i++)
    {
        send_buffer_to_OLED(tx_buf2, 0, i);
        HAL_Delay(5);
    }
    HAL_Delay(200);
    for (int i = 191; i >= 0; i--)
    {
        send_buffer_to_OLED(tx_buf2, 0, i);
        HAL_Delay(5);
    }
    HAL_Delay(2000);
    //SSD1322_API_set_display_mode(SSD1322_MODE_OFF);
    /*==================================== DEMO CODE END ============================================*/
}

/*
256 X 64 size.

5~14 pt => 
*/

void SSD1322_buf_data(uint8_t *tx_buf, float data){//, uint16_t x, uint16_t y
    char buf[10];
    memset(buf, ' ', sizeof(buf));
    sprintf(buf, "%.1f", data);
    //strcpy(buf+3, "ww w");
    
    set_buffer_size(256, 64);
    fill_buffer(tx_buf, 0);
    draw_string_tiny(tx_buf, buf, 10, 45);
    
    // select_font(&FreeMono12pt7b);
    // draw_text(tx_buf, buf, 10, 45, 15);
    send_buffer_to_OLED(tx_buf,0, 0);
}

void SSD1322_layout_test(uint8_t *tx_buf, uint32_t ENC_1, uint32_t ENC_2, bool bt1, bool bt2, bool sl_l, bool sl_r){
    select_font(&FreeMono12pt7b);
    fill_buffer(tx_buf, 0);
    char buf1[32];
    char buf2[32];
    int bufload = 0;

    strcpy(buf1+bufload, "E_R:");
    bufload += 4;
    sprintf(buf1+bufload, "%i", ENC_1);
    if(ENC_1>99){
        bufload +=3;
    }
    else if(ENC_1>9){
        bufload +=2;
    }
    else
        bufload +=1;
    strcpy(buf1+bufload, "|E_L:");
    bufload+=5;
    sprintf(buf1+bufload, "%i", ENC_2);

    if(bt1)
      draw_line(tx_buf, 0, 30, 60, 30, 15);
    if(bt2)
      draw_line(tx_buf, 0, 30, 60, 30, 15);
      
    strcpy(buf2, "S_R:");
    if(sl_r)
      strcpy(buf2+4, " ON");
    else
      strcpy(buf2+4, "OFF");

    strcpy(buf2+7, "S_L:");
    if(sl_l)
      strcpy(buf2+11, " ON");
    else
      strcpy(buf2+11, "OFF");

    draw_text(tx_buf, buf1, 10, 20, 15);
    draw_text(tx_buf, buf2, 10, 45, 15);
    send_buffer_to_OLED(tx_buf, 0 , 0);

}

void SSD1322_SSE(uint8_t *tx_buf){
    set_buffer_size(256, 64); 
    fill_buffer(tx_buf, 0);
    draw_bitmap_4bpp(tx_buf, SSE, 0, 0, 256, 64);
    send_buffer_to_OLED(tx_buf, 0, 0);
}

