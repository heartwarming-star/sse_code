/**
 ****************************************************************************************
 *
 * \file SSD1322_GFX.c
 *
 * \brief Simple GFX library to draw some basic shapes on OLED display.
 *
 *
 * Copyright (C) 2020 Wojciech Klimek
 * MIT license:
 * https://github.com/wjklimek1/SSD1322_OLED_library
 *
 ****************************************************************************************
 */

//====================== Includes ====================//
#include "../SSD1322_OLED_lib/Fonts/ProggyTinyFont.h"   // 방금 보여주신 폰트 데이터 헤더
#include "../SSD1322_OLED_lib/Fonts/Font.h"
#include "../SSD1322_OLED_lib/SSD1322_API.h"
#include "../SSD1322_OLED_lib/SSD1322_GFX.h"

#include <stdlib.h>
#include <math.h>


const GFXfont *gfx_font = NULL;     //pointer to Adafruit font that is currently selected

uint16_t _buffer_height = 64;       //buffer dimensions used to determine if pixel is within array bounds
uint16_t _buffer_width = 256;      //by default buffer size is equal to OLED size
uint16_t _max_buffer_size = 6;
uint8_t _brightness = 0xf;
//====================== set buffer size ========================//
/**
 *  @brief Overwrites expected frame buffer dimensions
 *
 *	Buffer size is used in draw_pixel() function to determine if pixel is within array bounds.
 *
 *  By default frame buffer size is 256x64 - equal to size of OLED screen. You may want to change it,
 *  for example to use scrolling effet.
 *
 *  @param[in] buffer_width
 *             new x size of a buffer in pixels
 *  @param[in] buffer_height
 *  		   new y size of a buffer in pixels
 */


void set_buffer_size(uint16_t buffer_width, uint16_t buffer_height)
{
	_buffer_width = buffer_width;
	_buffer_height = buffer_height;
}

//====================== fill buffer ========================//
/**
 *  @brief Fill buffer with specified brightness
 *
 *  @param[in] frame_buffer
 *             array of pixel values
 *  @param[in] brightness
 *             brightness value of pixel (range 0-15 dec or 0x00-0x0F hex)
 */
void fill_buffer(uint8_t *frame_buffer, uint8_t brightness)
{
	uint8_t byte_value = (brightness << 4) | brightness;
	uint32_t buffer_size = _buffer_height * _buffer_width / 2;
	while (buffer_size--)
	{
		*frame_buffer++ = byte_value;
	}
}

//====================== draw pixel ========================//
/**
 *  @brief Draws one pixel on frame buffer
 *
 *  Draws pixel of specified brightness on given coordinates on frame buffer.
 *  Pixels drawn outside buffer outline are ignored to avoid overwriting
 *  memory outside frame buffer - "segfault".
 *
 *  @param[in] frame_buffer
 *             array of pixel values
 *  @param[in] x
 *             horizontal coordinate of pixel
 *  @param[in] y
 *             vertical coordinate of pixel
 *  @param[in] brightness
 *             brightness value of pixel (range 0-15 dec or 0x00-0x0F hex)
 */
void draw_pixel(uint8_t *frame_buffer, uint16_t x, uint16_t y, uint8_t brightness)
{
	if(x > (_buffer_width-1) || y > (_buffer_height-1))
		return;

	if ((y * _buffer_width + x) % 2 == 1)
	{
		frame_buffer[((y * _buffer_width) + x) / 2] = (frame_buffer[((y * _buffer_width) + x) / 2] & 0xF0) | brightness;
	}
	else
	{
		frame_buffer[((y * _buffer_width) + x) / 2] = (frame_buffer[((y * _buffer_width) + x) / 2] & 0x0F) | (brightness << 4);
	}
}

//====================== draw vertical line ========================//
/**
 *  @brief Draws vertical line in frame buffer
 *
 *  Draws vertical line of specified brightness on given coordinates on frame buffer.
 *
 *  @param[in] frame_buffer
 *             array of pixel values
 *  @param[in] x
 *             horizontal coordinate of line
 *  @param[in] y0
 *             beginning of the line
 *  @param[in] y1
 *             end of the line
 * 	@param[in] brightness
 *             brightness value of pixels (range 0-15 dec or 0x00-0x0F hex)
 */
void draw_vline(uint8_t *frame_buffer, uint16_t x, uint16_t y0, uint16_t y1, uint8_t brightness)
{
	if(y0 < y1)
	{
		for (uint16_t i = y0; i <= y1; i++)
		{
			draw_pixel(frame_buffer, x, i, brightness);
		}
	}
	else
	{
		for (uint16_t i = y1; i <= y0; i++)
		{
			draw_pixel(frame_buffer, x, i, brightness);
		}
	}
}

//====================== draw horizontal line ========================//
/**
 *  @brief Draws horizontal line in frame buffer
 *
 *  Draws horizontal line of specified brightness on given coordinates on frame buffer.
 *
 *  @param[in] frame_buffer
 *             array of pixel values
 *  @param[in] y
 *             vertical coordinate of line
 *  @param[in] x0
 *             beginning of the line
 *  @param[in] x1
 *             end of the line
 * 	@param[in] brightness
 *             brightness value of pixels (range 0-15 dec or 0x00-0x0F hex)
 */
void draw_hline(uint8_t *frame_buffer, uint16_t y, uint16_t x0, uint16_t x1, uint8_t brightness)
{
	if(x0 < x1)
	{
		for (uint16_t i = x0; i <= x1; i++)
		{
			draw_pixel(frame_buffer, i, y, brightness);
		}
	}
	else
	{
		for (uint16_t i = x1; i <= x0; i++)
		{
			draw_pixel(frame_buffer, i, y, brightness);
		}
	}
}

//====================== draw sloping line ========================//
/**
 *  @brief Draws sloping line
 *
 *  Can be also used to draw vertical and horizontal lines.
 *
 *  @param[in] frame_buffer
 *             array of pixel values
 *  @param[in] x0
 *             x position of line beginning
 *  @param[in] y0
 *             y position of line beginning
 *  @param[in] x1
 *             x position of line ending
 *  @param[in] y1
 *             y position of line ending
 * 	@param[in] brightness
 *             brightness value of pixels (range 0-15 dec or 0x00-0x0F hex)
*/
void draw_line(uint8_t *frame_buffer, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t brightness)
{
	//handle horizontal and vertical lines with appropriate functions
	if (x0 == x1)
	{
		draw_vline(frame_buffer, x0, y0, y1, brightness);
	}
	if (y0 == y1)
	{
		draw_hline(frame_buffer, y0, x0, x1, brightness);
	}

	int16_t steep = abs(y1 - y0) > abs(x1 - x0);
	if (steep == 1)
	{
		uint16_t tmp = y0;
		y0 = x0;
		x0 = tmp;
		tmp = y1;
		y1 = x1;
		x1 = tmp;
	}

	if (x0 > x1)
	{
		uint16_t tmp = x0;
		x0 = x1;
		x1 = tmp;
		tmp = y0;
		y0 = y1;
		y1 = tmp;
	}

	int16_t dx, dy;
	dx = x1 - x0;
	dy = abs(y1 - y0);

	int16_t err = dx / 2;
	int16_t ystep;

	if (y0 < y1)
	{
		ystep = 1;
	}
	else
	{
		ystep = -1;
	}

	for (; x0 <= x1; x0++)
	{
		if (steep)
		{
			draw_pixel(frame_buffer, y0, x0, brightness);
		}
		else
		{
			draw_pixel(frame_buffer, x0, y0, brightness);
		}
		err -= dy;
		if (err < 0)
		{
			y0 += ystep;
			err += dx;
		}
	}
}

//====================== draw antialiased sloping line using Xiaolin Wu's aghoritm ========================//
/**
 *  @brief Draws antialiased sloping line using Xiaolin Wu's aghoritm.
 *
 *  Doesn't work for lines with brightness = 1, probably because of rounding errors.
 *  Can be also used to draw vertical and horizontal lines.
 *
 *  @param[in] frame_buffer
 *             array of pixel values
 *  @param[in] x0
 *             x position of line beginning
 *  @param[in] y0
 *             y position of line beginning
 *  @param[in] x1
 *             x position of line ending
 *  @param[in] y1
 *             y position of line ending
 * 	@param[in] brightness
 *             brightness value of pixels (range 0-15 dec or 0x00-0x0F hex)
*/
void draw_AA_line(uint8_t *frame_buffer, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t brightness)
{
	//handle horizontal and vertical lines with appropriate functions
	if (x0 == x1)
	{
		draw_vline(frame_buffer, x0, y0, y1, brightness);
	}
	if (y0 == y1)
	{
		draw_hline(frame_buffer, y0, x0, x1, brightness);
	}

	uint8_t steep = abs(y1 - y0) > abs(x1 - x0);

	if (steep)
	{
		uint16_t tmp = y0;
		y0 = x0;
		x0 = tmp;
		tmp = y1;
		y1 = x1;
		x1 = tmp;
	}
	if (x0 > x1)
	{
		uint16_t tmp = x0;
		x0 = x1;
		x1 = tmp;
		tmp = y0;
		y0 = y1;
		y1 = tmp;
	}

	float dx = x1 - x0;
	float dy = y1 - y0;
	float gradient = dy / dx;

	// handle first endpoint
	float xend = round(x0);
	float yend = y0 + gradient * (xend - x0);
	float xgap =  1 - ((x0 + 0.5) - floor(x0 + 0.5));
	float xpxl1 = xend; // this will be used in the main loop
	float ypxl1 = floor(yend);
	if (steep)
	{
		draw_pixel(frame_buffer, ypxl1, xpxl1, (1-(yend - (floor(yend))) * xgap)*brightness);
		draw_pixel(frame_buffer, ypxl1 + 1, xpxl1, (yend - (floor(yend)) * xgap)*brightness);
	}
	else
	{
		draw_pixel(frame_buffer, xpxl1, ypxl1, (1-(yend - (floor(yend))) * xgap)*brightness);
		draw_pixel(frame_buffer, xpxl1, ypxl1 + 1, (yend - (floor(yend)) * xgap)*brightness);
	}

	float intery = yend + gradient; // first y-intersection for the main loop

	// handle second endpoint
	xend = round(x1);
	yend = y1 + gradient * (xend - x1);
	xgap = (x1 + 0.5) - floor(x1 + 0.5);
	float xpxl2 = xend; //this will be used in the main loop
	float ypxl2 = floor(yend);
	if (steep)
	{
		draw_pixel(frame_buffer, ypxl2, xpxl2, (1 - (yend - floor(yend)) * xgap)*brightness);
		draw_pixel(frame_buffer, ypxl2 + 1, xpxl2, ((yend - floor(yend)) * xgap)*brightness);
	}
	else
	{
		draw_pixel(frame_buffer, xpxl2, ypxl2, (1 - (yend - floor(yend)) * xgap)*brightness);
		draw_pixel(frame_buffer, xpxl2, ypxl2 + 1, ((yend - floor(yend)) * xgap)*brightness);
	}

	// main loop
	if (steep)
	{
		for (int x = xpxl1 + 1; x <= xpxl2 - 1; x++)
		{
			draw_pixel(frame_buffer, floor(intery), x, (1 - (intery - floor(intery)))*brightness);
			draw_pixel(frame_buffer, floor(intery) + 1, x, (intery - floor(intery))*brightness);
			intery = intery + gradient;
		}
	}
	else
	{
		for (int x = xpxl1 + 1; x <= xpxl2 - 1; x++)
		{
			draw_pixel(frame_buffer, x, floor(intery), (1 - (intery - floor(intery)))*brightness);
			draw_pixel(frame_buffer, x, floor(intery) + 1, (intery - floor(intery))*brightness);
			intery = intery + gradient;
		}
	}
}



//====================== draw empty rectangle ========================//
/**
 *  @brief Draws empty rectangle on frame buffer
 *
 *  @param[in] frame_buffer
 *             array of pixel values
 *  @param[in] x0
 *             x position of first corner
 *  @param[in] y0
 *             y position of first corner
 *  @param[in] x1
 *             x position of second corner
 *  @param[in] y1
 *             y position of second corner
 * 	@param[in] brightness
 *             brightness value of pixels (range 0-15 dec or 0x00-0x0F hex)
 */
void draw_rect(uint8_t *frame_buffer, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t brightness)
{
	draw_vline(frame_buffer, x0, y0, y1, brightness);
	draw_vline(frame_buffer, x1, y0, y1, brightness);
	draw_hline(frame_buffer, y0, x0, x1, brightness);
	draw_hline(frame_buffer, y1, x0, x1, brightness);
}

//====================== draw filled rectangle ========================//
/**
 *  @brief Draws filled rectangle on frame buffer
 *
 *  @param[in] frame_buffer
 *             array of pixel values
 *  @param[in] x0
 *             x position of first corner
 *  @param[in] y0
 *             y position of first corner
 *  @param[in] x1
 *             x position of second corner
 *  @param[in] y1
 *             y position of second corner
 * 	@param[in] brightness
 *             brightness value of pixels (range 0-15 dec or 0x00-0x0F hex)
 */
void draw_rect_filled(uint8_t *frame_buffer, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t brightness)
{
	for (uint16_t i = x0; i <= x1; i++)
	{
		for (uint16_t j = y0; j <= y1; j++)
		{
			draw_pixel(frame_buffer, i, j, brightness);
		}
	}
}

//====================== draw empty circle ========================//
/**
 *  @brief Draws empty circle on frame buffer
 *
 *  @param[in] frame_buffer
 *             array of pixel values
 *  @param[in] x0
 *             x position of circle's center
 *  @param[in] y0
 *             y position of circle's center
 *  @param[in] r
 *             radius of the circle (pixels)
 * 	@param[in] brightness
 *             brightness value of pixels (range 0-15 dec or 0x00-0x0F hex)
 */
void draw_circle(uint8_t *frame_buffer, uint16_t x0, uint16_t y0, uint16_t r, uint8_t brightness)
{
  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;

  draw_pixel(frame_buffer, x0, y0 + r, brightness);
  draw_pixel(frame_buffer, x0, y0 - r, brightness);
  draw_pixel(frame_buffer, x0 + r, y0, brightness);
  draw_pixel(frame_buffer, x0 - r, y0, brightness);

  while (x < y)
  {
    if (f >= 0)
    {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;

    draw_pixel(frame_buffer, x0 + x, y0 + y, brightness);
    draw_pixel(frame_buffer, x0 - x, y0 + y, brightness);
    draw_pixel(frame_buffer, x0 + x, y0 - y, brightness);
    draw_pixel(frame_buffer, x0 - x, y0 - y, brightness);
    draw_pixel(frame_buffer, x0 + y, y0 + x, brightness);
    draw_pixel(frame_buffer, x0 - y, y0 + x, brightness);
    draw_pixel(frame_buffer, x0 + y, y0 - x, brightness);
    draw_pixel(frame_buffer, x0 - y, y0 - x, brightness);
  }
}

//====================== draw bitmap ========================//
/**
 *  @brief Draws 8 bits per pixel bitmap to frame buffer.
 *
 *  Only 4 first bits of grayscale byte are written to frame buffer. For example, if pixel value in bitmap
 *  is 0x14 (0b00010100), this byte is cut in half (0b0001 0100) and only first half (0b0001) is used.
 *
 *  You can use any image to bitmap converter as long as it gives you an 8-bit grayscale color depth array.
 *
 *  @param[in] frame_buffer
 *             array of pixel values
 *  @param[in] bitmap
 *  		   array with pixels to write to frame buffer
 *  @param[in] x0
 *             x position of top left bitmap corner
 *  @param[in] y0
 *             y position of top left bitmap corner
 *  @param[in] x_size
 *             width of bitmap in pixels
 *  @param[in] y_size
 *             height of bitmap in pixels
 */
void draw_bitmap_8bpp(uint8_t *frame_buffer, const uint8_t *bitmap, uint16_t x0, uint16_t y0, uint16_t x_size, uint16_t y_size)
{
	uint16_t bitmap_pos = 0;

	for (uint16_t i = y0; i < y0 + y_size; i++)
	{
		for (uint16_t j = x0; j < x0 + x_size; j++)
		{
			draw_pixel(frame_buffer, j, i, bitmap[bitmap_pos] >> 4);
			bitmap_pos++;
		}
	}
}

//====================== draw 4-bit bitmap ========================//
/**
 *  @brief Draws bitmap with 4 bit per pixel grayscale depth to frame buffer.
 *
 * 	Writes bitmap where 2 pixels are coded in a single byte.
 *
 * 	WARNING: This function is still untested!
 *
 *  @param[in] frame_buffer
 *             array of pixel values
 *  @param[in] bitmap
 *  		   array with pixels to write to frame buffer
 *  @param[in] x0
 *             x position of top left bitmap corner
 *  @param[in] y0
 *             y position of top left bitmap corner
 *  @param[in] x_size
 *             width of bitmap in pixels
 *  @param[in] y_size
 *             height of bitmap in pixels
 */
void draw_bitmap_4bpp(uint8_t *frame_buffer, const uint8_t *bitmap, uint16_t x0, uint16_t y0, uint16_t x_size, uint16_t y_size)
{
	uint16_t bitmap_pos = 0;       //byte index in bitmap array
	uint16_t processed_pixels = 0;
	uint8_t pixel_parity = 0;      //if pixel is even = 0; odd = 1

	for (uint16_t i = y0; i < y0 + y_size; i++)
	{
		for (uint16_t j = x0; j < x0 + x_size; j++)
		{
			pixel_parity = processed_pixels % 2;

			if(pixel_parity == 0)
			{
				draw_pixel(frame_buffer, j, i, bitmap[bitmap_pos] >> 4);
				processed_pixels++;
			}
			else
			{
				draw_pixel(frame_buffer, j, i, bitmap[bitmap_pos]);
				processed_pixels++;
				bitmap_pos++;
			}
		}
	}
}

//====================== select font ========================//
/**
 *  @brief Select font to write text
 *
 *  This function has to be called at least once to define font.
 *  Next call is needed only when you want to change font to other one.
 *
 *  @param[in] new_gfx_font
 *             pointer to font structure
 */
void select_font(const GFXfont *new_gfx_font)
{
	gfx_font = new_gfx_font;
}

//====================== draw single character ========================//
/**
 *  @brief Draw single character
 *
 *	To draw character font has to be selected.
 *
 *  @param[in] frame_buffer
 *             array of pixel values
 *  @param[in] c
 *             ASCII character to draw in buffer
 *  @param[in] x
 *             x position of bottom left corner of character
 *  @param[in] y
 *             y position of bottom left corner of character
 * 	@param[in] brightness
 *             brightness value of pixels (range 0-15 dec or 0x00-0x0F hex)
 */
void draw_char(uint8_t *frame_buffer, uint8_t c, uint16_t x, uint16_t y, uint8_t brightness)
{
	if(gfx_font == NULL)
		return;

	c -= (uint8_t)gfx_font->first;          //convert input char to corresponding byte from font array
    GFXglyph *glyph = gfx_font->glyph + c;  //get pointer of glyph corresponding to char
    uint8_t *bitmap = gfx_font->bitmap;     //get pointer of char bitmap

    uint16_t bo = glyph->bitmapOffset;
    uint8_t width = glyph->width;
    uint8_t height = glyph->height;

    int8_t x_offset = glyph->xOffset;
    int8_t y_offset = glyph->yOffset;

    //decide for background brightness or font brightness
    uint8_t bit = 0;
    uint8_t bits = 0;
    uint8_t y_pos = 0;
    uint8_t x_pos = 0;

	for (y_pos = 0; y_pos < height; y_pos++)
	{
		for (x_pos = 0; x_pos < width; x_pos++)
		{
			if (!(bit++ & 7))
			{
				bits = (*(const unsigned char *)(&bitmap[bo++]));
			}
			if (bits & 0x80)
			{
				draw_pixel(frame_buffer, x + x_offset + x_pos, y + y_offset+y_pos, brightness);
			}
			else
			{

			}
			bits <<= 1;
		}
	}
}

//====================== draw string ========================//
/**
 *  @brief Draw single character
 *
 *	To draw string font has to be selected.
 *
 *	WARNING: This works only for NULL-terminated strings!
 *
 *  @param[in] frame_buffer
 *             array of pixel values
 *  @param[in] text
 *             string with ASCII values
 *  @param[in] x
 *             x position of bottom left corner of first character
 *  @param[in] y
 *             y position of bottom left corner of first character
 * 	@param[in] brightness
 *             brightness value of pixels (range 0-15 dec or 0x00-0x0F hex)
 */
void draw_text(uint8_t *frame_buffer, const char* text, uint16_t x, uint16_t y, uint8_t brightness)
{
    while (*text)
    {
        draw_char(frame_buffer, *text, x, y, brightness);
        x = x + gfx_font->glyph[*text-32].xAdvance;
        text++;
    }
}

//====================== send frame buffer to OLED ========================//
/**
 *  @brief Sends frame buffer to OLED display.
 *
 *  Transmits frame buffer to OLED display.
 *
 *  If your frame buffer size is equal to OLED size, leave start_x and start_y at 0.
 *  If you use bigger frame buffer, you can use this parameters to choose region of your frame buffer
 *  that will be displayed on OLED. This is particularly useful for scrolling.
 *
 *  @param[in] frame_buffer
 *             array of pixel values
 *  @param[in] start_x
 *             x position of frame buffer part that will be displayed on OLED. Useful for horizontal scrolling.
 *  @param[in] start_y
 *             y position of frame buffer part that will be displayed on OLED Useful for vertical scrolling.

 */
void send_buffer_to_OLED(uint8_t *frame_buffer, uint16_t start_x, uint16_t start_y)
{
	SSD1322_API_set_window(0, 63, 0, 127);
	SSD1322_API_send_buffer(frame_buffer + (start_y * OLED_WIDTH / 2) + start_x, 8192);
}



void draw_char_proggy_tiny(uint8_t *frame_buffer, char c, int16_t x, int16_t y)
{
    // 폰트에 없는 범위(ASCII 0x20 미만, 0x7E 초과)는 그냥 무시
    if (c < 0x20 || c > 0x7E)
        return;

    // ' ' (0x20)은 배열의 인덱스 0에 해당
    uint8_t char_index = c - 0x20;

    // ProggyTinyFontBitmaps[char_index][행][열]
    for (int row = 0; row < PROGGY_FONT_HEIGHT; row++)
    {
        for (int col = 0; col < PROGGY_FONT_WIDTH; col++)
        {
            // 픽셀 밝기를 꺼내서
            uint8_t pixel_val = ProggyTinyFontBitmaps[char_index][row][col];

            // 혹시 값이 4비트 범위를 넘을 수도 있으니 0~15로 제한
            if (pixel_val > 15) {
                pixel_val = 15;
            }

            // (x+col, y+row)에 픽셀 찍기
            draw_pixel(frame_buffer, x + col, y + row, pixel_val);
        }
    }
}

void draw_string_proggy_tiny(uint8_t *frame_buffer, const char *text, int16_t x, int16_t y)
{
    while (*text)
    {
        // 현재 문자
        char c = *text++;

        // 그 문자 한 글자 찍기
        draw_char_proggy_tiny(frame_buffer, c, x, y);

        // 다음 글자는 x좌표를 FONT_WIDTH만큼 이동해서 표시
        x += PROGGY_FONT_WIDTH;

        // 혹시 한 칸 간격을 더 주고 싶다면 x += (FONT_WIDTH + 1); 처럼 해도 됨
    }
}

uint16_t draw_char_tiny(uint8_t *frame_buffer, char c, int16_t x, int16_t y)
{
    // 폰트에 없는 범위(ASCII 0x20 미만, 0x7E 초과)는 그냥 무시
    if (c < 0x20 || c > 0x7E)
        return;

    // ' ' (0x20)은 배열의 인덱스 0에 해당
    uint8_t char_index = c - 0x20;
	uint16_t width = 0;
    // ProggyTinyFontBitmaps[char_index][행][열]
    for (int row = 0; row < FONT_HEIGHT; row++)
    {
        for (int col = 0; col < FONT_WIDTH; col++)
        {
            // 픽셀 밝기를 꺼내서
            uint8_t pixel_val = FontBitmaps[char_index][row][col];

            // 혹시 값이 4비트 범위를 넘을 수도 있으니 0~15로 제한
			if (row==0 && pixel_val == 0x8){
				width = col;
				break;
			}
			else if(pixel_val == 0x8){
				break;
			}

            if (pixel_val > 15) {
                pixel_val = _brightness;
            }

            // (x+col, y+row)에 픽셀 찍기
            draw_pixel(frame_buffer, x + col, y + row, pixel_val);
        }
    }
	return width;
}

void draw_string_tiny(uint8_t *frame_buffer, const char *text, int16_t x, int16_t y)
{
    while (*text)
    {
        // 현재 문자
        char c = *text++;

        // 그 문자 한 글자 찍기
        

        // 다음 글자는 x좌표를 FONT_WIDTH만큼 이동해서 표시
		uint16_t tmp = draw_char_tiny(frame_buffer, c, x, y);
		if(tmp == 0){
			x += FONT_WIDTH;
		}
		else{
			x += (tmp+1);
		}

        // 혹시 한 칸 간격을 더 주고 싶다면 x += (FONT_WIDTH + 1); 처럼 해도 됨
    }
}

void draw_int_tiny(uint8_t *frame_buffer, int data, int16_t x, int16_t y){
	char buf[_max_buffer_size];
	if(data < 99999){
		sprintf(buf, "%i", data);
		draw_string_tiny(frame_buffer, buf, x, y);
	}
	return;
} 

void draw_float_tiny(uint8_t *frame_buffer, float data, int16_t x, int16_t y, int pos, int max_data){
	//4자리 고정.
	char buf[_max_buffer_size];
	if(data < max_data){
		switch (pos){
			case 0:
				sprintf(buf, "%.0f", data);
				break;
			
			case 1:
				sprintf(buf, "%.1f", data);
				break;

			default:
				break;
		}
		draw_string_tiny(frame_buffer, buf, x, y);
	}
	return;
}
uint16_t draw_char_tiny_2x(uint8_t *frame_buffer, char c, int16_t x, int16_t y)
{
    // 폰트에 없는 범위(ASCII 0x20 미만, 0x7E 초과)는 그냥 무시
    if (c < 0x20 || c > 0x7E)
        return 0;

    // ' ' (0x20)은 배열의 인덱스 0에 해당
    uint8_t char_index = c - 0x20;
	uint16_t width = 0;
    // ProggyTinyFontBitmaps[char_index][행][열]
    for (int row = 0; row < FONT_HEIGHT; row++)
    {
        for (int col = 0; col < FONT_WIDTH; col++)
        {
            // 픽셀 밝기를 꺼내서
            uint8_t pixel_val = FontBitmaps[char_index][row][col];

            // 혹시 값이 4비트 범위를 넘을 수도 있으니 0~15로 제한
			if (row==0 && pixel_val == 0x8){
				width = col;
				break;
			}
			else if(pixel_val == 0x8){
				break;
			}

            if (pixel_val > 15) {
                pixel_val = _brightness;
            }

            // (x+col, y+row)에 픽셀 찍기
			for(int x_tmp = 0; x_tmp <  2; x_tmp++){
				for(int y_tmp = 0; y_tmp < 2; y_tmp++){
					draw_pixel(frame_buffer, x + col*2 + x_tmp, y + row*2 + y_tmp, pixel_val);
				}
			}
        }
    }
	return width;
}

void draw_string_tiny_2x(uint8_t *frame_buffer, const char *text, int16_t x, int16_t y)
{
    while (*text)
    {
        // 현재 문자
        char c = *text++;

        // 그 문자 한 글자 찍기
        

        // 다음 글자는 x좌표를 FONT_WIDTH만큼 이동해서 표시
		uint16_t tmp = draw_char_tiny_2x(frame_buffer, c, x, y);
		if(tmp == 0){
			x += FONT_WIDTH*2;
		}
		else{
			x += (tmp*2+1);
		}

        // 혹시 한 칸 간격을 더 주고 싶다면 x += (FONT_WIDTH + 1); 처럼 해도 됨
    }
}

uint16_t draw_char_tiny_3x(uint8_t *frame_buffer, char c, int16_t x, int16_t y)
{
    // 폰트에 없는 범위(ASCII 0x20 미만, 0x7E 초과)는 그냥 무시
    if (c < 0x20 || c > 0x7E)
        return 0;

    // ' ' (0x20)은 배열의 인덱스 0에 해당
    uint8_t char_index = c - 0x20;
	uint16_t width = 0;
    // ProggyTinyFontBitmaps[char_index][행][열]
    for (int row = 0; row < FONT_HEIGHT; row++)
    {
        for (int col = 0; col < FONT_WIDTH; col++)
        {
            // 픽셀 밝기를 꺼내서
            uint8_t pixel_val = FontBitmaps[char_index][row][col];

            // 혹시 값이 4비트 범위를 넘을 수도 있으니 0~15로 제한
			if (row==0 && pixel_val == 0x8){
				width = col;
				break;
			}
			else if(pixel_val == 0x8){
				break;
			}

            if (pixel_val > 15) {
                pixel_val = _brightness;
            }

            // (x+col, y+row)에 픽셀 찍기
			for(int x_tmp = 0; x_tmp <  3; x_tmp++){
				for(int y_tmp = 0; y_tmp < 3; y_tmp++){
					draw_pixel(frame_buffer, x + col*3 + x_tmp, y + row*3 + y_tmp, pixel_val);
				}
			}
        }
    }
	return width;
}

void draw_string_tiny_3x(uint8_t *frame_buffer, const char *text, int16_t x, int16_t y)
{
    while (*text)
    {
        // 현재 문자
        char c = *text++;

        // 그 문자 한 글자 찍기
        

        // 다음 글자는 x좌표를 FONT_WIDTH만큼 이동해서 표시
		uint16_t tmp = draw_char_tiny_3x(frame_buffer, c, x, y);
		if(tmp == 0){
			x += FONT_WIDTH*3;
		}
		else{
			x += (tmp*3+1);
		}

        // 혹시 한 칸 간격을 더 주고 싶다면 x += (FONT_WIDTH + 1); 처럼 해도 됨
    }
}

void update_brightness(uint8_t data){
	if(data == 0x8){
		_brightness = 0x7;
	}
	if(data < 0xf){
		_brightness = data;
	}
	else{
		_brightness = 0xf;
	}
}