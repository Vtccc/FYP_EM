#include "DEV_Config.h"
#include "EPD.h"
#include "GUI_Paint.h"
#include "imagedata.h"
#include <stdlib.h>

void setup() {
    printf("EPD_3IN0G_test Demo\r\n");
    DEV_Module_Init();

    printf("e-Paper Init and Clear...\r\n");
    EPD_3IN0G_Init();

    EPD_3IN0G_Clear(EPD_3IN0G_WHITE); // White
    DEV_Delay_ms(2000);

    //Create a new image cache
    UBYTE *BlackImage;
    UWORD Imagesize = ((EPD_3IN0G_WIDTH % 4 == 0)? (EPD_3IN0G_WIDTH / 4 ): (EPD_3IN0G_WIDTH / 4 + 1)) * EPD_3IN0G_HEIGHT;
    if((BlackImage = (UBYTE *)malloc(Imagesize)) == NULL) {
        printf("Failed to apply for black memory...\r\n");
        while (1);
    }
    printf("Paint_NewImage\r\n");
    Paint_NewImage(BlackImage, EPD_3IN0G_WIDTH, EPD_3IN0G_HEIGHT, 0, EPD_3IN0G_WHITE);
    Paint_SetScale(4);

#if 1   // show bmp
    printf("show BMP-----------------\r\n");
    EPD_3IN0G_Init();
    Paint_SelectImage(BlackImage);
    Paint_DrawBitMap(gImage_3in0g);
    EPD_3IN0G_Display(BlackImage);
    DEV_Delay_ms(2000);
#endif

#if 1   // Drawing on the image
    //1.Select Image
    EPD_3IN0G_Init();
    printf("SelectImage:BlackImage\r\n");
    Paint_SelectImage(BlackImage);
    Paint_Clear(EPD_3IN0G_WHITE);

    // 2.Drawing on the image
    printf("Drawing:BlackImage\r\n");
    Paint_DrawPoint(10, 80, EPD_3IN0G_RED, DOT_PIXEL_1X1, DOT_STYLE_DFT);
    Paint_DrawPoint(10, 90, EPD_3IN0G_YELLOW, DOT_PIXEL_2X2, DOT_STYLE_DFT);
    Paint_DrawPoint(10, 100, EPD_3IN0G_BLACK, DOT_PIXEL_3X3, DOT_STYLE_DFT);
    Paint_DrawLine(20, 70, 70, 120, EPD_3IN0G_RED, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    Paint_DrawLine(70, 70, 20, 120, EPD_3IN0G_RED, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    Paint_DrawRectangle(20, 70, 70, 120, EPD_3IN0G_YELLOW, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    Paint_DrawRectangle(80, 70, 130, 120, EPD_3IN0G_YELLOW, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    Paint_DrawCircle(45, 95, 20, EPD_3IN0G_BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    Paint_DrawCircle(105, 95, 20, EPD_3IN0G_BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    Paint_DrawLine(85, 95, 125, 95, EPD_3IN0G_RED, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);
    Paint_DrawLine(105, 75, 105, 115, EPD_3IN0G_YELLOW, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);
    Paint_DrawString_EN(10, 0, "Red, yellow, white and black", &Font16, EPD_3IN0G_RED, EPD_3IN0G_YELLOW);
    Paint_DrawString_EN(10, 35, "Four color e-Paper", &Font12, EPD_3IN0G_YELLOW, EPD_3IN0G_BLACK);
    Paint_DrawString_CN(10, 125, "微雪电子", &Font24CN, EPD_3IN0G_RED, EPD_3IN0G_WHITE);
    Paint_DrawNum(10, 50, 123456, &Font12, EPD_3IN0G_RED, EPD_3IN0G_WHITE);

    printf("EPD_Display\r\n");
    EPD_3IN0G_Display(BlackImage);
    DEV_Delay_ms(3000);
#endif

#if 1   // Drawing on the image
    //1.Select Image
    EPD_3IN0G_Init();
    printf("SelectImage:BlackImage\r\n");
    Paint_SelectImage(BlackImage);
    Paint_Clear(EPD_3IN0G_WHITE);

    // 2.Drawing on the image
    printf("Drawing:BlackImage\r\n");
    Paint_DrawRectangle(1, 1, 167, 86, EPD_3IN0G_RED, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    Paint_DrawRectangle(1, 314, 167, 399, EPD_3IN0G_YELLOW, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    Paint_DrawRectangle(59, 1, 109, 399, EPD_3IN0G_BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);

    printf("EPD_Display\r\n");
    EPD_3IN0G_Display(BlackImage);
    DEV_Delay_ms(3000);
#endif

#if 1   // Drawing on the image
    //1.Select Image
    EPD_3IN0G_Init();
    printf("SelectImage:BlackImage\r\n");
    Paint_SelectImage(BlackImage);
    Paint_Clear(EPD_3IN0G_WHITE);

    int hNumber, hWidth, vNumber, vWidth;
    hNumber = 20;
	hWidth = EPD_3IN0G_HEIGHT/hNumber; // 400/20=20
    vNumber = 8;
	vWidth = EPD_3IN0G_WIDTH/vNumber; // 168/8=21

    // 2.Drawing on the image
    printf("Drawing:BlackImage\r\n");
	for(int i=0; i<hNumber; i++) {  // horizontal
		Paint_DrawRectangle(1, 1+i*hWidth, EPD_3IN0G_WIDTH, hWidth*(1+i), EPD_3IN0G_BLACK + (i % 2), DOT_PIXEL_1X1, DRAW_FILL_FULL);
	}
	for(int i=0; i<vNumber; i++) {  // vertical
        if(i%2) {
            Paint_DrawRectangle(1+i*vWidth, 1, vWidth*(i+1), EPD_3IN0G_HEIGHT, EPD_3IN0G_YELLOW + (i/2%2), DOT_PIXEL_1X1, DRAW_FILL_FULL);
        }
	}
    printf("EPD_Display\r\n");
    EPD_3IN0G_Display(BlackImage);
    DEV_Delay_ms(3000);
#endif

    EPD_3IN0G_Init();
    printf("Clear...\r\n");
    EPD_3IN0G_Clear(EPD_3IN0G_WHITE);

    printf("Goto Sleep...\r\n");
    EPD_3IN0G_Sleep();
    free(BlackImage);
    BlackImage = NULL;
    DEV_Delay_ms(2000);//important, at least 2s
    // close 5V
    printf("close 5V, Module enters 0 power consumption ...\r\n");
}

/* The main loop -------------------------------------------------------------*/
void loop()
{
  //
}
