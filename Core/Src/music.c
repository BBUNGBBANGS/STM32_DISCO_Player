
#include "music.h"
#include "lcd.h"

FATFS SDFatFs; /* File system object for SD card logical drive */
FIL MyFile; /* File object */
DIR dir;                    // Directory
FILINFO fno;                // File Info
char SDPath[4]; /* SD card logical drive path */
uint16_t Music_file_count;

void Music_SdCard_Init(void)
{
    f_mount(&SDFatFs, (TCHAR const*) SDPath, 0);
}

void Music_File_Read(void)
{
    BSP_LCD_SetBackColor(LCD_COLOR_DARKGRAY);
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    f_opendir(&dir, "/");   // Open Root
    do
    {
        uint8_t tx_buf[50] = {0,};
        f_readdir(&dir, &fno);
        if (fno.fname[0] != 0)
        {
            Music_file_count++;
        }
        if ((Music_file_count > 1) && (Music_file_count < 8))
        {
            sprintf((char*)tx_buf, "%s", (char *)fno.fname);
            BSP_LCD_DisplayStringAt(37, 30 * (Music_file_count - 1) + 6, (uint8_t *)&tx_buf, LEFT_MODE);
        }

    } while(fno.fname[0] != 0);

    f_closedir(&dir);
    Music_file_count = Music_file_count - 1;
}
 
void Music_Play(void)
{

}