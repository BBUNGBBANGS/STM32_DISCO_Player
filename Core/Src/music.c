
#include "music.h"
#include "lcd.h"

FATFS SDFatFs; /* File system object for SD card logical drive */
FIL MyFile; /* File object */
DIR dir;                    // Directory
FILINFO fno;                // File Info
char SDPath[4]; /* SD card logical drive path */
uint16_t Music_file_count;

static uint8_t Music_File_Decision(char *name);

void Music_SdCard_Init(void)
{
    uint8_t tx_buf[100] = {0,};
    uint32_t byteswritten; /* File write count */
    uint8_t wtext[] = "SD card test in STM32F746G-DISCO board"; 
    f_mount(&SDFatFs, (TCHAR const*) SDPath, 0);
    f_open(&MyFile, "dummy.txt", FA_CREATE_ALWAYS | FA_WRITE);
    f_write(&MyFile, wtext, sizeof(wtext), (void *) &byteswritten);
    f_close(&MyFile);
}

void Music_File_Read(void)
{
    uint8_t retval = 0;
    uint8_t counter = 0;
    BSP_LCD_SetBackColor(LCD_COLOR_DARKGRAY);
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    f_opendir(&dir, "/");   // Open Root
    do
    {
        uint8_t tx_buf[50] = {0,};
        f_readdir(&dir, &fno);
        if (fno.fname[0] != 0)
        {
            counter++;
        }
        if ((counter > 1) && (counter < 8))
        {
            retval = Music_File_Decision((char *)fno.fname);
            if(retval == 1)
            {
                Music_file_count++;
                sprintf((char*)tx_buf, "%s", (char *)fno.fname);
                BSP_LCD_DisplayStringAt(37, 30 * (Music_file_count) + 6, (uint8_t *)&tx_buf, LEFT_MODE);
            }
        }

    } while(fno.fname[0] != 0);

    f_closedir(&dir);
}
 
void Music_Play(void)
{

}

static uint8_t Music_File_Decision(char *name)
{
    uint8_t counter = 0;
    while(name[counter] != 0)
    {
        counter++;
    }

    if(counter>0)
    {
        if((name[counter-3] == 'm')&&(name[counter-2] == 'p')&&(name[counter-1] == '3'))
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
    
    return 0;
}