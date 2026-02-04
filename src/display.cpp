#include <RA8876_t3.h>
#include <Audio.h>
#include <si5351.h>
#include <SD.h>
#include <SPI.h>

#include <TimeLib.h>

#include "display.h"
#include "decode_ft8.h"
#include "ADIF.h"
#include "button.h"
#include "main.h"
#include "gen_ft8.h"
#include "ini.h"
#include "autoseq_engine.h"

File stationData_File;

static const int max_log_messages = 9;
display_message_details log_messages[max_log_messages];
char current_message[22];

static int old_rtc_hour = -1;

void display_value(int x, int y, int value)
{
  char string[5];
  sprintf(string, "%4i", value);

  tft.textColor(YELLOW, BLACK);
  tft.setCursor(x, y);
  tft.setFontSize(2, true);
  tft.write(string, 5);
}

void show_wide(uint16_t x, uint16_t y, int variable)
{
  char string[7];
  sprintf(string, "%6i", variable);
  tft.textColor(YELLOW, BLACK);
  tft.setCursor(x, y);
  tft.setFontSize(2, true);
  tft.write(string, 7);
}

void display_time(int x, int y)
{
  getTeensy3Time();
  char string[10];
  sprintf(string, "%2.2i:%2.2i:%2.2i", hour(), minute(), second());

  if (hour() < old_rtc_hour)
  {
    Init_Log_File();
    display_date(650, 30);
    //clear_auto_memories();
  }

  old_rtc_hour = hour();

  tft.textColor(WHITE, BLACK);
  tft.setCursor(x, y);
  tft.setFontSize(2, true);
  tft.write(string, 8);
}

void display_date(int x, int y)
{
  getTeensy3Time();
  char string[11];
  sprintf(string, "%2.2i/%2.2i/%4.4i", day(), month(), year());
  tft.textColor(WHITE, BLACK);
  tft.setCursor(x, y);
  tft.setFontSize(2, true);
  tft.write(string, 11);
}

static int setup_station_call(const char *call_part)
{
  int result = 0;
  if (call_part != NULL)
  {
    size_t i = strlen(call_part);
    result = i > 0 && i < sizeof(Station_Call) ? 1 : 0;
    if (result != 0)
    {
      strcpy(Station_Call, call_part);
    }
  }
  return result;
}

static int setup_locator(const char *locator_part)
{
  int result = 0;
  if (locator_part != NULL)
  {
    size_t i = strlen(locator_part);
    result = i > 0 && i < sizeof(Station_Locator) ? 1 : 0;
    if (result != 0)
    {
      strcpy(Station_Locator, locator_part);
      memcpy(Short_Station_Locator, locator_part, 4);
      Short_Station_Locator[4] = 0;
    }
  }
  return result;
}

enum
{
  FreeText1,
  FreeText2
};

static int setup_free_text(const char *free_text, int field_id)
{
  int result = 0;
  if (free_text != NULL)
  {
    size_t i = strlen(free_text);
    switch (field_id)
    {
    case FreeText1:
      result = i < sizeof(Free_Text1) ? 1 : 0;
      if (i > 0 && result != 0)
        strcpy(Free_Text1, free_text);
      break;
    case FreeText2:
      result = i < sizeof(Free_Text2) ? 1 : 0;
      if (i > 0 && result != 0)
        strcpy(Free_Text2, free_text);
      break;
    default:
      result = 1;
    }
  }
  return result;
}

bool open_stationData_file(void)
{
  Station_Call[0] = 0;
  Station_Locator[0] = 0;
  Short_Station_Locator[0] = 0;
  Free_Text1[0] = 0;
  Free_Text2[0] = 0;

  char read_buffer[256];
  if (!SD.begin(BUILTIN_SDCARD))
  {
    tft.textColor(RED, BLACK);
    tft.setCursor(0, 300);
    tft.setFontSize(2, true);
    tft.write("SD Card not found", 17);
    return false;
  }
  else
  {
    size_t bytes_read = 0;
    stationData_File = SD.open("StationData.ini", FILE_READ);

    if (stationData_File)
    {
      stationData_File.seek(0);
      bytes_read = stationData_File.read(read_buffer, 256);
      ini_data_t ini_data;
      parse_ini(read_buffer, bytes_read, &ini_data);
      const ini_section_t *section = get_ini_section(&ini_data, "Station");
      if (section != NULL)
      {
        setup_station_call(get_ini_value_from_section(section, "Call"));
        setup_locator(get_ini_value_from_section(section, "Locator"));
      }

      section = get_ini_section(&ini_data, "FreeText");
      if (section != NULL)
      {
        setup_free_text(get_ini_value_from_section(section, "1"), FreeText1);
        setup_free_text(get_ini_value_from_section(section, "2"), FreeText2);
      }

      section = get_ini_section(&ini_data, "BandData");
      if (section != NULL)
      {
        // see BandIndex
        static const char *bands[NumBands] = {"40", "30", "20", "17", "15", "12", "10"};
        for (int idx = _40M; idx <= _10M; ++idx)
        {
          const char *band_data = get_ini_value_from_section(section, bands[idx]);
          if (band_data != NULL)
          {
            size_t band_data_size = strlen(band_data) + 1;
            if (band_data_size < BAND_DATA_SIZE)
            {
              sBand_Data[idx].Frequency = (uint16_t)(atof(band_data) * 1000);
              memcpy(sBand_Data[idx].display, band_data, band_data_size);
            }
          }
        }
      }

      stationData_File.close();
    }
    else
    {
      stationData_File = SD.open("StationData.txt", FILE_READ);

      if (stationData_File)
      {
        stationData_File.seek(0);
        stationData_File.read(read_buffer, 64);
      }

      char *call_part, *locator_part = NULL, *free_text1_part = NULL, *free_text2_part = NULL;

      call_part = strtok(read_buffer, ":");

      if (call_part != NULL)
        locator_part = strtok(NULL, ":");

      if (locator_part != NULL)
        free_text1_part = strtok(NULL, ":");

      if (free_text1_part != NULL)
        free_text2_part = strtok(NULL, ":");

      stationData_File.close();

      if (call_part != NULL)
      {
        setup_station_call(call_part);
      }
      if (locator_part != NULL)
      {
        setup_locator(locator_part);
      }
      if (free_text1_part != NULL)
      {
        setup_free_text(free_text1_part, FreeText1);
      }
      if (free_text2_part != NULL)
      {
        setup_free_text(free_text2_part, FreeText2);
      }
    }
  }
  return true;
}

void display_station_data(int x, int y)
{
  char str[13];
  sprintf(str, "%7s %4s", Station_Call, Short_Station_Locator);

  tft.textColor(YELLOW, BLACK);
  tft.setCursor(x, y);
  tft.setFontSize(2, true);
  tft.write(str, 13);
}

void display_revision_level(void)
{
  tft.textColor(YELLOW, BLACK);
  tft.setFontSize(2, true);

  tft.setCursor(0, 100);
  tft.write("RA8876_DX FT8", 13);

  tft.setCursor(0, 130);
  tft.write("Hardware: V3.0", 14);

  tft.setCursor(0, 160);
  tft.write("Firmware: V1.5", 16);

  tft.setCursor(0, 190);
  tft.write("W5BAA - WB2CBA", 14);

  tft.setCursor(0, 220);

  if (Band_Minimum == _20M)
    tft.write("Five Band Board", 15);
  else
    tft.write("Seven Band Board", 16);

  tft.textColor(GREEN, BLACK);
  tft.setCursor(0, 270);
  tft.write("Please Wait While", 17);
  tft.setCursor(0, 300);
  tft.write("Gears & Pulleys", 15);
  tft.setCursor(0, 330);
  tft.write("Are Aligned", 12);

  delay(5000);

  tft.fillRect(0, 100, 300, 400, BLACK);
}

void show_decimal(uint16_t x, uint16_t y, float variable)
{
  char str[23];
  int units, fraction;
  float remainder;
  units = (int)variable;
  remainder = variable - units;
  fraction = (int)(remainder * 1000);
  if (fraction < 0)
    fraction = fraction * -1;
  sprintf(str, "%3i.%3i", units, fraction);

  tft.textColor(YELLOW, BLACK);
  tft.setCursor(x, y);
  tft.setFontSize(2, true);
  tft.write(str, 8);
}

void Be_Patient(void)
{
  tft.fillRect(0, 100, 300, 400, BLACK);
  tft.textColor(YELLOW, BLACK);
  tft.setCursor(0, 120);
  tft.setFontSize(2, true);
  tft.write("Patience, Synching", 19);
}

void update_message_log_display(int mode)
{
  const char blank[] = "                      ";

  for (int i = 0; i < max_log_messages - 1; i++)
  {
    strcpy(log_messages[i].message, blank);
    strcpy(log_messages[i].message, log_messages[i + 1].message);
    log_messages[i].text_color = log_messages[i + 1].text_color;
  }

  if (mode)
  {
    strcpy(log_messages[max_log_messages - 1].message, blank);
    strcpy(log_messages[max_log_messages - 1].message, current_message);
    log_messages[max_log_messages - 1].text_color = 1;
  }
  else
  {
    strcpy(log_messages[max_log_messages - 1].message, blank);
    strcpy(log_messages[max_log_messages - 1].message, current_message);
    log_messages[max_log_messages - 1].text_color = 0;
  }

  log_display_flag = 1;
}

void display_logged_messages(void)
{
  clear_qso_region();

  for (int i = 0; i < max_log_messages; i++)
  {
    if (log_messages[i].text_color)
      display_line(true, i + 2, Black, Yellow, log_messages[i].message);
    else
      display_line(true, i + 2, Black, Red, log_messages[i].message);
  }
}
