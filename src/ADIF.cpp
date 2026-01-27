/*
 * ADIF.c
 *
 *  Created on: Jun 18, 2023
 *      Author: Charley
 */
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include <SD.h>
#include <RA8876_t3.h>
#include <Audio.h>
#include <si5351.h>

#include <TimeLib.h>

#include "arm_math.h"

#include "button.h"
#include "display.h"
#include "ADIF.h"
#include "gen_ft8.h"
#include "decode_ft8.h"
#include "main.h"
#include "Maps.h"
#include "Geodesy.h"

static const double EARTH_RAD = 6371; // radius in km

#include "EM29_4000.h"  //the picture
#include "EM29_8000.h"  //the picture
#include "EM29_16000.h" //the picture
#include "JM29_4000.h"  //the picture
#include "JM29_8000.h"  //the picture
#include "JM29_16000.h" //the picture

static char log_rtc_date_string[9];
static char log_rtc_time_string[9];

static char file_name_string[24];

static int ADIF_distance;
static int ADIF_map_distance;
static int ADIF_map_bearing;

static int16_t start_x;
static int16_t start_y;
static int16_t center_x;
static int16_t center_y;

static int16_t map_width;
static int16_t map_height;
static int16_t map_center_x;
static int16_t map_center_y;

static char map_locator[7];
static int map_key_index = 0;
static Map_Memory stored_log_entries[100] = {0};
static int number_logged = 0;
static float Station_Latitude, Station_Longitude;
static float Map_Latitude, Map_Longitude;
static float Target_Latitude, Target_Longitude;

static float QTH_Distance;
static float QTH_Bearing;

static File Log_File;

// convert degrees to radians
inline double deg2rad(double deg)
{
  return deg * (PI / 180.0);
}

inline double rad2deg(double rad)
{
  return (rad * 180) / PI;
}

static void make_date(void)
{
  getTeensy3Time();
  sprintf(log_rtc_date_string, "%04i%02i%02i", year(), month(), day());
}

void make_time(void)
{
  getTeensy3Time();
  sprintf(log_rtc_time_string, "%02i%02i%02i", hour(), minute(), second());
}

void make_File_Name(void)
{
  make_date();
  sprintf((char *)file_name_string, "%s.adi", log_rtc_date_string);
}

static void write_log_data(char *data)
{
  Log_File = SD.open(file_name_string, FILE_WRITE);
  Log_File.println(data);
  Log_File.close();
}

void Open_Log_File(void)
{
  Log_File = SD.open(file_name_string, FILE_WRITE);

  if (Log_File.size() == 0)
  {
    Log_File.println("ADIF EXPORT");
    Log_File.println("<eoh>");
  }

  Log_File.close();
}

void Init_Log_File(void)
{
  make_File_Name();
  Open_Log_File();
}

static void draw_vector(float distance, float bearing, int size, int color)
{
  float vector_scale = MapFiles[map_key_index].map_scale;
  float max_distance = MapFiles[map_key_index].max_distance;

  float vector_magnitude;
  float QTH_vector_magnitude;

  int plot_X, plot_Y;
  int plot_QTH_X, plot_QTH_Y;

  if (distance <= max_distance)
    vector_magnitude = distance / vector_scale;
  else
    vector_magnitude = 199;

  plot_X = (center_x) + (int)(sin(bearing * PI / 180) * vector_magnitude);
  plot_Y = (center_y) - (int)(cos(bearing * PI / 180) * vector_magnitude);

  QTH_vector_magnitude = QTH_Distance / vector_scale;

  plot_QTH_X = (center_x) + (int)(sin(QTH_Bearing * PI / 180) * QTH_vector_magnitude);
  plot_QTH_Y = (center_y) - (int)(cos(QTH_Bearing * PI / 180) * QTH_vector_magnitude);

  switch (color)
  {
  case 0: // Yellow
    tft.drawLine(plot_X, plot_Y, plot_QTH_X, plot_QTH_Y, YELLOW);
    tft.drawCircleFill(plot_X, plot_Y, size, YELLOW);
    break;
  case 1: // White
    tft.drawLine(plot_X, plot_Y, plot_QTH_X, plot_QTH_Y, WHITE);
    tft.drawCircleFill(plot_X, plot_Y, size, WHITE);
    break;
  case 2: // Green
    tft.drawLine(plot_X, plot_Y, plot_QTH_X, plot_QTH_Y, 0x0400);
    tft.drawCircleFill(plot_X, plot_Y, size, 0x0400);
    break;
  case 3: // Red
    tft.drawLine(plot_X, plot_Y, plot_QTH_X, plot_QTH_Y, RED);
    tft.drawCircleFill(plot_X, plot_Y, size, RED);
    break;
  }
}

// distance (km) on earth's surface from point 1 to point 2
static double distance(double lat1, double lon1, double lat2, double lon2)
{
  double lat1r = deg2rad(lat1);
  double lon1r = deg2rad(lon1);
  double lat2r = deg2rad(lat2);
  double lon2r = deg2rad(lon2);

  return acos(sin(lat1r) * sin(lat2r) + cos(lat1r) * cos(lat2r) * cos(lon2r - lon1r)) * EARTH_RAD;
}

float Target_Distance(const char *target)
{
  LatLong ll = QRAtoLatLong(target);
  if (ll.isValid)
  {
    Target_Latitude = ll.latitude;
    Target_Longitude = ll.longitude;
  }
  else
  {
    Target_Latitude = Target_Longitude = 0.0;
  }

  return distance(Station_Latitude, Station_Longitude, Target_Latitude, Target_Longitude);
}

static float Map_Distance(const char *target)
{
  LatLong ll = QRAtoLatLong(map_locator);
  if (ll.isValid)
  {
    Map_Latitude = ll.latitude;
    Map_Longitude = ll.longitude;
  }
  else
  {
    Map_Latitude = Map_Longitude = 0.0;
  }

  ll = QRAtoLatLong(target);
  if (ll.isValid)
  {
    Target_Latitude = ll.latitude;
    Target_Longitude = ll.longitude;
  }
  else
  {
    Target_Latitude = Target_Longitude = 0.0;
  }
  return distance(Map_Latitude, Map_Longitude, Target_Latitude, Target_Longitude);
}

static double bearing(double lat1, double long1, double lat2, double long2)
{
  double dlon = deg2rad(long2 - long1);
  lat1 = deg2rad(lat1);
  lat2 = deg2rad(lat2);
  double a1 = sin(dlon) * cos(lat2);
  double a2 = sin(lat1) * cos(lat2) * cos(dlon);
  a2 = cos(lat1) * sin(lat2) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0)
  {
    a2 += (2 * PI);
  }
  return rad2deg(a2);
}

float Map_Bearing(const char *target)
{
  LatLong ll = QRAtoLatLong(map_locator);
  if (ll.isValid)
  {
    Map_Latitude = ll.latitude;
    Map_Longitude = ll.longitude;
  }
  else
  {
    Map_Latitude = Map_Longitude = 0.0;
  }

  ll = QRAtoLatLong(target);
  if (ll.isValid)
  {
    Target_Latitude = ll.latitude;
    Target_Longitude = ll.longitude;
  }
  else
  {
    Target_Latitude = Target_Longitude = 0.0;
  }

  return bearing(Map_Latitude, Map_Longitude, Target_Latitude, Target_Longitude);
}

static unsigned num_digits(int num)
{
  int count = 0;
  if ((num <= -100) && (num > -1000))
    count = 4;
  else if (((num >= 100) && (num < 1000)) || ((num <= -10) && (num > -100)))
    count = 3;
  else if (((num >= 10) && (num < 100)) || ((num <= -1) && num > -10))
    count = 2;
  else if (num >= 0 && num < 10)
    count = 1;
  return count;
}

static const char *trim_front(const char *ptr)
{
  while (isspace((int)*ptr))
    ++ptr;
  return ptr;
}

static unsigned num_chars(const char *ptr)
{
  return (unsigned)strlen(trim_front(ptr));
}

void write_ADIF_Log()
{
  static char log_line[300];
  char freq[10];

  make_time();
  make_date();

  sprintf(freq, "%u.%.3u", sBand_Data[BandIndex].Frequency / 1000, sBand_Data[BandIndex].Frequency % 1000);

  int offset = sprintf(log_line, "<call:%1u>%s ", num_chars(Target_Call), trim_front(Target_Call));
  int target_locator_len = num_chars(Target_Locator);
  if (target_locator_len > 0)
    offset += sprintf(log_line + offset, "<gridsquare:%1u>%s ", target_locator_len, trim_front(Target_Locator));
  offset += sprintf(log_line + offset, "<mode:3>FT8<qso_date:%1u>%s ", num_chars(log_rtc_date_string), trim_front(log_rtc_date_string));
  offset += sprintf(log_line + offset, "<time_on:%1u>%s ", num_chars(log_rtc_time_string), trim_front(log_rtc_time_string));
  offset += sprintf(log_line + offset, "<freq:%1u>%s ", num_chars(freq), trim_front(freq));
  offset += sprintf(log_line + offset, "<station_callsign:%1u>%s ", num_chars(Station_Call), trim_front(Station_Call));
  offset += sprintf(log_line + offset, "<my_gridsquare:%1u>%s ", num_chars(Station_Locator), trim_front(Station_Locator));

  int rsl_len = num_digits(Target_RSL);
  if (rsl_len > 0)
    offset += sprintf(log_line + offset, "<rst_sent:%1u:N>%i ", rsl_len, Target_RSL);

  rsl_len = num_digits(Station_RSL);
  if (rsl_len > 0)
    offset += sprintf(log_line + offset, "<rst_rcvd:%1u:N>%i ", rsl_len, Station_RSL);

  strcpy(log_line + offset, "<tx_pwr:4>0.5 <eor>");

  // Force NULL termination
  log_line[sizeof(log_line) - 1] = 0;

  write_log_data(log_line);
  if (Auto_QSO) store_logged_CQ_Call(Target_Call);

  LatLong ll = QRAtoLatLong(Target_Locator);
  if (ll.isValid)
    ADIF_distance = Target_Distance(Target_Locator);
  else
    ADIF_distance = 0;

  if (ADIF_distance > 0)
  {
    ADIF_map_distance = Map_Distance(Target_Locator);
    ADIF_map_bearing = Map_Bearing(Target_Locator);

    draw_vector(ADIF_map_distance, ADIF_map_bearing, 3, 3);
    stored_log_entries[number_logged].ADIF_map_distance = ADIF_map_distance;
    stored_log_entries[number_logged].ADIF_map_bearing = ADIF_map_bearing;
    number_logged++;
  }
}

static void draw_QTH(void)
{
  QTH_Distance = Map_Distance(Station_Locator);
  QTH_Bearing = Map_Bearing(Station_Locator);

  draw_vector(QTH_Distance, QTH_Bearing, 3, 2);
}

static void drawImage(uint16_t image_width, uint16_t image_height, uint16_t image_x, uint16_t image_y, const uint16_t *image)
{
  start_x = (1023 - image_width);
  start_y = (100);
  center_x = (start_x + image_x);
  center_y = (100 + image_y);

  tft.fillRect(588, 100, 435, 435, BLACK);

  tft.writeRect(start_x, start_y, image_width, image_height, image);

  tft.drawLine(center_x - 10, center_y, center_x + 10, center_y, RED); // This puts cross hair on map at map center
  tft.drawLine(center_x, center_y - 10, center_x, center_y + 10, RED);
}

static void draw_stored_entries(void)
{
  for (int j = 0; j < number_logged && j < (int)(sizeof(stored_log_entries) / sizeof(stored_log_entries[0])); j++)
    draw_vector(stored_log_entries[j].ADIF_map_distance, stored_log_entries[j].ADIF_map_bearing, 3, 3);
}

void draw_map(int16_t index)
{
  map_width = MapFiles[index].map_width;
  map_height = MapFiles[index].map_height;
  map_center_x = MapFiles[index].map_center_x;
  map_center_y = MapFiles[index].map_center_y;

  switch (index)
  {
  case 0:
    drawImage(map_width, map_height, map_center_x, map_center_y, (uint16_t *)EM29_4000);
    break;

  case 1:
    drawImage(map_width, map_height, map_center_x, map_center_y, (uint16_t *)EM29_8000);
    break;

  case 2:
    drawImage(map_width, map_height, map_center_x, map_center_y, (uint16_t *)EM29_16000);
    break;

  case 3:
    drawImage(map_width, map_height, map_center_x, map_center_y, (uint16_t *)JM29_4000);
    break;

  case 4:
    drawImage(map_width, map_height, map_center_x, map_center_y, (uint16_t *)JM29_8000);
    break;

  case 5:
    drawImage(map_width, map_height, map_center_x, map_center_y, (uint16_t *)JM29_16000);
    break;
  }

  strcpy(map_locator, MapFiles[index].map_locator);
  map_key_index = index;
  draw_QTH();
  drawButton(14);
  drawButton(15);
  draw_stored_entries();
}

void set_Station_Coordinates()
{
  LatLong ll = QRAtoLatLong(Station_Locator);
  if (ll.isValid)
  {
    Station_Latitude = ll.latitude;
    Station_Longitude = ll.longitude;
  }
  else
  {
    Station_Latitude = Station_Longitude = 0.0;
  }
}




