#include <si5351.h>

#include <RA8876_t3.h>
#include "button.h"
#include "display.h"
#include "gen_ft8.h"
#include "traffic_manager.h"
#include "decode_ft8.h"
#include "Process_DSP.h"
#include <EEPROM.h>
#include <Wire.h>
#include "locator.h"
#include <TimeLib.h>
#include "options.h"
#include "ADIF.h"
#include <Audio.h>

#define Board_PIN 2
#define Relay_PIN 3
#define RxSw_PIN 4
#define TxSw_PIN 5

#define CQSOTA 18
#define CQPOTA 19
#define QRPP 20
#define StandardCQ 17
#define FreeText1 21
#define FreeText2 22
#define CQFree 4

extern int Transmit_Armned;

extern RA8876_t3 tft;
extern AudioControlSGTL5000 sgtl5000_1;
extern Si5351 si5351;
extern AudioAmplifier amp1;
extern AudioAmplifier        in_left_amp;
extern AudioAmplifier        in_right_amp;
extern uint16_t currentFrequency;
#define USB 2

uint16_t draw_x, draw_y, touch_x, touch_y;
int test;

#define SCREEN_WIDTH 1024
#define SCREEN_HEIGHT 600
#define ft8_shift 6.25

extern int master_decoded;
extern void sync_FT8(void);
extern uint16_t cursor_freq;
extern int Tune_On;
extern int Auto_QSO_State;

extern uint16_t cursor_line;
uint16_t display_cursor_line;
extern int offset_freq;
extern int xmit_flag;

int start_up_offset_freq;
extern int logging_on;
int button_delay = 5;

uint8_t RX_volume;
int RF_Gain;

int FT8_Touch_Flag;
int FT8_Message_Touch;
int FT_8_TouchIndex;
int FT_8_MessageIndex;

int CQ_Flag;
int QSO_Flag;
int Beacon_State;
int Target_Flag;
int Beacon_On;
int Auto_Sync;
extern int QSO_xmit;

uint16_t start_freq;
int Arm_Tune;
int Clock_Correction;
int32_t cal_factor = 2000;
int bfo_offset;
int BandIndex;
int Band_Minimum;
int QSO_Fix;

int CQ_Mode_Index;
int Free_Index;

int freq_scale = 10;
int Map_Index;
int Map_Max = 6;

extern char Free_Text1[];
extern char Free_Text2[];

#define numButtons 23
#define button_height 100
#define button_height_Low 60

#define button_width 80
#define line0 20
#define line1 539
#define line2 120
#define line3 200
#define line4 300
#define line5 400

#define numBands 7

FreqStruct sBand_Data[] = {

    {7074,
     "7074"},

    {10136,
     "10136"},

    {14074,
     "14074"},

    {18100,
     "18100"},

    {21074,
     "21074"},

    {24915,
     "24915"},

    {28074,
     "28074"}};

ButtonStruct sButtonData[] = {

    {// button 0  Clear Message Displays
     /*text0*/ "Clr ",
     /*text1*/ "Clr ",
     /*blank*/ "    ",
     /*Active*/ 1,
     /*Displayed*/ 1,
     /*state*/ 0,
     /*x*/ 0,
     /*y*/ line1,
     /*w*/ button_width,
     /*h*/ button_height},

    {// button 1 control Beaconing / CQ chasing
     /*text0*/ "QSO ",
     /*text1*/ "Becn",
     /*blank*/ "    ",
     /*Active*/ 1,
     /*Displayed*/ 1,
     /*state*/ 0,
     /*x*/ 80,
     /*y*/ line1,
     /*w*/ button_width,
     /*h*/ button_height},

    {// button 2 control Tune
     /*text0*/ "Tune",
     /*text1*/ "Tune",
     /*blank*/ "    ",
     /*Active*/ 1,
     /*Displayed*/ 1,
     /*state*/ 0,
     /*x*/ 160,
     /*y*/ line1,
     /*w*/ button_width,
     /*h*/ button_height},

    {// button 3 display R/T status
     /*text0*/ " Rx ",
     /*text1*/ " Tx ",
     /*blank*/ "    ",
     /*Active*/ 1,
     /*Displayed*/ 1,
     /*state*/ 0,
     /*x*/ 240,
     /*y*/ line1,
     /*w*/ 0, // setting the width and height to 0 turns off touch response , display only
     /*h*/ 0},

    {// button 4 CQ or free mode
     /*text0*/ " CQ ",
     /*text1*/ "Free",
     /*blank*/ "    ",
     /*Active*/ 1,
     /*Displayed*/ 1,
     /*state*/ 0,
     /*x*/ 320,
     /*y*/ line1,
     /*w*/ button_width,
     /*h*/ button_height},

    {// button 5 Fixed or Receive Freq message response
     /*text0*/ "Fixd",
     /*text1*/ "Freq",
     /*blank*/ "   ",

     /*Active*/ 1,
     /*Displayed*/ 1,

     /*state*/ 0,
     /*x*/ 400,
     /*y*/ line1,
     /*w*/ button_width,
     /*h*/ button_height},

    {// button 6 Sync FT8
     /*text0*/ "Sync",
     /*text1*/ "Sync",
     /*blank*/ "    ",
     /*Active*/ 1,
     /*Displayed*/ 1,
     /*state*/ 0,
     /*x*/ 480,
     /*y*/ line1,
     /*w*/ button_width,
     /*h*/ button_height

    },

    {// button 7 Lower Gain
     /*text0*/ " G- ",
     /*text1*/ " G- ",
     /*blank*/ "    ",
     /*Active*/ 2,
     /*Displayed*/ 1,
     /*state*/ 0,
     /*x*/ 560,
     /*y*/ line1,
     /*w*/ button_width,
     /*h*/ button_height

    },

    {// button 8 Raise Gain
     /*text0*/ " G+ ",
     /*text1*/ " G+ ",
     /*blank*/ "    ",
     /*Active*/ 2,
     /*Displayed*/ 1,
     /*state*/ 0,
     /*x*/ 680,
     /*y*/ line1,
     /*w*/ button_width,
     /*h*/ button_height

    },

    {// button 9 Lower Cursor
     /*text0*/ " F- ",
     /*text1*/ " F- ",
     /*blank*/ "    ",
     /*Active*/ 2,
     /*Displayed*/ 1,
     /*state*/ 0,
     /*x*/ 780,
     /*y*/ line1,
     /*w*/ button_width,
     /*h*/ button_height

    },

    {// button 10 Raise Cursor
     /*text0*/ " F+ ",
     /*text1*/ " F+ ",
     /*blank*/ "    ",
     /*Active*/ 2,
     /*Displayed*/ 1,
     /*state*/ 0,
     /*x*/ 930,
     /*y*/ line1,
     /*w*/ button_width,
     /*h*/ button_height

    },

    {// button 11 Band Down
     /*text0*/ "Band-",
     /*text1*/ "Band-",
     /*blank*/ "    ",
     /*Active*/ 0,
     /*Displayed*/ 1,
     /*state*/ 0,
     /*x*/ 0, // 700 - 700 = 0
     /*y*/ line2,
     /*w*/ button_width,
     /*h*/ button_height

    },

    {// button 12 Band Up
     /*text0*/ "Band+",
     /*text1*/ "Band+",
     /*blank*/ "     ",
     /*Active*/ 0,
     /*Displayed*/ 1,
     /*state*/ 0,
     /*x*/ 200, // 900 - 700 = 200
     /*y*/ line2,
     /*w*/ button_width,
     /*h*/ button_height

    },

    {// button 13 Save Band Data
     /*text0*/ "Save",
     /*text1*/ "Save",
     /*blank*/ "    ",
     /*Active*/ 0,
     /*Displayed*/ 1,
     /*state*/ 0,
     /*x*/ 300, // 800-700 = 100
     /*y*/ line2,
     /*w*/ button_width,
     /*h*/ button_height

    },

    {// button 14 Xmit for Calibration
     /*text0*/ "Xmit",
     /*text1*/ "Xmit",
     /*blank*/ "    ",
     /*Active*/ 0,
     /*Displayed*/ 1,
     /*state*/ 0,
     /*x*/ 400, // 800 - 700 = 100
     /*y*/ line2,
     /*w*/ button_width,
     /*h*/ button_height

    },

    {// button 15 Map Scale Down

     /*text0*/ "Map-",
     /*text1*/ "Map-",
     /*blank*/ "    ",
     /*Active*/ 0,
     /*Displayed*/ 1,
     /*state*/ 0,
     /*x*/ 500, // 800 - 700 = 100
     /*y*/ 480,
     /*w*/ button_width,
     /*h*/ button_height

    },

    {// button 16 Map Scale Up
     /*text0*/ "Map+",
     /*text1*/ "Map+",
     /*blank*/ "    ",
     /*Active*/ 0,
     /*Displayed*/ 1,
     /*state*/ 0,
     /*x*/ 500, // 800 - 700 = 100
     /*y*/ 400,
     /*w*/ button_width,
     /*h*/ button_height

    },

    {// button 17 Standard CQ
     /*text0*/ " CQ ",
     /*text1*/ " CQ ",
     /*blank*/ "    ",
     /*Active*/ 0,
     /*Displayed*/ 1,
     /*state*/ 1,
     /*x*/ 0,
     /*y*/ line3,
     /*w*/ button_width,
     /*h*/ button_height},

    {// button 18 CQ SOTA
     /*text0*/ "SOTA",
     /*text1*/ "SOTA",
     /*blank*/ "    ",
     /*Active*/ 0,
     /*Displayed*/ 1,
     /*state*/ 0,
     /*x*/ 100,
     /*y*/ line3,
     /*w*/ button_width,
     /*h*/ button_height},

    {// button 19 CQ POTA
     /*text0*/ "POTA",
     /*text1*/ "POTA",
     /*blank*/ "    ",
     /*Active*/ 0,
     /*Displayed*/ 1,
     /*state*/ 0,
     /*x*/ 200,
     /*y*/ line3,
     /*w*/ button_width,
     /*h*/ button_height},

    {// button 20 CQ QRP
     /*text0*/ "QRP ",
     /*text1*/ "QRP ",
     /*blank*/ "   ",
     /*Active*/ 0,
     /*Displayed*/ 1,
     /*state*/ 0,
     /*x*/ 300,
     /*y*/ line3,
     /*w*/ button_width,
     /*h*/ button_height},

    {// button 21 Free Text 1
     /*text0*/ "Fr-1",
     /*text1*/ "Fr-1",
     /*blank*/ "    ",
     /*Active*/ 0,
     /*Displayed*/ 1,
     /*state*/ 0,
     /*x*/ 0,
     /*y*/ line4,
     /*w*/ button_width,
     /*h*/ button_height},

    {// button 22 Free Text 2
     /*text0*/ "Fr-2",
     /*text1*/ "Fr-2",
     /*blank*/ "    ",
     /*Active*/ 0,
     /*Displayed*/ 1,
     /*state*/ 0,
     /*x*/ 0,
     /*y*/ line5,
     /*w*/ button_width,
     /*h*/ button_height},

};

void drawButton(uint16_t i)
{

  tft.setFontSize(2, true);
  if (sButtonData[i].Active > 0)
  {

    if (sButtonData[i].state == 1)
      tft.textColor(WHITE, RED);
    else
      tft.textColor(WHITE, BLUE);

    if (sButtonData[i].state == 1)
    {
      tft.setCursor(sButtonData[i].x + 7, sButtonData[i].y + 20);
      tft.write(sButtonData[i].text1, 4);
    }
    else
    {
      tft.setCursor(sButtonData[i].x + 7, sButtonData[i].y + 20);
      tft.write(sButtonData[i].text0, 4);
    }
  }
}

void display_all_buttons(void)
{
  drawButton(0);
  drawButton(1);
  drawButton(2);
  drawButton(3);
  drawButton(4);
  drawButton(5);
  drawButton(6);
  drawButton(7);
  drawButton(8);
  drawButton(9);
  drawButton(10);
}

void checkButton(void)
{
  uint16_t i;

  for (i = 0; i < numButtons; i++)
  {

    if (testButton(i) == 1)
    {

      switch (sButtonData[i].Active)
      {

      case 0:

        break;

      case 1:
        sButtonData[i].state = !sButtonData[i].state;
        drawButton(i);
        executeButton(i);
        break;

      case 2:
        executeButton(i);
        break;

      case 3:
        executeCalibrationButton(i);

        break;
      }
    }
  }
}

void executeButton(uint16_t index)
{

  switch (index)
  {

  case 0:

    sButtonData[0].state = 1;
    drawButton(0);
    delay(40);

    clear_xmit_messages();
    terminate_QSO();
    Auto_QSO_State = 0;
    QSO_xmit = 0;
    clear_reply_message_box();
    clear_log_stored_data();
    clear_log_messages();

    erase_CQ();
    sButtonData[0].state = 0;
    drawButton(0);

    break;

  case 1:

    if (!sButtonData[1].state)
    {
      Beacon_On = 0;
      Beacon_State = 0;
      clear_reply_message_box();
      clear_log_messages();
      clear_log_stored_data();
    }
    else
    {
      Beacon_On = 1;
      clear_reply_message_box();
      clear_log_stored_data();
      clear_log_messages();
      Beacon_State = 1;
    }

    delay(5);
    break;

  case 2:

    if (!sButtonData[2].state)
    {
      tune_Off_sequence();
      Tune_On = 0;
      Arm_Tune = 0;
      xmit_flag = 0;
      receive_sequence();
      erase_Cal_Display();
      delay(5);
    }
    else
    {
      Tune_On = 1; // Turns off display of FT8 traffic
      setup_Cal_Display();
      Arm_Tune = 0;
    }

    break;

  case 3:

    // no code required, all dependent stuff works off of button state
    break;

  case 4:
    if (!sButtonData[4].state && Free_Index != 0)
    {
      Free_Index = 0;
    }
    break;

  case 5:

    delay(10);
    if (sButtonData[5].state == 1)
      QSO_Fix = 1;
    else
      QSO_Fix = 0;

    break;

  case 6:

    if (!sButtonData[6].state)
      Auto_Sync = 0;
    else
    {
      Auto_Sync = 1;
      Be_Patient();
    }

    break;

  case 7: // Lower Gain

    if (RF_Gain >= 2)
      RF_Gain--;
    display_value(620, 559, RF_Gain);
    set_RF_Gain(RF_Gain);

    break;

  case 8: // Raise Gain

    if (RF_Gain < 32)
      RF_Gain++;
    display_value(620, 559, RF_Gain);
    set_RF_Gain(RF_Gain);

    break;

  case 9: // Lower Freq

    if (cursor_line > 0)
    {
      cursor_line--;
      cursor_freq = (uint16_t)((float)(cursor_line + ft8_min_bin) * ft8_shift);
      display_cursor_line = 2 * cursor_line;
      display_value(870, 559, cursor_freq);
    }

    break;

  case 10: // Raise Freq

    if (cursor_line <= (ft8_buffer - ft8_min_bin - 2))
    {
      cursor_line++;
      cursor_freq = (uint16_t)((float)(cursor_line + ft8_min_bin) * ft8_shift);
      display_cursor_line = 2 * cursor_line;
      display_value(870, 559, cursor_freq);
    }
    break;

  case 14: // Transmit for Calibration   17

    if (!sButtonData[14].state)
    {

      tune_Off_sequence();
      Arm_Tune = 0;
      xmit_flag = 0;
      receive_sequence();
    }
    else
    {
      transmit_sequence();
      delay(10);
      xmit_flag = 1;
      tune_On_sequence();
      Arm_Tune = 1;
    }

    break;
  }
}

void executeCalibrationButton(uint16_t index)
{

  switch (index)
  {

  case 11: // Lower Band

    if (BandIndex > Band_Minimum)
    {
      BandIndex--;
      show_wide(90, 140, sBand_Data[BandIndex].Frequency); // 790 - 700 = 90
    }

    break;

  case 12: // Raise Band

    if (BandIndex < _10M)
    {
      BandIndex++;
      show_wide(90, 140, sBand_Data[BandIndex].Frequency); // 790 - 700 = 90
    }

    break;

  case 13: // Save Band Changes

    Options_SetValue(0, BandIndex);
    Options_StoreValue(0);
    start_freq = sBand_Data[BandIndex].Frequency;
    show_wide(680, 0, (int)start_freq);

    set_Rcvr_Freq();
    delay(10);

    SelectFilterBlock();

    sButtonData[13].state = 1;
    drawButton(13);
    delay(10);
    sButtonData[13].state = 0;
    drawButton(13);

    break;

  case 15: // Lower Map_Index

    if (Map_Index > 0)
    {
      Map_Index--;
      draw_map(Map_Index);
      Options_SetValue(1, Map_Index);
      Options_StoreValue(1);
      Map_Index = Options_GetValue(1);
    }

    sButtonData[15].state = 1;
    drawButton(15);
    delay(10);
    sButtonData[15].state = 0;
    drawButton(15);

    break;

  case 16: // Raise Map_Index

    if (Map_Index < Map_Max)
    {
      Map_Index++;
      draw_map(Map_Index);
      Options_SetValue(1, Map_Index);
      Options_StoreValue(1);
      Map_Index = Options_GetValue(1);
    }

    sButtonData[16].state = 1;
    drawButton(16);
    delay(10);
    sButtonData[16].state = 0;
    drawButton(16);

    break;

  case StandardCQ:
    if (!sButtonData[StandardCQ].state)
    {
      CQ_Mode_Index = 0;
      reset_buttons(CQSOTA, CQPOTA, QRPP, " CQ ");
      sButtonData[StandardCQ].state = 1;
      drawButton(StandardCQ);
    }
    display_value(0, 300, StandardCQ);
    display_value(100, 300, Free_Index);

    break;

  case CQSOTA:
    if (!sButtonData[CQSOTA].state)
    {
      CQ_Mode_Index = 1;
      reset_buttons(StandardCQ, CQPOTA, QRPP, "SOTA");
      sButtonData[CQSOTA].state = 1;
      drawButton(CQSOTA);
    }
    display_value(0, 300, CQSOTA);
    display_value(100, 300, Free_Index);
    break;

  case CQPOTA:
    if (!sButtonData[CQPOTA].state)
    {
      CQ_Mode_Index = 2;
      reset_buttons(StandardCQ, CQSOTA, QRPP, "POTA");
      sButtonData[CQPOTA].state = 1;
      drawButton(CQPOTA);
    }
    display_value(0, 300, CQPOTA);
    display_value(100, 300, Free_Index);
    break;

  case QRPP:
    if (!sButtonData[QRPP].state)
    {
      CQ_Mode_Index = 3;
      reset_buttons(StandardCQ, CQSOTA, CQPOTA, "QRP ");
      sButtonData[QRPP].state = 1;
      drawButton(QRPP);
    }
    display_value(0, 300, QRPP);
    display_value(100, 300, Free_Index);
    break;

  case 21:
    Free_Index = 0;
    sButtonData[FreeText2].state = 0;
    drawButton(FreeText2);

    if (!sButtonData[FreeText1].state)
    {
      Free_Index = 1;
      sButtonData[FreeText1].state = 1;
      drawButton(FreeText1);
    }
    else
    {
      sButtonData[FreeText1].state = 0;
      drawButton(FreeText1);
    }

    display_value(100, 300, Free_Index);
    break;

  case FreeText2:
    Free_Index = 0;
    sButtonData[FreeText1].state = 0;
    drawButton(FreeText1);

    if (!sButtonData[FreeText2].state)
    {
      Free_Index = 2;
      sButtonData[FreeText2].state = 1;
      drawButton(FreeText2);
    }
    else
    {
      sButtonData[FreeText2].state = 0;
      drawButton(FreeText2);
    }

    display_value(100, 300, Free_Index);
    break;
  }
}

void set_xmit_button(bool state)
{
  sButtonData[3].state = state;
  drawButton(3);
  delay(button_delay);
}

void init_RxSw_TxSw(void)
{

  pinMode(TxSw_PIN, OUTPUT);
  digitalWrite(TxSw_PIN, HIGH);
  delay(10);
  pinMode(RxSw_PIN, OUTPUT);
  digitalWrite(RxSw_PIN, LOW);
}

void Init_BoardVersionInput(void)
{
  pinMode(Board_PIN, INPUT_PULLUP);
}

void RLY_Select_20to40(void)
{
  digitalWrite(Relay_PIN, HIGH);
}

void RLY_Select_10to17(void)
{
  digitalWrite(Relay_PIN, LOW);
}

void Init_BandSwitchOutput(void)
{
  pinMode(Relay_PIN, OUTPUT);
  digitalWrite(Relay_PIN, LOW);
}

void Check_Board_Version(void)
{
  Band_Minimum = _20M;

  // GPIO Pin 6 is grounded for new model
  if (digitalRead(Board_PIN) == 0)
  {
    Init_BandSwitchOutput();
    Band_Minimum = _40M;
  }
}

void SelectFilterBlock(void)
{
  if (Band_Minimum == _40M)
  {
    if (BandIndex < _17M) // i.e. 40M, 30M or 20M
      RLY_Select_20to40();
    else
      RLY_Select_10to17();
  }
}

void transmit_sequence(void)
{

  digitalWrite(RxSw_PIN, HIGH);
  delay(10);
  digitalWrite(TxSw_PIN, LOW);
  set_xmit_button(true);
}

void receive_sequence(void)
{

  digitalWrite(TxSw_PIN, HIGH);
  delay(10);
  digitalWrite(RxSw_PIN, LOW);
  set_xmit_button(false);
}

void set_RF_Gain(int rfgain)
{

  float gain_setpoint;
  gain_setpoint = (float)rfgain / 32.0;
  amp1.gain(gain_setpoint);
}

void set_Attenuator_Gain(float att_gain){

      in_left_amp.gain(att_gain);
      in_right_amp.gain(att_gain);
  }



void terminate_transmit_armed(void)
{

  Transmit_Armned = 0;
  ft8_receive_sequence();
  delay(2);
  receive_sequence();
  sButtonData[3].state = false;
  drawButton(3);
}

int testButton(uint8_t index)
{

  if ((draw_x > sButtonData[index].x) && (draw_x < sButtonData[index].x + sButtonData[index].w) && (draw_y > sButtonData[index].y) && (draw_y <= sButtonData[index].y + sButtonData[index].h))
  {

    tft.drawCircle(draw_x, draw_y, 5, YELLOW);
    return 1;
  }
  else

    return 0;
}

int FT8_Touch(void)
{

  int y_test;

  if (draw_x < 320 && (draw_y > 100 && draw_y < 500))
  {
    y_test = draw_y - 100;

    FT_8_TouchIndex = y_test / 40;
    return 1;
  }

  else

    return 0;
}

int Xmit_message_Touch(void)
{

  int y_test;
  if ((draw_x > 400 && draw_x < 640) && (draw_y > 380 && draw_y < 550))
  {
    y_test = draw_y - 380;

    FT_8_MessageIndex = y_test / 40;

    return 1;
  }

  else
    return 0;
}

void check_WF_Touch(void)
{
  if (draw_x < 600 && draw_y < 90)
  {

    display_cursor_line = draw_x;
    cursor_line = display_cursor_line / 2;
    cursor_freq = (uint16_t)((float)(cursor_line + ft8_min_bin) * ft8_shift);
    display_value(870, 559, cursor_freq);
  }
}

void set_startup_freq(void)
{

  display_cursor_line = 224;
  cursor_line = display_cursor_line / 2;
  cursor_freq = (uint16_t)((float)(cursor_line + ft8_min_bin) * ft8_shift);
  display_value(870, 559, cursor_freq);
}

#define MAXTOUCHLIMIT 10
int touch_count;

void process_touch(void)
{

  if (touch_count <= MAXTOUCHLIMIT)
  {

    if (tft.touched())
    { // if touched(true) detach isr
      // at this point we need to fill the FT5206 registers...
      tft.updateTS(); // now we have the data inside library

      uint16_t coordinates[MAXTOUCHLIMIT][2]; // to hold coordinates
      tft.getTScoordinates(coordinates);      // done
      // now coordinates has the x,y of all touches
      for (uint8_t i = 0; i < tft.getTouches(); i++)
      {
        if (i == 0)
        {
          draw_x = SCREEN_WIDTH - coordinates[i][0];
          draw_y = SCREEN_HEIGHT - coordinates[i][1];
        }
      }
      tft.enableCapISR(); // rearm ISR if needed (touched(true))

      touch_count++;
    }
  }

  if (touch_count >= 10)
  {
    checkButton();
    FT8_Touch_Flag = FT8_Touch();
    FT8_Message_Touch = Xmit_message_Touch();
    check_WF_Touch();
    touch_count = 0;
  }
}

void setup_Cal_Display(void)
{

  clear_reply_message_box();
  // tft.fillRect(400, 100, 624, 439,BLACK );
  tft.fillRect(0, 100, 600, 439, BLACK);
  erase_CQ();

  sButtonData[11].Active = 3;
  sButtonData[12].Active = 3;
  sButtonData[13].Active = 3;
  sButtonData[14].Active = 1;
  sButtonData[15].Active = 3;
  sButtonData[16].Active = 3;
  sButtonData[17].Active = 3;
  sButtonData[18].Active = 3;
  sButtonData[19].Active = 3;
  sButtonData[20].Active = 3;
  sButtonData[21].Active = 3;
  sButtonData[22].Active = 3;

  for (int i = 11; i < 23; i++)
    drawButton(i);

  show_wide(90, 140, start_freq); // 790 - 700 = 90

  display_Free_Text();
}

void display_Free_Text(void)
{

  tft.setFontSize(2, true);
  tft.textColor(WHITE, BLACK);

  tft.setCursor(100, line4 + 20);
  tft.write(Free_Text1, 14);

  tft.setCursor(100, line5 + 20);
  tft.write(Free_Text2, 14);
}

void reset_buttons(int btn1, int btn2, int btn3, const char *button_text)
{
  sButtonData[btn1].state = 0;
  drawButton(btn1);
  sButtonData[btn2].state = 0;
  drawButton(btn2);
  sButtonData[btn3].state = 0;
  drawButton(btn3);
  sButtonData[4].text0 = (char *)button_text;
  drawButton(4);

  display_value(100, 300, CQ_Mode_Index);
}

void update_CQFree_button()
{
  sButtonData[CQFree].state = 0;
  drawButton(CQFree);
}

void erase_Cal_Display(void)
{
  clear_reply_message_box();
  tft.fillRect(0, 100, 600, 439, BLACK); // move tune to left hand pane
  erase_CQ();
  for (int i = 11; i < 23; i++)
    sButtonData[i].Active = 0;
}

void EEPROMWriteInt(int address, int value)
{
  uint16_t internal_value = 32768 + value;

  byte byte1 = internal_value >> 8;
  byte byte2 = internal_value & 0xFF;
  EEPROM.write(address, byte1);
  EEPROM.write(address + 1, byte2);
}

int EEPROMReadInt(int address)
{
  uint16_t byte1 = EEPROM.read(address);
  uint16_t byte2 = EEPROM.read(address + 1);
  uint16_t internal_value = (byte1 << 8 | byte2);

  return (int)internal_value - 32768;
}

void Set_Cursor_Frequency(void)
{
  cursor_freq = (uint16_t)((float)(cursor_line + ft8_min_bin) * ft8_shift);

  display_value(800, 300, cursor_freq);
}

const uint64_t F_boot = 11229600000ULL;

void start_Si5351(void)
{
  si5351.init(SI5351_CRYSTAL_LOAD_0PF, 26000000, 0);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_2MA);
  si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_2MA);
  si5351.set_freq(F_boot, SI5351_CLK1);
  delay(10);
  si5351.output_enable(SI5351_CLK1, 1);
  delay(20);
  set_Rcvr_Freq();
}
