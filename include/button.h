
#pragma once

struct ButtonStruct
{
  const char *text0;
  const char *text1;
  const char *blank;
  int Active;
  int Displayed;
  int state;
  const uint16_t x;
  const uint16_t y;
  const uint16_t w;
  const uint16_t h;

};

struct FreqStruct
{
  uint16_t Frequency;
  const char *display;

};

enum BandIndex
{
  _40M = 0,
  _30M = 1,
  _20M = 2,
  _17M = 3,
  _15M = 4,
  _12M = 5,
  _10M = 6,
  NumBands = 7
};

void drawButton(uint16_t i);
void checkButton(void);
void executeButton(uint16_t index);
void display_all_buttons(void);
int FT8_Touch(void);
int Xmit_message_Touch(void);
void transmit_sequence(void);
void receive_sequence(void);
void process_touch(void);
void check_WF_Touch(void);
void set_startup_freq(void);
void terminate_transmit_armed(void);
void EEPROMWriteInt(int address, int value);
int EEPROMReadInt(int address);
void set_xmit_button(bool state);
void init_RxSw_TxSw(void);
void setup_Cal_Display(void);
void erase_Cal_Display(void);
void executeCalibrationButton(uint16_t index);
void start_Si5351(void);
void set_RF_Gain(int rfgain);
void set_Attenuator_Gain(float att_gain);
void Set_Cursor_Frequency(void);
void Init_BoardVersionInput(void);
void RLY_Select_20to40(void);
void RLY_Select_10to17(void);
void Init_BandSwitchOutput(void);
void Check_Board_Version(void);
void SelectFilterBlock(void);
void reset_buttons(int btn1, int btn2, int btn3, const char *button_text);
void update_CQFree_button();
void display_Free_Text(void);
void sync_FT8(void);
int testButton(uint8_t index);

extern int QSO_xmit;
extern int Beacon_State;
extern int Beacon_On;
extern uint8_t RX_volume;
extern int RF_Gain;
extern uint16_t display_cursor_line;
extern int Auto_Sync;
extern struct ButtonStruct sButtonData[];
extern int FT_8_TouchIndex;
extern int FT8_Message_Touch;
extern int Map_Index;
extern uint16_t start_freq;
extern int Band_Minimum;
extern struct FreqStruct sBand_Data[];
extern int BandIndex;
extern int Free_Index;
extern int CQ_Mode_Index;
extern int QSO_Fix;
extern int FT8_Touch_Flag;







