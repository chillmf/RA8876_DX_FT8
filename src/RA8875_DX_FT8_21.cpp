

#include <SPI.h>
#include <TimeLib.h>
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include <si5351.h>
#include <RA8876_t3.h>

#include "arm_math.h"

#include "Arduino.h"
#include "Process_DSP.h"
#include "decode_ft8.h"
#include "WF_Table.h"
#include "display.h"
#include "button.h"
#include "traffic_manager.h"
#include "AudioStream.h"
#include "filters.h"
#include "constants.h"
#include "gen_ft8.h"
#include "options.h"
#include "ADIF.h"
#include "AudioSDRpreProcessor.h"
#include "main.h"
#include "Geodesy.h"
#include "PskInterface.h"
#include "autoseq_engine.h"
#include "ADIF.h"

#define SCREEN_WIDTH 1024
#define SCREEN_HEIGHT 600
#define RA8876_CS 10
#define RA8876_RESET 9
#define BACKLITE 6 // My copy of the display is set for external backlight control
#define CTP_INT 27 // Use an interrupt capable pin such as pin 2 (any pin on a Teensy)

#define MAXTOUCHLIMIT 10

// Used for skipping the TX slot
int was_txing = 0;
bool clr_pressed = false;
bool free_text = false;
bool tx_pressed = false;
int log_display_flag;

// Autoseq TX text buffer
static char autoseq_txbuf[MAX_MSG_LEN];
// Autoseq current QSO state text
static char autoseq_state_str[MAX_LINE_LEN];

static bool worked_qsos_in_display = false;

RA8876_t3 tft = RA8876_t3(RA8876_CS, RA8876_RESET);
Si5351 si5351;

AudioSDRpreProcessor preProcessor;
AudioInputI2S i2s1;            // xy=120,212
AudioEffectMultiply multiply2; // xy=285,414
AudioEffectMultiply multiply1; // xy=287,149
AudioSynthWaveformSine sine1;  // xy=289,244
AudioFilterFIR fir1;           // xy=471,153
AudioFilterFIR fir2;           // xy=472,413
AudioMixer4 mixer1;            // xy=666,173
AudioMixer4 mixer2;            // xy=675,406
AudioAmplifier amp1;           // xy=859,151
AudioOutputI2S i2s2;           // xy=868,258
AudioRecordQueue audioQueue;   // xy=1027,149

AudioAmplifier left_amp;
AudioAmplifier right_amp;

AudioAmplifier in_left_amp;
AudioAmplifier in_right_amp;

AudioConnection c11(i2s1, 0, in_left_amp, 0);
AudioConnection c12(i2s1, 1, in_right_amp, 0);

AudioConnection c13(in_left_amp, 0, preProcessor, 0);
AudioConnection c14(in_right_amp, 0, preProcessor, 1);

AudioConnection patchCord1(preProcessor, 0, multiply1, 0);
AudioConnection patchCord2(preProcessor, 1, multiply2, 1);
AudioConnection patchCord3(multiply2, fir2);
AudioConnection patchCord4(multiply1, fir1);
AudioConnection patchCord5(sine1, 0, multiply1, 1);
AudioConnection patchCord6(sine1, 0, multiply2, 0);
AudioConnection patchCord7(fir1, 0, mixer1, 0);
AudioConnection patchCord8(fir1, 0, mixer2, 0);
AudioConnection patchCord9(fir2, 0, mixer1, 1);
AudioConnection patchCord10(fir2, 0, mixer2, 1);

AudioConnection c3(mixer2, 0, amp1, 0);
AudioConnection patchCord13(amp1, audioQueue);

AudioConnection c4(mixer1, 0, left_amp, 0);
AudioConnection c5(mixer2, 0, right_amp, 0);

AudioConnection c6(left_amp, 0, i2s2, 0);
AudioConnection c7(right_amp, 0, i2s2, 1);

AudioControlSGTL5000 sgtl5000; // xy=404,516

q15_t __attribute__((aligned(4))) dsp_buffer[FFT_BASE_SIZE * 3];
q15_t __attribute__((aligned(4))) dsp_output[FFT_SIZE * 2];
q15_t __attribute__((aligned(4))) input_gulp[FFT_BASE_SIZE * 5];

char Station_Call[11];         // six character call sign + /0
char Station_Locator[7];       // up to six character locator  + /0
char Short_Station_Locator[5]; // four character locator  + /0

uint16_t currentFrequency;

uint32_t current_time, start_time, ft8_time;
int ft8_flag;
int FT_8_counter;
int ft8_marker;
int decode_flag;
int WF_counter;
int xmit_flag;
int ft8_xmit_counter;
int DSP_Flag;
int master_decoded;

uint16_t cursor_freq;
uint16_t cursor_line;
int Tune_On;

float xmit_level = 0.8;

int QSO_xmit;
int slot_state = 0;
int target_slot;

static void process_data();
static void update_synchronization();

// Helper function for updating TX region display
void tx_display_update(void)
{
  if (Tune_On || worked_qsos_in_display)
  {
    return;
  }
  if (xmit_flag)
  {
    display_txing_message(autoseq_txbuf);
  }
  else
  {
    display_queued_message(autoseq_txbuf);
  }

  autoseq_get_qso_state(autoseq_state_str);
  display_qso_state(autoseq_state_str);
}

void setup(void)
{
  Serial.begin(9600);

  if (CrashReport)
  {
    Serial.print(CrashReport);
  }

  init_RxSw_TxSw();

  setSyncProvider(getTeensy3Time);

  Serial.println("hello charley");

  delay(10);

  Wire.begin();
  Wire1.begin();
  pinMode(BACKLITE, OUTPUT);
  digitalWrite(BACKLITE, HIGH);

  tft.begin();
  tft.backlight(true);
  tft.useCapINT(CTP_INT); // we use the capacitive chip Interrupt out!
  tft.setTouchLimit(MAXTOUCHLIMIT);
  tft.enableCapISR(true); // capacitive touch screen interrupt it's armed
  tft.fillRect(0, 0, 1024, 600, BLACK);

  Init_BoardVersionInput();
  Check_Board_Version();
  Options_Initialize();

  start_Si5351();
  init_DSP();
  set_startup_freq();

  AudioMemory(100);
  preProcessor.startAutoI2SerrorDetection();

  RX_volume = 10;
  RF_Gain = 20;

  sgtl5000.enable();
  sgtl5000.inputSelect(AUDIO_INPUT_LINEIN);
  sgtl5000.lineInLevel(RX_volume);
  sgtl5000.lineOutLevel(31);
  sgtl5000.volume(0.4);

  sine1.amplitude(1.0);
  sine1.frequency(10000);
  sine1.phase(0);

  fir1.begin(FIR_I, NUM_COEFFS);
  fir2.begin(FIR_Q, NUM_COEFFS);

  mixer1.gain(0, 0.4);
  mixer1.gain(1, -0.4);

  mixer2.gain(0, 0.4);
  mixer2.gain(1, 0.4);

  set_RF_Gain(RF_Gain);
  set_Attenuator_Gain(1.0);

  left_amp.gain(0.2);
  right_amp.gain(0.2);

  audioQueue.begin();

  start_time = millis();

  open_stationData_file();

  set_Station_Coordinates();

  LatLong ll = QRAtoLatLong(Station_Locator);
  if (ll.isValid)
  {
    show_decimal(680, 60, ll.latitude);
    show_decimal(880, 60, ll.longitude);
  }

  display_all_buttons();
  display_date(650, 30);
  display_station_data(820, 0);

  display_revision_level();

  display_value(620, 559, RF_Gain);

  Init_Log_File();
  draw_map(Map_Index);

  autoseq_init(Station_Call, Station_Locator);
}

//charley is a dope without hope
void loop()
{
  const int offset_index = 8;
  if (!decode_flag)
    process_data();

  if (DSP_Flag)
  {
    process_FT8_FFT();

    if (xmit_flag)
    {
      if (!Tune_On)
      {
        if (ft8_xmit_counter >= offset_index && ft8_xmit_counter < 79 + offset_index)
        {
          set_FT8_Tone(tones[ft8_xmit_counter - offset_index]);
        }

        ft8_xmit_counter++;

        if (ft8_xmit_counter == 80 + offset_index)
        {
          xmit_flag = 0;
          terminate_transmit_armed();
        }
      }
    }

    display_time(880, 30);

    DSP_Flag = 0;
  }

  if (decode_flag && !Tune_On && !xmit_flag) // start of servicing FT_Decode
  {

    master_decoded = ft8_decode();

    display_messages(new_decoded, master_decoded);

    for (int i = 0; i < master_decoded; ++i)
    {
      if (strindex(new_decoded[i].call_to, Station_Call) >= 0)
      {
        char received_message[22];
        sprintf(received_message, "%s %s %s", new_decoded[i].call_to, new_decoded[i].call_from, new_decoded[i].locator);
        strcpy(current_message, received_message);
        update_message_log_display(0);
      }
    }

    if (!was_txing)
    {
      for (int i = 0; i < master_decoded; i++)
      {
        // TX is (potentially) necessary
        if (autoseq_on_decode(&new_decoded[i]))
        {
          // Fetch TX msg
          if (autoseq_get_next_tx(autoseq_txbuf))
          {
            queue_custom_text(autoseq_txbuf);
            QSO_xmit = 1;
            tx_display_update();
            break;
          }
        }
      }

      // No valid response has received to advance auto sequencing.
      // Check TX retry is needed?
      // Yes => QSO_xmit = True;
      // No  => check in beacon mode?
      //       Yes => start_cq, QSO_xmit = True;
      //       No  => QSO_xmit = False;
      if (!QSO_xmit)
      {
        // Check if retry is necessary
        if (autoseq_get_next_tx(autoseq_txbuf))
        {
          queue_custom_text(autoseq_txbuf);
          QSO_xmit = 1;
        }
        else if (Beacon_On)
        {
          target_slot = slot_state ^ 1; // toggle the slot
          autoseq_start_cq();
          autoseq_get_next_tx(autoseq_txbuf);
          queue_custom_text(autoseq_txbuf);
          QSO_xmit = 1;
          tx_display_update();
        }
      }
    }

    decode_flag = 0;

  } // end of sevicing FT_Decode

  process_touch();

  if (clr_pressed)
  {
    terminate_QSO();
    QSO_xmit = 0;
    was_txing = 0;
    autoseq_init(Station_Call, Station_Locator);
    autoseq_txbuf[0] = '\0';
    tx_display_update();
    clr_pressed = false;
  }
  
  if (tx_pressed)
  {
    worked_qsos_in_display = display_worked_qsos();
    tx_pressed = false;
    tx_display_update();
  }

  if (!Tune_On && log_display_flag == 1)
  {
    display_logged_messages();
    log_display_flag = 0;
  }

  if (!Tune_On && FT8_Touch_Flag && FT_8_TouchIndex < master_decoded)
  {
    process_selected_Station(master_decoded, FT_8_TouchIndex);
    autoseq_on_touch(&new_decoded[FT_8_TouchIndex]);
    autoseq_get_next_tx(autoseq_txbuf);
    queue_custom_text(autoseq_txbuf);
    QSO_xmit = 1;
    FT8_Touch_Flag = 0;
    tx_display_update();
  }

  update_synchronization();
  getTime();
}

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

static void process_data()
{
  if (audioQueue.available() >= num_que_blocks)
  {
    for (int i = 0; i < num_que_blocks; i++)
    {
      memcpy(input_gulp + block_size * i,
             audioQueue.readBuffer(),
             AUDIO_BLOCK_SAMPLES * sizeof(uint16_t));
      audioQueue.freeBuffer();
    }

    const size_t length = FFT_SIZE;
    memmove(dsp_buffer,
            dsp_buffer + FFT_BASE_SIZE,
            length * sizeof(q15_t));
    for (int i = 0; i < FFT_BASE_SIZE; i++)
    {
      dsp_buffer[length + i] = input_gulp[i * 5]; // decimation by 5
    }

    DSP_Flag = 1;
  }
}

void update_synchronization()
{
  uint32_t current_time = millis();
  ;
  ft8_time = current_time - start_time;

  // Update slot and reset RX
  int current_slot = ft8_time / 15000 % 2;
  if (current_slot != slot_state)
  {
    // toggle the slot state
    slot_state ^= 1; 
    if (was_txing)
    {
      autoseq_tick();
    }
    was_txing = 0;

    ft8_flag = 1;
    FT_8_counter = 0;
    ft8_marker = 1;
    WF_counter = 0;
    tx_display_update();
  }

  // Check if TX is intended
  if (QSO_xmit && target_slot == slot_state && FT_8_counter < 29)
  {
    setup_to_transmit_on_next_DSP_Flag(); // TODO: move to main.c
    QSO_xmit = 0;
    was_txing = 1;
    // Partial TX, set the TX counter based on current ft8_time
    ft8_xmit_counter = (ft8_time % 15000) / 160; // 160ms per symbol

    // Log the TX
    if (strindex(autoseq_txbuf, "CQ") < 0)
    {
      strcpy(current_message, autoseq_txbuf);
      update_message_log_display(1);
    }

    tx_display_update();
  }
}

void sync_FT8(void)
{
  setSyncProvider(getTeensy3Time);
  Teensy3Clock.set(now()); // set the RTC
  start_time = millis();
  ft8_flag = 1;
  FT_8_counter = 0;
  ft8_marker = 1;
  WF_counter = 0;
}
