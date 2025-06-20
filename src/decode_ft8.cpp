/*
 * decode_ft8.c
 *
 *  Created on: Sep 16, 2019
 *      Author: user
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <ctype.h>
#include "gen_ft8.h"
#include "unpack.h"
#include "ldpc.h"
#include "decode.h"
#include "constants.h"
#include "encode.h"
#include <TimeLib.h>
#include "Process_DSP.h"
#include "display.h"
#include "decode_ft8.h"
#include <RA8876_t3.h>
#include "ADIF.h"
#include "button.h"

extern RA8876_t3 tft;

extern int left_hand_message;

char blank[] = "                          ";
int blank_length = 26;

const int kLDPC_iterations = 20;
const int kMax_candidates = 20;
const int kMax_decoded_messages = 20;
size_t kMax_message_length = 20;
const int kMin_score = 40; // Minimum sync score threshold for candidates

int strindex(const char *s, const char *t);

extern uint8_t export_fft_power[];

extern int ND;
extern int NS;
extern int NN;
// Define the LDPC sizes
extern int N;
extern int K;
extern int M;
extern int K_BYTES;

display_message_details display[10];

Decode new_decoded[20];

Calling_Station Answer_CQ[100];
int log_size = 50;

static int num_calls; // number of unique calling stations

int message_limit = 10;

int Auto_QSO_State; // chh

extern int Beacon_On;
extern int Station_RSL;
extern char Target_Locator[]; // four character locator  + /0
extern char Station_Call[];

extern int QSO_Fix;
extern int slot_state;
extern int target_slot;
extern int target_freq;
extern int RSL_sent;
extern int RR73_sent;
extern uint16_t cursor_freq;

extern char Target_Call[];
extern int Target_RSL; // four character RSL  + /0

extern uint16_t cursor_line;
extern uint16_t display_cursor_line;

extern time_t getTeensy3Time();

extern int FT8_Touch_Flag;

extern char current_message[];

int ft8_decode(void)
{

  // Find top candidates by Costas sync score and localize them in time and frequency
  Candidate candidate_list[kMax_candidates];

  int num_candidates = find_sync(export_fft_power, ft8_msg_samples, ft8_buffer, kCostas_map, kMax_candidates, candidate_list, kMin_score);
  char decoded[kMax_decoded_messages][kMax_message_length];

  const float fsk_dev = 6.25f; // tone deviation in Hz and symbol rate

  // Go over candidates and attempt to decode messages
  int num_decoded = 0;

  for (int idx = 0; idx < num_candidates; ++idx)
  {
    Candidate cand = candidate_list[idx];
    float freq_hz = (cand.freq_offset + cand.freq_sub / 2.0f) * fsk_dev;

    float log174[N];
    extract_likelihood(export_fft_power, ft8_buffer, cand, kGray_map, log174);

    // bp_decode() produces better decodes, uses way less memory
    uint8_t plain[N];
    int n_errors = 0;
    bp_decode(log174, kLDPC_iterations, plain, &n_errors);

    if (n_errors > 0)
      continue;

    // Extract payload + CRC (first K bits)
    uint8_t a91[K_BYTES];
    pack_bits(plain, K, a91);

    // Extract CRC and check it
    uint16_t chksum = ((a91[9] & 0x07) << 11) | (a91[10] << 3) | (a91[11] >> 5);
    a91[9] &= 0xF8;
    a91[10] = 0;
    a91[11] = 0;
    uint16_t chksum2 = crc(a91, 96 - 14);
    if (chksum != chksum2)
      continue;

    char message[kMax_message_length];

    char field1[14];
    char field2[14];
    char field3[7];
    int rc = unpack77_fields(a91, field1, field2, field3);
    if (rc < 0)
      continue;

    sprintf(message, "%s %s %s ", field1, field2, field3);

    // Check for duplicate messages (TODO: use hashing)
    bool found = false;
    for (int i = 0; i < num_decoded; ++i)
    {
      if (0 == strcmp(decoded[i], message))
      {
        found = true;
        break;
      }
    }

    int raw_RSL;
    int display_RSL;
    int received_RSL;

    getTeensy3Time();
    char rtc_string[10]; // print format stuff
    sprintf(rtc_string, "%2i%2i%2i", hour(), minute(), second());
    for (int j = 0; j < 9; j++)
      if (rtc_string[j] == 32)
        rtc_string[j] = 48;

    if (!found && num_decoded < kMax_decoded_messages)
    {
      if (strlen(message) < kMax_message_length)
      {
        strcpy(decoded[num_decoded], message);

        new_decoded[num_decoded].sync_score = cand.score;
        new_decoded[num_decoded].freq_hz = (int)freq_hz;
        strcpy(new_decoded[num_decoded].field1, field1);
        strcpy(new_decoded[num_decoded].field2, field2);
        strcpy(new_decoded[num_decoded].field3, field3);
        strcpy(new_decoded[num_decoded].decode_time, rtc_string);

        new_decoded[num_decoded].slot = slot_state;

        raw_RSL = (float)cand.score;
        display_RSL = (int)((raw_RSL - 160)) / 6;
        new_decoded[num_decoded].snr = display_RSL;

        char Test_Locator[] = "    ";
        char FT8_Message[] = "    ";

        strcpy(Test_Locator, new_decoded[num_decoded].field3);

        if (validate_locator(Test_Locator) == 1)
        {
          strcpy(new_decoded[num_decoded].target_locator, Test_Locator);
          new_decoded[num_decoded].sequence = Seq_Locator;
        }

        strcpy(FT8_Message, new_decoded[num_decoded].field3);

        if (strindex(FT8_Message, "73") >= 0 || strindex(FT8_Message, "RR73") >= 0 || strindex(FT8_Message, "RRR") >= 0)
        {
          new_decoded[num_decoded].RR73 = 1;
        }
        else
        {
          if (FT8_Message[0] == 82)
          {
            FT8_Message[0] = 32;
            new_decoded[num_decoded].RR73 = 2; // chh_traffuc
            new_decoded[num_decoded].sequence = Seq_Locator;
          }

          received_RSL = atoi(FT8_Message);
          if (received_RSL < 30) // Prevents an 73 being decoded as a received RSL
          {
            new_decoded[num_decoded].received_snr = received_RSL;
          }
        }

        ++num_decoded;
      }
    }
  } // End of big decode loop

  return num_decoded;
}

void display_messages(int decoded_messages)
{
  char CQ[] = "CQ";
  char blank[] = "                  ";

  tft.fillRect(0, 100, 300, 400, BLACK);

  for (int i = 0; i < decoded_messages && i < message_limit; i++)
  {

    strcpy(display[i].message, blank);

    if (strcmp(CQ, new_decoded[i].field1) == 0)
    {

      if (strcmp(Station_Call, new_decoded[i].field2) != 0)
      {
        sprintf(display[i].message, "%s %s %s %2i", new_decoded[i].field1, new_decoded[i].field2, new_decoded[i].field3, new_decoded[i].snr);
      }
      else
        sprintf(display[i].message, "%s %s %s", new_decoded[i].field1, new_decoded[i].field2, new_decoded[i].field3);
      display[i].text_color = 1;
    }

    else
    {
      sprintf(display[i].message, "%s %s %s", new_decoded[i].field1, new_decoded[i].field2, new_decoded[i].field3);
      display[i].text_color = 0;
    }

    tft.setFontSize(2, true);

    for (int j = 0; j < decoded_messages && j < message_limit; j++)
    {
      if (display[j].text_color == 0)
        tft.textColor(WHITE, BLACK);
      else
        tft.textColor(GREEN, BLACK);
      tft.setCursor(0, 100 + j * 40);
      tft.write(display[j].message, 18);
    }
  }
}

int validate_locator(const char *QSO_locator)
{
  uint8_t A1, A2, N1, N2;
  uint8_t test = 0;

  A1 = QSO_locator[0] - 65;
  A2 = QSO_locator[1] - 65;
  N1 = QSO_locator[2] - 48;
  N2 = QSO_locator[3] - 48;

  if (A1 >= 0 && A1 <= 17)
    test++;
  if (A2 > 0 && A2 < 17)
    test++; // block RR73 Artic and Anartica

  if (N1 >= 0 && N1 <= 9)
    test++;
  if (N2 >= 0 && N2 <= 9)
    test++;

  if (test == 4)
    return 1;
  return 0;
}

int strindex(const char *s, const char *t)
{
  int i, j, k, result;

  result = -1;

  for (i = 0; s[i] != '\0'; i++)
  {
    for (j = i, k = 0; t[k] != '\0' && s[j] == t[k]; j++, k++)
      ;
    if (k > 0 && t[k] == '\0')
      result = i;
  }
  return result;
}

void clear_log_stored_data(void)
{
  const char call_blank[] = "       ";
  const char locator_blank[] = "    ";

  for (int i = 0; i < log_size; i++)
  {
    Answer_CQ[i].number_times_called = 0;
    strcpy(Answer_CQ[i].call, call_blank);
    strcpy(Answer_CQ[i].locator, locator_blank);
    Answer_CQ[i].RSL = 0;
    Answer_CQ[i].RR73 = 0;
    Answer_CQ[i].received_RSL = 99;
    Answer_CQ[i].sequence = Seq_RSL;
  }
}

void clear_decoded_messages(void)
{
  const char call_blank[] = "       ";
  const char locator_blank[] = "    ";

  for (int i = 0; i < kMax_decoded_messages; i++)
  {
    strcpy(new_decoded[i].field1, call_blank);
    strcpy(new_decoded[i].field2, call_blank);
    strcpy(new_decoded[i].field3, locator_blank);
    strcpy(new_decoded[i].target_locator, locator_blank);
    new_decoded[i].freq_hz = 0;
    new_decoded[i].sync_score = 0;
    new_decoded[i].received_snr = 99;
    new_decoded[i].slot = 0;
    new_decoded[i].RR73 = 0;
    new_decoded[i].sequence = Seq_RSL;
  }
}

char field1[14];
char field2[14];
char field3[7];

int Check_Calling_Stations(int num_decoded)
{
  int Beacon_Reply_Status = 0;
  char received_message[40];

  for (int i = 0; i < num_decoded; i++)
  { // check to see if being called
    int old_call;
    int old_call_address = 0;

    if (strindex(new_decoded[i].field1, Station_Call) >= 0)
    {
      old_call = 0;

      for (int j = 0; j < num_calls; j++)
      {
        if (strcmp(Answer_CQ[j].call, new_decoded[i].field2) == 0)
        {
          old_call = Answer_CQ[j].number_times_called;
          old_call++;
          Answer_CQ[j].number_times_called = old_call;
          old_call_address = j;
        }
      }

      if (old_call == 0)
      {
        sprintf(received_message, "%s %s %s", new_decoded[i].field1, new_decoded[i].field2, new_decoded[i].field3);
        strcpy(current_message, received_message);

        update_message_log_display(0);

        strcpy(Target_Call, new_decoded[i].field2);

        if (Beacon_On == 1)
          {Target_RSL = new_decoded[i].snr;
          strcpy(Target_Locator, new_decoded[i].target_locator);
          }

        if (new_decoded[i].received_snr != 99)
          Station_RSL = new_decoded[i].received_snr;

        if (Beacon_On == 1) // migraion
        {
          if (new_decoded[i].sequence == Seq_Locator)
            set_reply(Reply_RSL);
          else
            set_reply(Reply_R_RSL);
        }

        Beacon_Reply_Status = 1;

        strcpy(Answer_CQ[num_calls].call, new_decoded[i].field2);
        strcpy(Answer_CQ[num_calls].locator, new_decoded[i].target_locator);
        Answer_CQ[num_calls].RSL = Target_RSL;
        Answer_CQ[num_calls].received_RSL = Station_RSL;
        Answer_CQ[num_calls].sequence = new_decoded[i].sequence;

        num_calls++;
        break;
      }

      if (old_call >= 1 && old_call < 5)
      {

        sprintf(received_message, "%s %s %s", new_decoded[i].field1, new_decoded[i].field2, new_decoded[i].field3);
        strcpy(current_message, received_message);
        update_message_log_display(0);

        if (new_decoded[i].RR73 == 1)
          RR73_sent = 1;

       
        strcpy(Target_Call, Answer_CQ[old_call_address].call);
       // strcpy(Target_Locator, Answer_CQ[old_call_address].locator);
        Target_RSL = Answer_CQ[old_call_address].RSL;
        

        if (new_decoded[i].received_snr != 99)
          Answer_CQ[old_call_address].received_RSL = new_decoded[i].received_snr;

        Station_RSL = Answer_CQ[old_call_address].received_RSL;

        if (Answer_CQ[old_call_address].RR73 == 0)
        {
          if (Beacon_On == 1)
          {
            // if (new_decoded[i].RR73 == 1)  //chh_ft8_traffic
            strcpy(Target_Locator, Answer_CQ[old_call_address].locator);
            if (new_decoded[i].RR73 > 0)
            {
              if (Answer_CQ[old_call_address].sequence == Seq_Locator)
                // if this is a  locator response send Beacon 73
                set_reply(Reply_Beacon_73);
              else
                // if this is a RSL response send QSO 73
                set_reply(Reply_QSO_73);

              if (new_decoded[i].RR73 == 1)
              {
                Answer_CQ[old_call_address].RR73 = 1;
                write_ADIF_Log();
              }
            }
            else
            {
              if (Answer_CQ[old_call_address].sequence == Seq_Locator)
                // if this is a  locator response send RSL
                set_reply(Reply_RSL);
              else
                // if this is a RSL response send R_RSL
                set_reply(Reply_R_RSL);
            }
          }

          Beacon_Reply_Status = 1;
        } // Check for RR73 =1
      } // check for old call
    } // check for station call
  } // check to see if being called

  return Beacon_Reply_Status;
}

void process_selected_Station(int stations_decoded, int TouchIndex)
{
  if (stations_decoded > 0 && TouchIndex <= stations_decoded)
  {
    strcpy(Target_Call, new_decoded[TouchIndex].field2);
    strcpy(Target_Locator, new_decoded[TouchIndex].target_locator);
    Target_RSL = new_decoded[TouchIndex].snr;
    target_slot = new_decoded[TouchIndex].slot;
    target_freq = new_decoded[TouchIndex].freq_hz;

    if (QSO_Fix == 1)
      set_QSO_Xmit_Freq(target_freq);

    compose_messages();
    Auto_QSO_State = 1;
    RSL_sent = 0;
    RR73_sent = 0;
  }

  FT8_Touch_Flag = 0;
}

void set_QSO_Xmit_Freq(int freq)
{
  float cursor_value;

  cursor_freq = freq;
  display_value(870, 559, cursor_freq);

  cursor_value = (float)freq / FFT_Resolution;
  cursor_line = (uint16_t)(cursor_value - ft8_min_bin);
  display_cursor_line = 2 * cursor_line;
}
