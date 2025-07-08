/*
 * decode_ft8.h
 *
 *  Created on: Nov 2, 2019
 *      Author: user
 */

#ifndef DECODE_FT8_H_
#define DECODE_FT8_H_

#define MAX_MSG_LEN 40

int ft8_decode(void);

typedef enum _Sequence
{
    Seq_RSL = 0,
    Seq_Locator
} Sequence;

typedef struct
{
    char field1[14];
    char field2[14];
    char field3[7];
    int freq_hz;
    char decode_time[10];
    int sync_score;
    int snr;
    int received_snr;
    //int distance;
    //int bearing;
    //int map_distance;
    //int map_bearing;
    char target_locator[7];
    int slot;
    int RR73;
    Sequence sequence;
} Decode;

typedef struct
{
    char message[40];
    int text_color;
} display_message_details;

typedef struct
{
    int number_times_called;
    char call[14];
    char locator[7];
    int RSL;
    int received_RSL;
    int RR73;
    //int distance;
    //int map_distance;
    //int map_bearing;
    Sequence sequence;
} Calling_Station;

int Check_Calling_Stations(int num_decoded);
void display_details(int decoded_messages);
void display_messages(int decoded_messages);
void clear_display_details(void);
void display_selected_call(int index);

void process_selected_Station(int stations_decoded, int TouchIndex);
void set_QSO_Xmit_Freq(int freq);

void clear_decoded_messages(void);
void clear_log_stored_data(void);

int strindex(const char *s, const char *t);
int validate_locator(const char *QSO_locator);

#endif /* DECODE_FT8_H_ */
