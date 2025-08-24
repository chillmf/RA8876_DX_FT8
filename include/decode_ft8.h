/*
 * decode_ft8.h
 *
 *  Created on: Nov 2, 2019
 *      Author: user
 */

#ifndef DECODE_FT8_H_
#define DECODE_FT8_H_

int ft8_decode(void);

enum Sequence
{
    Seq_RSL = 0,
    Seq_Locator
};

struct Decode
{
    char field1[14];
    char call_to[14];
    char call_from[14];
    char locator[7];
    int freq_hz;
    char decode_time[10];
    int sync_score;
    int snr;
    int received_snr;
    char target_locator[7];
    int slot;
    Sequence sequence;
};

struct display_message_details
{
    char message[22];
    int text_color;
};

struct Calling_Station
{
    int number_times_called;
    char call[14];
    char locator[7];
    int RSL;
    int received_RSL;
    Sequence sequence;
};

typedef enum _MsgColor
{
    Black = 0,
    White,
    Red,
    Green,
    Blue,
    Yellow,
    LastColor
} MsgColor;

const uint32_t lcd_color_map[LastColor] = {
        0x0000, // BLACK
        0xffff,  // WHITE
        0xf800, // RED
        0x07e0, // GREEN
        0x001f, // BLUE
        0xffe0 // YELLOW
};





void process_selected_Station(int stations_decoded, int TouchIndex);

void display_line(     bool right,    int line,    MsgColor background,    MsgColor textcolor,    const char *text);
void display_messages(Decode new_decoded[], int decoded_messages);
void clear_rx_region(void);
void clear_qso_region(void);
void display_queued_message(const char* msg);
void display_txing_message(const char*msg);
void display_qso_state(const char *txt);
char *add_worked_qso(void);
bool display_worked_qsos(void);

int strindex(const char *s, const char *t);

extern struct Decode new_decoded[];
extern size_t kMax_message_length;
extern int was_txing;

#endif /* DECODE_FT8_H_ */
