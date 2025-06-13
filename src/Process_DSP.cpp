#include "RA8876_t3.h"
extern RA8876_t3 tft;

#include "Process_DSP.h"
#include "WF_Table.h"
#include "arm_math.h"
#include "decode_ft8.h"
#include "display.h"
#include <TimeLib.h>
#include "traffic_manager.h"
#include "button.h"

extern q15_t __attribute__((aligned(4))) dsp_buffer[];
extern q15_t __attribute__((aligned(4))) dsp_output[];
static q15_t __attribute__((aligned(4))) window_dsp_buffer[FFT_SIZE];

uint8_t WF_index[900];
float window[FFT_SIZE];

extern uint16_t cursor_line;
extern uint16_t display_cursor_line;

extern int ft8_flag, FT_8_counter, ft8_marker, decode_flag, WF_counter;
extern int num_decoded_msg;

int master_offset, offset_step;
extern int CQ_Flag;
extern int QSO_Flag;
extern int Transmit_Armned;
extern int Auto_Sync;
extern void sync_FT8(void);
extern ButtonStruct sButtonData[];
extern int xmit_flag;

int offset_index;

q15_t FFT_Scale[FFT_SIZE * 2];
q15_t FFT_Magnitude[FFT_SIZE];
int32_t FFT_Mag_10[FFT_SIZE / 2];
uint8_t FFT_Buffer[FFT_SIZE / 2];
float mag_db[FFT_SIZE / 2 + 1];

arm_rfft_instance_q15 fft_inst;
arm_cfft_radix4_instance_q15 aux_inst;

uint8_t export_fft_power[ft8_msg_samples * ft8_buffer * 4];

void init_DSP(void)
{
  // arm_rfft_init_q15(&fft_inst, &aux_inst, FFT_SIZE, 0, 1);
  arm_rfft_init_q15(&fft_inst, FFT_SIZE, 0, 1);
  for (int i = 0; i < FFT_SIZE; ++i)
    window[i] = ft_blackman_i(i, FFT_SIZE);
  offset_step = (int)ft8_buffer * 4;
  offset_index = 8;
}

int max_bin, max_bin_number;

float ft_blackman_i(int i, int N)
{
  const float alpha = 0.16f; // or 2860/18608
  const float a0 = (1 - alpha) / 2;
  const float a1 = 1.0f / 2;
  const float a2 = alpha / 2;

  float x1 = cosf(2 * (float)M_PI * i / (N - 1));
  // float x2 = cosf(4 * (float)M_PI * i / (N - 1));
  float x2 = 2 * x1 * x1 - 1; // Use double angle formula
  return a0 - a1 * x1 + a2 * x2;
}

// Compute FFT magnitudes (log power) for each timeslot in the signal
void extract_power(int offset)
{
  // Loop over two possible time offsets (0 and block_size/2)
  for (int time_sub = 0; time_sub <= input_gulp_size / 2; time_sub += input_gulp_size / 2)
  {
    for (int i = 0; i < FFT_SIZE; i++)
      window_dsp_buffer[i] = (q15_t)((float)dsp_buffer[i + time_sub] * window[i]);

    arm_rfft_q15(&fft_inst, window_dsp_buffer, dsp_output);
    arm_shift_q15(&dsp_output[0], 5, &FFT_Scale[0], FFT_SIZE * 2);
    // arm_shift_q15(&dsp_output[0], 6, &FFT_Scale[0], FFT_SIZE * 2 );
    arm_cmplx_mag_squared_q15(&FFT_Scale[0], &FFT_Magnitude[0], FFT_SIZE);

    for (int j = 0; j < FFT_SIZE / 2; j++)
    {
      FFT_Mag_10[j] = 10 * (int32_t)FFT_Magnitude[j];
      mag_db[j] = 5.0 * log((float)FFT_Mag_10[j] + 0.1);
      // mag_db[j] =  10.0 * log((float)FFT_Mag_10[j] + 0.1);
    }

    // Loop over two possible frequency bin offsets (for averaging)
    for (int freq_sub = 0; freq_sub < 2; ++freq_sub)
    {
      for (int j = 0; j < ft8_buffer; ++j)
      {
        float db1 = mag_db[j * 2 + freq_sub];
        float db2 = mag_db[j * 2 + freq_sub + 1];
        float db = (db1 + db2) / 2;

        int scaled = (int)(db);
        export_fft_power[offset] = (scaled < 0) ? 0 : ((scaled > 255) ? 255 : scaled);
        ++offset;
      }
    }
  }
}

void process_FT8_FFT(void)
{
  if (ft8_flag == 1)
  {
    master_offset = offset_step * FT_8_counter;
    extract_power(master_offset);

    update_offset_waterfall(master_offset);

    FT_8_counter++;
    if (FT_8_counter == ft8_msg_samples)
    {
      ft8_flag = 0;
      decode_flag = 1;
    }
  }
}

const int max_noise_count = 3;
const int max_noise_free_sets_count = 3;
static int noise_free_sets_count = 0;

void update_offset_waterfall(int offset)
{
  for (int j = ft8_min_bin; j < ft8_buffer; j++)
    FFT_Buffer[j] = export_fft_power[j + offset];

  int bar;
  for (int x = ft8_min_bin; x < ft8_buffer; x++)
  {
    bar = FFT_Buffer[x];
    if (bar > 63)
      bar = 63;

    WF_index[x] = bar / 4;
  }

  if (Auto_Sync)
  {
    // count the amount of noise seen in the FFT
    int noise_count = 0;

    for (int x = ft8_min_bin; x < ft8_buffer; x++)
    {
      if ((FFT_Buffer[x] > 0) && (++noise_count >= max_noise_count))
        break;
    }

    // if less than the maximum noise count in the FFT
    if (noise_count < max_noise_count)
    {
      // if sufficient noise free FFT data is seen it's time to synchronise
      if (++noise_free_sets_count >= max_noise_free_sets_count)
      {
        sync_FT8();
        Auto_Sync = 0;
        noise_free_sets_count = 0;
        sButtonData[6].state = 0;
        drawButton(6);
      }
    }
    else
    {
      noise_free_sets_count = 0;
    }
  }

  /*
  const size_t rowLength = 2 * (ft8_buffer - ft8_min_bin);
  // two pixels for each datum point
  uint16_t rowBuffer[rowLength * 2] = {0};

  for (int k = ft8_min_bin; k < ft8_buffer; k++)
  {
    int x = 2 * (k - ft8_min_bin);
    uint16_t color;
    if (xmit_flag == 0)
    {
      color = WFPalette[WF_index[k]];
    }
    else
    {
      if (x >= display_cursor_line && x <= display_cursor_line + 16)
        color = WFPalette[WF_index[k]];
      else
        color = BLACK;
    }

    // two pixels for each datum point
    int row = x * 2;
    rowBuffer[row + 1] = rowBuffer[row] = color;
  }

  // Draw the cursor line in red
  if (display_cursor_line >= 0 && display_cursor_line < rowLength)
  {
    // two pixels for each datum point
    int row = display_cursor_line * 2;
    rowBuffer[row] = rowBuffer[row + 1] = RED;
  }

  tft.writeRect(0, WF_counter, rowLength, 1, rowBuffer);
  */

for (int k = ft8_min_bin; k< ft8_buffer; k++) {
        
      if(xmit_flag == 0 )
      tft.drawPixel(2*(k-ft8_min_bin), WF_counter, WFPalette[WF_index[k]]);  
      else {
      if( 2*(k-ft8_min_bin) >= display_cursor_line && 2*(k-ft8_min_bin) <= display_cursor_line + 16) tft.drawPixel(2*(k-ft8_min_bin), WF_counter, WFPalette[WF_index[k]]);
      else
      tft.drawPixel(2*(k-ft8_min_bin), WF_counter, BLACK);   
      }
    
       tft.drawPixel(display_cursor_line, WF_counter,RED);
      
      }



  WF_counter++;
}
