/*
 * autoseq_engine.c – FT8 CQ/QSO auto‑sequencing engine (revised)
 *
 * This version consumes the **Decode** structure defined in decode_ft8.h, the
 * native output of *ft8_decode()* in the DX‑FT8 transceiver firmware, so no
 * wrapper conversion is required.  All logic remains the same as the QEX/WSJT‑X
 * state diagram (Tx1…Tx6) presented earlier.
 *
 */

#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "autoseq_engine.h"
#include "gen_ft8.h" // For accessing CQ_Mode_Index and saving Target_*
#include "ADIF.h"    // For write_ADIF_Log()
#include "button.h"  // For BandIndex

/***** Compile‑time knobs *****/
#define MAX_TX_RETRY 2

/***** Identifiers for the six canonical FT8 messages *****/
typedef enum
{
    TX_UNDEF = 0,
    TX1, /*  <DXCALL> <MYCALL> <GRID>            */
    TX2, /*  <DXCALL> <MYCALL> ##                */
    TX3, /*  <DXCALL> <MYCALL> R##               */
    TX4, /*  <DXCALL> <MYCALL> RR73 (or RRR)     */
    TX5, /*  <DXCALL> <MYCALL> 73                */
    TX6  /*  CQ <MYCALL> <GRID>                  */
} tx_msg_t;

/***** High‑level auto‑sequencer states *****/
typedef enum
{
    AS_IDLE = 0,
    AS_REPLYING,
    AS_REPORT,
    AS_ROGER_REPORT,
    AS_ROGERS,
    AS_SIGNOFF,
    AS_CALLING,
} autoseq_state_t;

/***** Control‑block *****/
typedef struct
{
    autoseq_state_t state;
    tx_msg_t next_tx;
    tx_msg_t rcvd_msg_type;

    char mycall[14];
    char mygrid[7];
    char dxcall[14];
    char dxgrid[7];
    int snr_tx; /* SNR we report to DX (‑dB) */
    int retry_counter;
    int retry_limit;
    bool logged; /* true => QSO logged */

} autoseq_ctx_t;

static autoseq_ctx_t ctx;

/*************** Forward declarations ****************/
static void set_state(autoseq_state_t s, tx_msg_t first_tx, int limit);
static void format_tx_text(tx_msg_t id, char out[MAX_MSG_LEN]);
static void parse_rcvd_msg(const Decode *msg);
// Internal helper called by autoseq_on_touch() and autoseq_on_decode()
static bool generate_response(const Decode *msg, bool override);
static void write_worked_qso();
/******************************************************/

/* ====================================================
 *  Public API implementation
 * ==================================================== */

void autoseq_init(const char *myCall, const char *myGrid)
{
    memset(&ctx, 0, sizeof(ctx));
    strncpy(ctx.mycall, myCall, sizeof(ctx.mycall) - 1);
    strncpy(ctx.mygrid, myGrid, sizeof(ctx.mygrid) - 1);
    ctx.state = AS_IDLE;
}

static const char CQ[3] = "CQ";

void autoseq_start_cq(void)
{
    memcpy(ctx.dxcall, CQ, sizeof(CQ));
    set_state(AS_CALLING, TX6, 0); /* infinite CQ loop */
}

/* === Called for selected decode (manual response) === */
void autoseq_on_touch(const Decode *msg)
{
    if (!msg)
        return;
    
    parse_rcvd_msg(msg);
    if (strncmp(msg->call_to, ctx.mycall, 14) != 0)
    {
        // Not addresses me, treat it as if it's a CQ/TX6
        ctx.rcvd_msg_type = TX6;
    }
    else
    {
        // Addressed me
        generate_response(msg, true);
        return;
    }

    // Must be handling TX6
    strncpy(ctx.dxcall, msg->call_from, sizeof(ctx.dxcall) - 1);
    strncpy(ctx.dxgrid, msg->locator, sizeof(ctx.dxgrid) - 1);
    ctx.snr_tx = msg->snr;
    set_state(Skip_Tx1 ? AS_REPORT : AS_REPLYING,
              Skip_Tx1 ? TX2 : TX1, MAX_TX_RETRY);
}

/* === Called for **every** new decode (auto response) === */
/* Return whether TX is needed */
bool autoseq_on_decode(const Decode *msg)
{
    if (!msg)
        return false;

    // Not addresses me, return false
    if (strncmp(msg->call_to, ctx.mycall, 14) != 0)
    {
        return false;
    }

    // If the current QSO is still in progress, we ignore different dxcall
    if (ctx.state == AS_REPLYING || ctx.state == AS_REPORT || ctx.state == AS_ROGER_REPORT)
    {
        if (strncmp(msg->call_from, ctx.dxcall, sizeof(ctx.dxcall)) != 0)
        {
            return false;
        }
    }

    parse_rcvd_msg(msg);

    return generate_response(msg, false);
}

/* === Provide the message we should transmit this slot (if any) === */
bool autoseq_get_next_tx(char out_text[MAX_MSG_LEN])
{
    format_tx_text(ctx.next_tx, out_text);

    if (ctx.next_tx == TX_UNDEF)
        return false;

    /* Bump retry counter */
    if (ctx.retry_limit)
    {
        if (ctx.retry_counter >= ctx.retry_limit)
        {
            ctx.state = AS_SIGNOFF; /* give up */
        }
    }
    return true;
}

/* === Populate the string for displaying the current QSO state  === */
void autoseq_get_qso_state(char out_text[MAX_LINE_LEN])
{
    if (!out_text)
    {
        return;
    }

    out_text[0] = '\0';
    // IDEL state is treated as no active QSO
    if (ctx.state == AS_IDLE)
    {
        return;
    }

    const char states[][5] = {
        "",     // AS_IDLE
        "RPLY", // AS_REPLYING
        "RPRT", // AS_REPORT
        "RRPT", // AS_ROGER_REPORT
        "RGRS", // AS_ROGERS
        "SOFF", // AS_SIGNOFF
        "CALL", // AS_CALLING
    };

    snprintf(out_text, MAX_LINE_LEN,
             " %.4s tried:%1u",
             states[ctx.state],
             ctx.retry_counter
    );
}

/* === Slot timer / time‑out manager === */
void autoseq_tick(void)
{
    switch (ctx.state)
    {
    case AS_REPLYING:
        if (ctx.retry_counter < ctx.retry_limit)
        {
            ctx.next_tx = TX1;
            ctx.retry_counter++;
        }
        else
        {
            ctx.state = AS_SIGNOFF;
            ctx.next_tx = TX5;
        }
        break;

    case AS_REPORT:
        if (ctx.retry_counter < ctx.retry_limit)
        {
            ctx.next_tx = TX2;
            ctx.retry_counter++;
        }
        else
        {
            ctx.state = AS_SIGNOFF;
            ctx.next_tx = TX5;
        }
        break;

    case AS_ROGER_REPORT:
        if (ctx.retry_counter < ctx.retry_limit)
        {
            ctx.next_tx = TX3;
            ctx.retry_counter++;
        }
        else
        {
            ctx.state = AS_SIGNOFF;
            ctx.next_tx = TX5;
        }
        break;

    case AS_ROGERS:
        if (ctx.retry_counter < ctx.retry_limit)
        {
            ctx.next_tx = TX4;
            ctx.retry_counter++;
        }
        else
        {
            ctx.state = AS_IDLE;
            ctx.next_tx = TX_UNDEF;
            ctx.logged = false;
        }
        break;

    case AS_CALLING: // CQ is controlled by Beacon_On, so it's only once
    case AS_SIGNOFF:
        ctx.state = AS_IDLE;
        ctx.next_tx = TX_UNDEF;
        ctx.logged = false;
        break;
    default:
        break;
    }
}

/* ================================================================
 *                Internal helpers
 * ================================================================ */

static void set_state(autoseq_state_t s, tx_msg_t first_tx, int limit)
{
    ctx.state = s;
    ctx.next_tx = first_tx;
    ctx.retry_counter = 0;
    ctx.retry_limit = limit;
}

/* Build printable FT8 text ("<CALL> <CALL> <LOC/RPT>") */
static void format_tx_text(tx_msg_t id, char out[MAX_MSG_LEN])
{
    if (!out)
    {
        return;
    }

    out[0] = '\0';

    const char *cq_str;

    switch (id)
    {
    case TX1:
        snprintf(out, MAX_MSG_LEN, "%s %s %s", ctx.dxcall, ctx.mycall, ctx.mygrid);
        break;
    case TX2:
        snprintf(out, MAX_MSG_LEN, "%s %s %+d", ctx.dxcall, ctx.mycall, ctx.snr_tx);
        Target_RSL = ctx.snr_tx;
        break;
    case TX3:
        snprintf(out, MAX_MSG_LEN, "%s %s R%+d", ctx.dxcall, ctx.mycall, ctx.snr_tx);
        Target_RSL = ctx.snr_tx;
        break;
    case TX4:
        snprintf(out, MAX_MSG_LEN, "%s %s RR73", ctx.dxcall, ctx.mycall);
        if (!ctx.logged)
        {
            write_ADIF_Log();
            write_worked_qso();
            ctx.logged = true;
        }
        break;
    case TX5:
        snprintf(out, MAX_MSG_LEN, "%s %s 73", ctx.dxcall, ctx.mycall);
        if (!ctx.logged)
        {
            write_ADIF_Log();
            write_worked_qso();
            ctx.logged = true;
        }
        break;
    case TX6:
        if (!free_text)
        {
            switch (CQ_Mode_Index)
            {
            case 1:
                cq_str = "CQ SOTA";
                break;
            case 2:
                cq_str = "CQ POTA";
                break;
            case 3:
                cq_str = "CQ QRP";
                break;
            default:
                cq_str = CQ;
                break;
            }
            snprintf(out, MAX_MSG_LEN, "%s %s %s", cq_str, ctx.mycall, ctx.mygrid);
        }
        else
        {
            switch (Free_Index)
            {
            case 0:
                strncpy(out, Free_Text1, 13);
                break;
            case 1:
                strncpy(out, Free_Text2, 13);

                break;
            default:
                break;
            }
        }
        break;
    default:
        break;
    }
}

static void parse_rcvd_msg(const Decode *msg)
{
    ctx.rcvd_msg_type = TX_UNDEF;
    if (msg->sequence == Seq_Locator)
    {
        ctx.rcvd_msg_type = TX1;
        strncpy(ctx.dxgrid, msg->locator, sizeof(ctx.dxgrid) - 1);
    }
    else
    {
        if (strcmp(msg->locator, "73") == 0)
        {
            ctx.rcvd_msg_type = TX5;
        }
        else if (strcmp(msg->locator, "RR73") == 0)
        {
            ctx.rcvd_msg_type = TX4;
        }
        else if (strcmp(msg->locator, "RRR") == 0)
        {
            ctx.rcvd_msg_type = TX4;
        }
        else if (msg->locator[0] == 'R')
        {
            ctx.rcvd_msg_type = TX3;
        }
        else
        {
            ctx.rcvd_msg_type = TX2;
        }
    }
}

// Internal helper called by autoseq_on_touch() and autoseq_on_decode()
static bool generate_response(const Decode *msg, bool override)
{
    if (!msg || ctx.rcvd_msg_type == TX_UNDEF)
    {
        return false;
    }

    // Update the DX call and SNR
    strncpy(ctx.dxcall, msg->call_from, sizeof(ctx.dxcall) - 1);
    ctx.snr_tx = msg->snr;

    if (override)
    {
        // Reset own internal state to macth rcve_msg_type
        switch (ctx.rcvd_msg_type)
        {
        case TX1:
            set_state(AS_CALLING, TX_UNDEF, 0);
            break;
        case TX2:
            set_state(AS_REPLYING, TX_UNDEF, 0);
            break;
        case TX3:
            set_state(AS_REPORT, TX_UNDEF, 0);
            break;
        case TX4:
            set_state(AS_ROGER_REPORT, TX_UNDEF, 0);
            break;
        case TX5:
            set_state(AS_ROGERS, TX_UNDEF, 0);
        // case TX6 already handled by autoseq_on_touch()
        default:
            break;
        }
    }
    // Populating Target_Call
    strncpy(Target_Call, msg->call_from, 7);

    // Populating Station_RSL
    if (ctx.rcvd_msg_type == TX2 || ctx.rcvd_msg_type == TX3)
    {
        Station_RSL = msg->received_snr;
    }

    // After CQ TX, state goes back to IDLE. Need to distinguish between Beacon and QSO mode
    if (ctx.state == AS_IDLE)
    {
        ctx.state = Beacon_On ? AS_CALLING : AS_IDLE;
    }

    switch (ctx.state)
    {
    /* ------------------------------------------------ CALLING (we sent CQ) */
    case AS_CALLING:
        switch (ctx.rcvd_msg_type)
        {
        case TX1:
            // Populate Target_Locator
            strncpy(Target_Locator, msg->locator, 5);
            set_state(AS_REPORT, TX2, MAX_TX_RETRY);
            return true;
        case TX2:
            set_state(AS_ROGER_REPORT, TX3, MAX_TX_RETRY);
            return true;
        case TX3:
            set_state(AS_ROGERS, TX4, MAX_TX_RETRY);
            return true;
        default:
            return false;
        }

    /* ------------------------------------------------ REPLYING (we sent Tx1) */
    case AS_REPLYING:
        switch (ctx.rcvd_msg_type)
        {
        // Since we sent TX1, it doesn't make sense to respond to TX1
        case TX2:
            set_state(AS_ROGER_REPORT, TX3, MAX_TX_RETRY);
            return true;
        case TX3:
            set_state(AS_ROGERS, TX4, MAX_TX_RETRY);
            return true;

        // QSO complete without signal report exchange
        case TX4:
        case TX5:
            set_state(AS_SIGNOFF, TX5, 0);
            return true;
        default:
            return false;
        }

    /* ------------------------------------------------ REPORT sent, waiting Roger */
    case AS_REPORT:
        switch (ctx.rcvd_msg_type)
        {
        case TX3:
            set_state(AS_ROGERS, TX4, MAX_TX_RETRY);
            return true;
        // QSO complete without signal report exchange
        case TX4:
        case TX5:
            set_state(AS_SIGNOFF, TX5, 0);
            return true;
        default:
            return false;
        }

    /* ------------------------------------------------ Roger‑Report sent */
    case AS_ROGER_REPORT:
        switch (ctx.rcvd_msg_type)
        {
        // QSO complete
        case TX4:
        case TX5: // Be polite, echo back 73
            set_state(AS_SIGNOFF, TX5, 0);
            return true;
        default:
            return false;
        }

    case AS_ROGERS:
        switch (ctx.rcvd_msg_type)
        {
        // QSO complete
        case TX4:
        case TX5:
            set_state(AS_IDLE, TX_UNDEF, 0);
            break;
        default:
            break;
        }
        return false;

    // Since 73 is sent only once, this should never be reached
    case AS_SIGNOFF:
        switch (ctx.rcvd_msg_type)
        {
        // DX hasn't received our TX5. Retry
        case TX4:
            break;
        default:
            set_state(AS_IDLE, TX_UNDEF, 0);
            break;
        }
        return false;

    default:
        break;
    }
    return false;
}

static void write_worked_qso()
{
    static const char band_strs[NumBands][4] = {
        "40", "30", "20", "17", "15", "12", "10"};
    char *buf = add_worked_qso();
    int printed = snprintf(buf, MAX_LINE_LEN, "%.3s %.12s",
                           band_strs[BandIndex],
                           ctx.dxcall);
    if (printed < 0)
    {
        return;
    }
    char rsl[5]; // space + sign + 2 digits + null = 5
    // Check if RX RSL would fit
    int needed = snprintf(rsl, sizeof(rsl), " %d", Station_RSL);
    if (printed + needed <= MAX_LINE_LEN - 1)
    {
        strncpy(buf + printed, rsl, needed + 1);
        printed += needed;
    }
    else
    {
        return;
    }
    // Check if TX RSL would fit
    needed = snprintf(rsl, sizeof(rsl), " %d", Target_RSL);
    if (printed + needed <= MAX_LINE_LEN - 1)
    {
        strncpy(buf + printed, rsl, needed + 1);
    }
}
