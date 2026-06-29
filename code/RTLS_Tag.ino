/*

---

Project Name : RTLS-System (for GitHub)
About        : Single-Side Two-Way Ranging implementation using a Tag-Anchor structure
About        : Public Release Version

Module       : Makerfabs - ESP32 UWB DW3000

---

*/

/*
@file : RTLS_Tag
@author : haseo0211

@version V 1.0.0  @date  2025.09.21  ---> Implemented SS-TWR and measured antenna delay
@version V 1.0.1  @date  2025.09.25  ---> Tuned timing parameters to improve Tag reception stability for Anchor responses
@version V 1.0.2  @date  2025.11.02  ---> Refactored code; added distance extraction functions, extended to 4 anchors, and added position calculation
@version V 1.0.3  @date  2025.11.23  ---> Optimized code and modified the positioning matrix
@version V 1.0.4  @date  2025.11.30  ---> Fixed numerical errors
@version V 1.1    @date  2026.06.29  ---> Generalized position calculation using anchor coordinates

@brief The functions used in this code can be found in the API guide.
@brief Refer to the official Makerfabs documentation for detailed API descriptions.
@brief 1 UUS = 1.0256 us
@brief Variable definitions: tag_poll_time, tag_resp_time, anchor_poll_time, anchor_resp_time
@brief   tag_poll_time     : Time when the Tag transmits the Poll message
@brief   tag_resp_time     : Time when the Tag receives the Response message
@brief   anchor_poll_time  : Time when the Anchor receives the Poll message
@brief   anchor_resp_time  : Time when the Anchor transmits the Response message
@brief
@brief When using Bluetooth, insufficient RAM may cause repeated reboot issues.
@brief For stable data transmission from the ESP32, use a wired Serial connection.
@brief
@brief This code is a simple public release version for constructing a basic RTLS system.
*/

#include "dw3000.h"  /* Makerfabs Open Source Library */
#include "SPI.h"     /* SPI communication */
#include <math.h>

/*
External global variables defined in another source file.
dwt_txconfig_t is a structure that contains DW3000 TX configuration values,
including power, bandwidth, and calibration parameters.
This file does not modify those values directly.
*/
extern SPISettings _fastSPI;
extern dwt_txconfig_t txconfig_options;

#define PIN_RST 27
#define PIN_IRQ 34 /* IRQ = Interrupt Request. Indicates events such as RX and TX. */
#define PIN_SS 4

#define TX_ANT_DLY 16385                                            // Adjustable antenna delay parameter
#define RX_ANT_DLY 16385                                            // Calibrate the antenna delay at approximately 1 m or 2 m

/* Common TX/RX frame indices */
#define ALL_MSG_COMMON_LEN 10                                       // Common message length
#define ALL_MSG_SN_IDX 2                                            // Sequence number index
#define RESP_MSG_anchor_poll_time_IDX 10                            // Index of the Anchor Poll RX timestamp
#define RESP_MSG_anchor_resp_time_IDX 14                            // Index of the Anchor Response TX timestamp

/* Timing parameters */
#define POLL_TX_TO_RESP_RX_DLY_UUS  240                             // Delay before switching to RX mode after Poll transmission

/* Adjust this parameter to improve reception stability. */
#define RESP_RX_TIMEOUT_UUS         520                             // RX timeout duration for the Tag

/* Ranging cycle period */
#define RNG_DELAY_MS 20                                             // ms, approximately 50 Hz

#define NUM_ANCHORS 4

/*
Anchor coordinates.
Modify only this part according to the actual anchor configuration.
*/
static const double anchor_pos[NUM_ANCHORS][3] = {
{0.0, 0.0, 0.0},     // Anchor 1
{10.0, 0.0, 0.0},    // Anchor 2
{10.0, 10.0, 0.0},   // Anchor 3
{0.0, 10.0, 0.0}     // Anchor 4
};

/*
Communication configuration structure.
Refer to the API guide for detailed descriptions.
The Tag and Anchors must use the same communication configuration.
*/
static dwt_config_t config = {
5,                  /* Channel number: select either 5 or 9 */
DWT_PLEN_128,       /* Preamble length: available range is 64 to 4096 */
DWT_PAC8,           /* Preamble acquisition chunk size, used in RX only */
9,                  /* TX preamble code */
9,                  /* RX preamble code */
1,                  /* Start Frame Delimiter type, 1 => DW8 */
DWT_BR_6M8,         /* Data rate: 6.8 Mbps */
DWT_PHRMODE_STD,    /* PHY Header mode: Standard */
DWT_PHRRATE_STD,    /* PHY Header rate: 850 kbps */
(129 + 8 - 8),
DWT_STS_MODE_OFF,   /* STS (Scrambled Timestamp Sequence) OFF */
DWT_STS_LEN_64,     /* STS length: not used */
DWT_PDOA_M0         /* PDOA mode OFF */
};

/* Variable definitions */
/* Note that rx_msg and tx_msg are different between the Anchor and Tag. */

/* Communication frames */
/* Poll messages transmitted to each Anchor */
static uint8_t tx_poll_msg[NUM_ANCHORS][12] = {
{0x41, 0x88, 0, 0xCA, 0xDE, 'A', 'N', 'C', '1', 0xE0, 0, 0},
{0x41, 0x88, 0, 0xCA, 0xDE, 'A', 'N', 'C', '2', 0xE0, 0, 0},
{0x41, 0x88, 0, 0xCA, 0xDE, 'A', 'N', 'C', '3', 0xE0, 0, 0},
{0x41, 0x88, 0, 0xCA, 0xDE, 'A', 'N', 'C', '4', 0xE0, 0, 0}
};

/* Response messages expected from each Anchor */
static uint8_t rx_resp_msg[NUM_ANCHORS][20] = {
{0x41, 0x88, 0, 0xCA, 0xDE, 'N', 'C', 'A', '1', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0x41, 0x88, 0, 0xCA, 0xDE, 'N', 'C', 'A', '2', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0x41, 0x88, 0, 0xCA, 0xDE, 'N', 'C', 'A', '3', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0x41, 0x88, 0, 0xCA, 0xDE, 'N', 'C', 'A', '4', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};

/* Status register */
static uint32_t status_reg = 0;

/* Sequence number, repeated from 0 to 255 */
static uint8_t frame_seq_nb = 0;

/* RX buffer */
static uint8_t rx_buffer[20];

/* Distance variables */
static double distance[NUM_ANCHORS] = {
0.0, 0.0, 0.0, 0.0
};

/* Position variable */
static double pos[3];

# /*

# ************************************************************ Function Definitions *********************************************************

*/

/* Tag-Anchor communication function */
double tag_anchor_communicate(uint8_t *tx_poll_msg, uint16_t poll_len, uint8_t *rx_resp_msg, double pre_distance)
{
double tof;
double distance;

/* Fill the sequence number field in the Poll message. */
tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;

dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);     // Clear the TX complete flag in the status register
dwt_writetxdata(poll_len, tx_poll_msg, 0);                       // Write tx_poll_msg to the DW3000 TX buffer from offset 0
dwt_writetxfctrl(poll_len, 0 , 1);                               // Configure TX frame length, buffer offset, and ranging flag

/* Start communication. */

/*
After TX is completed, the device waits for 240 UUS and switches to RX mode.
The Tag waits for a Response message until the RX timeout occurs.
*/
dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

/*
Read the status register and wait until one of the following events occurs:
successful RX, RX timeout, or RX error.
*/
while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
{
};

/* Increment the sequence number. */
frame_seq_nb++;

if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)                       // Data received successfully
{
uint32_t frame_len;
dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);    // Clear the RX good frame flag to prevent event conflicts

```
frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;        // Read the received frame length

if (frame_len <= sizeof(rx_buffer))                             // Check whether the received frame fits in the RX buffer
{
  dwt_readrxdata(rx_buffer, frame_len, 0);                      // Read the received frame into rx_buffer from offset 0

  /*
     The sequence number changes every frame.
     Set it to 0 before comparing the received frame with the expected Response message.
  */
  rx_buffer[ALL_MSG_SN_IDX] = 0;

  if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)  // Check whether the first 10 bytes match the expected Response frame
  {
    uint32_t tag_poll_time, tag_resp_time, anchor_poll_time, anchor_resp_time;
    int32_t round_time, delay_time;
    float clockOffsetRatio;

    /* Tag timestamp values can be read directly from the DW3000. */
    tag_poll_time = dwt_readtxtimestamplo32();
    tag_resp_time = dwt_readrxtimestamplo32();

    // Clock offset correction
    clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

    // Extract Anchor timestamp values from the received frame
    resp_msg_get_ts(&rx_buffer[RESP_MSG_anchor_poll_time_IDX], &anchor_poll_time);
    resp_msg_get_ts(&rx_buffer[RESP_MSG_anchor_resp_time_IDX], &anchor_resp_time);

    // Time calculation
    round_time = tag_resp_time - tag_poll_time;
    delay_time = anchor_resp_time - anchor_poll_time;

    tof = ((round_time - delay_time * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
    distance = tof * SPEED_OF_LIGHT;

    return distance;
  }
}
```

}
else                                                               // Data was not received successfully
{
//Serial.println("Timeout/Error occurred");

```
// Clear RX timeout and RX error flags before starting the next ranging cycle
dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

return pre_distance;                                             // Return the previous distance if reception fails
```

}

return pre_distance;                                               // Return the previous distance for unexpected frames or invalid frame length
}

double determinant_3x3(double matrix[3][3])
{
return
matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]) -
matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0]) +
matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);
}

bool solve_3x3(double A[3][3], double b[3], double x[3])
{
double det_A = determinant_3x3(A);

if (fabs(det_A) < 1e-9) {
return false;
}

double A_x[3][3] = {
{b[0], A[0][1], A[0][2]},
{b[1], A[1][1], A[1][2]},
{b[2], A[2][1], A[2][2]}
};

double A_y[3][3] = {
{A[0][0], b[0], A[0][2]},
{A[1][0], b[1], A[1][2]},
{A[2][0], b[2], A[2][2]}
};

double A_z[3][3] = {
{A[0][0], A[0][1], b[0]},
{A[1][0], A[1][1], b[1]},
{A[2][0], A[2][1], b[2]}
};

x[0] = determinant_3x3(A_x) / det_A;
x[1] = determinant_3x3(A_y) / det_A;
x[2] = determinant_3x3(A_z) / det_A;

return true;
}

/*
Position calculation function.
Matrix A and vector b are automatically calculated using the anchor coordinates.
*/
void cal_position(double ranges[NUM_ANCHORS], double pos[3])
{
double A[3][3];
double b[3];

double norm_p1 =
anchor_pos[0][0] * anchor_pos[0][0] +
anchor_pos[0][1] * anchor_pos[0][1] +
anchor_pos[0][2] * anchor_pos[0][2];

for (int i = 1; i < NUM_ANCHORS; i++) {
A[i - 1][0] = 2.0 * (anchor_pos[i][0] - anchor_pos[0][0]);
A[i - 1][1] = 2.0 * (anchor_pos[i][1] - anchor_pos[0][1]);
A[i - 1][2] = 2.0 * (anchor_pos[i][2] - anchor_pos[0][2]);

```
double norm_pi =
  anchor_pos[i][0] * anchor_pos[i][0] +
  anchor_pos[i][1] * anchor_pos[i][1] +
  anchor_pos[i][2] * anchor_pos[i][2];

b[i - 1] =
  (ranges[0] * ranges[0] - ranges[i] * ranges[i]) +
  (norm_pi - norm_p1);
```

}

bool solved = solve_3x3(A, b, pos);

if (!solved) {
pos[0] = 0.0;
pos[1] = 0.0;
pos[2] = 0.0;
}
}

/*
Output function.
This function can be used for Python data reception or debugging.
*/
void output()
{
/*
Serial.print(F("distance1=")); Serial.print(distance[0], 3);
Serial.print(F(" || "));

Serial.print(F("distance2=")); Serial.print(distance[1], 3);
Serial.print(F(" || "));

Serial.print(F("distance3=")); Serial.print(distance[2], 3);
Serial.print(F(" || "));

Serial.print(F("distance4=")); Serial.println(distance[3], 3);
*/

Serial.print(F("x =")); Serial.print(pos[0], 3);
Serial.print(F(" || "));

Serial.print(F("y =")); Serial.print(pos[1], 3);
Serial.print(F(" || "));

Serial.print(F("z =")); Serial.println(pos[2], 3);
}

# /*

# ***************************************************************** Main Loop ***************************************************************

*/

void setup()
{
delay(1500);                 // Power stabilization delay
Serial.begin(115200);
Serial.println("\n[BOOT] Tag firmware starting...");

UART_init();

_fastSPI = SPISettings(16000000L, MSBFIRST, SPI_MODE0);           // SPI settings for DW3000 communication

spiBegin(PIN_IRQ, PIN_RST);
spiSelect(PIN_SS);                                                // Select the DW3000 SPI device

delay(2);                                                         // Short delay for DW3000 setup

while (!dwt_checkidlerc())                                        // Check whether the device is in IDLE_RC state
{
Serial.print("IDLE FAILED\r\n");
while (1)
;
}

if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)                     // Initialize the DW3000 driver
{
Serial.print("INIT FAILED\r\n");
while (1)
;
}

dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);               // Enable LED functions

if (dwt_configure(&config))                                       // Apply the communication configuration
{
Serial.print("CONFIG FAILED\r\n");
while (1)
;
}

/* Apply the TX RF configuration defined in txconfig_options. */
dwt_configuretxrf(&txconfig_options);

/* Apply antenna delay calibration values. */
dwt_setrxantennadelay(RX_ANT_DLY);
dwt_settxantennadelay(TX_ANT_DLY);

/* Configure TX-to-RX delay and RX timeout. */
dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

/* Enable LNA and PA mode. */
dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

/* Print a message after setup is complete. */
Serial.println("Tag");
Serial.println("Setup complete.");
}

void loop()
{
// Tag-Anchor communication section
for (int i = 0; i < NUM_ANCHORS; i++) {
distance[i] = tag_anchor_communicate(tx_poll_msg[i], (uint16_t)sizeof(tx_poll_msg[i]), rx_resp_msg[i], distance[i]);
}

// Position calculation section
cal_position(distance, pos);

/*
Output section.
Since the output function introduces additional processing,
enable or disable it as needed.
*/
output();

delay(RNG_DELAY_MS);
}
