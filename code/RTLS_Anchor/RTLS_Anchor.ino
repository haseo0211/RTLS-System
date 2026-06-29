/*
*****************************************************************************************************************************************

 * Project Name : RTLS-System (for GitHub)
 * About        : Single-Side Two-Way Ranging implementation using a Tag-Anchor structure
 * About        : Public Release Version
 * 
 * Module : Makerfabs - ESP32 UWB DW3000  
 * 
***************************************************************************************************************************************** 
 */

/*
 * @file : RTLS_Anchor
 * @author : haseo0211

 * @version V 1.0.0  @date  2025.09.22  ---> Implemented SS-TWR
 * @version V 1.0.1  @date  2025.09.25  ---> Tuned timing parameters to improve Tag reception stability for Anchor responses
 * @version V 1.0.2  @date  2025.11.02  ---> Modified communication frame addresses (synchronized with the function in the Tag communication code)
 * @version
 * @version
 * @version
 * 
 * @brief The functions used in this code can be found in the API guide.
 * @brief Refer to the official Makerfabs documentation for detailed API descriptions.
 * @brief 1UUS = 1.0256us
 * @brief Variable definitions: anchor_poll_time, anchor_resp_time
 * @brief            anchor_poll_time : Time when the Anchor receives the Poll message
 * @brief            anchor_resp_time : Time when the Anchor transmits the Response message
 * @brief
 * @brief
 * @brief For details, refer to the Tag code used as the main code.
 * @brief ===============================================================================================================================
 */

#include "dw3000.h"   /* Makerfabs OpenSource Library */
#include "SPI.h"      /* SPI communication */

/* Use global variables defined in another source file */
/* dwt_txconfig_t is a structure that contains DW3000 TX power, bandwidth, and calibration values */
extern SPISettings _fastSPI;
extern dwt_txconfig_t txconfig_options;   

/* Pin definitions */
#define PIN_RST 27
#define PIN_IRQ 34   /* IRQ = Interrupt Request - Indicates events such as RX and TX */
#define PIN_SS 4

/* Antenna delay (requires calibration through experiments) */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

/* TX/RX frame-related definitions */
#define ALL_MSG_COMMON_LEN 10        // Message size
#define ALL_MSG_SN_IDX 2             // Sequence number index
#define RESP_MSG_anchor_poll_time_IDX 10   // Index where the Poll reception time is stored
#define RESP_MSG_anchor_resp_time_IDX 14   // Index where the Response transmission time is stored
#define RESP_MSG_TS_LEN 4            // Timestamp length
#define FINAL_Ra_IDX (ALL_MSG_COMMON_LEN) // The first 10 Final indices are the common indices (Ra data is inserted after this)



/* Timing parameters */
/* Determines how many UUS after receiving the Poll from the Tag the Anchor transmits the Response */
/* In theory, reception stability can be improved by tuning this parameter, but if the value is too small, delayed transmission is more likely to fail */
/* Avoid setting this value below 400 if possible */
#define POLL_RX_TO_RESP_TX_DLY_UUS 500   // Delay time from Poll reception to Response transmission (UUS unit)

/* Communication configuration structure - Refer to the API guide */
/* The Tag and Anchor must use the same communication configuration */
static dwt_config_t config = {
    5,                /* channel number, select either {5, 9} */
    DWT_PLEN_128,     /* preamble length */
    DWT_PAC8,         /* preamble acquisition chunk size */
    9,                /* TX preamble Code */
    9,                /* RX preamble Code */
    1,                /* Start Frame Delimiter Type */
    DWT_BR_6M8,       /* dataRate = 6.8Mbps */
    DWT_PHRMODE_STD,  /* PHY Header Mode */
    DWT_PHRRATE_STD,  /* PHY Header Rate = 850kbps */
    (129 + 8 - 8),    /* SFD timeout */
    DWT_STS_MODE_OFF, /* STS off */
    DWT_STS_LEN_64,   /* STS length (not used) */
    DWT_PDOA_M0       /* PDOA mode off */
};

/* TX/RX message definitions */
static uint8_t rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'A', 'N', 'C', '1', 0xE0, 0, 0};   // Poll to be received
static uint8_t tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'N', 'C', 'A', '1', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // Response to be transmitted


/* Sequence number, repeated from 0 to 255 */
static uint8_t frame_seq_nb = 0;

/* RX buffer */
static uint8_t rx_buffer[20];

/* Status register */
static uint32_t status_reg = 0; 

/* Time variables */
static uint64_t anchor_poll_time;   // Poll reception time
static uint64_t anchor_resp_time;   // Response transmission time


void setup()
{
  delay(1500);                               // Wait for power stabilization
  Serial.begin(115200);
  Serial.println("\n[BOOT] Anchor FW starting...");
  
  UART_init();                               // UART initialization

  _fastSPI = SPISettings(16000000L, MSBFIRST, SPI_MODE0);  // SPI configuration
  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_SS);                         // Select SPI slave

  delay(2);                                  // Wait for DW3000 initialization

  while (!dwt_checkidlerc())                 // Check IDLE_RC state
  {
    UART_puts("IDLE FAILED\r\n");
    while (1);
  }

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)   // Initialize the DW3000 driver
  {
    UART_puts("INIT FAILED\r\n");
    while (1);
  }

  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);   // Enable LED functions for debugging

  if (dwt_configure(&config))                           // Apply communication configuration
  {
    UART_puts("CONFIG FAILED\r\n");
    while (1);
  }

  dwt_configuretxrf(&txconfig_options);               // Apply RF transmission parameters
  dwt_setrxantennadelay(RX_ANT_DLY);                  // Calibrate RX antenna delay
  dwt_settxantennadelay(TX_ANT_DLY);                  // Calibrate TX antenna delay
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);   // Enable LNA/PA

  Serial.println("Anchor");
  Serial.println("Setup over........");
}

void loop()
{
  dwt_rxenable(DWT_START_RX_IMMEDIATE);   // Switch to RX mode immediately

  /* Wait until Poll reception or error occurs */
  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & 
          (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR))) { }

  if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)   // Normal reception
  {
    uint32_t frame_len;
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);   // Clear RX event bit

    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;       // Read frame length
    if (frame_len <= sizeof(rx_buffer))
    {
      dwt_readrxdata(rx_buffer, frame_len, 0);                     // Read RX data
      rx_buffer[ALL_MSG_SN_IDX] = 0;                               // Ignore sequence number before comparison

      if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0) // Check Poll message
      {
        uint32_t resp_tx_time;
        int ret;

        anchor_poll_time = get_rx_timestamp_u64();   // Store Poll reception time

        /* Calculate Response transmission time */
        resp_tx_time = (anchor_poll_time + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
        dwt_setdelayedtrxtime(resp_tx_time);

        anchor_resp_time = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;   // Response transmission time with antenna delay applied

        resp_msg_set_ts(&tx_resp_msg[RESP_MSG_anchor_poll_time_IDX], anchor_poll_time);   // Insert Poll reception time
        resp_msg_set_ts(&tx_resp_msg[RESP_MSG_anchor_resp_time_IDX], anchor_resp_time);   // Insert Response transmission time

        /* Transmit Response message */
        tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
        dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0); // Write data to TX buffer
        dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1);          // Configure TX control
        ret = dwt_starttx(DWT_START_TX_DELAYED);              // Start delayed transmission

        if (ret == DWT_SUCCESS)
        {
          while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) { }   // Wait for transmission completion
          dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);                 // Clear TX complete flag
          frame_seq_nb++;                                                             // Increment sequence number
        }
      }
    }
  }
  else
  {
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);   // Clear RX error flag
  }
}
