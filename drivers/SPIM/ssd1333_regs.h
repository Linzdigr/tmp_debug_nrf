/*
  Copyright (c) 2022 Kiwi devices Inc.

  @author Yvan FEREZ
  @version 0.1
 */

#ifndef __SSD1333_REGS_H__
#define __SSD1333_REGS_H__

// Timing Delays
#define SSD1333_DELAYS_HWFILL       (3)         // Fill delay
#define SSD1333_DELAYS_HWLINE       (1)         // Line delay
#define SSD1333_RESET_DELAY_MS      5

// SSD1333 Commands
#define SSD1333_CMD_WRITEINTORAM    0x5C        // Enable MCU to Write into RAM
#define SSD1333_CMD_READFROMRAM     0x5D        // Enable MCU to Read from RAM
#define SSD1333_CMD_FILL            0x26        // Fill enable/disable
#define SSD1333_CMD_SETCOLUMN       0x15        // Set column address
#define SSD1333_CMD_SETROW          0x75        // Set row adress
#define SSD1333_CMD_SETREMAP        0xA0        // Set re-map & data format - Default => 0x50
#define SSD1333_CMD_STARTLINE       0xA1        // Set display start line - Set Vertical Scroll by RAM

/* Acceleration commands */
#define SSD1333_CMD_DRAWLINE        0x21        // Draw line
#define SSD1333_CMD_DRAWRECT        0x22        // Draw rectangle

/* Time related commands */
#define SSD1333_CMD_SETMULTIPLEX    0xA8        // Set multiplex ratio
#define SSD1333_CMD_CLOCKDIV        0xB3        // Set display clock divide ratio/oscillator frequency - Default => 0x90
#define SSD1333_CMD_PHASELEN        0xB1        // Phase 1 (Reset) & Phase 2 (Pre-Charge) Period Adjustment - Default => 0x84 (16 Display Clocks [Phase 2] / 8 Display Clocks [Phase 1])
#define SSD1333_CMD_PRECHARGE_P_A   0x8A        // Phase 1 and 2 period adjustment - Default => 0x08 (8 Display Clocks)
#define SSD1333_CMD_PRECHARGE_P_B   0x8B        // Phase 1 and 2 period adjustment - Default => 0x08 (8 Display Clocks)
#define SSD1333_CMD_PRECHARGE_P_C   0x8C        // Phase 1 and 2 period adjustment - Default => 0x08 (8 Display Clocks)

/* Drive current / levels related commands */
#define SSD1333_CMD_SETMASTER       0xAD        // Set master configuration (IREF) - 0x80: Select external IREF, 0x90: Enable Internal IREF during display On
#define SSD1333_CMD_CONTRAST_COLOR  0xC1        // Contrast for color channels
#define SSD1333_CMD_MASTERCURRENT   0xC7        // Master contrast current control
#define SSD1333_CMD_PRECHARGE_V     0xBB        // Set second pre-charge speed for colors
#define SSD1333_CMD_PRECHARGE_PH1_2 0xB1        // Set first pre-charge speed
#define SSD1333_CMD_PRECHARGE_P     0xB6        // Set pre-charge speed
#define SSD1333_CMD_PRECHARGE_PH4   0xB6        // Set final pre-charge speed
#define SSD1333_CMD_VCOMH           0xBE        // Set Vcomh voltage (deselect V)
#define SSD1333_CMD_LINEAR_GST      0xB9        // Set linear Gray Scale Table at default

/* Modes / global state related commands */
#define SSD1333_CMD_LOCK            0xFD        // (Un)lock IC from hearing CMD (except 0xFD ofc) - 0x12 Unlock / 0x16 Lock
#define SSD1333_CMD_SETPWRSVMODE    0xB0        // Set power saving mode
#define SSD1333_CMD_DISPLAYOFF      0xAE        // Display OFF (sleep mode)
#define SSD1333_CMD_DISPLAYON       0xAF        // Normal Brightness Display ON
#define SSD1333_CMD_POWERMODE       0xB0        // Power save mode
#define SSD1333_CMD_DISPLAYOFFSET   0xA2        // Set display offset
#define SSD1333_CMD_DISPLAYALLOFF   0xA4        // Set entire display OFF
#define SSD1333_CMD_DISPLAYALLON    0xA5        // Set entire display ON
#define SSD1333_CMD_NORMALDISPLAY   0xA6        // Normal display
#define SSD1333_CMD_INVERTDISPLAY   0xA7        // Invert display

// Some SSD1333 Data values for OLED 160*128px display
#define SSD1333_MAX_COL             0x9F        // 160-1
#define SSD1333_MAX_ROW             0x7F        // 128-1
#define	Brightness                  0x0F

#endif   /* __SSD1333_REGS_H__ */
