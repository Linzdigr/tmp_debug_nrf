/*
 * Copyright (c) 2022 Kiwi devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT solomon_ssd1333

#include <logging/log.h>
LOG_MODULE_REGISTER(ssd1333, CONFIG_DISPLAY_LOG_LEVEL);

#include <string.h>
#include <device.h>
#include <init.h>
#include <drivers/gpio.h>
#include <drivers/i2c.h>
#include <drivers/spi.h>
#include <nrfx_spim.h>

#include "ssd1333_regs.h"
#include <display/cfb.h>

typedef uint8_t BYTE;

#define SPI_NO_DATA       NULL

struct ssd1333_config {
	struct spi_dt_spec bus;
	struct gpio_dt_spec dc_gpio;
  struct gpio_dt_spec cs_gpio;
  struct gpio_dt_spec mosi_gpio;
  struct gpio_dt_spec miso_gpio;
  struct gpio_dt_spec reset_gpio;
  struct gpio_dt_spec sck_gpio;
  bool orientation;
  uint16_t height;
  uint16_t width;
  BYTE vcom;
  BYTE multiplex;
  BYTE adress_start;
};

struct ssd1333_data {
  uint8_t contrast;
  uint8_t scan_mode;
};

BYTE SSD1333_DATA_ADDR_0[] = {0x0};             // Address 0
BYTE SSD1333_DATA_UNLOCK[] = {0x12};            // IC unlock
BYTE SSD1333_DATA_LOCK[] = {0x16};              // IC lock
BYTE SSD1333_DATA_DISPLAYOFFSET_DEF[] = {0x0};  // Display offset default value
BYTE SSD1333_DATA_EXT_IREF[] = {0x80};          // Select external IREF
BYTE SSD1333_DATA_PRECHARGE_8P[] = {0x08};      // Set Second Pre-Charge Period as 8 Clock
BYTE SSD1333_DATA_CLOCKDIV_90FPS[] = {0xD1};    // Set Clock as 90 Frames/Sec
BYTE SSD1333_DATA_MULTIPLEX_1_64[] = {SSD1333_MAX_ROW}; // 1/128 duty
BYTE SSD1333_DATA_PRECHARGE_V_40[] = {0x17};    // Set Pre-Charge Voltage Level as 0.40*VCC
BYTE SSD1333_DATA_COM_DES_PIN_V[] = {0x05};     // Set Common Pins Deselect Voltage Level as 0.82*VCC
BYTE SSD1333_DATA_MASTERCURR_DEF[] = {0xAF};    // Set contrast R,G,B
BYTE SSD1333_DATA_CONTRAST_DEF[] = {0x70, 0x70, 0x70};    // Set contrast R,G,B
BYTE SSD1333_DATA_COLOR_REMAP[] = {0x74};       // Color Map remap
BYTE SSD1333_DATA_PHASELENGHT_DEF[] = {0x22};   // Phase length period (def. 0x84)
BYTE SSD1333_DATA_ADDR_RANGE_X[] = {0x08, 0xA7}; // Had to do +8 ...
BYTE SSD1333_DATA_ADDR_RANGE_Y[] = {0x0, SSD1333_MAX_ROW}; // End adress 0x7F

BYTE SSD1333_DATA_CUSTOM_GRAYSCALE_TABLE[] = {0xB8, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x15, 0x17, 0x19, 0x1B, 0x1D, 0x1F, 0x21, 0x23, 0x25, 0x27, 0x2A, 0x2D, 0x30, 0x33, 0x36, 0x39, 0x3C, 0x3F, 0x42, 0x45, 0x48, 0x4C, 0x50, 0x54, 0x58, 0x5C, 0x60, 0x64, 0x68, 0x6C, 0x70, 0x74, 0x78, 0x7D, 0x82, 0x87, 0x8C, 0x91, 0x96, 0x9B, 0xA0, 0xA5, 0xAA, 0xAF, 0xB4};

/* Methods definition */

static inline bool ssd1333_bus_ready(const struct device *dev) {
  // printk("[SSD1333] BUS READY CHECK\n");
  const struct ssd1333_config *config = dev->config;

  if(gpio_pin_configure_dt(&config->dc_gpio, GPIO_OUTPUT_INACTIVE) < 0) {
    return false;
  }

  return spi_is_ready(&config->bus);
}

static inline int ssd1333_write_bus(const struct device *dev, BYTE cmd, BYTE *data, size_t data_len) {
  // printk("[SSD1333] WRITE BUS\n");
  const struct ssd1333_config *config = dev->config;

  BYTE *tx_buffer = k_malloc(data_len+1);
  
  tx_buffer[0] = cmd;

  if(data_len != 0 && data) {
    memcpy(tx_buffer+1, data, data_len);
  }

  nrf_spim_tx_buffer_set(NRF_SPIM3, tx_buffer, data_len+1);
  nrf_spim_rx_buffer_set(NRF_SPIM3, NULL, 0);

  nrf_spim_task_trigger(NRF_SPIM3, NRF_SPIM_TASK_START);

  while(!nrf_spim_event_check(NRF_SPIM3, NRF_SPIM_EVENT_ENDTX)) {
    k_usleep(5);
  }

  k_free(tx_buffer);
  nrf_spim_event_clear(NRF_SPIM3, NRF_SPIM_EVENT_ENDTX);
}

static void format65kColors(BYTE* buf, BYTE _r, BYTE _g, BYTE _b) {
  if(!buf) {
    // TODO Error handling
    return;
  }
  buf[0] = (_r << 3) | (_g >> 3);
  buf[1] = (_g << 5) | (_b);
}

static void resetPixelPos(const struct device *dev) {
  ssd1333_write_bus(dev, SSD1333_CMD_SETCOLUMN, SSD1333_DATA_ADDR_RANGE_X, sizeof(SSD1333_DATA_ADDR_RANGE_X));
  ssd1333_write_bus(dev, SSD1333_CMD_SETROW, SSD1333_DATA_ADDR_RANGE_Y, sizeof(SSD1333_DATA_ADDR_RANGE_Y));
}

int ssd1333_init_device(const struct device *dev) {
  // printk("[SSD1333] INIT DEV\n");

  const struct ssd1333_config *config = dev->config;

  /* Reset if pin connected */
  if(config->reset_gpio.port) {
    k_msleep(SSD1333_RESET_DELAY_MS);
    gpio_pin_set_dt(&config->reset_gpio, 1);
    k_msleep(SSD1333_RESET_DELAY_MS);
    gpio_pin_set_dt(&config->reset_gpio, 0);
  }

  // Initialization Sequence
  ssd1333_write_bus(dev, SSD1333_CMD_LOCK, SSD1333_DATA_UNLOCK, sizeof(SSD1333_DATA_UNLOCK));
  ssd1333_write_bus(dev, SSD1333_CMD_DISPLAYOFF, SPI_NO_DATA, 0);
  ssd1333_write_bus(dev, SSD1333_CMD_CLOCKDIV, SSD1333_DATA_CLOCKDIV_90FPS, sizeof(SSD1333_DATA_CLOCKDIV_90FPS)); // 7:4 = Oscillator Frequency, 3:0 = CLK Div Ratio - (A[3:0]+1 = 1..16)
  ssd1333_write_bus(dev, SSD1333_CMD_SETMULTIPLEX, SSD1333_DATA_MULTIPLEX_1_64, sizeof(SSD1333_DATA_MULTIPLEX_1_64)); // 0x3F 1/64 duty
  ssd1333_write_bus(dev, SSD1333_CMD_DISPLAYOFFSET, SSD1333_DATA_DISPLAYOFFSET_DEF, sizeof(SSD1333_DATA_DISPLAYOFFSET_DEF));
  ssd1333_write_bus(dev, SSD1333_CMD_STARTLINE, SSD1333_DATA_ADDR_0, sizeof(SSD1333_DATA_ADDR_0));
  ssd1333_write_bus(dev, SSD1333_CMD_SETREMAP, SSD1333_DATA_COLOR_REMAP, sizeof(SSD1333_DATA_COLOR_REMAP));
  ssd1333_write_bus(dev, SSD1333_CMD_SETMASTER, SSD1333_DATA_EXT_IREF, sizeof(SSD1333_DATA_EXT_IREF));
  ssd1333_write_bus(dev, SSD1333_CMD_MASTERCURRENT, SSD1333_DATA_MASTERCURR_DEF, sizeof(SSD1333_DATA_MASTERCURR_DEF));
  ssd1333_write_bus(dev, SSD1333_CMD_CONTRAST_COLOR, SSD1333_DATA_CONTRAST_DEF, sizeof(SSD1333_DATA_CONTRAST_DEF));
  ssd1333_write_bus(dev, SSD1333_CMD_LINEAR_GST, SPI_NO_DATA, 0);
  ssd1333_write_bus(dev, SSD1333_CMD_PHASELEN, SSD1333_DATA_PHASELENGHT_DEF, sizeof(SSD1333_DATA_PHASELENGHT_DEF));
  ssd1333_write_bus(dev, SSD1333_CMD_PRECHARGE_V, SSD1333_DATA_PRECHARGE_V_40, sizeof(SSD1333_DATA_PRECHARGE_V_40));  // Set Pre-Charge Voltage Level as 0.40*VCC
  ssd1333_write_bus(dev, SSD1333_CMD_PRECHARGE_P, SSD1333_DATA_PRECHARGE_8P, sizeof(SSD1333_DATA_PRECHARGE_8P));
  ssd1333_write_bus(dev, SSD1333_CMD_VCOMH, SSD1333_DATA_COM_DES_PIN_V, sizeof(SSD1333_DATA_COM_DES_PIN_V));  // Set Common Pins Deselect Voltage Level as 0.82*VCC
  ssd1333_write_bus(dev, SSD1333_CMD_NORMALDISPLAY, SPI_NO_DATA, 0);
  ssd1333_write_bus(dev, SSD1333_CMD_DISPLAYON, SPI_NO_DATA, 0);

  resetPixelPos(dev);

  return 0;
}

int ssd1333_init(const struct device *dev) {
  // printk("[SSD1333] MAINT INIT\n");
  const struct ssd1333_config *config = dev->config;
  
  nrf_spim_disable(NRF_SPIM3);

  nrf_spim_pins_set(NRF_SPIM3, 27, 32, NRF_SPIM_PIN_NOT_CONNECTED);
  nrf_spim_dcx_pin_set(NRF_SPIM3, config->dc_gpio.pin);
  nrf_spim_dcx_cnt_set(NRF_SPIM3, 1);
  nrf_spim_configure(NRF_SPIM3, NRF_SPIM_MODE_0, NRF_SPIM_BIT_ORDER_MSB_FIRST);
  nrf_spim_csn_configure(NRF_SPIM3, 40, NRF_SPIM_CSN_POL_LOW, 0);
  nrf_spim_frequency_set(NRF_SPIM3, NRF_SPIM_FREQ_8M);
  // nrf_spim_int_enable(NRF_SPIM3, NRF_SPIM_INT_ENDTX_MASK);
  nrf_spim_enable(NRF_SPIM3);

  if(ssd1333_init_device(dev)) {
    LOG_ERR("Failed to initialize device!");
    return -EIO;
  }

  return 0;
}

static int ssd1333_blanking_off(const struct device *dev) {
  // printk("[SSD1333] BlOFF\n");
  return -ENOTSUP;
}

static int ssd1333_blanking_on(const struct device *dev) {
  // printk("[SSD1333] BlON\n");
  return -ENOTSUP;
}

int ssd1333_write(const struct device *dev, const uint16_t x, const uint16_t y,
        const struct display_buffer_descriptor *desc,
        const void *buf) {
  // printk("[SSD1333] WRITE\n");
  size_t buf_len;

  if(desc->pitch < desc->width) {
    LOG_ERR("Pitch is smaller than width");
    return -1;
  }

  buf_len = desc->buf_size; // / 8 ???
  if(buf == NULL || buf_len == 0U) {
    LOG_ERR("Display buffer is not available");
    return -1;
  }

  if(desc->pitch > desc->width) {
    LOG_ERR("Unsupported mode");
    return -1;
  }

  // if((y & 0x7) != 0U) {
  //   LOG_ERR("Unsupported origin y %u", y);
  //   return -1;
  // }

  /* Check we do not overflow x or y from buffer display_buffer_descriptor */
  struct display_capabilities capabilities;
  display_get_capabilities(dev, &capabilities);

  if((x + desc->width) > capabilities.x_resolution) {
    LOG_ERR("Unsupported origin x %u with buffer width setled to %u (screen overflow)", x, desc->width);
    return -1;
  }

  if((y + desc->height) > capabilities.y_resolution) {
    LOG_ERR("Unsupported origin y %u with buffer height setled to %u (screen overflow)", y, desc->height);
    return -1;
  }

  LOG_DBG("x %u, y %u, pitch %u, width %u, height %u, buf_len %u",
    x, y, desc->pitch, desc->width, desc->height, buf_len);

  BYTE x_range[] = {SSD1333_DATA_ADDR_RANGE_X[0] + x, SSD1333_DATA_ADDR_RANGE_X[0] + x + (desc->width - 1)};
  BYTE y_range[] = {SSD1333_DATA_ADDR_RANGE_Y[0] + y, SSD1333_DATA_ADDR_RANGE_Y[0] + y + (desc->height - 1)};

  // if(x_range[0] != 40 || x_range[1] != 135 || y_range[0] != 55 || y_range[1] != 75) {
  //   return 0;
  // }

  // uint64_t hash = 0;

  // for(size_t i = 0; i < buf_len; i++) {
  //   hash += ((BYTE *)buf)[i];
  // }

  // LOG_WRN("x: %u-%u | y: %u-%u -> hash %u", x_range[0], x_range[1], y_range[0], y_range[1], hash);

  // uint32_t total_rows = y2 - y1;
  // uint32_t total_cols = x2 - x1;
 
  if(ssd1333_write_bus(dev, SSD1333_CMD_SETCOLUMN, x_range, sizeof(x_range)) || ssd1333_write_bus(dev, SSD1333_CMD_SETROW, y_range, sizeof(y_range))) {
    LOG_ERR("Failed to write command");
    return -1;
  }

  return ssd1333_write_bus(dev, SSD1333_CMD_WRITEINTORAM, (BYTE *)buf, buf_len);
}

static int ssd1333_read(const struct device *dev, const uint16_t x,
                        const uint16_t y,
                        const struct display_buffer_descriptor *desc,
                        void *buf)
{
  // printk("[SSD1333] READ\n");
  LOG_ERR("Unsupported");
  return -ENOTSUP;
}

static void *ssd1333_get_framebuffer(const struct device *dev)
{
  // printk("[SSD1333] 0\n");
  LOG_ERR("Unsupported");
  return NULL;
}

static int ssd1333_set_brightness(const struct device *dev,
                                  const uint8_t brightness)
{
  // printk("[SSD1333] SET BRIGHTNESS\n");
  LOG_WRN("Unsupported");
  return -ENOTSUP;
}

static int ssd1333_set_contrast(const struct device *dev, const uint8_t contrast) {
  return ssd1333_write_bus(dev, SSD1333_CMD_CONTRAST_COLOR, SSD1333_DATA_CONTRAST_DEF, sizeof(SSD1333_DATA_CONTRAST_DEF));
}

static void ssd1333_get_capabilities(const struct device *dev, struct display_capabilities *caps) {
  // printk("[SSD1333] GET CAPABILITIES\n");
	const struct ssd1333_config *config = dev->config;

  memset(caps, 0, sizeof(struct display_capabilities));
  caps->x_resolution = config->width;
  caps->y_resolution = config->height;
  caps->supported_pixel_formats = PIXEL_FORMAT_RGB_565;
  caps->current_pixel_format = PIXEL_FORMAT_RGB_565;
  caps->screen_info = 0;
  caps->current_orientation = DISPLAY_ORIENTATION_NORMAL;
}

static int ssd1333_set_orientation(const struct device *dev,
                                   const enum display_orientation
                                       orientation)
{
  LOG_ERR("Unsupported");
  return -ENOTSUP;
}

static int ssd1333_set_pixel_format(const struct device *dev, const enum display_pixel_format pf) {
  // printk("[SSD1333] SET PX FORMAT\n");
  if (pf == PIXEL_FORMAT_MONO10) {
    return 0;
  }
  LOG_ERR("Unsupported");
  return -ENOTSUP;
}
/* End of methods definition */

static struct display_driver_api ssd1333_driver_api = {
  .blanking_on = ssd1333_blanking_on,
  .blanking_off = ssd1333_blanking_off,
  .write = ssd1333_write,
  .read = ssd1333_read,
  .get_framebuffer = ssd1333_get_framebuffer,
  .set_brightness = ssd1333_set_brightness,
  .set_contrast = ssd1333_set_contrast,
  .get_capabilities = ssd1333_get_capabilities,
  .set_pixel_format = ssd1333_set_pixel_format,
  .set_orientation = ssd1333_set_orientation
};

#define SSD1333_DEFINE(inst)                                                \
  static const struct ssd1333_config ssd1333_config_##inst = {              \
    .bus = SPI_DT_SPEC_INST_GET(                                            \
      inst, SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8), 0),    \
    .dc_gpio = GPIO_DT_SPEC_INST_GET(inst, data_ctrl_gpios),                \
    .reset_gpio = GPIO_DT_SPEC_INST_GET(inst, reset_gpios),                 \
    .width = DT_INST_PROP(inst, width),                                     \
    .height = DT_INST_PROP(inst, height),                                   \
  };                                                                        \
                                                                            \
  static struct ssd1333_data ssd1333_driver_data_##inst;                    \
                                                                            \
  DEVICE_DT_INST_DEFINE(inst,                                               \
                        ssd1333_init,                                       \
                        NULL,                                               \
                        &ssd1333_driver_data_##inst,                        \
                        &ssd1333_config_##inst,                             \
                        POST_KERNEL, CONFIG_DISPLAY_INIT_PRIORITY,          \
                        &ssd1333_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SSD1333_DEFINE)
