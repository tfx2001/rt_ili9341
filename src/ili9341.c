/*
 * Copyright (c) 2021, tfx2001 <2479727366@qq.com>
 *
 * SPDX-License-Identifier: MIT License
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-01-24     tfx2001      the first version
 */

#include <math.h>
#include <rtdevice.h>
#include <rtthread.h>
#include <stdlib.h>
#include <string.h> // For memcpy

#include "ili9341.h"

#define DBG_TAG "pkgs.ili9341"
#ifdef PKG_USING_ILI9341_DEBUG
#define DBG_LVL DBG_LOG
#else
#define DBG_LVL DBG_ERROR
#endif
#include <rtdbg.h>

#define MADCTL_MY 0x80  ///< Bottom to top
#define MADCTL_MX 0x40  ///< Right to left
#define MADCTL_MV 0x20  ///< Reverse Mode
#define MADCTL_ML 0x10  ///< LCD refresh Bottom to top
#define MADCTL_RGB 0x00 ///< Red-Green-Blue pixel order
#define MADCTL_BGR 0x08 ///< Blue-Green-Red pixel order
#define MADCTL_MH 0x04  ///< LCD refresh right to left

static rt_err_t ili9341_device_control(rt_device_t dev, int cmd, void *args);
static void ili9341_device_blit_line(const char *pixel, int x, int y,
                                     rt_size_t size);

// clang-format off
static const uint8_t init_cmd[] = {
    0xEF, 3, 0x03, 0x80, 0x02,
    0xCF, 3, 0x00, 0xC1, 0x30,
    0xED, 4, 0x64, 0x03, 0x12, 0x81,
    0xE8, 3, 0x85, 0x00, 0x78,
    0xCB, 5, 0x39, 0x2C, 0x00, 0x34, 0x02,
    0xF7, 1, 0x20,
    0xEA, 2, 0x00, 0x00,
    ILI9341_PWCTR1  , 1, 0x23,             // Power control VRH[5:0]
    ILI9341_PWCTR2  , 1, 0x10,             // Power control SAP[2:0];BT[3:0]
    ILI9341_VMCTR1  , 2, 0x3e, 0x28,       // VCM control
    ILI9341_VMCTR2  , 1, 0x86,             // VCM control2
    ILI9341_MADCTL  , 1, 0x48,             // Memory Access Control
    ILI9341_VSCRSADD, 1, 0x00,             // Vertical scroll zero
    ILI9341_PIXFMT  , 1, 0x55,
    ILI9341_FRMCTR1 , 2, 0x00, 0x18,
    ILI9341_DFUNCTR , 3, 0x08, 0x82, 0x27, // Display Function Control
    0xF2, 1, 0x00,                         // 3Gamma Function Disable
    ILI9341_GAMMASET , 1, 0x01,             // Gamma curve selected
    ILI9341_GMCTRP1 , 15, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, // Set Gamma
      0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00,
    ILI9341_GMCTRN1 , 15, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, // Set Gamma
      0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F,
    ILI9341_SLPOUT  , 0x80,                // Exit Sleep
    ILI9341_DISPON  , 0x80,                // Display on
    0x00                                   // End of list
};
// clang-format on

struct ili9341_lcd_device {
    struct rt_device parent;

    struct rt_device_graphic_info lcd_info;

    struct rt_semaphore lcd_lock;

    struct rt_device_graphic_ops graphic_ops;

    struct rt_spi_device *spi_device;

    rt_base_t reset_pin;
    rt_base_t dc_pin;
};

static struct ili9341_lcd_device _lcd;

void ili9341_send_command_data(uint8_t cmd, uint8_t *data, rt_size_t data_len) {
    /* Set D/C pin to 0 */
    rt_pin_write(_lcd.dc_pin, PIN_LOW);
    rt_spi_send(_lcd.spi_device, &cmd, 1);

    if (data_len) {
        rt_pin_write(_lcd.dc_pin, PIN_HIGH);
        rt_spi_send(_lcd.spi_device, data, data_len);
    }
}

void ili9341_send_data(uint8_t *data, rt_size_t data_len) {
    rt_pin_write(_lcd.dc_pin, PIN_HIGH);
    rt_spi_send(_lcd.spi_device, data, data_len);
}

void ili9341_reset(void) {
    rt_pin_write(_lcd.reset_pin, PIN_LOW);
    rt_thread_mdelay(20);
    rt_pin_write(_lcd.reset_pin, PIN_HIGH);
    rt_thread_mdelay(200);
}

rt_err_t ili9341_init(const char *dev_name, rt_base_t dc_pin,
                      rt_base_t reset_pin) {
    rt_err_t result = RT_EOK;
    struct rt_device *device = &_lcd.parent;

    _lcd.spi_device = (struct rt_spi_device *)rt_device_find(dev_name);
    if (_lcd.spi_device == RT_NULL) {
        LOG_E("can not find %s device", dev_name);
        return -RT_ERROR;
    }

    _lcd.reset_pin = reset_pin;
    _lcd.dc_pin = dc_pin;
    rt_pin_mode(_lcd.reset_pin, PIN_MODE_OUTPUT);
    rt_pin_mode(_lcd.dc_pin, PIN_MODE_OUTPUT);
    ili9341_reset();

    /* init ILI9341 */
    uint8_t cmd, x, num_args;
    uint8_t *addr = init_cmd;
    while ((cmd = *(addr++)) > 0) {
        x = *(addr++);
        num_args = x & 0x7F;
        ili9341_send_command_data(cmd, addr, num_args);
        addr += num_args;
        if (x & 0x80)
            rt_thread_mdelay(150);
    }

    /* init lcd_lock semaphore */
    result = rt_sem_init(&_lcd.lcd_lock, "lcd_lock", 0, RT_IPC_FLAG_FIFO);
    if (result != RT_EOK) {
        LOG_E("init semaphore failed!\n");
        result = -RT_ENOMEM;
        goto __exit;
    }

    /* config LCD dev info */
    _lcd.lcd_info.height = 320;
    _lcd.lcd_info.width = 240;
    _lcd.lcd_info.bits_per_pixel = 16;
    _lcd.lcd_info.pixel_format = RTGRAPHIC_PIXEL_FORMAT_RGB565;
    _lcd.lcd_info.framebuffer = RT_NULL;

    device->type = RT_Device_Class_Graphic;
#ifdef RT_USING_DEVICE_OPS
    device->ops = &lcd_ops;
#else
    device->init = RT_NULL;
    device->control = ili9341_device_control;

    _lcd.graphic_ops.blit_line = ili9341_device_blit_line;
    _lcd.parent.user_data = (void *)&_lcd.graphic_ops;
#endif

    /* register lcd device */
    rt_device_register(device, "lcd", RT_DEVICE_FLAG_RDWR);

__exit:

    return result;
}

rt_err_t ili9341_device_control(rt_device_t dev, int cmd, void *args) {
    switch (cmd) {
    case RTGRAPHIC_CTRL_GET_INFO:
        RT_ASSERT(args != RT_NULL);
        *(struct rt_device_graphic_info *)args = _lcd.lcd_info;
        break;
    }
}

void ili9341_set_rotation(uint8_t m) {
    uint8_t rotation;

    rotation = m % 4; // can't be higher than 3
    switch (rotation) {
    case 0:
        m = (MADCTL_MX | MADCTL_BGR);
        break;
    case 1:
        m = (MADCTL_MV | MADCTL_BGR);
        break;
    case 2:
        m = (MADCTL_MY | MADCTL_BGR);
        break;
    case 3:
        m = (MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR);
        break;
    }

    ili9341_send_command_data(ILI9341_MADCTL, &m, 1);
}

void ili9341_set_addrwindow(uint16_t x1, uint16_t y1, uint16_t w, uint16_t h) {
    uint16_t x2 = (x1 + w - 1);
    uint16_t y2 = (y1 + h - 1);
    uint8_t data_buf[4];

    data_buf[0] = (uint8_t)(x1 >> 8);
    data_buf[1] = (uint8_t)(x1 & 0xFF);
    data_buf[2] = (uint8_t)(x2 >> 8);
    data_buf[3] = (uint8_t)(x2 & 0xFF);
    ili9341_send_command_data(ILI9341_CASET, data_buf, 4); // Column address set

    data_buf[0] = (uint8_t)(y1 >> 8);
    data_buf[1] = (uint8_t)(y1 & 0xFF);
    data_buf[2] = (uint8_t)(y2 >> 8);
    data_buf[3] = (uint8_t)(y2 & 0xFF);
    ili9341_send_command_data(ILI9341_PASET, data_buf, 4); // Row address set
    ili9341_send_command_data(ILI9341_RAMWR, RT_NULL, 0);  // Write to RAM
}

void ili9341_flush(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                   uint8_t *data, rt_size_t data_len) {
    ili9341_set_addrwindow(x, y, w, h);
    ili9341_send_data(data, data_len);
}

void ili9341_draw_pixel(uint16_t x, uint16_t y, uint16_t color) {
    uint16_t color_swap = (color << 8) | (color >> 8);
    ili9341_flush(x, y, 1, 1, (uint8_t *)&color_swap, 2);
}

void ili9341_device_blit_line(const char *pixel, int x, int y, rt_size_t size) {
    ili9341_flush(x, y, size, 1, pixel, size * 2);
}