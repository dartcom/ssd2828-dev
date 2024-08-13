#ifndef DCS_H
#define DCS_H


#define DCS_enter_idle_mode         0x39
#define DCS_enter_invert_mode       0x21
#define DCS_enter_normal_mode       0x13
#define DCS_enter_partial_mode      0x12
#define DCS_enter_sleep_mode        0x10
#define DCS_exit_idle_mode          0x38
#define DCS_exit_invert_mode        0x20
#define DCS_exit_sleep_mode         0x11
#define DCS_get_address_mode        0x0B
#define DCS_get_blue_channel        0x08
#define DCS_get_diagnostic_result   0x0F
#define DCS_get_display_mode        0x0D
#define DCS_get_green_channel       0x07
#define DCS_get_pixel_format        0x0C
#define DCS_get_power_mode          0x0A
#define DCS_get_red_channel         0x06
#define DCS_get_scanline            0x45
#define DCS_get_signal_mode         0x0E
#define DCS_nop                     0x00
#define DCS_read_DDB_continue       0xA8
#define DCS_read_DDB_start          0xA1
#define DCS_read_memory_continue    0x3E
#define DCS_read_memory_start       0x2E
#define DCS_set_address_mode        0x36
#define DCS_set_column_address      0x2A
#define DCS_set_display_off         0x28
#define DCS_set_display_on          0x29
#define DCS_set_gamma_curve         0x26
#define DCS_set_page_address        0x2B
#define DCS_set_partial_columns     0x31
#define DCS_set_partial_rows        0x30
#define DCS_set_pixel_format        0x3A
#define DCS_set_scroll_area         0x33
#define DCS_set_scroll_start        0x37
#define DCS_set_tear_off            0x34
#define DCS_set_tear_on             0x35
#define DCS_set_tear_scanline       0x44
#define DCS_soft_reset              0x01
#define DCS_write_LUT               0x2D
#define DCS_write_memory_continue   0x3C
#define DCS_write_memory_start      0x2C


#endif