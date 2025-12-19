/**
 * @file lv_conf.h
 * Configuration file for LVGL v9.4.0
 */

#ifndef LV_CONF_H
#define LV_CONF_H

#include <stdint.h>

/*====================
   COLOR SETTINGS
 *====================*/

/*Color depth: 1 (1 byte per pixel), 8 (RGB332), 16 (RGB565), 32 (XRGB8888)*/
#define LV_COLOR_DEPTH 16

/*====================
   MEMORY SETTINGS
 *====================*/

/*1: use custom malloc/free, 0: use the built-in `lv_malloc_builtin()` and `lv_free_builtin()`*/
#define LV_USE_BUILTIN_MALLOC 1

/*Size of the memory available for `lv_malloc()` in bytes (>= 2kB)*/
#define LV_MEM_SIZE (128U * 1024U)          /*[bytes]*/

/*Set an address for the memory pool instead of allocating it as a normal array. Can be in external SRAM too.*/
#define LV_MEM_ADR 0     /*0: unused*/

/*====================
   HAL SETTINGS
 *====================*/

/*Default Dot Per Inch. Used to initialize default sizes such as widgets sized, style paddings.*/
#define LV_DPI_DEF 130     /*[px/inch]*/

/*====================
   FONT USAGE
 *====================*/

/*Montserrat fonts with ASCII range and some symbols*/
#define LV_FONT_MONTSERRAT_8  0
#define LV_FONT_MONTSERRAT_10 0
#define LV_FONT_MONTSERRAT_12 0
#define LV_FONT_MONTSERRAT_14 1
#define LV_FONT_MONTSERRAT_16 0
#define LV_FONT_MONTSERRAT_18 0
#define LV_FONT_MONTSERRAT_20 0
#define LV_FONT_MONTSERRAT_22 0
#define LV_FONT_MONTSERRAT_24 0
#define LV_FONT_MONTSERRAT_26 0
#define LV_FONT_MONTSERRAT_28 0
#define LV_FONT_MONTSERRAT_30 0
#define LV_FONT_MONTSERRAT_32 0
#define LV_FONT_MONTSERRAT_34 0
#define LV_FONT_MONTSERRAT_36 0
#define LV_FONT_MONTSERRAT_38 0
#define LV_FONT_MONTSERRAT_40 0
#define LV_FONT_MONTSERRAT_42 0
#define LV_FONT_MONTSERRAT_44 0
#define LV_FONT_MONTSERRAT_46 0
#define LV_FONT_MONTSERRAT_48 0

/*Demonstrate special features*/
#define LV_FONT_MONTSERRAT_28_COMPRESSED 0  /*bpp = 3*/
#define LV_FONT_DEJAVU_16_PERSIAN_HEBREW 0  /*Hebrew, Arabic, Persian letters*/
#define LV_FONT_SIMSUN_16_CJK            0  /*1000 most common CJK radicals*/

/*Pixel perfect monospace fonts*/
#define LV_FONT_UNSCII_8  0
#define LV_FONT_UNSCII_16 0

/*====================
   THEME USAGE
 *====================*/

/*A simple, impressive and very complete theme*/
#define LV_USE_THEME_DEFAULT 1
#if LV_USE_THEME_DEFAULT
    /*0: Light mode; 1: Dark mode*/
    #define LV_THEME_DEFAULT_DARK 0
    /*1: Enable grow on press*/
    #define LV_THEME_DEFAULT_GROW 1
    /*Default transition time in [ms]*/
    #define LV_THEME_DEFAULT_TRANSITION_TIME 80
#endif /*LV_USE_THEME_DEFAULT*/

/*A very simple theme that is a good starting point for a custom theme*/
#define LV_USE_THEME_SIMPLE 1

/*A theme designed for monochrome displays*/
#define LV_USE_THEME_MONO 1

/*====================
   WIDGETS
 *====================*/

#define LV_USE_LABEL    1
#define LV_USE_BUTTON   1
#define LV_USE_IMAGE    1
#define LV_USE_ARC      1
#define LV_USE_BAR      1
#define LV_USE_CALENDAR 0
#define LV_USE_CHART    0
#define LV_USE_CHECKBOX 1
#define LV_USE_DROPDOWN 1
#define LV_USE_KEYBOARD 0
#define LV_USE_LED      1
#define LV_USE_LINE     1
#define LV_USE_LIST     1
#define LV_USE_MSGBOX   1
#define LV_USE_ROLLER   1
#define LV_USE_SLIDER   1
#define LV_USE_SWITCH   1
#define LV_USE_TABLE    0
#define LV_USE_TEXTAREA 1

/*====================
   OTHERS
 *====================*/

/*1: Enable API to take snapshot for object*/
#define LV_USE_SNAPSHOT 0

/*1: Enable system monitor component*/
#define LV_USE_SYSMON   0

/*1: Enable the runtime performance profiler*/
#define LV_USE_PERF_MONITOR 0

/*1: Enable all log levels*/
#define LV_USE_LOG 1
#if LV_USE_LOG
    /*How important log should be added:
     *LV_LOG_LEVEL_TRACE       A lot of logs to give detailed information
     *LV_LOG_LEVEL_INFO        Log important events
     *LV_LOG_LEVEL_WARN        Log if something unwanted happened but didn't cause a problem
     *LV_LOG_LEVEL_ERROR       Only critical issue, when the system may fail
     *LV_LOG_LEVEL_USER        Only logs added by the user
     *LV_LOG_LEVEL_NONE        Do not log anything*/
    #define LV_LOG_LEVEL LV_LOG_LEVEL_WARN
    
    /*1: Print the log with 'printf';
     *0: User need to register a callback with `lv_log_register_print_cb()`*/
    #define LV_LOG_PRINTF 1
#endif  /*LV_USE_LOG*/

#endif /*LV_CONF_H*/