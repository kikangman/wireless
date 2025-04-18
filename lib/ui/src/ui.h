// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.4
// LVGL version: 8.3.6
// Project name: esp32_7inch

#ifndef _ESP32_7INCH_UI_H
#define _ESP32_7INCH_UI_H

#ifdef __cplusplus
extern "C"
{
#endif

#if defined __has_include
#if __has_include("lvgl.h")
#include "lvgl.h"
#elif __has_include("lvgl/lvgl.h")
#include "lvgl/lvgl.h"
#else
#include "lvgl.h"
#endif
#else
#include "lvgl.h"
#endif


#ifdef __cplusplus
extern "C" {
#endif

void ui_DistanceDetailScreen_screen_init(void);

// 이 변수는 main.cpp에 정의되어 있고 다른 파일에서 사용할 수 있게 extern으로 선언
extern lv_obj_t *ui_DistanceDetailScreen;

void distance_box_event_cb(lv_event_t *e);

#ifdef __cplusplus
}
#endif



#include "ui_helpers.h"
#include "components/ui_comp.h"
#include "components/ui_comp_hook.h"
#include "ui_events.h"
    // SCREEN: ui_Screen1
    void ui_Screen1_screen_init(void);
    extern lv_obj_t *ui_Screen1;
    extern lv_obj_t *ui_Label1;
    extern lv_obj_t *ui____initial_actions0;
    extern lv_obj_t *valueLabel;  // 센서 값 표시용 전역 라벨 추가
    extern lv_obj_t *valueLabel2; // 두 번째 센서값 표시용 라벨

    extern const lv_font_t lv_font_montserrat_20;
    extern const lv_font_t lv_font_montserrat_24;
    extern const lv_font_t lv_font_montserrat_32;

    LV_FONT_DECLARE(ui_font_Font1);
    LV_FONT_DECLARE(ui_font_Font2);

    void ui_init(void);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
