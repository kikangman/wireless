#include <Arduino.h>
#include <lvgl.h>
#include <ui.h>
#include <Arduino_GFX_Library.h>

#include <WiFi.h>
#include <esp_now.h>

#define TFT_BL 2

// #define GFX_BL DF_GFX_BL

Arduino_ESP32RGBPanel *bus = new Arduino_ESP32RGBPanel(
    GFX_NOT_DEFINED /* CS */, GFX_NOT_DEFINED /* SCK */, GFX_NOT_DEFINED /* SDA */,
    41 /* DE */, 40 /* VSYNC */, 39 /* HSYNC */, 42 /* PCLK */,
    14 /* R0 */, 21 /* R1 */, 47 /* R2 */, 48 /* R3 */, 45 /* R4 */,
    9 /* G0 */, 46 /* G1 */, 3 /* G2 */, 8 /* G3 */, 16 /* G4 */, 1 /* G5 */,
    15 /* B0 */, 7 /* B1 */, 6 /* B2 */, 5 /* B3 */, 4 /* B4 */
);

Arduino_RPi_DPI_RGBPanel *gfx = new Arduino_RPi_DPI_RGBPanel(
    bus,
    //  800 /* width */, 0 /* hsync_polarity */, 8/* hsync_front_porch */, 2 /* hsync_pulse_width */, 43/* hsync_back_porch */,
    //  480 /* height */, 0 /* vsync_polarity */, 8 /* vsync_front_porch */, 2/* vsync_pulse_width */, 12 /* vsync_back_porch */,
    //  1 /* pclk_active_neg */, 16000000 /* prefer_speed */, true /* auto_flush */);

    800 /* width */, 0 /* hsync_polarity */, 210 /* hsync_front_porch */, 30 /* hsync_pulse_width */, 16 /* hsync_back_porch */,
    480 /* height */, 0 /* vsync_polarity */, 22 /* vsync_front_porch */, 13 /* vsync_pulse_width */, 10 /* vsync_back_porch */,
    1 /* pclk_active_neg */, 12000000 /* prefer_speed */, true /* auto_flush */);

#include "touch.h"

/*Change to your screen resolution*/
static uint32_t screenWidth;
static uint32_t screenHeight;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *disp_draw_buf;
static lv_disp_drv_t disp_drv;

// Distance 상세 차트용 전역 변수
lv_obj_t *ui_DistanceDetailScreen;
lv_obj_t *distanceChart;
lv_chart_series_t *distanceSeries;


#if LV_USE_LOG != 0
/* Serial debugging */
void my_print(const char *buf)
{
  Serial.printf(buf);
  Serial.flush();
}
#endif


void ui_DistanceDetailScreen_screen_init() {
  ui_DistanceDetailScreen = lv_obj_create(NULL);
  lv_obj_clear_flag(ui_DistanceDetailScreen, LV_OBJ_FLAG_SCROLLABLE);

  // 🔙 뒤로가기 버튼
  lv_obj_t *back_btn = lv_btn_create(ui_DistanceDetailScreen);
  lv_obj_set_size(back_btn, 60, 40);
  lv_obj_align(back_btn, LV_ALIGN_TOP_LEFT, 10, 10);
  lv_obj_t *back_label = lv_label_create(back_btn);
  lv_label_set_text(back_label, "←");
  lv_obj_center(back_label);
  lv_obj_add_event_cb(back_btn, [](lv_event_t *e){
    lv_scr_load(ui_Screen1);  // 메인 화면으로 돌아감
  }, LV_EVENT_CLICKED, NULL);

  // 📊 차트 생성
  distanceChart = lv_chart_create(ui_DistanceDetailScreen);
  lv_obj_set_size(distanceChart, 700, 300);
  lv_obj_align(distanceChart, LV_ALIGN_CENTER, 0, 30);

  lv_chart_set_type(distanceChart, LV_CHART_TYPE_LINE);
  lv_chart_set_range(distanceChart, LV_CHART_AXIS_PRIMARY_Y, 0, 1000);  // 거리값 범위
  lv_chart_set_point_count(distanceChart, 50);                         // 50개 포인트 표시
  lv_chart_set_update_mode(distanceChart, LV_CHART_UPDATE_MODE_SHIFT);

  distanceSeries = lv_chart_add_series(distanceChart, lv_palette_main(LV_PALETTE_ORANGE), LV_CHART_AXIS_PRIMARY_Y);
}


// 센서 데이터 저장용 전역 변수 및 플래그
char sensorDataBuffer[64];
volatile bool sensorDataUpdated = false;
void onReceive(const uint8_t *mac_addr, const uint8_t *data, int len)
{

  int copyLen = min(len, (int)(sizeof(sensorDataBuffer) - 1)); // <-- 타입 캐스팅 추가
  memcpy(sensorDataBuffer, data, copyLen);
  sensorDataBuffer[copyLen] = '\0';

  sensorDataUpdated = true;
  Serial.printf("수신된 메시지: %s\n", sensorDataBuffer);
}

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

#if (LV_COLOR_16_SWAP != 0)
  gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#else
  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#endif

  lv_disp_flush_ready(disp);
}

/*Read the touchpad*/
void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
  if (touch_has_signal())
  {
    if (touch_touched())
    {
      data->state = LV_INDEV_STATE_PR;

      /*Set the coordinates*/
      data->point.x = touch_last_x;
      data->point.y = touch_last_y;
      // Serial.print( "Data x " );
      // Serial.println( data->point.x );
      // Serial.print( "Data y " );
      // Serial.println( data->point.y );
    }
    else if (touch_released())
    {
      data->state = LV_INDEV_STATE_REL;
    }
  }
  else
  {
    data->state = LV_INDEV_STATE_REL;
  }
}

// 전역
unsigned long main_t = 0;

void setup()
{
  Serial.begin(115200); /* prepare for possible serial debug */

  // setup

  String LVGL_Arduino = "Hello Arduino! ";
  LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

  Serial.println(LVGL_Arduino);
  Serial.println("I am LVGL_Arduino");

#if LV_USE_LOG != 0
  lv_log_register_print_cb(my_print); /* register print function for debugging */
#endif

  gfx->begin();
#ifdef TFT_BL
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  ledcSetup(0, 300, 8);
  ledcAttachPin(TFT_BL, 0);
  ledcWrite(0, 255); /* Screen brightness can be modified by adjusting this parameter. (0-255) */

#endif
  lv_init();

  // Init touch device
  pinMode(TOUCH_GT911_RST, OUTPUT);
  digitalWrite(TOUCH_GT911_RST, LOW);
  delay(10);
  digitalWrite(TOUCH_GT911_RST, HIGH);
  delay(10);
  touch_init();
  //  touch.setTouch( calData );

  screenWidth = gfx->width();
  screenHeight = gfx->height();
#ifdef ESP32
  disp_draw_buf = (lv_color_t *)heap_caps_malloc(sizeof(lv_color_t) * screenWidth * screenHeight / 4, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
#else
  disp_draw_buf = (lv_color_t *)malloc(sizeof(lv_color_t) * screenWidth * screenHeight / 4);
#endif
  if (!disp_draw_buf)
  {
    Serial.println("LVGL disp_draw_buf allocate failed!");
  }
  else
  {
    lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, NULL, screenWidth * screenHeight / 4);

    /* Initialize the display */
    lv_disp_drv_init(&disp_drv);
    /* Change the following line to your display resolution */
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    /* Initialize the (dummy) input device driver */
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);

    // ESP-NOW 초기화 추가 (setup 맨 마지막에 추가)
    WiFi.mode(WIFI_STA);
    delay(100);

    if (esp_now_init() != ESP_OK)
    {
      Serial.println("ESP-NOW 초기화 실패!");
      return;
    }

    esp_now_register_recv_cb(onReceive);  // 올바르게 바뀐 콜백함수
    Serial.println("ESP-NOW 초기화 완료, 데이터 수신 대기중...");

    Serial.println("Setup done");
  }

  // 작화툴에서 생성한 LVGL코드를 실행하기 위한 코드
  ui_init();
}

void loop()
{
  lv_timer_handler(); /* let the GUI do its work */

  if (sensorDataUpdated)
  {
    lv_label_set_text(valueLabel, sensorDataBuffer); // 기존 텍스트 표시
  
    // 🔢 차트용 숫자 추출
    int distanceVal = 0;
    sscanf(sensorDataBuffer, "%d", &distanceVal); // "123 mm" -> 123
  
    // 📈 차트에 값 추가 (상세 화면일 경우만)
    if (distanceChart && distanceSeries)
    {
      lv_chart_set_next_value(distanceChart, distanceSeries, distanceVal);
      lv_chart_refresh(distanceChart);
    }
  
    sensorDataUpdated = false;
  }
  
  delay(5);
}
