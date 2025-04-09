#include <Arduino.h>
#include <lvgl.h>
#include <ui.h>
#include <Arduino_GFX_Library.h>

#include <WiFi.h>
#include <esp_now.h>

#define TFT_BL 2

Arduino_ESP32RGBPanel *bus = new Arduino_ESP32RGBPanel(
    GFX_NOT_DEFINED, GFX_NOT_DEFINED, GFX_NOT_DEFINED,
    41, 40, 39, 42,
    14, 21, 47, 48, 45,
    9, 46, 3, 8, 16, 1,
    15, 7, 6, 5, 4);

Arduino_RPi_DPI_RGBPanel *gfx = new Arduino_RPi_DPI_RGBPanel(
    bus,
    800, 0, 210, 30, 16,
    480, 0, 22, 13, 10,
    1, 12000000, true);

#include "touch.h"

static uint32_t screenWidth;
static uint32_t screenHeight;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *disp_draw_buf;
static lv_disp_drv_t disp_drv;

// Distance 상세 차트용 전역 변수
lv_obj_t *ui_DistanceDetailScreen;
lv_obj_t *distanceChart;
lv_chart_series_t *distanceSeries;

// 센서 박스 및 값 표시용 라벨
lv_obj_t *sensorBoxes[8];
lv_obj_t *sensorValues[8];
lv_obj_t *sensorLabels[8];
unsigned long sensorLastUpdate[8]; // 센서별 마지막 수신 시각

#if LV_USE_LOG != 0
void my_print(const char *buf)
{
  Serial.printf(buf);
  Serial.flush();
}
#endif

void ui_DistanceDetailScreen_screen_init()
{
  ui_DistanceDetailScreen = lv_obj_create(NULL);
  lv_obj_clear_flag(ui_DistanceDetailScreen, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t *back_btn = lv_btn_create(ui_DistanceDetailScreen);
  lv_obj_set_size(back_btn, 60, 40);
  lv_obj_align(back_btn, LV_ALIGN_TOP_LEFT, 10, 10);
  lv_obj_t *back_label = lv_label_create(back_btn);
  lv_label_set_text(back_label, "←");
  lv_obj_center(back_label);
  lv_obj_add_event_cb(back_btn, [](lv_event_t *e)
                      { lv_scr_load(ui_Screen1); }, LV_EVENT_CLICKED, NULL);

  distanceChart = lv_chart_create(ui_DistanceDetailScreen);
  lv_obj_set_size(distanceChart, 700, 300);
  lv_obj_align(distanceChart, LV_ALIGN_CENTER, 0, 30);

  lv_chart_set_type(distanceChart, LV_CHART_TYPE_LINE);
  lv_chart_set_range(distanceChart, LV_CHART_AXIS_PRIMARY_Y, 0, 500);
  lv_chart_set_point_count(distanceChart, 50);
  lv_chart_set_update_mode(distanceChart, LV_CHART_UPDATE_MODE_SHIFT);

  distanceSeries = lv_chart_add_series(distanceChart, lv_palette_main(LV_PALETTE_ORANGE), LV_CHART_AXIS_PRIMARY_Y);
}

char sensorDataBuffer[64];
volatile bool sensorDataUpdated = false;

typedef struct struct_message
{
  int device_id;
  int sensor_type;
  union
  {
    struct
    {
      int distance_mm;
    } distance;
    struct
    {
      float temperature, humidity;
    } th;
    struct
    {
      float pitch, roll, yaw;
    } imu;
    struct
    {
      int bpm;
    } heart;
  } data;
} struct_message;

struct_message incomingData;

void onReceive(const uint8_t *mac_addr, const uint8_t *data, int len)
{
  if (len == sizeof(struct_message))
  {
    memcpy(&incomingData, data, sizeof(struct_message));
    if (incomingData.sensor_type == 0)
    {
      snprintf(sensorDataBuffer, sizeof(sensorDataBuffer), "%d mm", incomingData.data.distance.distance_mm);
    }
    else
    {
      snprintf(sensorDataBuffer, sizeof(sensorDataBuffer), "Other sensor type: %d", incomingData.sensor_type);
    }
    sensorDataUpdated = true;
    Serial.printf("📅 [%d] 거리 수신: %s\n", incomingData.device_id, sensorDataBuffer);
  }
  else
  {
    Serial.printf("❌ 알 수 없는 데이터 (크기: %d)\n", len);
  }
}

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

void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
  if (touch_has_signal())
  {
    if (touch_touched())
    {
      data->state = LV_INDEV_STATE_PR;
      data->point.x = touch_last_x;
      data->point.y = touch_last_y;
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

unsigned long main_t = 0;

void setup()
{
  Serial.begin(115200);
  String LVGL_Arduino = "Hello Arduino! ";
  LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();
  Serial.println(LVGL_Arduino);
  Serial.println("I am LVGL_Arduino");

#if LV_USE_LOG != 0
  lv_log_register_print_cb(my_print);
#endif

  gfx->begin();
#ifdef TFT_BL
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  ledcSetup(0, 300, 8);
  ledcAttachPin(TFT_BL, 0);
  ledcWrite(0, 255);
#endif

  lv_init();

  pinMode(TOUCH_GT911_RST, OUTPUT);
  digitalWrite(TOUCH_GT911_RST, LOW);
  delay(10);
  digitalWrite(TOUCH_GT911_RST, HIGH);
  delay(10);
  touch_init();

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
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);

    WiFi.mode(WIFI_STA);
    delay(100);

    if (esp_now_init() != ESP_OK)
    {
      Serial.println("ESP-NOW 초기화 실패!");
      return;
    }

    esp_now_register_recv_cb(onReceive);
    Serial.println("ESP-NOW 초기화 완료, 데이터 수신 대기중...");
    Serial.println("Setup done");
  }

  ui_init();
}

void loop() {
  lv_timer_handler();

  if (sensorDataUpdated) {
    int id = incomingData.device_id;
    if (id >= 0 && id < 8 && sensorValues[id]) {
      lv_label_set_text(sensorValues[id], sensorDataBuffer);
      lv_obj_set_style_bg_color(sensorBoxes[id], lv_palette_main(LV_PALETTE_GREEN), LV_PART_MAIN);
      sensorLastUpdate[id] = millis();

      // 센서 종류 표시
      if (sensorLabels[id]) {
        const char *typeText = "Unknown";
        switch (incomingData.sensor_type) {
          case 0: typeText = "Distance"; break;
          case 1: typeText = "Temp/Humid"; break;
          case 2: typeText = "IMU"; break;
          case 3: typeText = "Heart"; break;
        }
        lv_label_set_text(sensorLabels[id], typeText);
        Serial.printf("센서 [%d] 타입: %s\n", id, typeText);
      }

    } else {
      Serial.printf("❗ device_id %d에 대한 레이블 없음\n", id);
    }

    int distanceVal = 0;
    sscanf(sensorDataBuffer, "%d", &distanceVal);

    if (distanceChart && distanceSeries) {
      lv_chart_set_next_value(distanceChart, distanceSeries, distanceVal);
      lv_chart_refresh(distanceChart);
    }

    sensorDataUpdated = false;
  }

  // 타임아웃 검사
  unsigned long now = millis();
  const unsigned long TIMEOUT_MS = 3000;
  for (int i = 0; i < 8; i++) {
    if (sensorValues[i] && (now - sensorLastUpdate[i] > TIMEOUT_MS)) {
      lv_obj_set_style_bg_color(sensorBoxes[i], lv_palette_main(LV_PALETTE_GREY), LV_PART_MAIN);
      lv_label_set_text(sensorValues[i], "Offline");
    }
  }

  delay(5);
}