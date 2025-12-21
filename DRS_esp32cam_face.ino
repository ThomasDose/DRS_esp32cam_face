#include <Arduino.h>
#include <list>

#include "esp_camera.h"
#include "human_face_detect_msr01.hpp"

// ---------------------------
// AI-Thinker ESP32-CAM pin map (OV2640)
// ---------------------------
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// ---------------------------
// Motor GPIO mapping (as you defined)
// Vertical: GPIO22/GPIO27
// Horizontal: GPIO23/GPIO24
// ---------------------------
static const int PIN_V1 = 13;  // UP
static const int PIN_V2 = 12;  // DOWN
static const int PIN_H1 = 14;  // LEFT
static const int PIN_H2 = 15;  // RIGH

// ---------------------------
// Performance / behavior knobs
// ---------------------------
static const framesize_t FRAME_SIZE = FRAMESIZE_QVGA;     // 320x240 recommended (try 240x240 if needed)
static const pixformat_t PIXFORMAT  = PIXFORMAT_RGB565;   // detector-friendly (no JPEG decode)
static const int         XCLK_FREQ  = 20000000;

static const int DETECT_EVERY_N = 2;          // run detector every Nth frame
static const int DEADZONE_X = 18;             // pixels around center = no horizontal move
static const int DEADZONE_Y = 18;             // pixels around center = no vertical move
static const int HYSTERESIS = 6;              // prevents chatter (enter/exit band)

// Detector strictness (less “sensitive” than super permissive)
static HumanFaceDetectMSR01 detector(0.15F, 0.5F, 10, 0.2F);

// ---------------------------
// State
// ---------------------------
enum AxisCmd : int8_t { AX_STOP = 0, AX_NEG = -1, AX_POS = +1 }; // H: NEG=LEFT POS=RIGHT, V: NEG=UP POS=DOWN

struct MotorState {
  AxisCmd h;  // horizontal
  AxisCmd v;  // vertical
  bool operator==(const MotorState& o) const { return h == o.h && v == o.v; }
  bool operator!=(const MotorState& o) const { return !(*this == o); }
};

static uint32_t frameNo = 0;

static MotorState lastOut = {AX_STOP, AX_STOP};
static bool lastHadFace = false;
static int lastCx = 0, lastCy = 0;

static uint32_t lastSignalMs = 0;

// ---------------------------
// Helpers
// ---------------------------
static bool init_camera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;

  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;

  config.pin_xclk  = XCLK_GPIO_NUM;
  config.pin_pclk  = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href  = HREF_GPIO_NUM;

  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;

  config.pin_pwdn  = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = XCLK_FREQ;

  config.pixel_format = PIXFORMAT;
  config.frame_size   = FRAME_SIZE;

  config.jpeg_quality = 12;                 // ignored for RGB565
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.grab_mode    = CAMERA_GRAB_LATEST;
  config.fb_count     = 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("CAM init failed: 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_framesize(s, FRAME_SIZE);
    s->set_pixformat(s, PIXFORMAT);

    // neutral image settings
    s->set_brightness(s, 0);
    s->set_contrast(s, 0);
    s->set_saturation(s, 0);
    s->set_gain_ctrl(s, 1);
    s->set_exposure_ctrl(s, 1);
    s->set_awb_gain(s, 1);
  }

  return true;
}

static void motor_apply(const MotorState &st) {
  // Vertical
  if (st.v == AX_NEG) {            // UP
    digitalWrite(PIN_V1, HIGH);
    digitalWrite(PIN_V2, LOW);
  } else if (st.v == AX_POS) {     // DOWN
    digitalWrite(PIN_V1, LOW);
    digitalWrite(PIN_V2, HIGH);
  } else {                         // STOP
    digitalWrite(PIN_V1, LOW);
    digitalWrite(PIN_V2, LOW);
  }

  // Horizontal
  if (st.h == AX_NEG) {            // LEFT
    digitalWrite(PIN_H1, HIGH);
    digitalWrite(PIN_H2, LOW);
  } else if (st.h == AX_POS) {     // RIGHT
    digitalWrite(PIN_H1, LOW);
    digitalWrite(PIN_H2, HIGH);
  } else {                         // STOP
    digitalWrite(PIN_H1, LOW);
    digitalWrite(PIN_H2, LOW);
  }
}

static const char* axis_to_str(AxisCmd c, bool horizontal) {
  if (c == AX_STOP) return "STOP";
  if (horizontal) return (c == AX_NEG) ? "LEFT" : "RIGHT";
  return (c == AX_NEG) ? "UP" : "DOWN";
}

static void bestFace(std::list<dl::detect::result_t> &results,
                     bool &hasFace, int &x, int &y, int &w, int &h) {
  hasFace = false;
  int bestArea = 0;

  for (auto &r : results) {
    int b0 = r.box[0], b1 = r.box[1], b2 = r.box[2], b3 = r.box[3];

    int xx, yy, ww, hh;
    // handle both common formats
    if (b2 > b0 && b3 > b1) {        // x1,y1,x2,y2
      xx = b0; yy = b1; ww = b2 - b0; hh = b3 - b1;
    } else {                         // x,y,w,h
      xx = b0; yy = b1; ww = b2; hh = b3;
    }

    if (ww <= 0 || hh <= 0) continue;
    int area = ww * hh;
    if (area > bestArea) {
      bestArea = area;
      x = xx; y = yy; w = ww; h = hh;
      hasFace = true;
    }
  }
}

// Hysteresis decision: uses last command to reduce chatter
static AxisCmd decide_axis(int delta, int deadzone, AxisCmd lastCmd) {
  // delta: (cx - centerX) or (cy - centerY)
  int enter = deadzone;
  int exit  = deadzone - HYSTERESIS;  // smaller band to keep current movement

  if (lastCmd == AX_STOP) {
    if (delta < -enter) return AX_NEG;
    if (delta >  enter) return AX_POS;
    return AX_STOP;
  } else if (lastCmd == AX_NEG) {
    // keep moving NEG until we are close enough to center
    if (delta > -exit) return AX_STOP;
    return AX_NEG;
  } else { // AX_POS
    if (delta <  exit) return AX_STOP;
    return AX_POS;
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  setCpuFrequencyMhz(240);

  pinMode(PIN_V1, OUTPUT);
  pinMode(PIN_V2, OUTPUT);
  pinMode(PIN_H1, OUTPUT);
  pinMode(PIN_H2, OUTPUT);
  motor_apply({AX_STOP, AX_STOP});

  Serial.printf("PSRAM: %s size=%u free=%u\n",
                psramFound() ? "YES" : "NO",
                ESP.getPsramSize(),
                ESP.getFreePsram());

  if (!init_camera()) {
    Serial.println("Camera init FAILED.");
    while (true) delay(1000);
  }

  lastSignalMs = millis();
  Serial.println("OK. Motor timing mode active. Serial prints ONLY on motor state changes.");
}

void loop() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return;

  frameNo++;
  bool doDetect = ((frameNo % DETECT_EVERY_N) == 0);

  bool hasFace = lastHadFace;
  int x=0,y=0,w=0,h=0;

  if (doDetect) {
    std::list<dl::detect::result_t> &results =
      detector.infer((uint16_t*)fb->buf, {(int)fb->height, (int)fb->width, 3});

    bestFace(results, hasFace, x, y, w, h);

    if (hasFace) {
      lastCx = x + w/2;
      lastCy = y + h/2;
    }
    lastHadFace = hasFace;
  }

  // derive desired motor state from last known face (only if face currently present)
  MotorState desired = lastOut;

  if (lastHadFace) {
    int centerX = fb->width  / 2;
    int centerY = fb->height / 2;

    int dx = lastCx - centerX;  // + => face right of center
    int dy = lastCy - centerY;  // + => face below center

    desired.h = decide_axis(dx, DEADZONE_X, lastOut.h);
    desired.v = decide_axis(dy, DEADZONE_Y, lastOut.v);
  } else {
    // no face: do NOT emit repeated STOP.
    desired = lastOut; // keep whatever we had; we only stop once when face is lost.
    if (lastOut.h != AX_STOP || lastOut.v != AX_STOP) {
      desired = {AX_STOP, AX_STOP}; // stop once on face loss
    }
  }

  esp_camera_fb_return(fb);

  // Only output GPIO + timing if motor state changes
  if (desired != lastOut) {
    uint32_t now = millis();
    uint32_t dt  = now - lastSignalMs;
    lastSignalMs = now;

    motor_apply(desired);

    // Print ONLY on motor signal output
    // Format: DT(ms) H=... V=... cx cy (or -1 -1)
    if (lastHadFace) {
      Serial.printf("DT %lu  H=%s  V=%s  FACE %d %d\n",
                    (unsigned long)dt,
                    axis_to_str(desired.h, true),
                    axis_to_str(desired.v, false),
                    lastCx, lastCy);
    } else {
      Serial.printf("DT %lu  H=%s  V=%s  FACE -1 -1\n",
                    (unsigned long)dt,
                    axis_to_str(desired.h, true),
                    axis_to_str(desired.v, false));
    }

    lastOut = desired;
  }
}