// terraPen status lighting  -- Seeed XIAO ESP32C3 version
//
// Listens to FluidNC's telnet channel and drives a WS2812B strip by machine state.
// Uses Adafruit_NeoPixel rather than FastLED: FastLED's RMT path on the C3 has
// known pixel-offset bugs and can corrupt output once WiFi is active.
//
// Board: Seeed XIAO ESP32C3 (Arduino: "XIAO_ESP32C3")
//   XIAO pin D2 == GPIO4. If you're on a different Seeed C3 board, just pick any
//   free GPIO and set DATA_PIN accordingly.
//
// IMPORTANT: plug in the u.FL external antenna. The XIAO C3's WiFi range without
// it is poor, and a flaky link here means the lights silently stop telling the truth.
//
// Wiring:
//   GPIO4 -> ~330R -> strip DIN
//   Strip 5V from a SEPARATE supply -- NOT the XIAO's 5V pad
//   Strip GND tied to XIAO GND
//   1000uF cap across strip 5V/GND
//
// Libraries: Adafruit_NeoPixel

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>
#include "secrets.h"   // defines WIFI_SSID / WIFI_PASS -- gitignored, copy from secrets.h.example

// ---------- config ----------
const char* FLUID_HOST  = "terrapen";   // hostname without .local
const uint16_t FLUID_PORT = 23;
const int REPORT_INTERVAL_MS = 50;

#define DATA_PIN    4      // XIAO D2
#define NUM_LEDS    39
#define BTN_PIN     9      // XIAO BOOT button (GPIO9, strapping pin -- fine at runtime)

// Position-follow (M_RUN comet): strip is 39 LEDs @ 16mm pitch = 608mm,
// mounted along a 660mm rail. Machine X travel we track is 0..600mm, so
// ~30mm (~2 LEDs) at each end never lights up.
const float LED_PITCH_MM      = 16.0f;
const float X_TRAVEL_MM       = 600.0f;
const int   STRIP_MARGIN_LEDS = 2;

// Calibration only -- the taper itself is symmetric by construction (see
// renderComet). If the lit LEDs sit consistently to one side of the real
// tool head, the strip's physical mounting doesn't line up with the assumed
// 0..X_TRAVEL_MM window; nudge this (mm, either sign) until it's centered.
const float POS_OFFSET_MM = 20.0f;

const uint8_t BRIGHT_WORK  = 255;
const uint8_t BRIGHT_IDLE  = 60;
const uint8_t BRIGHT_ALERT = 200;

const uint32_t MIN_JOB_MS   = 5000;
const uint32_t CELEBRATE_MS = 12000;
const uint32_t FRAME_MS     = 20;    // 50fps. Don't push this on a single-core C3.

// ---------- state ----------
enum Mode { M_BOOT, M_IDLE, M_RUN, M_HOLD, M_ALARM, M_HOMING, M_DONE };

Adafruit_NeoPixel strip(NUM_LEDS, DATA_PIN, NEO_GRB + NEO_KHZ800);
WiFiClient fluid;
WebServer server(80);
Preferences prefs;

Mode mode = M_BOOT;
bool partyMode = false;

// Web-configurable, persisted in flash. Film mode forces a steady comet with
// no party mode (button included) and a fixed brightness -- for filming/
// timelapses, where the comet's real job is shadow-free task lighting, not
// a show.
bool filmMode = false;
uint8_t filmBrightness = 200;
float cometRadiusLeds = 3.5f;   // half-width of the comet, in LEDs

char lineBuf[192];
size_t lineLen = 0;

uint32_t runStartedAt = 0, doneAt = 0, lastRxAt = 0;
uint32_t lastReconnectTry = 0, lastFrame = 0;
uint16_t rainbowHue = 0;

float posX = 0;
float wcoX = 0;
bool havePos = false;

uint8_t sineTab[256];

// ---------- small maths helpers (replacing FastLED's) ----------
uint8_t sin8t(uint8_t theta) { return sineTab[theta]; }

// oscillate between lo and hi at the given bpm
uint8_t beat(uint8_t bpm, uint8_t lo, uint8_t hi) {
  uint32_t phase = (millis() * bpm * 256UL) / 60000UL;
  uint16_t v = sin8t(phase & 0xFF);
  return lo + ((uint32_t)v * (hi - lo)) / 255;
}

void buildSineTable() {
  for (int i = 0; i < 256; i++)
    sineTab[i] = (uint8_t)(127.5 + 127.4 * sin(i * 2.0 * PI / 256.0));
}

// ---------- web UI ----------
void handleRoot() {
  char page[900];
  snprintf(page, sizeof(page),
    "<!DOCTYPE html><html><head><meta name=viewport content='width=device-width,initial-scale=1'>"
    "<title>terraPen lights</title><style>"
    "body{font-family:sans-serif;max-width:420px;margin:2em auto;padding:0 1em}"
    "label{display:block;margin-top:1.2em}"
    "input[type=range]{width:100%%}"
    "button{margin-top:1.5em;padding:0.6em 1.2em;font-size:1em}"
    "</style></head><body>"
    "<h2>terraPen lights</h2>"
    "<form method=POST action=/set>"
    "<label><input type=checkbox name=film %s> Film mode (steady white, no party)</label>"
    "<label>Brightness: <span id=bv>%d</span>"
    "<input type=range name=bright min=10 max=255 value=%d oninput=\"bv.textContent=this.value\"></label>"
    "<label>Comet width: <span id=rv>%.1f</span> LEDs"
    "<input type=range name=radius min=1 max=8 step=0.5 value=%.1f oninput=\"rv.textContent=this.value\"></label>"
    "<button type=submit>Apply</button>"
    "</form></body></html>",
    filmMode ? "checked" : "", filmBrightness, filmBrightness, cometRadiusLeds, cometRadiusLeds);
  server.send(200, "text/html", page);
}

void handleSet() {
  filmMode = server.hasArg("film");
  if (server.hasArg("bright")) filmBrightness = (uint8_t)constrain(server.arg("bright").toInt(), 10, 255);
  if (server.hasArg("radius")) cometRadiusLeds = constrain(server.arg("radius").toFloat(), 1.0f, 8.0f);
  if (filmMode) partyMode = false;   // never mid-rainbow on a take

  prefs.putBool("filmMode", filmMode);
  prefs.putUChar("filmBright", filmBrightness);
  prefs.putFloat("cometR", cometRadiusLeds);

  server.sendHeader("Location", "/");
  server.send(303);
}

// ---------- machine-readable API (terraTouch dial) ----------
// The dial polls /status every 3s and pushes changes to /set and /party.
// It resolves us as terrapen-leds.local, the same name we advertise below.
const char* modeName(Mode m) {
  switch (m) {
    case M_IDLE:   return "IDLE";
    case M_RUN:    return "RUN";
    case M_HOLD:   return "HOLD";
    case M_ALARM:  return "ALARM";
    case M_HOMING: return "HOMING";
    case M_DONE:   return "DONE";
    default:       return "BOOT";
  }
}

void handleStatus() {
  char json[160];
  snprintf(json, sizeof(json),
    "{\"filmMode\":%s,\"brightness\":%u,\"radius\":%.1f,\"party\":%s,\"mode\":\"%s\"}",
    filmMode ? "true" : "false", filmBrightness, cometRadiusLeds,
    partyMode ? "true" : "false", modeName(mode));
  server.send(200, "application/json", json);
}

// Unlike /set this must answer 200 -- the dial rejects anything else outright.
void handleParty() {
  if (!filmMode) partyMode = !partyMode;   // same rule as the button: never mid-take
  char json[32];
  snprintf(json, sizeof(json), "{\"party\":%s}", partyMode ? "true" : "false");
  server.send(200, "application/json", json);
}

// ---------- setup ----------
void setup() {
  Serial.begin(115200);
  pinMode(BTN_PIN, INPUT_PULLUP);
  buildSineTable();

  strip.begin();
  strip.setBrightness(BRIGHT_IDLE);
  strip.clear();
  strip.show();

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  prefs.begin("terrapen", false);
  filmMode       = prefs.getBool("filmMode", false);
  filmBrightness = prefs.getUChar("filmBright", 200);
  cometRadiusLeds = prefs.getFloat("cometR", 3.5f);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/set", HTTP_POST, handleSet);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/party", HTTP_POST, handleParty);
  server.begin();
}

// ---------- connection ----------
void connectToFluidNC() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (millis() - lastReconnectTry < 3000) return;
  lastReconnectTry = millis();

  static bool mdnsUp = false;
  if (!mdnsUp) {
    mdnsUp = MDNS.begin("terrapen-leds");
    if (mdnsUp) MDNS.addService("http", "tcp", 80);   // reachable at http://terrapen-leds.local/
  }

  IPAddress ip = MDNS.queryHost(FLUID_HOST, 2000);
  if (ip == IPAddress((uint32_t)0)) {
    Serial.println("mDNS lookup failed, retrying");
    return;
  }

  if (fluid.connect(ip, FLUID_PORT, 3000)) {
    fluid.setNoDelay(true);
    // Auto-reporting is per-channel and must be re-issued after every reconnect.
    fluid.print("\n");
    fluid.printf("$Report/Interval=%d\n", REPORT_INTERVAL_MS);
    lastRxAt = millis();
    Serial.println("connected to FluidNC");
  }
}

// ---------- parsing ----------
void applyState(const char* state) {
  Mode next = mode;

  if      (!strncmp(state, "Run",   3)) next = M_RUN;
  else if (!strncmp(state, "Jog",   3)) next = M_RUN;
  else if (!strncmp(state, "Hold",  4)) next = M_HOLD;
  else if (!strncmp(state, "Door",  4)) next = M_HOLD;
  else if (!strncmp(state, "Alarm", 5)) next = M_ALARM;
  else if (!strncmp(state, "Home",  4)) next = M_HOMING;
  else if (!strncmp(state, "Sleep", 5)) next = M_IDLE;
  else if (!strncmp(state, "Idle",  4)) {
    bool wasJob = (mode == M_RUN) && (millis() - runStartedAt > MIN_JOB_MS);
    if (wasJob) { next = M_DONE; doneAt = millis(); }
    else if (mode == M_DONE && millis() - doneAt < CELEBRATE_MS) next = M_DONE;
    else next = M_IDLE;
  }

  if (next == M_RUN && mode != M_RUN) runStartedAt = millis();
  mode = next;
}

void handleLine(char* line) {
  lastRxAt = millis();

  if (line[0] == '<') {
    char* end = strpbrk(line + 1, "|>");
    if (!end) return;
    char saved = *end;
    *end = '\0';
    applyState(line + 1);
    *end = saved;             // restore so MPos further along the line is still readable

    // FluidNC reports WCO (work coordinate offset) only occasionally, not on
    // every line, so latch it and apply it to whichever MPos we get next.
    // WPos (job/sheet-relative, what the sender shows you) = MPos - WCO.
    char* wco = strstr(end, "WCO:");
    if (wco) wcoX = atof(wco + 4);

    char* wpos = strstr(end, "WPos:");
    if (wpos) {
      posX = atof(wpos + 5);
      havePos = true;
    } else {
      char* mpos = strstr(end, "MPos:");
      if (mpos) {
        posX = atof(mpos + 5) - wcoX;
        havePos = true;
      }
    }
    return;
  }

  // Optional: a (MSG,PARTY) comment in your gcode may surface here as [MSG:PARTY].
  if (strstr(line, "[MSG:") && strstr(line, "PARTY")) partyMode = !partyMode;
}

void pumpSerialStream() {
  while (fluid.available()) {
    char c = fluid.read();
    if (c == '\n' || c == '\r') {
      if (lineLen) { lineBuf[lineLen] = '\0'; handleLine(lineBuf); lineLen = 0; }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    }
  }
}

// ---------- effects ----------
void fillAll(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < NUM_LEDS; i++) strip.setPixelColor(i, r, g, b);
}

float xToLedIndex(float x) {
  x = constrain(x + POS_OFFSET_MM, 0.0f, X_TRAVEL_MM);
  x = X_TRAVEL_MM - x;      // X=0 is the far end of the strip, not the near end
  float usableLeds = (NUM_LEDS - 1) - 2 * STRIP_MARGIN_LEDS;
  return STRIP_MARGIN_LEDS + (x / X_TRAVEL_MM) * usableLeds;
}

void renderComet() {
  fillAll(0, 0, 0);
  if (!havePos) { fillAll(255, 250, 240); return; }   // no position yet, fall back to solid white

  float center = xToLedIndex(posX);   // deliberately not rounded -- keeps the fade continuous
  int lo = (int)floorf(center - cometRadiusLeds);
  int hi = (int)ceilf(center + cometRadiusLeds);
  for (int idx = lo; idx <= hi; idx++) {
    if (idx < 0 || idx >= NUM_LEDS) continue;
    float d = idx - center;
    if (fabsf(d) > cometRadiusLeds) continue;
    // Raised-cosine taper: 1.0 at the exact position, 0.0 at the radius edge,
    // smooth in between -- no LED ever pops straight from off to full bright.
    float b = 0.5f * (1.0f + cosf(PI * d / cometRadiusLeds));
    uint8_t bright = (uint8_t)(b * 255);
    strip.setPixelColor(idx, (255UL * bright) / 255, (250UL * bright) / 255, (240UL * bright) / 255);
  }
}

void render() {
  uint32_t t = millis();

  if (partyMode) {
    strip.setBrightness(BRIGHT_ALERT);
    for (int i = 0; i < NUM_LEDS; i++) {
      uint16_t h = rainbowHue + (i * 65536L / NUM_LEDS);
      strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(h)));
    }
    rainbowHue += 512;
    return;
  }

  switch (mode) {
    case M_RUN:
      strip.setBrightness(BRIGHT_WORK);
      renderComet();                              // warm-white dot follows the carriage's X
      break;

    case M_DONE:
      if (t - doneAt > CELEBRATE_MS) { mode = M_IDLE; break; }
      strip.setBrightness(beat(30, 90, 255));
      fillAll(0, 255, 60);                       // green
      break;

    case M_HOMING: {
      // Sequential fill down the strip, hold lit, blank, repeat -- like a
      // modern car's dynamic indicator rather than a wrapping dot-chase.
      strip.setBrightness(BRIGHT_ALERT);
      const uint32_t SWEEP_MS = 600, HOLD_MS = 150, GAP_MS = 150;
      const uint32_t CYCLE_MS = SWEEP_MS + HOLD_MS + GAP_MS;
      uint32_t phase = t % CYCLE_MS;

      int lit;
      if (phase < SWEEP_MS)             lit = (int)((phase * (uint32_t)NUM_LEDS) / SWEEP_MS);
      else if (phase < SWEEP_MS + HOLD_MS) lit = NUM_LEDS;
      else                                lit = -1;   // gap: all off

      for (int i = 0; i < NUM_LEDS; i++)
        strip.setPixelColor(i, i < lit ? 255 : 0, i < lit ? 90 : 0, 0);   // indicator orange
      break;
    }

    case M_HOLD:
      strip.setBrightness(beat(60, 60, BRIGHT_ALERT));
      fillAll(255, 130, 0);                      // amber
      break;

    case M_ALARM:
      strip.setBrightness(((t / 300) % 2) ? BRIGHT_ALERT : 20);
      fillAll(255, 0, 0);
      break;

    case M_BOOT:
      strip.setBrightness(beat(20, 10, 50));
      fillAll(0, 40, 255);                       // dim blue breathing = not connected
      break;

    case M_IDLE:
    default:
      strip.setBrightness(BRIGHT_IDLE);
      for (int i = 0; i < NUM_LEDS; i++) {       // slow warm shimmer
        uint8_t v = sin8t((i * 6) + (t / 24));
        strip.setPixelColor(i, 40 + (v >> 2), 22 + (v >> 3), 4);
      }
      break;
  }

  // Film mode wants even, predictable exposure -- flatten out whatever
  // brightness the mode above chose (including the beat()-driven pulsing)
  // in favor of one steady level you dial in from the web UI.
  if (filmMode) strip.setBrightness(filmBrightness);
}

// ---------- loop ----------
void loop() {
  server.handleClient();

  static uint32_t lastBtn = 0;
  static uint32_t lowSince = 0;
  static bool armed = true;
  bool btnState = digitalRead(BTN_PIN);
  // GPIO9's weak internal pull-up picks up stepper-driver EMI while the
  // plotter is moving, so a plain edge trigger fires on noise. Require the
  // pin to sit LOW for 30ms straight (real presses clear that easily,
  // motor-switching spikes mostly don't) and re-arm only once it's seen HIGH.
  // Disabled entirely in film mode -- no accidental party mode mid-take.
  if (btnState == LOW) {
    if (lowSince == 0) lowSince = millis();
    if (armed && !filmMode && millis() - lowSince > 30 && millis() - lastBtn > 400) {
      partyMode = !partyMode;
      lastBtn = millis();
      armed = false;
    }
  } else {
    lowSince = 0;
    armed = true;
  }

  if (!fluid.connected()) {
    mode = M_BOOT;
    fluid.stop();
    connectToFluidNC();
  } else {
    pumpSerialStream();

    // FluidNC's auto-report gates its interval timer on motionState(), so a
    // parked machine transmits nothing at all -- silence does NOT imply a dead
    // link. Poke it with '?' (a realtime char: no newline, safe to inject
    // mid-stream) once a second whenever the channel goes quiet, so the
    // watchdog below is judging an unanswered question rather than an idle
    // machine. During a job the 50ms auto-reports keep us under the threshold
    // and this never fires.
    static uint32_t lastPoll = 0;
    if (millis() - lastRxAt > 1500 && millis() - lastPoll > 1000) {
      lastPoll = millis();
      fluid.print('?');
    }

    // Prolonged silence means the link died without a clean close.
    if (millis() - lastRxAt > 8000) { fluid.stop(); mode = M_BOOT; }
  }

  if (millis() - lastFrame >= FRAME_MS) {
    lastFrame = millis();
    render();
    strip.show();
  }
}
