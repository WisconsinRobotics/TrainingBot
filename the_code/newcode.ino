#include <WiFi.h>
#include <WebServer.h>

// ===========================================================================
// HOW TO CONNECT
// ===========================================================================
// 1. Upload this sketch to the ESP32-S3.
// 2. On your phone, go to WiFi settings and connect to:
//      Network name: ESP32-Controller
//      Password:     control123
// 3. Open a browser on your phone and go to:
//      http://192.168.4.1
//    (this is the ESP32's default address whenever it creates its own
//    network with WiFi.softAP() -- it's always this address, every time)
// ===========================================================================
//
// ===========================================================================
// CONTROL MAPPING  (what the joystick and buttons currently do)
// ===========================================================================
// JOYSTICK:
//   Drives the robot using "arcade drive":
//     y (up/down)   -> forward / backward speed
//     x (left/right)-> turning (mixed into left/right motor speeds)
//   leftSpeed  = y + x
//   rightSpeed = y - x
//   Each is constrained to -100..100 and converted to a direction + PWM duty
//   for its motor.
//
// BUTTON A (green, bottom):  Hold to turn ON the yellow LED (GPIO18). Demo only.
// BUTTON B (red, right):     Not wired to anything yet.
// BUTTON X (blue, left):     Not wired to anything yet.
// BUTTON Y (yellow, top):    Not wired to anything yet.
//
// Whenever you wire up something new, update BOTH:
//   1. The if/else logic inside handleControl()
//   2. This comment block, so it stays an accurate map of what does what
// ===========================================================================

// ---- The ESP32 creates its own WiFi network with this name/password ----
const char* ap_ssid     = "ESP32-Controller";
const char* ap_password = "control123";   // must be at least 8 characters
// --------------------------------------------------------------------------

// ---- Motor driver pins (L298N-style: 2 direction pins + 1 PWM enable pin per motor) ----
const int motorLeftPin1   = 35;
const int motorLeftPin2   = 36;
const int motorLeftEnable = 7;

const int motorRightPin1   = 37;
const int motorRightPin2   = 38;
const int motorRightEnable = 6;

const int pwmFreq       = 2000;
const int pwmResolution = 8; // 0-255 duty range

const int ledPin = 18; // Button A demo LED

WebServer server(80);

// ---------------------------------------------------------------------
// The phone-side controller page: a joystick + 4 buttons (A, B, X, Y).
// Sends x, y, a, b, c, d values to /control roughly every 150ms.
// ---------------------------------------------------------------------
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>ESP32 Controller</title>
  <meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
  <style>
    * { touch-action: none; user-select: none; -webkit-user-select: none; }
    body {
      margin: 0; background: #111; color: #fff; font-family: Arial, sans-serif;
      height: 100vh; display: flex; flex-direction: column; justify-content: space-between;
    }
    h1 { text-align: center; font-size: 18px; margin: 10px; color: #888; }
    .controls { display: flex; justify-content: space-between; align-items: center; padding: 20px; flex: 1; }
    #joystickBase {
      width: 150px; height: 150px; border-radius: 50%; background: #222;
      border: 2px solid #444; position: relative; flex-shrink: 0;
    }
    #joystickKnob {
      width: 60px; height: 60px; border-radius: 50%; background: #4CAF50;
      position: absolute; top: 45px; left: 45px;
    }
    .buttons {
      display: grid; grid-template-columns: 60px 60px 60px; grid-template-rows: 60px 60px 60px;
      gap: 5px; flex-shrink: 0;
    }
    .btn {
      border-radius: 50%; display: flex; align-items: center; justify-content: center;
      font-weight: bold; font-size: 18px; color: #fff;
      transition: transform 0.05s, filter 0.05s;
    }
    .btn.pressed {
      filter: brightness(1.5);
      transform: scale(0.88);
    }
    #btnY { grid-column: 2; grid-row: 1; background: #FFC107; }
    #btnX { grid-column: 1; grid-row: 2; background: #2196F3; }
    #btnB { grid-column: 3; grid-row: 2; background: #f44336; }
    #btnA { grid-column: 2; grid-row: 3; background: #4CAF50; }
    #status { text-align: center; color: #666; font-size: 12px; margin: 10px; }
  </style>
</head>
<body>
  <h1>ESP32 Controller</h1>
  <div class="controls">
    <div id="joystickBase">
      <div id="joystickKnob"></div>
    </div>
    <div class="buttons">
      <div class="btn" id="btnY">Y</div>
      <div class="btn" id="btnX">X</div>
      <div class="btn" id="btnB">B</div>
      <div class="btn" id="btnA">A</div>
    </div>
  </div>
  <div id="status">X:0 Y:0</div>

<script>
let joyX = 0, joyY = 0;
let btnA = 0, btnB = 0, btnC = 0, btnD = 0; // C = "X" button, D = "Y" button on screen

const base = document.getElementById('joystickBase');
const knob = document.getElementById('joystickKnob');
const statusEl = document.getElementById('status');
const maxDist = 45;

function setKnob(dx, dy) {
  knob.style.left = (45 + dx) + 'px';
  knob.style.top = (45 + dy) + 'px';
}

function handleMove(clientX, clientY) {
  const rect = base.getBoundingClientRect();
  const cx = rect.left + rect.width / 2;
  const cy = rect.top + rect.height / 2;
  let dx = clientX - cx;
  let dy = clientY - cy;
  const dist = Math.sqrt(dx * dx + dy * dy);
  if (dist > maxDist) {
    dx = dx / dist * maxDist;
    dy = dy / dist * maxDist;
  }
  setKnob(dx, dy);
  joyX = Math.round((dx / maxDist) * 100);
  joyY = Math.round((-dy / maxDist) * 100); // flip so up = positive
  statusEl.textContent = 'X:' + joyX + ' Y:' + joyY;
}

function resetJoystick() {
  setKnob(0, 0);
  joyX = 0; joyY = 0;
  statusEl.textContent = 'X:0 Y:0';
}

base.addEventListener('touchstart', e => handleMove(e.touches[0].clientX, e.touches[0].clientY));
base.addEventListener('touchmove', e => { e.preventDefault(); handleMove(e.touches[0].clientX, e.touches[0].clientY); });
base.addEventListener('touchend', resetJoystick);

// Mouse support too, so you can test this from a laptop browser
base.addEventListener('mousedown', e => {
  handleMove(e.clientX, e.clientY);
  function onMove(ev) { handleMove(ev.clientX, ev.clientY); }
  function onUp() {
    resetJoystick();
    document.removeEventListener('mousemove', onMove);
    document.removeEventListener('mouseup', onUp);
  }
  document.addEventListener('mousemove', onMove);
  document.addEventListener('mouseup', onUp);
});

function bindButton(id, setter) {
  const el = document.getElementById(id);
  el.addEventListener('touchstart', e => { e.preventDefault(); setter(1); el.classList.add('pressed'); });
  el.addEventListener('touchend', () => { setter(0); el.classList.remove('pressed'); });
  el.addEventListener('mousedown', () => { setter(1); el.classList.add('pressed'); });
  el.addEventListener('mouseup', () => { setter(0); el.classList.remove('pressed'); });
}
bindButton('btnA', v => btnA = v);
bindButton('btnB', v => btnB = v);
bindButton('btnX', v => btnC = v);
bindButton('btnY', v => btnD = v);

// Send the current state to the ESP32 every 150ms
setInterval(() => {
  fetch(`/control?x=${joyX}&y=${joyY}&a=${btnA}&b=${btnB}&c=${btnC}&d=${btnD}`);
}, 150);
</script>
</body>
</html>
)rawliteral";

void handleRoot() {
  server.send(200, "text/html", index_html);
}

// ---------------------------------------------------------------------
// Drives one motor.
//   enablePin:   the PWM-attached pin for this motor (its LEDC "channel"
//                is handled automatically by ledcAttach in setup()).
//   speedValue:  -100 (full reverse) to 100 (full forward), 0 = stop
// ---------------------------------------------------------------------
void driveMotor(int pin1, int pin2, int enablePin, int speedValue) {
  speedValue = constrain(speedValue, -100, 100);
  int duty = map(abs(speedValue), 0, 100, 0, 255);

  if (speedValue > 5) {
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
  } else if (speedValue < -5) {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
  } else {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
    duty = 0;
  }
  ledcWrite(enablePin, duty); // new core: ledcWrite takes the PIN, not a channel number
}

void handleControl() {
  int x = server.arg("x").toInt();   // JOYSTICK left(-)/right(+): -100 to 100 -> turning
  int y = server.arg("y").toInt();   // JOYSTICK down(-)/up(+):   -100 to 100 -> forward/back
  int a = server.arg("a").toInt();   // BUTTON A (green):  1 = held, 0 = released -> LED demo
  int b = server.arg("b").toInt();   // BUTTON B (red):    1 = held, 0 = released -> unused
  int c = server.arg("c").toInt();   // BUTTON X (blue):   1 = held, 0 = released -> unused
  int d = server.arg("d").toInt();   // BUTTON Y (yellow): 1 = held, 0 = released -> unused

  Serial.printf("X:%d Y:%d  A:%d B:%d C:%d D:%d\n", x, y, a, b, c, d);

  // -----------------------------------------------------------------
  // Arcade-drive mixing: combine forward/back (y) and turn (x) into
  // separate left/right motor speeds.
  // -----------------------------------------------------------------
  int leftSpeed  = constrain(y + x, -100, 100);
  int rightSpeed = constrain(y - x, -100, 100);

  driveMotor(motorLeftPin1, motorLeftPin2, motorLeftEnable, leftSpeed);
  driveMotor(motorRightPin1, motorRightPin2, motorRightEnable, rightSpeed);

  digitalWrite(ledPin, a ? HIGH : LOW); // BUTTON A: hold to light the yellow LED (demo)

  server.send(200, "text/plain", "OK");
}

void setup() {
  Serial.begin(115200);

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  pinMode(motorLeftPin1, OUTPUT);
  pinMode(motorLeftPin2, OUTPUT);
  pinMode(motorRightPin1, OUTPUT);
  pinMode(motorRightPin2, OUTPUT);

  // New ESP32 core (3.x) LEDC API: attach PWM directly to a pin, no
  // separate channel number needed. ledcWrite() then also takes that pin.
  ledcAttach(motorLeftEnable, pwmFreq, pwmResolution);
  ledcWrite(motorLeftEnable, 0);

  ledcAttach(motorRightEnable, pwmFreq, pwmResolution);
  ledcWrite(motorRightEnable, 0);

  WiFi.softAP(ap_ssid, ap_password);
  Serial.print("Connect your phone to WiFi network: ");
  Serial.println(ap_ssid);
  Serial.print("Then open in your browser: http://");
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/control", handleControl);
  server.begin();
}

void loop() {
  server.handleClient();
}
