#include <WiFi.h>
#include <WebServer.h>

// WiFi AP Credentials, please change
const char* ap_ssid     = "ESP32-Controller";
const char* ap_password = "control123";

//DONT CHANGE ANYTHING IN THIS HEADER
//START OF HEADER
WebServer server(80);

// Global Joystick values (-100 to 100) accessible anywhere in your code
int leftJoyX = 0;
int leftJoyY = 0;
int rightJoyX = 0;
int rightJoyY = 0;

// ---------------------------------------------------------------------
// Web Page: Two 2D Joysticks returning X & Y (-100..100) for both
// ---------------------------------------------------------------------
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>ESP32 Dual Joystick</title>
  <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
  <style>
    * { touch-action: none; user-select: none; -webkit-user-select: none; box-sizing: border-box; }
    body {
      margin: 0; background: #111; color: #fff; font-family: Arial, sans-serif;
      height: 100vh; display: flex; flex-direction: column; justify-content: center; align-items: center;
    }
    .container {
      display: flex; width: 100%; justify-content: space-around; align-items: center; flex: 1; max-height: 300px;
    }
    .joystickBase {
      width: 140px; height: 140px; border-radius: 50%; background: #222;
      border: 3px solid #444; position: relative;
    }
    .joystickKnob {
      width: 50px; height: 50px; border-radius: 50%; background: #4CAF50;
      position: absolute; top: 45px; left: 45px;
    }
    #status { padding: 15px; font-size: 16px; color: #aaa; text-align: center; }
  </style>
</head>
<body>
  <div class="container">
    <div class="joystickBase" id="joyLeftBase"><div class="joystickKnob" id="joyLeftKnob"></div></div>
    <div class="joystickBase" id="joyRightBase"><div class="joystickKnob" id="joyRightKnob"></div></div>
  </div>
  <div id="status">Left: (0, 0) | Right: (0, 0)</div>

<script>
let lx = 0, ly = 0, rx = 0, ry = 0;
const statusEl = document.getElementById('status');

function setupJoystick(baseId, knobId, isLeft) {
  const base = document.getElementById(baseId);
  const knob = document.getElementById(knobId);
  const maxDist = 45; 
  const center = 45;  
  let activeTouchId = null;

  function setKnob(dx, dy) {
    knob.style.left = (center + dx) + 'px';
    knob.style.top = (center + dy) + 'px';
  }

  function handleCoord(clientX, clientY) {
    const rect = base.getBoundingClientRect();
    const cx = rect.left + rect.width / 2;
    const cy = rect.top + rect.height / 2;
    let dx = clientX - cx;
    let dy = clientY - cy;
    const dist = Math.sqrt(dx * dx + dy * dy);
    
    if (dist > maxDist) {
      dx = (dx / dist) * maxDist;
      dy = (dy / dist) * maxDist;
    }
    
    setKnob(dx, dy);
    let valX = Math.round((dx / maxDist) * 100);
    let valY = Math.round((-dy / maxDist) * 100); // Invert Y so up is positive
    
    if (isLeft) { lx = valX; ly = valY; } 
    else { rx = valX; ry = valY; }
    
    updateStatus();
  }

  function reset() {
    setKnob(0, 0);
    activeTouchId = null;
    if (isLeft) { lx = 0; ly = 0; } 
    else { rx = 0; ry = 0; }
    updateStatus();
  }

  base.addEventListener('touchstart', e => {
    e.preventDefault();
    if (activeTouchId !== null) return;
    const touch = e.changedTouches[0];
    activeTouchId = touch.identifier;
    handleCoord(touch.clientX, touch.clientY);
  });

  base.addEventListener('touchmove', e => {
    e.preventDefault();
    if (activeTouchId === null) return;
    for (let i = 0; i < e.changedTouches.length; i++) {
      if (e.changedTouches[i].identifier === activeTouchId) {
        handleCoord(e.changedTouches[i].clientX, e.changedTouches[i].clientY);
        break;
      }
    }
  });

  const endHandler = e => {
    if (activeTouchId === null) return;
    for (let i = 0; i < e.changedTouches.length; i++) {
      if (e.changedTouches[i].identifier === activeTouchId) {
        reset();
        break;
      }
    }
  };

  base.addEventListener('touchend', endHandler);
  base.addEventListener('touchcancel', endHandler);

  // Mouse fallback for desktop testing
  base.addEventListener('mousedown', e => {
    handleCoord(e.clientX, e.clientY);
    function onMove(ev) { handleCoord(ev.clientX, ev.clientY); }
    function onUp() {
      reset();
      document.removeEventListener('mousemove', onMove);
      document.removeEventListener('mouseup', onUp);
    }
    document.addEventListener('mousemove', onMove);
    document.addEventListener('mouseup', onUp);
  });
}

function updateStatus() {
  statusEl.textContent = `Left: (${lx}, ${ly}) | Right: (${rx}, ${ry})`;
}

setupJoystick('joyLeftBase', 'joyLeftKnob', true);
setupJoystick('joyRightBase', 'joyRightKnob', false);

// Stream joystick data every 100ms
setInterval(() => {
  fetch(`/control?lx=${lx}&ly=${ly}&rx=${rx}&ry=${ry}`).catch(() => {});
}, 100);
</script>
</body>
</html>
)rawliteral";

//END OF HEADER


// Motor Driver Pins (L298N)
const int motorLeftPin1   = 37;
const int motorLeftPin2   = 38;
const int motorLeftEnable = 6;

const int motorRightPin1   = 35;
const int motorRightPin2   = 36;
const int motorRightEnable = 7;

const int pwmFreq       = 2000;
const int pwmResolution = 8; // 0-255 range

// Hardware-level motor execution helper
void driveMotor(int pin1, int pin2, int enablePin, int speedValue) {
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
  ledcWrite(enablePin, duty);
  Serial.printf("Enable Pin:%d  pin1:%d   pin2:%d   speed:%d\n", enablePin, pin1, pin2, speedValue);
}

// ---------------------------------------------------------------------
// CUSTOM HANDLER: Map your two joysticks however you want here!
// ---------------------------------------------------------------------
void processJoystickInputs() {
  // Example: Standard Tank Drive 
  // Left joystick Up/Down controls left motor.
  // Right joystick Up/Down controls right motor.

  int leftSpeed  = leftJoyY;
  int rightSpeed = rightJoyY;
  //Serial.printf("Left X:%d Y:%d Speed:%d| Right X:%d Y:%d Speed:%d\n", leftJoyX, leftJoyY, leftSpeed,rightJoyX, rightJoyY, rightSpeed);
  // If you wanted arcade drive on just the left stick, you would do:
  // int leftSpeed  = constrain(leftJoyY + leftJoyX, -100, 100);
  // int rightSpeed = constrain(leftJoyY - leftJoyX, -100, 100);

  driveMotor(motorLeftPin1, motorLeftPin2, motorLeftEnable, leftSpeed);
  driveMotor(motorRightPin1, motorRightPin2, motorRightEnable, rightSpeed);
}

void handleRoot() {
  server.send(200, "text/html", index_html);
}

void handleControl() {
  if (server.hasArg("lx") && server.hasArg("ly") && server.hasArg("rx") && server.hasArg("ry")) {
    leftJoyX  = server.arg("lx").toInt();
    leftJoyY  = server.arg("ly").toInt();
    rightJoyX = server.arg("rx").toInt();
    rightJoyY = server.arg("ry").toInt();

    // Call custom handler function
    processJoystickInputs();
  }

  server.send(200, "text/plain", "OK");
}

void setup() {
  Serial.begin(115200);

  pinMode(motorLeftPin1, OUTPUT);
  pinMode(motorLeftPin2, OUTPUT);
  pinMode(motorRightPin1, OUTPUT);
  pinMode(motorRightPin2, OUTPUT);
  pinMode(motorLeftEnable, OUTPUT);
  pinMode(motorRightEnable, OUTPUT);

  ledcAttach(motorLeftEnable, pwmFreq, pwmResolution);
  ledcWrite(motorLeftEnable, 0);

  ledcAttach(motorRightEnable, pwmFreq, pwmResolution);
  ledcWrite(motorRightEnable, 0);

  WiFi.softAP(ap_ssid, ap_password);
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/control", handleControl);
  server.begin();
}

void loop() {
  server.handleClient();
  // The global variables leftJoyX, leftJoyY, rightJoyX, rightJoyY 
  // can be read here if you prefer handling logic inside the main loop.
}