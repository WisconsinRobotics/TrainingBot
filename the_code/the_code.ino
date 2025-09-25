  #include <Bluepad32.h>

  const uint8_t maxGamepads = 1;
  GamepadPtr myGamepads[maxGamepads];
  //declare which pins you are using 

  const int freq= 2000;
  const int pwmChannelA = 0;
  const int pwmChannelB = 1;
  const int resolution = 8;

  // This callback gets called any time a new gamepad is connected.
  // Up to 4 gamepads can be connected at the same time.
  void onConnectedGamepad(GamepadPtr gp) {
    bool foundEmptySlot = false;
    for (int i = 0; i < maxGamepads; i++) {
      if (myGamepads[i] == nullptr) {
        Serial.printf("CALLBACK: Gamepad is connected, index=%d\n", i);
        // Additionally, you can get certain gamepad properties like:
        // Model, VID, PID, BTAddr, flags, etc.
        GamepadProperties properties = gp->getProperties();
        Serial.printf("Gamepad model: %s, VID=0x%04x, PID=0x%04x\n",
                      gp->getModelName().c_str(), properties.vendor_id,
                      properties.product_id);
        myGamepads[i] = gp;
        foundEmptySlot = true;
        break;
      }
    }
    if (!foundEmptySlot) {
      Serial.println(
          "CALLBACK: Gamepad connected, but could not found empty slot");
    }
  }


  void onDisconnectedGamepad(GamepadPtr gp) {
    bool foundGamepad = false;

    for (int i = 0; i < maxGamepads; i++) {
      if (myGamepads[i] == gp) {
        Serial.printf("CALLBACK: Gamepad is disconnected from index=%d\n", i);
        myGamepads[i] = nullptr;
        foundGamepad = true;
        break;
      }
    }

    if (!foundGamepad) {
      Serial.println(
          "CALLBACK: Gamepad disconnected, but not found in myGamepads");
    }
  }

  //View Gamepad State (axis, button press, etc.)
  void GamepadState() {
    for (int i = 0; i < maxGamepads; i++) {
      GamepadPtr myGamepad = myGamepads[i];
        if (myGamepad && myGamepad->isConnected()) {
        Serial.printf(
                "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: "
                "%4d, %4d, brake: %4d, throttle: %4d, misc: 0x%02x, gyro x:%6d y:%6d "
                "z:%6d, accel x:%6d y:%6d z:%6d\n",
                i,                        // Gamepad Index
                myGamepad->dpad(),        // DPAD
                myGamepad->buttons(),     // bitmask of pressed buttons
                myGamepad->axisX(),       // (-511 - 512) left X Axis
                myGamepad->axisY(),       // (-511 - 512) left Y axis
                myGamepad->axisRX(),      // (-511 - 512) right X axis
                myGamepad->axisRY(),      // (-511 - 512) right Y axis
                myGamepad->brake(),       // (0 - 1023): brake button
                myGamepad->throttle(),    // (0 - 1023): throttle (AKA gas) button
                myGamepad->miscButtons(), // bitmak of pressed "misc" buttons
                myGamepad->gyroX(),       // Gyro X
                myGamepad->gyroY(),       // Gyro Y
                myGamepad->gyroZ(),       // Gyro Z
                myGamepad->accelX(),      // Accelerometer X
                myGamepad->accelY(),      // Accelerometer Y
                myGamepad->accelZ()       // Accelerometer Z
            );
          }
      }
  }

  //maps Gamepad Values to Motor Output
  void GamepadMapping() {
    GamepadPtr myGamepad = myGamepads[0];  // Since maxGamepads = 1
  //write gamepad mapping logic here with if statements
  }
  void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200); 
    
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t *addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2],
                  addr[3], addr[4], addr[5]);

    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
    //Write pin outputs here

    BP32.forgetBluetoothKeys();
  }

  unsigned long lastPrint = 0;

  void loop() {
    BP32.update();
    GamepadMapping();
    //GamepadState();
    //if (millis() - lastPrint > 500) {
      //GamepadState();
      //lastPrint = millis();
    //}
  }
