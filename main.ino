#include <Bluepad32.h>
#include <DShotRMT.h>

// #define M1PIN 4;
// #define M2PIN 0;
// #define M3PIN 2;
// #define M4PIN 15;
// #define DELAY 100;

int M1PIN = 4;
int M2PIN = 0;
int M3PIN = 2;
int M4PIN = 15;
int DELAY = 1;

int target1 = 100;
int throttle1 = 100;

int target2 = 100;
int throttle2 = 100;

int target3 = 100;
int throttle3 = 100;

int target4 = 100;
int throttle4 = 100;


DShotRMT m1 = DShotRMT(M1PIN, RMT_CHANNEL_0);
DShotRMT m2 = DShotRMT(M2PIN, RMT_CHANNEL_1);
DShotRMT m3 = DShotRMT(M3PIN, RMT_CHANNEL_2);
DShotRMT m4 = DShotRMT(M4PIN, RMT_CHANNEL_3);

void motorArm(DShotRMT * motor){
  motor->sendThrottleValue(0);
  delayMicroseconds(50);
  motor->sendThrottleValue(100);
  delayMicroseconds(150);
  motor->sendThrottleValue(2047);
  delayMicroseconds(150);
  motor->sendThrottleValue(0);
  delayMicroseconds(50);
  motor->sendThrottleValue(0);
}

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            ControllerProperties properties = ctl->getProperties();
            Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                           properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }

    if (!foundController) {
        Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

void dumpGamepad(ControllerPtr ctl) {
    Serial.printf(
        "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
        "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
        ctl->index(),        // Controller Index
        ctl->dpad(),         // D-pad
        ctl->buttons(),      // bitmask of pressed buttons
        ctl->axisX(),        // (-511 - 512) left X Axis
        ctl->axisY(),        // (-511 - 512) left Y axis
        ctl->axisRX(),       // (-511 - 512) right X axis
        ctl->axisRY(),       // (-511 - 512) right Y axis
        ctl->brake(),        // (0 - 1023): brake button
        ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
        ctl->miscButtons(),  // bitmask of pressed "misc" buttons
        ctl->gyroX(),        // Gyro X
        ctl->gyroY(),        // Gyro Y
        ctl->gyroZ(),        // Gyro Z
        ctl->accelX(),       // Accelerometer X
        ctl->accelY(),       // Accelerometer Y
        ctl->accelZ()        // Accelerometer Z
    );
    Serial.println("Got to here.");
}

void alignThrottle(int inc, int * target, int * throttle){
  if ((inc / 2) < 2){
    return;
  }
  if ((*target) + inc - 1 < (*throttle) || (*throttle) < (*target) - inc + 1){
    if ((*target) < (*throttle)){
      (*throttle) -= inc;
    }
    else {
      (*throttle) += inc;
    }
  }
  else {
    return alignThrottle(inc/2, target, throttle);
  }
}

uint16_t calcTarget(uint16_t val){
  return ((1.9 * val) + 100);
}

// Arduino setup function. Runs in CPU 1
void setup() {
  Serial.begin(115200);
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But it might also fix some connection / re-connection issues.
  BP32.forgetBluetoothKeys();

  // Enables mouse / touchpad support for gamepads that support them.
  // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
  // - First one: the gamepad
  // - Second one, which is a "virtual device", is a mouse.
  // By default, it is disabled.
  BP32.enableVirtualDevice(false);

  m1.begin(DSHOT300);
  motorArm(&m1);
  m2.begin(DSHOT300);
  motorArm(&m2);
  // m4.begin(DSHOT300);
  // motorArm(&m4);
  // m3.begin(DSHOT300);
  // motorArm(&m3);
}

// void loop() {
//   bool dataUpdated = BP32.update();
//   // Serial.printf("throttle: %d, target: %d\n", throttle, target);

//   if (dataUpdated){
//       Serial.println("2. here");
//       for (auto myController : myControllers){
//         dumpGamepad(myController);
//       }
//   }
// }

void setThrottle(int button){
   if (button == 1){ // A
      target1 += 16;
      target2 -= 16;
      target3 -= 16;
      target4 -= 16;
   }
   else if (button == 2) { // B
      target1 -= 16;
      target2 += 16;
      target3 -= 16;
      target4 -= 16;
   }
   else if (button == 4) { // X
      target1 -= 16;
      target2 -= 16;
      target3 += 16;
      target4 -= 16;
   }
   else if (button == 8) { // Y
      target1 -= 16;
      target2 -= 16;
      target3 -= 16;
      target4 += 16;
   }
   if (target1 < 100){
    target1 = 100;
   }
   if (target2 < 100){
    target2 = 100;
   }
   if (target3 < 100){
    target3 = 100;
   }
   if (target4 < 100){
    target4 = 100;
   }
}

void loop() {
  bool dataUpdated = BP32.update();
  // Serial.printf("throttle: %d, target: %d\n", throttle, target);
  if (dataUpdated){
      for (auto myController : myControllers){
        if (myController && myController->isConnected() && myController->hasData()) {
          if (myController->isGamepad()){
            int button = myController->buttons();
            setThrottle(button);
            // target = (uint16_t) myController->throttle();
            
            // target = calcTarget(target);
            // if (myController->brake()){
            //   target = 0;
            // }
            // Serial.printf("target: %d\n", target);
          }
        }
      }
  }
  else {
    Serial.printf("throttle1 = %d, throttle2 = %d, throttle3 = %d, throttle4 = %d\n", throttle1, throttle2, throttle3, throttle4);
    // alignThrottle(32, &target1, &throttle1);
    // alignThrottle(32, &target2, &throttle2);
    // alignThrottle(32, &target3, &throttle3);
    // alignThrottle(32, &target4, &throttle4);
    m1.sendThrottleValue(throttle1);
    m2.sendThrottleValue(throttle2);
    // m3.sendThrottleValue(throttle3);
    // m4.sendThrottleValue(throttle4);
  }
  delay(10);
}
