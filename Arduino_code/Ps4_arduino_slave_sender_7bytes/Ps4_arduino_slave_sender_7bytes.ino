
#include <PS4BT.h>

#include <usbhub.h>
#include <Wire.h>
#include <stdint.h>
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>
#define SLAVE_ADDR 0x68
USB Usb;
BTD Btd(&Usb);
PS4BT PS4(&Btd);
uint16_t device_id = 0xFF45;
uint8_t active_command = 0xff;

byte x = 127, y = 127, w = 127;
byte psbt[7];


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); // Halt
  }
  Serial.print(F("\r\nPS4 Bluetooth Library Started"));

  Wire.begin(SLAVE_ADDR);               // join I2C bus with address #8
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent); // register event
}

void loop() {

  // put your main code here, to run repeatedly:
  Usb.Task();

  if (PS4.connected()) {
    if (PS4.getAnalogHat(LeftHatX) > 137 || PS4.getAnalogHat(LeftHatX) < 117 || PS4.getAnalogHat(LeftHatY) > 137 || PS4.getAnalogHat(LeftHatY) < 117 ) {
      x = PS4.getAnalogHat(LeftHatX);
      y = PS4.getAnalogHat(LeftHatY);
    }
    else {
      x = 127;
      y = 127;
    }

    w = map(PS4.getAnalogButton(L2) - PS4.getAnalogButton(R2), -255, 255, 0, 255);

  }

  else {
    x = 127;
    y = 127;
    w = 127;
  }
  //  Serial.print("X :");   // print the character
  //  Serial.print(x);
  //  Serial.print("     Y : ");
  //  Serial.print(y);
  //  Serial.print("     W : ");
  //  Serial.println(w);

  //placeholder for @psbt[3]
  uint8_t bt_val = 0;

  //CROSS check
  if (PS4.getButtonPress(CROSS))
    bt_val |= (uint8_t)(1 << 0);

  //TRIANGLE CHECK
  if (PS4.getButtonPress(TRIANGLE))
    bt_val |= (1 << 1);

  //SQUARE CHECK
  if (PS4.getButtonPress(SQUARE))
    bt_val |= (1 << 2);

  //CIRCLE CHECK
  if (PS4.getButtonPress(CIRCLE))
    bt_val |= (1 << 3);

  //RIGHT CHECK
  if (PS4.getButtonPress(RIGHT))
    bt_val |= (1 << 4);

  //LEFT CHECK
  if (PS4.getButtonPress(LEFT))
    bt_val |= (1 << 5);

  //DOWN CHECK
  if (PS4.getButtonPress(DOWN))
    bt_val |= (1 << 6);

  //UP CHECK
  if (PS4.getButtonPress(UP))
    bt_val |= (1 << 7);



  //placeholder for @psbt[4]
  uint8_t bt_val_1 = 0;

  //L1 check
  if (PS4.getButtonPress(L1))
    bt_val_1 |= (uint8_t)(1 << 0);

  //R1 CHECK
  if (PS4.getButtonPress(R1))
    bt_val_1 |= (1 << 1);

  //L3 CHECK
  if (PS4.getButtonPress(L3))
    bt_val_1 |= (1 << 2);

  //R3 CHECK
  if (PS4.getButtonPress(R3))
    bt_val_1 |= (1 << 3);

  //SHARE CHECK
  if (PS4.getButtonPress(SHARE))
    bt_val_1 |= (1 << 4);

  //OPTIONS CHECK
  if (PS4.getButtonPress(OPTIONS))
    bt_val_1 |= (1 << 5);

  //TOUCHPAD CHECK
  if (PS4.getButtonPress(TOUCHPAD))
    bt_val_1 |= (1 << 6);

  //PS CHECK
  if (PS4.getButtonClick(PS))
    PS4.disconnect();


  //    psbt[5] = 0;
  //    psbt[6] = 0;

  uint8_t tp_x = 0;
  uint8_t tp_y = 0;
  if ((PS4.isTouching(0))) {
    tp_x = PS4.getX(0);
    tp_x = map(tp_x, 0, 2040, 0, 255);


    tp_y = PS4.getY(0);
    tp_y = map(tp_y, 0, 1023, 0, 127);

  }
  else {
    tp_x = 0;
    tp_y = 0;
  }
  Serial.println(tp_x);
  Serial.println(tp_y);


  psbt[0] = x;
  psbt[1] = y;
  psbt[2] = w;
  psbt[3] = bt_val;
  psbt[4] = bt_val_1;
  psbt[5] = tp_x;
  psbt[6] = tp_y;


}

void receiveEvent(int bytes) {
  active_command = Wire.read();    // read one character from the I2C
}
void requestEvent() {

  if (active_command == 0x05)
  {

    // Wire.write(strlen(name));
    Wire.write(psbt, 7);
    // Wire.write((uint8_t*)&name_msg[32],18);
    active_command = 0xff;
  }
  //  Wire.write(psbt, 5);
}

//if (~(PS4.isTouching(0) || PS4.isTouching(1))) {
//  psbt[5] = 0;
//  psbt[6] = 0;
//}
//
//else {
//  tp_x = PS4.getX();
//  tp_x = map(tp_x, 0, 2040, 0, 255);
//  psbt[5] = tp_x;
//  tp_y = PS4.getY();
//  tp_y = map(tp_y, 0, 1023, 0, 127);
//
//  psbt[6] = tp_y;
//}
