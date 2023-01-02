#include <Arduino.h>
#include <JoyButton.h>
#include <JoyAnalog.h>



JoyButton analogButtonL = JoyButton(12);
JoyButton analogButtonR = JoyButton(11);

JoyButton selectButton = JoyButton(10);

JoyButton circleButton = JoyButton(9);
JoyButton triangleButton = JoyButton(8);
JoyButton squareButton = JoyButton(7);
JoyButton crossButton = JoyButton(6);

JoyButton leftButton = JoyButton(5);
JoyButton upButton = JoyButton(4);
JoyButton rightButton = JoyButton(3);
JoyButton downButton = JoyButton(2);

JoyButton startButton = JoyButton(1);

JoyButton leftUpperShoulderButton = JoyButton(21);
JoyButton leftLowerShoulderButton = JoyButton(20);

JoyButton rightUpperShoulderButton = JoyButton(19);
JoyButton rightLowerShoulderButton = JoyButton(18);

JoyAnalog leftAnalogX = JoyAnalog(17);
JoyAnalog leftAnalogY = JoyAnalog(16);

JoyAnalog rightAnalogX = JoyAnalog(15);
JoyAnalog rightAnalogY = JoyAnalog(14);

int buttons2hat(bool up, bool down, bool left, bool right){
  if(up && !left && !down && !right){
    return 0;
  }else if(up && left && !down && !right){
    return 45;
  }else if(!up && left && !down && !right){
    return 90;
  }else if(!up && left && down && !right){
    return 135;
  }else if(!up && !left && down && !right){
    return 180;
  }else if(!up && !left && down && right){
    return 225;
  }else if(!up && !left && !down && right){
    return 270;
  }else if(up && !left && !down && right){
    return 315;
  }else{
    return -1;
  }
}

void setup() {
  Joystick.useManualSend(true);
}

void loop() {
  bool up = upButton.handle();
  bool down = downButton.handle();
  bool left = leftButton.handle();
  bool right = rightButton.handle();

  Joystick.hat(buttons2hat(up, down, left, right));

  Joystick.button(1, squareButton.handle());
  Joystick.button(2, crossButton.handle());
  Joystick.button(3, circleButton.handle());
  Joystick.button(4, triangleButton.handle());

  Joystick.button(5, leftUpperShoulderButton.handle());
  Joystick.button(7, leftLowerShoulderButton.handle());
  Joystick.button(6, rightUpperShoulderButton.handle());
  Joystick.button(8, rightLowerShoulderButton.handle());

  Joystick.button(9, startButton.handle());
  Joystick.button(10, selectButton.handle());

  Joystick.X(leftAnalogX.handle());
  Joystick.Y(leftAnalogY.handle());

  Joystick.Z(rightAnalogX.handle());
  Joystick.Zrotate(rightAnalogY.handle());

  Joystick.button(11, analogButtonL.handle());
  Joystick.button(12, analogButtonR.handle());

  Joystick.send_now();
}


// // PIN_D7 corresponds to physical pin 12
// #define BTN_PIN 1
// // PIN_F0 corresponds to physical pin 21
// // https://www.pjrc.com/teensy/td_digital.html
// // https://www.pjrc.com/teensy/card2b.pdf
// #define X_AXIS_PIN 14
// #define JOY_NUM 1
// // minimum value read from potentiometer
// #define POTENTIOMETER_MIN 300
// // maximum value read from potentiometer
// #define POTENTIOMETER_MAX 343
// #define ADC_MIN 0
// #define ADC_MAX 1023

// Bounce debouncer = Bounce();

// int mapPotentiometerValue(int rawValue) {
//   int translatedValue = map(rawValue, POTENTIOMETER_MIN, POTENTIOMETER_MAX, 0, 1023);
//   // ensure we stay between ADC_MIN and ADC_MAX
//   return max(ADC_MIN, min(ADC_MAX, translatedValue));
// }

// void setup()
// {
//   pinMode(LED_BUILTIN, OUTPUT);
//   pinMode(BTN_PIN, INPUT_PULLUP);
//   debouncer.attach(BTN_PIN);
//   debouncer.interval(1); // interval in ms
// }

// void loop()
// {
//   // try to update btn state only if GPIO state has changed
//   if(debouncer.update())
//   {
//     // we are in pull-up configuration so high state mean btn is released
//     if (debouncer.read())
//     {
//       digitalWrite(LED_BUILTIN, LOW);
//       Joystick.button(JOY_NUM, LOW);
//     } else {
//       digitalWrite(LED_BUILTIN, HIGH);
//       Joystick.button(JOY_NUM, HIGH);
//     }
//   }
//   // update joystick X axis with analog value for potentiometer
//   int xValue = analogRead(X_AXIS_PIN);
//   // map potentiometer values to full ADC 10 bits range
//   int translatedValue = mapPotentiometerValue(xValue);
//   Joystick.X(translatedValue);
// }
