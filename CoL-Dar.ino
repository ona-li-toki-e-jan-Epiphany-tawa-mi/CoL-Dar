#include <LiquidCrystal.h>

// TODO Ensure type conversions are accurate.
// Language options?))
// Have adjustable states be saved after setting them. 
// Add rolling average for inductance calculations.
// Move all strings into map.
// Add two sensitivity options for sliders.

// Config.
long numberOfSamples = 10000;
double shuntResistance = 51.0;
int LCDBrightness = 255/2;
int LCDContrast = 0;

// Pins.
#define INDUCTOR_CHARGE_PIN 1
#define BUTTON_PIN 3
#define LCD_E_PIN 10
#define LCD_D4_PIN 7
#define LCD_D5_PIN 6
#define LCD_D6_PIN 5
#define LCD_D7_PIN 4
#define INDICATOR_LED_PIN 8
// Measures the voltage drop of the inductor in reference to ground.
#define INDUCTOR_VOLTAGE_PIN A1
// Measures the voltage drop of the shunt in reference to ground.
#define SHUNT_VOLTAGE_PIN A0
#define LCD_RS_PIN A4
#define ENCODER_CLOCK_PIN 2
#define ENCODER_DATA_PIN A3
#define LCD_BACKLIGHT_PIN 9
#define LCD_CONTRAST_PIN 11



LiquidCrystal LCD(LCD_RS_PIN, LCD_E_PIN, LCD_D4_PIN, LCD_D5_PIN, LCD_D6_PIN, LCD_D7_PIN);

enum StringMappings{e_none = -1, e_exitMenu = 0, e_testInductor = 1, e_testerSettings = 2, e_samplingRate = 3, e_shuntResistance = 4, e_displaySettings = 5, e_brightness = 6, e_contrast = 7, e_samplingRate2 = 8, e_shuntResistance2 = 9};
const char* stringMap[] = {"Return", "Test Inductor", "Tester Settings", "Sampling Rate", "Shunt Resistance", "Display", "Brightness", "Contrast", "S. Rate", "S. Value"};

void printMappedString(StringMappings stringKey) {
  if (stringKey != e_none)
    LCD.print(stringMap[stringKey]);
}

byte loadingBar[][8] = {{B00000, B00000, B00000, B00000, B00000, B00000, B00000, B00000}, {B10000, B10000, B10000, B10000, B10000, B10000, B10000, B10000}, {B11000, B11000, B11000, B11000, B11000, B11000, B11000, B11000}, {B11100, B11100, B11100, B11100, B11100, B11100, B11100, B11100}, 
  {B11110, B11110, B11110, B11110, B11110, B11110, B11110, B11110}, {B11111, B11111, B11111, B11111, B11111, B11111, B11111, B11111}};

volatile int encoderSteps = 0;

/**
 * Measures the speed and direction of the encoder.
 */
/// INTERRUPT ///
void encoderInterrupt() {
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = millis();

  if (interruptTime - lastInterruptTime > 5) {
    if (!digitalRead(ENCODER_DATA_PIN)) {
      encoderSteps++;
  
    } else
      encoderSteps--;

    lastInterruptTime = interruptTime;
  }
}

volatile bool buttonPressed = false;

/**
 * Tells if the button is pressed.
 */
/// INTERRUPT ///
void buttonInterrupt() {
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = millis();

  if (interruptTime - lastInterruptTime > 5) {
    buttonPressed = true;
    lastInterruptTime = interruptTime;
  }
}

/**
 * Tests if the button is pressed.
 * Successive calls without button presses in between will return false, regardless of the initial state.
 * 
 * @return The state of the button.
 */
bool isButtonPressed() {
  bool wasButtonPressed = buttonPressed;
  buttonPressed = false;
  return wasButtonPressed;
}

class Menu {
  public:
    Menu(StringMappings displayName, Menu** lowerMenus, byte lowerMenusLength) {
      this->displayName = displayName;
      this->lowerMenus = lowerMenus;
      this->lowerMenusLength = lowerMenusLength;
      isAction = false;

      for (byte i = 0; i < lowerMenusLength; i++)
        lowerMenus[i]->upperMenu = this;
    }

    Menu(StringMappings displayName, void (*action)()) {
      this->displayName = displayName;
      this->action = action;
      isAction = true;
    }
  
    Menu* upperMenu;
    Menu** lowerMenus;
    byte lowerMenusLength;
    void (*action)();
    bool isAction;
    StringMappings displayName;
};

/**
 * Displays an adjustable slider.
 * 
 * @param changableVariable The address of the value to be changed by the slider.
 * @param minimum The minimum value that the slider can set.
 * @param maximum The maximum value that the slider can set.
 * @param sliderName The key to the string that should be displayed with the slider.
 */
template <typename N>
void displaySlider(N currentValue, N minimum, N maximum, StringMappings sliderName) {
  LCD.clear();
  printMappedString(sliderName);
  LCD.print(' ');
  LCD.print(currentValue);

  LCD.setCursor(0, 1);

  double sliderBarPercentage = (double) (currentValue - minimum) / (double) (maximum - minimum);
  byte sliderBarLength = (byte) (sliderBarPercentage * 16);

  for (byte i = 0; i < sliderBarLength; i++)
    LCD.write(5);

  byte sliderBarExtra = (byte) (sliderBarPercentage * 16 * 5) - sliderBarLength * 5;

  LCD.write(sliderBarExtra);
}

/**
 * Creates and runs an adjustable slider.
 * The function will exit when the user chooses.
 * 
 * @param changableVariable The address of the value to be changed by the slider.
 * @param minimum The minimum value that the slider can set.
 * @param maximum The maximum value that the slider can set.
 * @param sliderName The key to the string that should be displayed with the slider.
 * @param sensitivity How fast the value changes with the users input.
 */
template <typename N>
void adjustiableSlider(N* changableVariable, N minimum, N maximum, StringMappings sliderName, N sensitivity) {
  displaySlider(*changableVariable, minimum, maximum, sliderName);
  
  encoderSteps = 0;
  
  while (true) {
    if (encoderSteps) {
      N newValue = min(max(*changableVariable + encoderSteps * sensitivity, minimum), maximum);

      if (*changableVariable != newValue) {
        *changableVariable = newValue;
        encoderSteps = 0;

        displaySlider(newValue, minimum, maximum, sliderName);
      }
    }

    if (isButtonPressed())
      break;
  }
}

class MenuController {
  public:
    MenuController(Menu* mainMenu) {
      this->mainMenu = mainMenu;
      currentMenu = mainMenu;
    }

    void displayMenu() {
      LCD.clear();
      
      if (currentMenu->lowerMenusLength) {
        printMappedString(currentMenu->lowerMenus[cursorPosition]->displayName);

        if (cursorPosition + 1 < currentMenu->lowerMenusLength) {
          LCD.setCursor(0, 1);
          printMappedString(currentMenu->lowerMenus[cursorPosition + 1]->displayName);
        }

        LCD.home();
      
      } else
        LCD.print("Nothing here...");
    }
  
    void onButtonPress() {
      if (currentMenu->lowerMenusLength) {
        Menu* selectedMenu = currentMenu->lowerMenus[cursorPosition];

        if (selectedMenu->isAction) {
          selectedMenu->action();
        
        } else if (selectedMenu->displayName == e_exitMenu) {
          currentMenu = currentMenu->upperMenu;
          displayMenu();
          cursorPosition = 0;
        
        } else {
          currentMenu = selectedMenu;
          cursorPosition = 0;
        }
        
        displayMenu();
      
      } else if (currentMenu != mainMenu){
        // Returns to upper menu if nothing is inside the current one or the selected button exits the current menu.
        currentMenu = currentMenu->upperMenu;
        displayMenu();
        cursorPosition = 0;
      }
    }

    void onScroll(int scrollDistance) {
      if (currentMenu->lowerMenusLength) {
        byte oldPosition = cursorPosition;
        
        if ((int) cursorPosition + scrollDistance < 0) {
          cursorPosition = 0;
          
        } else if (cursorPosition + scrollDistance >= currentMenu->lowerMenusLength){
          cursorPosition = currentMenu->lowerMenusLength - 1;
          
        } else
          cursorPosition += scrollDistance;

        if (cursorPosition != oldPosition)
          displayMenu();
      }
    }

  private:
    Menu* mainMenu;
    Menu* currentMenu;
    byte cursorPosition = 0;
};



void testInductor();
void setInterruptState(bool state);

MenuController* menu = new MenuController(new Menu(e_none, new Menu*[3] {
  new Menu(e_testInductor, testInductor), 
  new Menu(e_testerSettings, new Menu*[3] {
    new Menu(e_samplingRate, []() {adjustiableSlider(&numberOfSamples, 1000L, 100000, e_samplingRate2, 1000L);}),
    new Menu(e_shuntResistance, []() {adjustiableSlider(&shuntResistance, 0.0, 1000.0, e_shuntResistance2, 0.05);}),
    new Menu(e_exitMenu, NULL, 0)
  }, 3), 
  new Menu(e_displaySettings, new Menu*[3] {
    new Menu(e_brightness, []() {
      adjustiableSlider(&LCDBrightness, 0, 255, e_brightness, 8);
      analogWrite(LCD_BACKLIGHT_PIN, LCDBrightness);
    }),
    new Menu(e_contrast, []() {
      adjustiableSlider(&LCDContrast, 0, 50, e_contrast, 3);
      analogWrite(LCD_CONTRAST_PIN, LCDContrast);
    }),
    new Menu(e_exitMenu, NULL, 0)
  }, 3)
}, 3));

unsigned long cursorTiming;

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(INDUCTOR_CHARGE_PIN, OUTPUT);
  pinMode(LCD_BACKLIGHT_PIN, OUTPUT);
  pinMode(INDICATOR_LED_PIN, OUTPUT);

  for (byte i = 0; i < 6; i++)
    LCD.createChar(i, loadingBar[i]);
  LCD.begin(16, 2);
  
  setInterruptState(true);

  menu->displayMenu();
  cursorTiming = millis();
}

void setInterruptState(bool state) {
  if (state) {
    analogWrite(LCD_BACKLIGHT_PIN, LCDBrightness);
    analogWrite(LCD_CONTRAST_PIN, LCDContrast);
    attachInterrupt(digitalPinToInterrupt(ENCODER_CLOCK_PIN), encoderInterrupt, LOW);
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonInterrupt, LOW);
    
  } else {
    digitalWrite(LCD_BACKLIGHT_PIN, LOW);
    digitalWrite(LCD_CONTRAST_PIN, LOW);
    detachInterrupt(digitalPinToInterrupt(ENCODER_CLOCK_PIN));
    detachInterrupt(digitalPinToInterrupt(BUTTON_PIN));
  }
}



bool showCursor = false;

void loop() {
  if (encoderSteps > 5) {
    encoderSteps = 0;
    menu->onScroll(1);
    
  } else if (encoderSteps < -5) {
    encoderSteps = 0;
    menu->onScroll(-1);
  }
  
  if (isButtonPressed()) {
    LCD.noCursor();
    menu->onButtonPress();
  }

  unsigned long currentTime = millis();
  if (abs(currentTime - cursorTiming) > 750) {
    showCursor = !showCursor;
    if (showCursor) { LCD.cursor(); } else LCD.noCursor();
    
    cursorTiming = currentTime;
  }
}

void testInductor() {
  // Disables non-essential interrupts to increase time-accuracy. 
  setInterruptState(false);
    
  double summedESR = 0;
  double summedInductance = 0;

  for (long int i = 0; i < numberOfSamples; i++) {
    digitalWrite(INDICATOR_LED_PIN, HIGH);
      
    // Measures how long the inductor takes to charge.
    int lastVoltage = -1;
    int currentVoltage;
    unsigned long endTime;
    unsigned long startTime = micros();
    
    digitalWrite(INDUCTOR_CHARGE_PIN, HIGH);
    
    while (true) {
      currentVoltage = analogRead(INDUCTOR_VOLTAGE_PIN);
      
      if (!(currentVoltage - lastVoltage)) {
        endTime = micros();
        break;
      }
      
      lastVoltage = currentVoltage;
    }
      
    int shuntVoltage = analogRead(SHUNT_VOLTAGE_PIN);
    
    digitalWrite(INDUCTOR_CHARGE_PIN, LOW);

    // Overflow protection.
    unsigned long deltaTime = endTime > startTime ? endTime - startTime : (unsigned long) -1 - startTime + endTime;
    // Caculates the circuit resistance from shunt. R = V / (Vs / Rs)
    double circuitResistance = 5.0 / (shuntVoltage * (5.0 / 1024.0) / shuntResistance);
    
    summedESR += circuitResistance - shuntResistance;
    // Calculates inductance. L = tR / 5
    summedInductance += deltaTime / 1000000.0 * circuitResistance / 5.0;
      
    digitalWrite(INDICATOR_LED_PIN, LOW);
  }

  setInterruptState(true);

  LCD.clear();
  LCD.print("ESR: ");
  LCD.print(summedESR / (double) numberOfSamples);
  LCD.setCursor(0, 1);
  LCD.print("L: ");
  LCD.print(summedInductance / (double) numberOfSamples);

  while (!isButtonPressed()) {}
  LCD.clear();
}
