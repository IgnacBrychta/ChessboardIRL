#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include "FastLED.h"
#include "A4988.h"
#include <EEPROM.h>

#define MS1 43 // microsteps
#define MS2 44
#define MS3 45
// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
#define RPM 120

#define BUFFER_SIZE 32
#define TIMER_PACKET_SIZE 12
#define CONFIG_PACKET_SIZE 1
#define INFORMATION_PACKET_SIZE 16
#define MOVE_PIECE_PACKET_SIZE 4

#define EEPROM_Electromagnet_x 0
#define EEPROM_Electromagnet_y 1

#define CHESSBOARD_SIZE 64
#define NUM_LEDS 256
#define DisplayColumns 2
#define HallCol1 A0
#define HallCol2 A1
#define HallCol3 A2
#define HallCol4 A3
#define HallCol5 A4
#define HallCol6 A5
#define HallCol7 A6
#define HallCol8 A7

#define HallRow1 22
#define HallRow2 23
#define HallRow3 24
#define HallRow4 25
#define HallRow5 26
#define HallRow6 27
#define HallRow7 28
#define HallRow8 29

#define DATA_PIN_LED_MATRIX 30
#define BUZZER 31

#define ForwardMotion_Steps 32
#define ForwardMotion_Direction 33
#define SidewaysMotion_Steps 34
#define SidewaysMotion_Direction 35

#define Electromagnet 36

#define ProhraTlacitkoBila 38
#define ProhraTlacitkoCerna 39
#define RemizaTlacitko 40
#define RestartTlacitko 41
#define PritelNaTelefonuTlacitko 42

#define DETECTION_THRESHOLD 600
#define LED_MATRIX_BRIGHTNESS 10

CRGB LED_Matrix[NUM_LEDS];
CRGB boardColor;
char buffer[BUFFER_SIZE];

bool figs[CHESSBOARD_SIZE];
int pieceMoved_index_1 = -1;
int pieceMoved_index_2 = -1;
uint8_t chessboardArrayIndex = 0;
char pressedButton = '0';
bool twoTimers = false;

LiquidCrystal_I2C lcd_White(0x27, INFORMATION_PACKET_SIZE, DisplayColumns);
LiquidCrystal_I2C lcd_Black(0x23, INFORMATION_PACKET_SIZE, DisplayColumns);
A4988 stepperMotor_forwardMotion(MOTOR_STEPS, ForwardMotion_Direction, ForwardMotion_Steps, MS1, MS2, MS3);
A4988 stepperMotor_sidewaysMotion(MOTOR_STEPS, SidewaysMotion_Direction, SidewaysMotion_Steps, MS1, MS2, MS3);

uint8_t stepperMotor_forwardMotion_Y = 0;
uint8_t stepperMotor_sidewaysMotion_X = 0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  SetPinModes();

  FastLED.addLeds<WS2812B, DATA_PIN_LED_MATRIX, RGB>(LED_Matrix, NUM_LEDS);
  FastLED.setBrightness(LED_MATRIX_BRIGHTNESS);
  ResetLEDmatrix();

  lcd_Black.init();
  lcd_Black.backlight();
  lcd_White.init();
  lcd_White.backlight();  
}
void LoadLastElectromagnetPosition() {
  stepperMotor_forwardMotion_Y = EEPROM.read(EEPROM_Electromagnet_x);
  stepperMotor_sidewaysMotion_X = EEPROM.read(EEPROM_Electromagnet_y);
}
void ResetLEDmatrix() {
  for(int i = 0; i < NUM_LEDS; i++) 
  {
    LED_Matrix[i] = CRGB::Black;
  }
  FastLED.show();
}
void SetPinModes() {
  pinMode(LED_BUILTIN, OUTPUT);
  // For reading each column of Hall effect sensors
  pinMode(HallCol1, INPUT);
  pinMode(HallCol2, INPUT);
  pinMode(HallCol3, INPUT);
  pinMode(HallCol4, INPUT);
  pinMode(HallCol5, INPUT);
  pinMode(HallCol6, INPUT);
  pinMode(HallCol7, INPUT);
  pinMode(HallCol8, INPUT);
  // For selecting which row of Hall effect sensors to read
  pinMode(HallRow1, OUTPUT);
  pinMode(HallRow2, OUTPUT);
  pinMode(HallRow3, OUTPUT);
  pinMode(HallRow4, OUTPUT);
  pinMode(HallRow5, OUTPUT);
  pinMode(HallRow6, OUTPUT);
  pinMode(HallRow7, OUTPUT);
  pinMode(HallRow8, OUTPUT);
  // For axis movement
  pinMode(ForwardMotion_Direction, OUTPUT);
  pinMode(ForwardMotion_Steps, OUTPUT);
  pinMode(SidewaysMotion_Direction, OUTPUT);
  pinMode(SidewaysMotion_Steps, OUTPUT);
  // Electromagnet
  pinMode(Electromagnet, OUTPUT);
  // RGB
  //pinMode(DATA_PIN_LED_MATRIX, OUTPUT);
  // Buttons
  pinMode(ProhraTlacitkoBila, INPUT);
  pinMode(ProhraTlacitkoCerna, INPUT);
  pinMode(RemizaTlacitko, INPUT); // dvě tlačítka pro jeden vstup
  pinMode(RestartTlacitko, INPUT); // jedno tlačítko
  pinMode(PritelNaTelefonuTlacitko, INPUT); // dvě tlačítka pro jeden vstup
}
bool CheckForChessboardChanges() {
  return false;
  bool figurkyZmeny[64];
  for(int row = 0; row < 8; row++) {
    digitalWrite(HallRow1 + row, HIGH); // activate Hall sensor row
    int index = row * 8;
    figurkyZmeny[index]   = IsChessPiecePresent(HallCol1);
    figurkyZmeny[index+1] = IsChessPiecePresent(HallCol2);
    figurkyZmeny[index+2] = IsChessPiecePresent(HallCol3);
    figurkyZmeny[index+3] = IsChessPiecePresent(HallCol4);
    figurkyZmeny[index+4] = IsChessPiecePresent(HallCol5);
    figurkyZmeny[index+5] = IsChessPiecePresent(HallCol6);
    figurkyZmeny[index+6] = IsChessPiecePresent(HallCol7);
    figurkyZmeny[index+7] = IsChessPiecePresent(HallCol8);
    digitalWrite(HallRow1 + row, LOW); // deactivate Hall sensor row
  }
  bool chessboardChanged = false;
  for(int i = 0; i < 64; i++) {
    if(figurkyZmeny[i] != figs[i]) {
      if(pieceMoved_index_1 == -1)
      {
        pieceMoved_index_1 = i;
      }
      else
      {
        pieceMoved_index_2 = i;
      }
      chessboardChanged = true;
    }
  }
  return chessboardChanged;
}
bool IsChessPiecePresent(int pin) {
  int analogValue = analogRead(pin);
  return analogValue > DETECTION_THRESHOLD;
}
void loop() {
  // put your main code here, to run repeatedly:
  int availableBytes = Serial.available();
  if(availableBytes > 0)
  {
    int packetType = Serial.read();
    if(packetType == 1) 
    {
      chessboardArrayIndex = 0;
      Serial.readBytes(buffer, BUFFER_SIZE);

      tone(BUZZER, 2000, 100);
      DecodeFigs();
      DecodeHighlighting();
    }
    else if(packetType == 2)
    {
      Serial.readBytes(buffer, TIMER_PACKET_SIZE);
    
      DecodeGameInfo();
    }
    else if(packetType == 3)
    {
      Serial.readBytes(buffer, CONFIG_PACKET_SIZE);
      Serial.println(F("arduino chess board init"));
      lcd_White.clear();
      lcd_Black.clear();
      ResetLEDmatrix();
      if(buffer[0] == '2')
      {
        twoTimers = true;
      }
      else
      {
        twoTimers = false;
      }
    }
    else if(packetType == 4)
    {
      lcd_White.clear();
      lcd_Black.clear();
      Serial.readBytes(buffer, INFORMATION_PACKET_SIZE);
      for(int i = 0; i < INFORMATION_PACKET_SIZE; i++)
      {
        char character = buffer[i];
        lcd_White.print(character);
        lcd_Black.print(character);
      }
    }
    else if(packetType == 5)
    {
      Serial.readBytes(buffer, MOVE_PIECE_PACKET_SIZE);
      uint8_t startRow = atoi((char*)buffer[0]);  
      uint8_t startColumn = atoi((char*)buffer[1]);
      uint8_t destRow = atoi((char*)buffer[2]);
      uint8_t destColumn = atoi((char*)buffer[3]);
      MoveOnChessboard(startRow, startColumn, destRow, destColumn);
    }
  }
  if(CheckForChessboardChanges())
  {
    Serial.println(F("piece moved"));
    Serial.print(pieceMoved_index_1);
    Serial.print(' ');
    Serial.print(pieceMoved_index_2);
    pieceMoved_index_1 = -1;
    pieceMoved_index_2 = -1;
  }
  if(CheckForButtonPress())
  {
    Serial.println(F("button pressed"));
    Serial.print(pressedButton);
    delay(350); // antispam
  }
}
void MoveOnChessboard(uint8_t startRow, uint8_t startColumn, uint8_t destRow, uint8_t destColumn) {
  const int someConst = 1; // change according to motor speed
  // set electromagnet in right (start) position
  int steps_vertical = startRow * 2 - stepperMotor_forwardMotion_Y * someConst;
  int steps_horizontal = startColumn * 2 - stepperMotor_sidewaysMotion_X * someConst;
  stepperMotor_forwardMotion.move(steps_vertical);
  stepperMotor_sidewaysMotion.move(steps_horizontal);
  digitalWrite(Electromagnet, HIGH);
  if(startRow == destRow)
  {
    if(startRow > 0)
    {
      // move piece vertically up by 1
      stepperMotor_forwardMotion.move(someConst);
    }
    else
    {
      // move piece vertically DOWN by 1
      stepperMotor_forwardMotion.move(-someConst);
    }
  }
  else if(startColumn == destColumn)
  {
    if(startColumn > 0)
    {
      // move piece horizontally left by 1
      stepperMotor_sidewaysMotion.move(someConst);
    }
    else
    {
      // move piece horizontally right by 1
      stepperMotor_sidewaysMotion.move(-someConst);
    }
    
  }
  uint8_t tilesMoved_vertical = -(startRow - destRow - 1);
  if(tilesMoved_vertical > 0) { tilesMoved_vertical -= 2; }
  steps_vertical = 2 * tilesMoved_vertical * someConst;
  stepperMotor_forwardMotion.move(steps_vertical);
  uint8_t tilesMoved_horizontal = -(startColumn - destColumn - 1);
  if(tilesMoved_horizontal > 0) { tilesMoved_horizontal -= 2; }
  steps_horizontal = 2 * tilesMoved_horizontal * someConst;
  stepperMotor_sidewaysMotion.move(steps_horizontal);

  steps_vertical = (tilesMoved_vertical - destRow) * 2 * someConst;
  stepperMotor_forwardMotion.move(steps_vertical);
  steps_horizontal = (tilesMoved_horizontal - destColumn) * 2 * someConst;
  stepperMotor_sidewaysMotion.move(steps_horizontal);

  digitalWrite(Electromagnet, LOW);
  // save electromagnet position for next move
  stepperMotor_forwardMotion_Y = destRow;
  stepperMotor_sidewaysMotion_X = destColumn;
  EEPROM.write(EEPROM_Electromagnet_x, stepperMotor_sidewaysMotion_X);
  EEPROM.write(EEPROM_Electromagnet_y, stepperMotor_forwardMotion_Y);
}
bool CheckForButtonPress() {
  if(digitalRead(ProhraTlacitkoBila) == HIGH)
  {
    pressedButton = '1';
    return true;
  }
  else if(digitalRead(ProhraTlacitkoCerna) == HIGH)
  {
    pressedButton = '2';
    return true;
  }
  else if(digitalRead(RemizaTlacitko) == HIGH)
  {
    pressedButton = '3';
    return true;
  }
  else if(digitalRead(RestartTlacitko) == HIGH)
  {
    pressedButton = '4';
    return true;
  }
  else if(digitalRead(PritelNaTelefonuTlacitko) == HIGH)
  {
    pressedButton = '5';
    return true;
  }
  return false;
}
void DecodeGameInfo() {
  // 12:34 56:79 [+|-] 99 [W|B]
  lcd_White.clear();
  lcd_Black.clear();
  if(twoTimers)
  {
    lcd_White.print(buffer[0]);
    lcd_White.print(buffer[1]);
    lcd_White.print(':');
    lcd_White.print(buffer[2]);
    lcd_White.print(buffer[3]);
    lcd_White.print(F("    "));
    if(buffer[8] == '+')
    {
      lcd_White.print('+');
    }
    else
    {
      lcd_White.print('-');
    }
    lcd_White.print(buffer[9]);
    lcd_White.print(buffer[10]);
    lcd_White.print(F("  "));
    lcd_White.print(buffer[11]);

    lcd_White.setCursor(0, 1);
    lcd_White.print(buffer[4]);
    lcd_White.print(buffer[5]);
    lcd_White.print(':');
    lcd_White.print(buffer[6]);
    lcd_White.print(buffer[7]);


    lcd_Black.print(buffer[4]);
    lcd_Black.print(buffer[5]);
    lcd_Black.print(':');
    lcd_Black.print(buffer[6]);
    lcd_Black.print(buffer[7]);
    lcd_Black.print(F("    "));
    if(buffer[8] == '+')
    {
      lcd_Black.print('-');
    }
    else
    {
      lcd_Black.print('+');
    }
    lcd_Black.print(buffer[9]);
    lcd_Black.print(buffer[10]);
    lcd_Black.print(F("  "));
    lcd_Black.print(buffer[11]);
    
    lcd_Black.setCursor(0, 1);
    lcd_Black.print(buffer[0]);
    lcd_Black.print(buffer[1]);
    lcd_Black.print(':');
    lcd_Black.print(buffer[2]);
    lcd_Black.print(buffer[3]);
  }
  else
  {
    if(buffer[11] == 'W')
    {
      lcd_White.print(buffer[0]);
      lcd_White.print(buffer[1]);
      lcd_White.print(':');
      lcd_White.print(buffer[2]);
      lcd_White.print(buffer[3]);
    }
    else {
      lcd_White.print(F("     "));
    }
    lcd_White.print(F("    "));
    if(buffer[8] == '+')
    {
      lcd_White.print('+');
    }
    else
    {
      lcd_White.print('-');
    }
    lcd_White.print(buffer[9]);
    lcd_White.print(buffer[10]);
    lcd_White.print(F("  "));
    lcd_White.print(buffer[11]);

    if(buffer[11] == 'B')
    {
      lcd_Black.print(buffer[0]);
      lcd_Black.print(buffer[1]);
      lcd_Black.print(':');
      lcd_Black.print(buffer[2]);
      lcd_Black.print(buffer[3]);
    }
    else {
      lcd_Black.print(F("     "));
    }
    lcd_Black.print(F("    "));
    if(buffer[8] == '+')
    {
      lcd_Black.print('-');
    }
    else
    {
      lcd_Black.print('+');
    }
    lcd_Black.print(buffer[9]);
    lcd_Black.print(buffer[10]);
    lcd_Black.print(F("  "));
    lcd_Black.print(buffer[11]);
  }
  
}
void DecodeHighlighting() {
  int index = 3;
  int byteCount = 0;
  int temp_iterace = 1;
  while(true) 
  {
    bool colorBit1 = GetBitFromByte(buffer[index], 2);
    bool colorBit2 = GetBitFromByte(buffer[index], 1);
    bool colorBit3 = GetBitFromByte(buffer[index], 0);
    uint8_t colorByte = 0;
    bitWrite(colorByte, 0, colorBit1);
    bitWrite(colorByte, 1, colorBit2);
    bitWrite(colorByte, 2, colorBit3);
    
    GetColorFromByte(colorByte);
    SetColor();

    colorBit1 = GetBitFromByte(buffer[index], 6);
    colorBit2 = GetBitFromByte(buffer[index], 5);
    colorBit3 = GetBitFromByte(buffer[index], 4);
    colorByte = 0;
    bitWrite(colorByte, 0, colorBit1);
    bitWrite(colorByte, 1, colorBit2);
    bitWrite(colorByte, 2, colorBit3);
    
    GetColorFromByte(colorByte);
    SetColor();
    temp_iterace++;
    byteCount++;
    index--;
    if(byteCount % 4 == 0)
    {
      index+=8; // next row
      byteCount = 0;
      chessboardArrayIndex+=16; // next row jump
      
      if(index == 35)
      {
        break;
      }
    }
  }
  FastLED.show();
}
void SetColor() {
                                                  // 46
  int modulo = chessboardArrayIndex % 16;         // 46 % 16 = 14
  int currentColumn = 16 - modulo;                // 16 - 14 = 2
  int indexPlus = currentColumn * 2 - 1;          // 2 * 4 - 1 = 3
  int index = chessboardArrayIndex + indexPlus;   // 46 + 3 = 49

  LED_Matrix[chessboardArrayIndex] = boardColor;
  LED_Matrix[index] = boardColor;
  chessboardArrayIndex++;


  modulo = chessboardArrayIndex % 16;         // 47 % 16 = 15
  currentColumn = 16 - modulo;                // 16 - 15 = 1
  indexPlus = currentColumn * 2 - 1;          // 1 * 2 - 1 = 1
  index = chessboardArrayIndex + indexPlus;   // 47 + 1 = 48

  LED_Matrix[chessboardArrayIndex] = boardColor;
  LED_Matrix[index] = boardColor;
  chessboardArrayIndex++;
}
void DecodeFigs() {
  int figIndex = 0;
  for(int i = 0; i < BUFFER_SIZE; i++) 
  {
    bool figPresent = GetBitFromByte(buffer[i], 7);
    figs[figIndex] = figPresent;
    figIndex++;
    //Serial.print(figPresent);
    figPresent = GetBitFromByte(buffer[i], 3);
    figs[figIndex] = figPresent;
    figIndex++;
    //Serial.print(figPresent);
    //if(figIndex % 8 == 0) {;
    //  Serial.println();
    //}
  }
}
bool GetBitFromByte(uint8_t b, int bitNumber) {
    return ((b >> bitNumber) & 1) == 1;
}
void GetColorFromByte(uint8_t colorNumber) {
  // NOT RGB, but GRB for some reason
  switch(colorNumber) {
    case 0:
      boardColor = CRGB::Black;
      break;
    case 0b100:
      boardColor = CRGB(148, 0, 198); // possible moves
      break;
    case 0b001:
      boardColor = CRGB(0, 255, 0); // king checkmated
      break;
    case 0b101:
      boardColor = CRGB(22, 148, 192);  // figs that can stop checkmate
      break;
    case 0b010:
      boardColor = CRGB(64, 0, 0); // last move start
      break;
    case 0b110:
      boardColor = CRGB(255, 0, 0); // last move end
      break;
    case 0b011:
      boardColor = CRGB(0, 128, 64); // possible moves; capture
      break;
  }
}