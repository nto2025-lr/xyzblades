// **************** НАСТРОЙКИ МАТРИЦЫ ****************
#define LED_PIN 13           // пин ленты
#define BRIGHTNESS 60       // стандартная маскимальная яркость (0-255)
#define CURRENT_LIMIT 2000    // лимит по току в миллиамперах, автоматически управляет яркостью (пожалей свой блок питания!) 0 - выключить лимит

#define WIDTH 16            // ширина матрицы
#define HEIGHT 16           // высота матрицы
#define SEGMENTS 1            // диодов в одном "пикселе" (для создания матрицы из кусков ленты)

#define COLOR_ORDER GRB       // порядок цветов на ленте. Если цвет отображается некорректно - меняйте. Начать можно с RGB

#define MATRIX_TYPE 0       // тип матрицы: 0 - зигзаг, 1 - последовательная
#define CONNECTION_ANGLE 0  // угол подключения: 0 - левый нижний, 1 - левый верхний, 2 - правый верхний, 3 - правый нижний
#define STRIP_DIRECTION 0   // направление ленты из угла: 0 - вправо, 1 - вверх, 2 - влево, 3 - вниз


#define DEBUG 0
#define NUM_LEDS WIDTH * HEIGHT
#include "FastLED.h"
CRGB leds[NUM_LEDS];


#include <Servo.h>


bool signl = 0;
bool flag = 1;
Servo myservo;




void setup() {
  Serial.begin(9600);
  randomSeed(analogRead(0) + analogRead(1));		// пинаем генератор случайных чисел

  // настройки ленты
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness(BRIGHTNESS);
  if (CURRENT_LIMIT > 0) FastLED.setMaxPowerInVoltsAndMilliamps(5, CURRENT_LIMIT);
  FastLED.clear();
  FastLED.show();

  // проверка правильности ориентации
  // левый нижний угол - начало координат
 
  

  Serial.begin(9600);
  pinMode(3, INPUT_PULLUP);// кнопка крышки
  pinMode(4, INPUT_PULLUP);// концевик
  pinMode(5, INPUT);//open
  pinMode(7, INPUT);//close
  myservo.attach(6);

}



void powwer(){
  FastLED.clear(true);

  drawPixelXY(1, 0 , CRGB::Blue);
  drawPixelXY(2, 0 , CRGB::Blue);
  drawPixelXY(3, 0 , CRGB::Blue);
  drawPixelXY(4, 0 , CRGB::Blue);
  drawPixelXY(9, 0 , CRGB::Blue);
  drawPixelXY(10, 0 , CRGB::Blue);
  drawPixelXY(13, 0 , CRGB::Blue);
  drawPixelXY(14, 0 , CRGB::Blue);

  drawPixelXY(1, 1 , CRGB::Blue);
  drawPixelXY(2, 1 , CRGB::Blue);
  drawPixelXY(3, 1 , CRGB::Blue);
  drawPixelXY(4, 1 , CRGB::Blue);
  drawPixelXY(9, 1 , CRGB::Blue);
  drawPixelXY(10, 1 , CRGB::Blue);
  drawPixelXY(13, 1 , CRGB::Blue);
  drawPixelXY(12, 1 , CRGB::Blue);

  drawPixelXY(1, 2 , CRGB::Blue);
  drawPixelXY(2, 2 , CRGB::Blue);
  drawPixelXY(9, 2 , CRGB::Blue);
  drawPixelXY(10 , 2 , CRGB::Blue);
  drawPixelXY(11, 2, CRGB::Blue);
  drawPixelXY(12, 2 , CRGB::Blue);

  drawPixelXY(1, 3 , CRGB::Blue);
  drawPixelXY(2, 3 , CRGB::Blue);
  drawPixelXY(9, 3 , CRGB::Blue);
  drawPixelXY(10 , 3 , CRGB::Blue);
  drawPixelXY(11, 3, CRGB::Blue);
  drawPixelXY(12, 3 , CRGB::Blue);
  drawPixelXY(3, 3 , CRGB::Blue);
  drawPixelXY(4, 3 , CRGB::Blue);

  drawPixelXY(1, 4 , CRGB::Blue);
  drawPixelXY(2, 4 , CRGB::Blue);
  drawPixelXY(9, 4 , CRGB::Blue);
  drawPixelXY(10 , 4 , CRGB::Blue);
  drawPixelXY(13, 4, CRGB::Blue);
  drawPixelXY(12, 4 , CRGB::Blue);
  drawPixelXY(3, 4 , CRGB::Blue);
  drawPixelXY(4, 4 , CRGB::Blue);

  drawPixelXY(1, 5 , CRGB::Blue);
  drawPixelXY(2, 5 , CRGB::Blue);
  drawPixelXY(9, 5 , CRGB::Blue);
  drawPixelXY(10 , 5 , CRGB::Blue);
  drawPixelXY(13, 5, CRGB::Blue);
  drawPixelXY(12, 5 , CRGB::Blue);

  drawPixelXY(1, 6 , CRGB::Blue);
  drawPixelXY(2, 6 , CRGB::Blue);
  drawPixelXY(9, 6 , CRGB::Blue);
  drawPixelXY(10 , 6 , CRGB::Blue);
  drawPixelXY(13, 6, CRGB::Blue);
  drawPixelXY(12, 6 , CRGB::Blue);
  drawPixelXY(3, 6 , CRGB::Blue);
  drawPixelXY(4, 6 , CRGB::Blue);

  drawPixelXY(1, 7 , CRGB::Blue);
  drawPixelXY(2, 7 , CRGB::Blue);
  drawPixelXY(9, 7 , CRGB::Blue);
  drawPixelXY(10 , 7 , CRGB::Blue);
  drawPixelXY(12, 7 , CRGB::Blue);
  drawPixelXY(3, 7 , CRGB::Blue);
  drawPixelXY(11, 7 , CRGB::Blue);
  drawPixelXY(4, 7 , CRGB::Blue);

  drawPixelXY(0, 9 , CRGB::Blue);
  drawPixelXY(1, 9 , CRGB::Blue);
  drawPixelXY(7, 9 , CRGB::Blue);
  drawPixelXY(14 , 9 , CRGB::Blue);
  drawPixelXY(12, 9 , CRGB::Blue);

  drawPixelXY(0, 10 , CRGB::Blue);
  drawPixelXY(1, 10 , CRGB::Blue);
  drawPixelXY(6, 10 , CRGB::Blue);
  drawPixelXY(8 , 10 , CRGB::Blue);
  drawPixelXY(11, 10 , CRGB::Blue);
  drawPixelXY(13, 10 , CRGB::Blue);
  drawPixelXY(15, 10 , CRGB::Blue);

  drawPixelXY(0, 11 , CRGB::Blue);
  drawPixelXY(1, 11 , CRGB::Blue);
  drawPixelXY(6, 11 , CRGB::Blue);
  drawPixelXY(8 , 11 , CRGB::Blue);
  drawPixelXY(2, 11 , CRGB::Blue);
  drawPixelXY(11, 11 , CRGB::Blue);
  drawPixelXY(15, 11 , CRGB::Blue);

  drawPixelXY(0, 12 , CRGB::Blue);
  drawPixelXY(1, 12 , CRGB::Blue);
  drawPixelXY(6, 12 , CRGB::Blue);
  drawPixelXY(8 , 12 , CRGB::Blue);
  drawPixelXY(3, 12 , CRGB::Blue);
  drawPixelXY(11, 12 , CRGB::Blue);
  drawPixelXY(15, 12 , CRGB::Blue);

  drawPixelXY(0, 13 , CRGB::Blue);
  drawPixelXY(1, 13 , CRGB::Blue);
  drawPixelXY(7, 13 , CRGB::Blue);
  drawPixelXY(2, 13 , CRGB::Blue);
  drawPixelXY(11, 13 , CRGB::Blue);
  drawPixelXY(15, 13 , CRGB::Blue);

  FastLED.show();
}



void loop() {
  Serial.println(digitalRead(4));

  if (digitalRead(4) == 0) {
   powwer();
   delay(1000);
   FastLED.clear(true);
   delay(1000);
  }



if ((digitalRead(7) == 1)||(digitalRead(5) == 1)) {

    if (digitalRead(5) == 1){



      drawPixelXY(2, 4 , CRGB::Red);
      myservo.write(90);
      delay(1000);
      flag = !flag;

    }
}
else {



  if (digitalRead(7) == 1){



    drawPixelXY(11, 12 , CRGB::Green);
    myservo.write(1);
    delay(1000);
    flag = !flag;
    FastLED.clear(true);
  }
                                   
}

  



}


