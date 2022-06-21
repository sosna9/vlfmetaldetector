// Wykrywacz metalu typu VLF
// 
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,20,4); 


#define max_ampAverage 200       // zmienna definiująca czułośc detektora
#define TIMER1  (257)            // dopasowanie częstotliwości sygnału nadawanego. Aby otrzymać wynikową częstotliwość należy podzielić
                                 // 16Mhz(częstotliwość CPU) / 8(timer0), a następnie wynik podzielić przez zmienną TIMER1. częstotliwość należy 
                                 // dobrać tak, aby w obwodzie powstawał rezonans. W tym przypadku jest to 7782Hz.
#define USE_3V3_AREF  (1)        // użycie 3V3 jako napięcie referencyjne

const int PinKalibracji = 2;    // pin kalibracji
const int buzzerPIN = 3;        // generacja dźwięku
const int coilTXpin = 5;        // pin cewki nadawczej
const int Timer0_pinIN = 4;      
const int Timer0_pinOUT = 9;

// Analog pin definitions
const int receiverInputPin = 0;          //sygnał z cewki odbiorczej

// zmienne używane w ISR
int16_t bins[4];                         // zebranie 4 razy wartości amplitudy w czasie 1 okresu
uint16_t liczbaprobek = 0;
const uint16_t l_prob_do_usred = 1024;   //liczba próbek do uśrednienia

// zmienne używane zarówno w ISR jak i poza nim
volatile int16_t srednia[4];            // przechowywanie kolejnych pobranych wartości odebranego sygnału, 
volatile uint32_t ticks = 0;            // zliczanie czasu
volatile bool probkagotowa = false;     // sprawdzenie ilości zebranych próbek

// zmienne używane poza rejestrami
int16_t kalibracja[4];                                          // zmienna pobrana podczas kalibracji, odejmowana od następnie pobranych próbek.
volatile uint8_t lastctr;
volatile uint16_t misses = 0;                                  // sprawdzenie poprawności działania ADC, powinno pozostać na 0
const double halfRoot2 = sqrt(0.5);
const double quarterPi = 3.1415927/4.0;
const double RadToDeg = 180.0/3.1415927;                       //przeliczenie radianów na stopnie
const float phaseAdjust = (45.0 * 32.0)/(float)(TIMER1 + 1);   // dostosowania przesunięcia 
float prog_czulosci = 8.0;                                     // im niższa tym większa czułość, uwaga na szumy.

void setup()
{
  lcd.init();
  lcd.backlight();
  pinMode(PinKalibracji, INPUT_PULLUP);  
  digitalWrite(coilTXpin, LOW);
  pinMode(coilTXpin, OUTPUT);         // wyjscie na cewkę nadawczą, generacja PWM
  digitalWrite(Timer0_pinOUT, LOW);
  pinMode(Timer0_pinOUT, OUTPUT);       // użycie timera 1 do zasilenia timera 0

void loop()
{
  while (!probkagotowa) {}
  uint32_t oldTicks = ticks;
  
  if (digitalRead(PinKalibracji) == LOW) // przycik kalibracji wciśnięty
  {
    // Wciśnięty przycisk kalibracji, zapisywane jest przesunięcie fazowe, a następnie odejmowane od wyników sygnału odebranego.
    for (int i = 0; i < 4; ++i)
    {
      kalibracja[i] = srednia[i];
    }
    probkagotowa = false;
    Serial.print("Calibrated: ");
    
    lcd.setCursor(0,0);
    lcd.print("Calibrating...  ");    
    for (int i = 0; i < 4; ++i)
    {
      Serial.write(' ');
      Serial.print(kalibracja[i]);
    lcd.setCursor(0,1);    
    lcd.print(' ');    
    lcd.print(kalibracja[4]); 
    lcd.print("        ");     
    }
    Serial.println();
  }
  
  else  //normalna praca, gdy przycisk kalibracji nie jest wciśnięty
  {  
    for (int i = 0; i < 4; ++i)
    {
      srednia[i] -= kalibracja[i];
    }
    const double f = 200.0;
    
    // eliminacja 3 harmonicznej
    double bin0 = (srednia[0] + halfRoot2 * (srednia[1] - srednia[3]))/f;
    double bin1 = (srednia[1] + halfRoot2 * (srednia[0] + srednia[2]))/f;
    double bin2 = (srednia[2] + halfRoot2 * (srednia[1] + srednia[3]))/f;
    double bin3 = (srednia[3] + halfRoot2 * (srednia[2] - srednia[0]))/f;
    probkagotowa = false;          // zakonczono wczytywanie wartości, więc rejestr jest gotowy na nowe próbki

    double amp1 = sqrt((bin0 * bin0) + (bin2 * bin2));
    double amp2 = sqrt((bin1 * bin1) + (bin3 * bin3));
    double ampAverage = (amp1 + amp2)/2.0;
    
    // ADC sample/hold 2 clocki po przepełnieniu
    double phase1 = atan2(bin0, bin2) * RadToDeg + 45.0;
    double phase2 = atan2(bin1, bin3) * RadToDeg;
  
   }
  while (ticks - oldTicks < 8000)
  {
  }      
}
