// Wykrywacz metalu typu VLF
// 
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,20,4); 


#define max_ampAverage 200       // sensitivity of detector, works same as analog potentiometer
#define TIMER1 (257)              // value to change for every other coil set. To get correct value, divide 16Mhz by 8 and then by
                                 // resonant frequency of your coil in circuit 
                                 // in my case, this frequency is 7782Hz.
#define USE_3V3_AREF  (1)        // use 3V3 as reference voltage

const int PIN_calibration = 2;    
const int PIN_buzzer = 3;         
const int coilTXpin = 5;         
const int Timer0_pinIN = 4;       //feed timer with output
const int Timer0_pinOUT = 9;

// Analog pin definitions
const int PIN_receiverinput = 0;          //sygnał z cewki odbiorczej

// zmienne używane w ISR
int16_t bins[4];                         // zebranie 4 razy wartości amplitudy w czasie 1 okresu
uint16_t num_of_samples = 0;
const uint16_t num_of_samples_to_mean = 1024;   //liczba próbek do uśrednienia

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
  pinMode(PIN_calibration, INPUT_PULLUP);  
  digitalWrite(coilTXpin, LOW);
  pinMode(coilTXpin, OUTPUT);         // wyjscie na cewkę nadawczą, generacja PWM
  digitalWrite(Timer0_pinOUT, LOW);
  pinMode(Timer0_pinOUT, OUTPUT);       // użycie timera 1 do zasilenia timera 0

  cli();
  // zatrzymanie timera0, który jest inicjowany przez Arduino
  TCCR0B = 0;        // zatrzymanie timera
   TIFR0 = 0x07;      // czyszczenie przerwań
  TIMSK0 = 0;        // wyłączenie przerwania
 
  //wyzwalanie ADC i odczytu 0 przy przepełnieniu timera1
  ADMUX = (1 << ADLAR);                   // użycie 3V3 jako napięcia odniesienia i przesunięcie w lewo
  ADCSRB = (1 << ADTS2) | (1 << ADTS1);   // wyzwalanie adc przy przepełnieniu 
  ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADATE) | (1 << ADPS2);  // włączenie adc, prescaler 16
  DIDR0 = 1;

  // ustawienie timera 1.
  // Prescaler = 1, phase correct PWM mode, TOP = ICR1A
  TCCR1A = (1 << COM1A1) | (1 << WGM11);
  TCCR1B = (1 << WGM12) | (1 << WGM13) | (1 << CS10);    // CTC mode, prescaler = 1
  TCCR1C = 0;
  OCR1AH = (TIMER1/2 >> 8);
  OCR1AL = (TIMER1/2 & 0xFF);
  ICR1H = (TIMER1 >> 8);
  ICR1L = (TIMER1 & 0xFF);
  TCNT1H = 0;
  TCNT1L = 0;
  TIFR1 = 0x07;      // czyszczenie przerwań
  TIMSK1 = (1 << TOIE1);

  // ustawienie timera 0
  // źródło zegara = T0, fast PWM mode, TOP (OCR0A) = 7, PWM output on OC0B
  TCCR0A = (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);
  TCCR0B = (1 << CS00) | (1 << CS01) | (1 << CS02) | (1 << WGM02);
  OCR0A = 7;
  OCR0B = 3;
  TCNT0 = 0;
  sei();
  while (!probkagotowa) {}    // odrzucenie pierwszej próbki
  misses = 0;
  probkagotowa = false;
  Serial.begin(19200); 
}
//przerwanie przepełnienia timera 0, inkrementacja licznika w celu zliczania czasu
ISR(TIMER1_OVF_vect)
{
  ++ticks;
  uint8_t ctr = TCNT0;
  int16_t val = (int16_t)(uint16_t)ADCH;    // odczyt 8 najbardziej znaczących bitów
  if (ctr != ((lastctr + 1) & 7))
  {
    ++misses;
  }
  lastctr = ctr;
  int16_t *p = &bins[ctr & 3];
  if (ctr < 4)
  {
    *p += (val);
    if (*p > 15000) *p = 15000;
  }
  else
  {
    *p -= val;
    if (*p < -15000) *p = -15000;
  } 
  if (ctr == 7)
  {
    ++num_of_samples;
    if (num_of_samples == num_of_samples_to_mean)
    {
      num_of_samples = 0;
      if (!probkagotowa)      //poprzednia próbka gotowa
      {
        memcpy((void*)srednia, bins, sizeof(srednia));
        probkagotowa = true;
      }
      memset(bins, 0, sizeof(bins));
    }
  }
}

void loop()
{
  while (!probkagotowa) {}
  uint32_t oldTicks = ticks;
  
  if (digitalRead(PIN_calibration) == LOW) // przycik kalibracji wciśnięty
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
  
    if (phase1 > phase2)
    {
      double temp = phase1;
      phase1 = phase2;
      phase2 = temp;
    }
    
    double phaseAverage = ((phase1 + phase2)/2.0) - phaseAdjust;
    if (phase2 - phase1 > 180.0)
    { 
      if (phaseAverage < 0.0)
      {
        phaseAverage += 180.0;
      }
      else
      {
        phaseAverage -= 180.0;
      }
    }
        
    // wypisanie osatecznie wyliczonej amplitudy sygnału
    if (ampAverage >= 0.0) Serial.write(' ');
    Serial.print(ampAverage, 1);
    Serial.write(' ');
    lcd.setCursor(0,0);
    lcd.print("          ");
    lcd.print(ampAverage);        

    if (phaseAverage >= 0.0) Serial.write(' ');
    Serial.print((int)phaseAverage);
    // decyzja czy znaleziono metal
    if (ampAverage >= prog_czulosci)
    { 
      //metale niemagnetyczne wywołują ujemne przesunięcie w fazie, natomiast magnetyczne, lekko ujemne, bądź dodatnie
      if (phaseAverage < -20.0)
      {
        Serial.print(" Non-ferrous");
        lcd.setCursor(0,0);        
        lcd.print("NonFerous ");      
      }
      else
      {
        Serial.print(" Ferrous");        
        lcd.setCursor(0,0);       
        lcd.print("Ferrous    ");                 
      }
      float temp = ampAverage;
      int sound = map (temp, 10, 200, 100, 1500);//scale amplitude to sound frequency
      tone(PIN_buzzer, sound,120);
       
      while (temp > prog_czulosci)
      {
        Serial.write('!');      
        temp -= (prog_czulosci/2);
      }
    }   
    Serial.println();
   
   }
  while (ticks - oldTicks < 8000)
  {
  }      
}
