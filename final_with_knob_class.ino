#include <U8g2lib.h>
#include <Wire.h>
#include <STM32FreeRTOS.h>
#include <math.h>
#include "knob.hpp"

#define Filter_Length 16
#define Filter_Mask 15
#define Sample_Length 8192
#define RC_Mask 8191
#define Polyphony_Max 10 //numbers of keys that can generate sound at once
#define init_tone 128

//keyboard geometry
#define piano 88
#define C4_Offset 39

#define music_size 128
#define poly_buff_size 1024

SemaphoreHandle_t keyArrayMutex;
QueueHandle_t msgOutQ;
//Pin definitions
//Row select and enable
const int RA0_PIN = D3;
const int RA1_PIN = D6;
const int RA2_PIN = D12;
const int REN_PIN = A5;

//Matrix input and output
const int C0_PIN = A2;
const int C1_PIN = D9;
const int C2_PIN = A6;
const int C3_PIN = D1;
const int OUT_PIN = D11;

//Audio analogue out
const int OUTL_PIN = A4;
const int OUTR_PIN = A3;

//Joystick analogue in
const int JOYY_PIN = A0;
const int JOYX_PIN = A1;

//Output multiplexer bits
const int DEN_BIT = 3;
const int DRST_BIT = 4;
const int HKOW_BIT = 5;
const int HKOE_BIT = 6;

//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

//DEFINE THE PHASE STEPSIZE ARRAY
const uint32_t stepSizes[] = {0x030B5BD9, 0x0339B3AD, 0x036ACCF7, 0x039ED1AA, 0x03D5EE37, 0x041051B4, 0x044E2E04,
                              0x048FB801, 0x04D527AA, 0x051EB851, 0x056CA8D2, 0x05BF3BC3};
uint8_t display_note = 255;

//GLOBAL VARIABLE
volatile uint32_t currentStepSize[Polyphony_Max] = {};

volatile uint8_t keypress = 12;//no key pressed
// volatile uint8_t keyArrayOld[7] = {0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF};//no longer used
volatile uint8_t keyArray[7];

//4 knob class instances;
Knob knobs[4] = {{0}, {8}, {8}, {16}};

volatile char noteMessage[] = "xxx";

//create a sine wave table
uint8_t sin8_t[256] = {};
int8_t cos8_t[256] = {};

volatile uint32_t Current_Sample = 0;
volatile uint32_t LP_Sample = 0;
volatile int32_t B_Sample = 0;
volatile uint8_t Gen_Mode = 2; //1 sin 2 square 4 saw 8 reserved
volatile bool sin_m_g = 1;//this is set by Gen_Mode
volatile bool square_m_g = 1;
volatile bool saw_m_g = 1;
volatile uint32_t Effect_Array[Filter_Length];
volatile uint8_t Effect_RC = 0;//rolling counter

volatile uint16_t Sample_Array[Sample_Length];
volatile uint16_t Write_RC = 0;
volatile uint16_t Read_RC = 4400; //set by delay knob.
volatile uint8_t Delay_Gain = 128; //set by delay knob. 0->no delay 255->max delay

//distortion effect variable, sample out of this range will be cut to max or min
volatile uint32_t Max_Amp = 9 << 28;
volatile uint32_t Dist_Sample = 0;
volatile uint8_t Dist_Gain = 255;

//tone bend
uint8_t tone_bend[Polyphony_Max] = {};

uint8_t tone_bend_here = 128;
uint8_t last_bend_here = 128;
int8_t local_shift=0;
bool bend_lock=0;

uint8_t last_bend[Polyphony_Max] = {};
uint32_t tune_stepsize[Polyphony_Max] = {};
uint32_t last_stepsize[Polyphony_Max] = {};
uint8_t Polyphony_RC = 0;
uint8_t Key_Id[Polyphony_Max] = {}; //id is assigned from 1
uint8_t JOYX_POS = 128;
uint16_t JOYY_POS = 512;
uint16_t JOYY_LAST = 512;

//key_info
//88 items represent every possible key
//key_map_no_sound records which keys are pressed
//key_map record what is played in the buffer/queue
bool key_map[piano] = {};
bool key_map_no_sound[piano] = {};

//envelope
//Amplitude will decay when there is at least one note are holded.
//It will only reset when no note or another note is played.
volatile uint16_t envelope_t = 255;
uint8_t decay_n = 64;
volatile uint8_t decay_d = 66; //needs to >=64
uint8_t Max_Volume = 255;

//open audio
bool play_open = 1;

int key_change = 0;
int last_key_change = 0;
//play the keyboard while the music is played in real time
//nokia ringtone note array
bool music_play = 1;
uint32_t MusicLowArray[music_size] = {0, 0, 0, 0x04000000, 0x80000000, 0, 0, 0x00080000, 0x80000000, 0, 0, 0x01000000, 0x80000000, 0x80000000};
uint32_t MusicMidArray[music_size] = {0, 0, 0, 0x02000000, 0x08000210, 0, 0x40000000, 0x00200000, 0x00800208, 0x40000000, 0x10000000, 0x00100000, 0x0800210, 0x10000110};
uint32_t MusicHighArray[music_size] = {0, 8, 2, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint32_t MusicTimeArray[music_size] = {100, 150, 150, 300, 300, 150, 150, 300, 300, 150, 180, 380, 400, 1000};//in ms

//auto accompanying : convert one or two notes to major or minor chord.
uint8_t Acco_Mode = 1;//0 free, 1 play nokia ringtone(stored music), 2 appegio, 3 accordine style, 4 tripple
volatile uint8_t tempo = 100; //change temple
volatile uint8_t chord_root = 39;
volatile uint8_t chord_third = 0;
volatile uint8_t chord_fifth = 0;
volatile bool start_ac = 1; // disable auto accomponying
uint8_t bar_count = 0;
bool root_set = 0;

//simplified band pass filters: need to work out a series of coefficients. min filter freq=85.9375, time=0.01s, samps length=22000/100=220 samples
volatile uint16_t boost_freq = 1600; //caution: due to limiting dynamic range, boost_freq must be at least an octave above the note.
volatile uint16_t last_boost_freq = 1600;
volatile uint8_t boost_MIX = 200; //0-> no effect, 255->full
int8_t B_Filter_Coe[Filter_Length];

//simplified low pass filters: need to work out a series of coefficients. min filter freq=85.9375, time=0.01s, samps length=22000/100=220 samples
volatile uint16_t pass_freq = 5000;
volatile uint16_t last_pass_freq = 5000;
volatile uint8_t pass_MIX = 128; //0-> no effect, 255->full
uint8_t LP_Filter_Coe[Filter_Length];

//control when joy stick is locked, it will only work on tone band effect, will not work on settings.
//Otherwise it set the menu. If key is pressed, it lock, otherwise unlock
volatile bool Lock_JOYX = 1;

volatile uint8_t Setting_Mode = 0;
volatile uint8_t Knob_Copy[4];//no use

//look up tables
const char *noteNames[] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B", " "};
const char intToHex[] = "0123456789ABCDEF";

//for serial
//uint8_t slave_octave = 4;

volatile char InMsg[] = "yyy";

bool refresh_f = 0; //this is used to trigger whether the machine is dead, it dead, the ISR will be jumped
bool last_refresh = 0;

//below is for version 2: this version has limitation on frequency now is 22Hz min size for each key=1024
uint8_t Poly_Buff[Polyphony_Max][poly_buff_size];
uint16_t Poly_Len[Polyphony_Max];
uint8_t key_count = 1;// never zero
#define Min_Poly_Len 255 //lower=faster, higher=more accurate tone

//LFO level
uint8_t LP_LFO = 128;
uint8_t B_LFO = 128;
uint8_t TONE_LFO = 128;
uint8_t LFO_Freq = 4; //0 to 16
bool Enable_LFO = 0;
uint8_t LFO_TONE_BEND = 128;

void setOutMuxBit(const uint8_t bitIdx, const bool value)
{
  digitalWrite(REN_PIN, LOW);
  digitalWrite(RA0_PIN, bitIdx & 0x01);
  digitalWrite(RA1_PIN, bitIdx & 0x02);
  digitalWrite(RA2_PIN, bitIdx & 0x04);
  digitalWrite(OUT_PIN, value);
  digitalWrite(REN_PIN, HIGH);
  delayMicroseconds(2);
  digitalWrite(REN_PIN, LOW);
}
//Functions to setup coefficients
void Get_B_Filter_Coefficient(int8_t *s_input, const uint16_t freq_in)
{
  for (int i = 0; i < Filter_Length; i++)
  {
    //B_Filter_Coe[i]=(int8_t) (((cosf((float)i/(float)Filter_Length*3.1415926*(float)freq_in/687.5))*127)); //more precise
    B_Filter_Coe[i] = cos8_t[(uint8_t)((float)i / (float)Filter_Length * (float)freq_in * 0.188651436993367)]; //faster
  }
}
void Get_LP_Filter_Coefficient(uint8_t *s_input, const uint16_t freq_in)
{
  uint16_t ones_index = int((float)Filter_Length / (float)freq_in * 687.5);
  if (ones_index == 0)
  {
    ones_index = 1;
  }
  if (ones_index > Filter_Length)
  {
    ones_index = Filter_Length;
  }
  for (int i = 0; i < Filter_Length; i++)
  {
    LP_Filter_Coe[i] = 0;
    if (i < ones_index)
    {
      LP_Filter_Coe[i] = 255 / ones_index;
    }
  }
}
void Get_sin8(uint8_t *s_dict)
{
  for (int i = 0; i < 256; i++)
  {
    s_dict[i] = (uint8_t)((sinf((float)i / 128.0 * 3.1415926)) * 127.0 + 127.0);
  }
}
void Get_cos8(int8_t *s_dict)
{
  for (int i = 0; i < 256; i++)
  {
    s_dict[i] = (int8_t)((cosf((float)i / 128.0 * 3.1415926)) * 127.0);
  }
}

void setup()
{
  // put your setup code here, to run once:
  keyArrayMutex = xSemaphoreCreateMutex();
  //Set pin directions
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);

  //Initialise display
  setOutMuxBit(DRST_BIT, LOW); //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH); //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH); //Enable display power supply

  //Initialise UART
  Serial.begin(115200);
  Serial.println("Hello World");

  Get_B_Filter_Coefficient(B_Filter_Coe, boost_freq);
  Get_LP_Filter_Coefficient(LP_Filter_Coe, pass_freq);
  Get_sin8(sin8_t);
  Get_cos8(cos8_t);

  //INITIALISE QUEUE
  msgOutQ = xQueueCreate(10, 4);

  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();

  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
      scanKeysTask,     /* Function that implements the task */
      "scanKeys",       /* Text name for the task */
      256,              /* Stack size in words, not bytes */
      NULL,             /* Parameter passed into the task */
      3,                /* Task priority */
      &scanKeysHandle); /* Pointer to store the task handle */

  TaskHandle_t displayUpdateHandle = NULL;
  xTaskCreate(
      displayUpdateTask,     /* Function that implements the task */
      "displayUpdate",       /* Text name for the task */
      256,                   /* Stack size in words, not bytes */
      NULL,                  /* Parameter passed into the task */
      1,                     /* Task priority */
      &displayUpdateHandle); /* Pointer to store the task handle */

  TaskHandle_t msgOutTaskHandle = NULL;
  xTaskCreate(
      msgOutTask,         /* Function that implements the task */
      "msgOut",           /* Text name for the task */
      32,                 /* Stack size in words, not bytes */
      NULL,               /* Parameter passed into the task */
      3,                  /* Task priority */
      &msgOutTaskHandle); /* Pointer to store the task handle */

  TaskHandle_t msgInTaskHandle = NULL;
  xTaskCreate(
      msgInTask,         /* Function that implements the task */
      "msgIn",           /* Text name for the task */
      32,                /* Stack size in words, not bytes */
      NULL,              /* Parameter passed into the task */
      5,                 /* Task priority */
      &msgInTaskHandle); /* Pointer to store the task handle */

  TaskHandle_t playMusicHandle = NULL;
  xTaskCreate(
      playMusicTask,     /* Function that implements the task */
      "playMusic",       /* Text name for the task */
      128,               /* Stack size in words, not bytes */
      NULL,              /* Parameter passed into the task */
      1,                 /* Task priority */
      &playMusicHandle); /* Pointer to store the task handle */

  TaskHandle_t updateChordHandle = NULL;
  xTaskCreate(
      updateChordTask,     /* Function that implements the task */
      "ChordUpdate",       /* Text name for the task */
      64,                  /* Stack size in words, not bytes */
      NULL,                /* Parameter passed into the task */
      1,                   /* Task priority */
      &updateChordHandle); /* Pointer to store the task handle */

  TaskHandle_t PolyphonyHandle = NULL;
  xTaskCreate(
      PolyphonyTask,     /* Function that implements the task */
      "PolyphonyTask",   /* Text name for the task */
      128,               /* Stack size in words, not bytes */
      NULL,              /* Parameter passed into the task */
      2,                 /* Task priority */
      &PolyphonyHandle); /* Pointer to store the task handle */

  TaskHandle_t LFOHandle = NULL;
  xTaskCreate(
      LFOTask,          /* Function that implements the task */
      "LFOTask",        /* Text name for the task */
      16,               /* Stack size in words, not bytes */
      NULL,             /* Parameter passed into the task */
      4,                /* Task priority */
      &LFOHandle); /* Pointer to store the task handle */
                        //
  vTaskStartScheduler();
}

void loop()
{
  // put your main code here, to run repeatedly:
  //scanKeysTask(NULL);//USED BEFORE, NO LONGER NEEDED AFTER THREADING
}

uint8_t readCols()
{
  uint8_t c0, c1, c2, c3;

  c0 = digitalRead(C0_PIN);
  c1 = digitalRead(C1_PIN);
  c2 = digitalRead(C2_PIN);
  c3 = digitalRead(C3_PIN);

  return ((c3 << 3) + (c2 << 2) + (c1 << 1) + c0);
}

void setRow(uint8_t rowIdx)
{
  bool x0 = true;
  bool x1 = true;
  bool x2 = true;

  if (rowIdx == 0 | rowIdx == 2 | rowIdx == 4 | rowIdx == 6)
  {
    x0 = false;
  };
  if (rowIdx == 0 | rowIdx == 1 | rowIdx == 4 | rowIdx == 5)
  {
    x1 = false;
  };
  if (rowIdx == 0 | rowIdx == 1 | rowIdx == 2 | rowIdx == 3)
  {
    x2 = false;
  };

  digitalWrite(REN_PIN, LOW);
  digitalWrite(RA0_PIN, x0);
  digitalWrite(RA1_PIN, x1);
  digitalWrite(RA2_PIN, x2);
  digitalWrite(REN_PIN, HIGH);
}

void Get_Samp(const uint32_t &phase, volatile uint32_t &samp_out, bool sin_m, bool square_m, bool saw_m, bool reserved_m) ///s_mode: 1=sin, 2=square, 4=saw, 8=white noise  >>2 avoid cliping.
{
  samp_out = 0;
  if (sin_m == 1)//sine wave mode
  {
    samp_out += (((uint32_t)(sin8_t[(uint8_t)(phase >> 24)])) << 24);

    samp_out = samp_out >> 2;
  }
  if (square_m == 1)//square wave mode
  {
    samp_out += 0;
    if (phase > 0x7fffffff)
    {
      samp_out += 0xffffffff >> 2;
    }
    if ((sin_m) == 1 || saw_m == 1)
    {
      samp_out = samp_out >> 1;
    }
  }
  if (saw_m == 1)//sawtooth wave mode
  {
    samp_out += (phase >> (2 + square_m + sin_m));
  }
  if (reserved_m == 1) //noise, reserved wave mode for future
  {
    samp_out += rand();
  }
}

void Set_Env()
{
  uint8_t tmpd;
  tmpd = 2 * knobs[1].readknob() + 64;
  u8g2.clearBuffer();
  u8g2.setCursor(2, 10);
  u8g2.print("Effect->Envelope");
  u8g2.setCursor(2, 20);
  u8g2.print("Decay per 0.1s:");
  u8g2.print(64 / (float)(tmpd));

  __atomic_store_n(&decay_d, tmpd, __ATOMIC_RELAXED);
}

void Set_Dist()
{
  uint32_t tmpMA;
  uint8_t tmpG;
  u8g2.clearBuffer();
  u8g2.setCursor(2, 10);
  u8g2.print("Effect->Overdrive");
  u8g2.setCursor(2, 20);
  u8g2.print("Clip at:");
  u8g2.print((float)(knobs[1].readknob() * 6.25));
  u8g2.setCursor(2, 30);
  u8g2.print("Level:");
  u8g2.print((float)(knobs[2].readknob() * 6.25));
  tmpMA = ((knobs[1].readknob() * 15 + 15) << 24) + 0x00FFFFFF;
  tmpG = (knobs[2].readknob()) * 15 + 15;
  __atomic_store_n(&Max_Amp, tmpMA, __ATOMIC_RELAXED);
  __atomic_store_n(&Dist_Gain, tmpG, __ATOMIC_RELAXED);
}

void Set_Echo()
{
  uint16_t tmpRC;
  uint8_t tmpG;
  u8g2.clearBuffer();
  u8g2.setCursor(2, 10);
  u8g2.print("Effect->Echo");
  u8g2.setCursor(2, 20);
  u8g2.print("Delay:");
  u8g2.print((float)(knobs[1].readknob() * 23.125));
  u8g2.print("ms");
  u8g2.setCursor(2, 30);
  u8g2.print("Level:");
  u8g2.print((float)(knobs[2].readknob() * 5.46875));
  tmpRC = Write_RC - (knobs[1].readknob()) * 512 - (!knobs[1].readknob());
  tmpG = (knobs[2].readknob()) * 14;
  __atomic_store_n(&Read_RC, tmpRC, __ATOMIC_RELAXED);
  __atomic_store_n(&Delay_Gain, tmpG, __ATOMIC_RELAXED);
}

void Set_LP()
{
  uint16_t tmpf;
  uint8_t tmpG;
  tmpf = 300 + knobs[1].readknob() * 1356;
  tmpG = (knobs[2].readknob()) * 15 + 15;
  u8g2.clearBuffer();
  u8g2.setCursor(2, 10);
  u8g2.print("Filter->Low Pass");
  u8g2.setCursor(2, 20);
  u8g2.print("Frequancy:");
  u8g2.print((tmpf));
  u8g2.print("Hz");
  u8g2.setCursor(2, 30);
  u8g2.print("Level:");
  u8g2.print((float)(knobs[2].readknob() * 6.25));
  __atomic_store_n(&pass_freq, tmpf, __ATOMIC_RELAXED);
  __atomic_store_n(&pass_MIX, tmpG, __ATOMIC_RELAXED);
}

void Set_BP()
{
  uint16_t tmpf;
  uint8_t tmpG;
  tmpf = 700 + knobs[1].readknob() * 200;
  tmpG = (knobs[2].readknob()) * 15 + 15;
  u8g2.clearBuffer();
  u8g2.setCursor(2, 10);
  u8g2.print("Filter->Band Pass");
  u8g2.setCursor(2, 20);
  u8g2.print("Frequancy:");
  u8g2.print((tmpf));
  u8g2.print("Hz");
  u8g2.setCursor(2, 30);
  u8g2.print("Level:");
  u8g2.print((float)(knobs[2].readknob() * 6.25));
  __atomic_store_n(&boost_freq, tmpf, __ATOMIC_RELAXED);
  __atomic_store_n(&boost_MIX, tmpG, __ATOMIC_RELAXED);
}

void sampleISR()
{
  static uint32_t phaseAcc[Polyphony_Max] = {};

  static bool skip_one = 0;

  static uint32_t last_currentSample = 0;
  static uint16_t dead_count = 0;

  static uint16_t Poly_Count[Polyphony_Max] = {};

  static uint32_t B_Cut = 0;
  static uint32_t LP_Cut = 0;
  static uint32_t Current_Add = 0;
/*
  if (refresh_f != last_refresh) // (this will not happen in version 2)if other task die due to too many key is pressed, the ISR will be skipped to free up resources
  {
    dead_count = 0;
    skip_one = 0;
    last_refresh = refresh_f;
  }
  else
  {
    if (dead_count > 11000){skip_one = 1;}
    else {dead_count++;}
  }
  if (!skip_one)
  {
    Current_Sample = 0;
    last_currentSample = 0;
    for (int i = 0; i < Polyphony_Max; i++)
    {
      Current_Sample += ((Poly_Buff[i][Poly_Count[i]]) << 23) / (key_count / 6 + 1);
      if (Current_Sample < last_currentSample)
      {
        Current_Sample = 0xffffffff;
      }
      last_currentSample = Current_Sample;
      (Poly_Count[i])++;
      if (Poly_Count[i] > Poly_Len[i])
      {
        Poly_Count[i] = 0;
      }
    }
    if (Current_Sample > Max_Amp){Dist_Sample = Max_Amp;}
    else{Dist_Sample = Current_Sample;}
    Current_Sample = (Current_Sample >> 2) + (Dist_Sample >> 8) * Dist_Gain;
    uint8_t count_c = 0;
    Effect_Array[Effect_RC & Filter_Mask] = Current_Sample; //store for the filter
    B_Sample = 0;
    LP_Sample = 0;
    while (count_c < Filter_Length) //convolution
    {
      B_Sample = B_Sample + (int32_t)(Effect_Array[(Effect_RC - count_c) & Filter_Mask] - 0x80000000) / 256 * (B_Filter_Coe[count_c]) / Filter_Length;
      LP_Sample = LP_Sample + (Effect_Array[(Effect_RC - count_c) & Filter_Mask]) / 256 * (LP_Filter_Coe[count_c]) / Filter_Length;
      count_c++;
    }
    B_Sample = B_Sample * 8; //do not merge this with the line below, offset will be wrong.
    B_Cut = (uint32_t)(B_Sample + 0x80000000) / 256 * boost_MIX;
    LP_Cut = LP_Sample / 16 * pass_MIX;
    if (B_Cut > (0xffffffff - LP_Cut)){Current_Sample = 0xffffffff;}
    else{Current_Sample = ((B_Cut + LP_Cut) >> 9) * envelope_t;}
    Current_Add = ((uint32_t)(Sample_Array[Read_RC & RC_Mask]) << 8) * Delay_Gain;
    if (Current_Add > (0xffffffff - Current_Sample)){Current_Sample = 0xffffffff;}
    else{Current_Sample = Current_Sample + Current_Add;}
    analogWrite(OUTR_PIN, Current_Sample >> 24);
    Sample_Array[Write_RC & RC_Mask] = (uint16_t)(Current_Sample >> 16);
    Write_RC++;
    Read_RC++;
    Effect_RC++;
  }
  */
/*
  if (refresh_f != last_refresh) // (this will not happen in version 2)if other task die due to too many key is pressed, the ISR will be skipped to free up resources
  {
    dead_count = 0;
    skip_one = 0;
    last_refresh = refresh_f;
  }
  else
  {
    if (dead_count > 11000)
    {
      skip_one = 1;
    }
    else
    {
      dead_count++;
    }
  }
  if (!skip_one)
  {*/
    Current_Sample = 0;
    last_currentSample = 0;

    for (int i = 0; i < Polyphony_Max; i++)
    {
      Current_Sample += ((Poly_Buff[i][Poly_Count[i]]) << 23) / (key_count / 6 + 1);
      if (Current_Sample < last_currentSample)
      {
        Current_Sample = 0xffffffff;
      }
      last_currentSample = Current_Sample;
      (Poly_Count[i])++;
      if (Poly_Count[i] > Poly_Len[i])
      {
        Poly_Count[i] = 0;
      }
    }

    if (Current_Sample > Max_Amp)
    {
      Dist_Sample = Max_Amp;
    }
    else
    {
      Dist_Sample = Current_Sample;
    }
    Current_Sample = (Current_Sample >> 2) + (Dist_Sample >> 8) * Dist_Gain;
    uint8_t count_c = 0;
    Effect_Array[Effect_RC & Filter_Mask] = Current_Sample; //store for the filter
    B_Sample = 0;
    LP_Sample = 0;

    while (count_c < Filter_Length) //convolution
    {
      //B_Sample = B_Sample + (int32_t)(Effect_Array[(Effect_RC - count_c) & Filter_Mask] - 0x80000000) / 256 * (B_Filter_Coe[count_c]) / Filter_Length;
      B_Sample = B_Sample + ((int32_t)(Effect_Array[(Effect_RC - count_c) & Filter_Mask] - 0x80000000)>>13)  * (B_Filter_Coe[count_c]) ;
   //   LP_Sample = LP_Sample + (Effect_Array[(Effect_RC - count_c) & Filter_Mask]) / 256 * (LP_Filter_Coe[count_c]) / Filter_Length;
     LP_Sample = LP_Sample + ((Effect_Array[(Effect_RC - count_c) & Filter_Mask]) >> 13) * (LP_Filter_Coe[count_c]);
      count_c++;
    }
    B_Sample = B_Sample << 3; //do not merge this with the line below, offset will be wrong.

    B_Cut = ((uint32_t)(B_Sample + 0x80000000)>>8) * boost_MIX;
    LP_Cut = (LP_Sample>>4) * pass_MIX;
    if (B_Cut > (0xffffffff - LP_Cut))
    {
      Current_Sample = 0xffffffff;
    }
    else
    {
      Current_Sample = ((B_Cut + LP_Cut) >> 9) * envelope_t;
    }

    Current_Add = ((uint32_t)(Sample_Array[Read_RC & RC_Mask]) << 8) * Delay_Gain;
    if (Current_Add > (0xffffffff - Current_Sample))
    {
      Current_Sample = 0xffffffff;
    }
    else
    {
      Current_Sample = Current_Sample + Current_Add;
    }

    analogWrite(OUTR_PIN, Current_Sample >> 24);
    Sample_Array[Write_RC & RC_Mask] = (uint16_t)(Current_Sample >> 16);
    Write_RC++;
    Read_RC++;
    Effect_RC++;
 /* }*/
}

void scanKeysTask(void *pvParameters)
{
  const TickType_t xFrequency = 50 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint16_t JOYX_POS_L = 512;
  uint16_t JOYY_POS_L = 512;
  static uint16_t local_envelope = Max_Volume;
  uint8_t update_count = 0;
  while (1)
  {
    refresh_f = !refresh_f;
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    //update envelope very three times
    if ((update_count)&0x3 == 0x3)
    {
      local_envelope = local_envelope * decay_n / decay_d;
      __atomic_store_n(&envelope_t, local_envelope, __ATOMIC_RELAXED);
    }
    update_count++;

    uint32_t localCurrentStepSize = 0;
    bool LocalKeyMap[piano] = {};
    bool Local_Lock_JOYX = 0;
    bool tmp_bend_lock=0;
    key_change = 0;
    for (int nrow = 0; nrow < 5; nrow++)
      {
        setRow(nrow);
        uint8_t keyoffset = nrow * 4;//each row has 4 columns
        delayMicroseconds(3);
        xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
        keyArray[nrow] = readCols();
        // read key presses
        if (nrow < 3)
        {
          if ((keyArray[nrow] & 0x1) == 0)
          {
            keypress = 0 + keyoffset;
            key_change += 1 << (0 + keyoffset);
            LocalKeyMap[((uint8_t)(C4_Offset + local_shift+ keyoffset))%piano] = 1;
          }
          if ((keyArray[nrow] & 0x2) == 0)
          {

            keypress = 1 + keyoffset;
            key_change += 1 << (1 + keyoffset);
            LocalKeyMap[((uint8_t)(1 + C4_Offset + keyoffset+local_shift))%piano] = 1;
          }
          if ((keyArray[nrow] & 0x4) == 0)
          {
            keypress = 2 + keyoffset;
            key_change += 1 << (2 + keyoffset);
            LocalKeyMap[((uint8_t)(2 + C4_Offset + local_shift+ keyoffset))%piano] = 1;
          }
          if ((keyArray[nrow] & 0x8) == 0)
          {
            keypress = 3 + keyoffset;
            key_change += 1 << (3 + keyoffset);
            LocalKeyMap[((uint8_t)(3 + C4_Offset + keyoffset+local_shift))%piano] = 1;
          }
        } //end reading tones
        ////////////////////////////////////////////////////////////////////////
        //ANOTHER method: octave is always 4 in this old version
        // if (nrow < 3) {
        //   for (int ncol = 0; ncol < 4; ncol++) { //scan cols
        //       if (!(keyArray[nrow] & (0x01 << ncol)) && (keyArrayOld[nrow] & (0x01 << ncol))) { //1 is not pressed, 0 is pressed
        //         //now 0 and previously 1, new press
        //         keypress = nrow * 4 + ncol;
        //         localCurrentStepSize = stepSizes[keypress];
        //         noteMessage[0] = 'P';
        //         noteMessage[1] = '4';
        //         noteMessage[2] = intToHex[keypress];
        //         xQueueSend( msgOutQ, (char*) noteMessage, portMAX_DELAY);
        //         __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
        //       }
        //       else if ((keyArray[nrow] & (0x01 << ncol)) && !(keyArrayOld[nrow] & (0x01 << ncol))) {
        //         //now is 1 and previously 0, new release
        //         //          keypress=nrow*4+ncol;
        //         keypress = 12;//no key [pressed]
        //         localCurrentStepSize = 0;
        //         noteMessage[0] = 'R';
        //         noteMessage[1] = '4';
        //         noteMessage[2] = intToHex[nrow * 4 + ncol];
        //         xQueueSend( msgOutQ, (char*) noteMessage, portMAX_DELAY);
        //         __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
        //       }
        //   }
        //   keyArrayOld[nrow] = keyArray[nrow];//update keyArrayOld[nrow]
        // }//end reading tones
        ///////////////////////////////////////////////////////////////////////
        if (nrow == 3)
        {
          for (int i = 2; i < 4; i++)
          {
            uint8_t localcurrentknobBA[4];
            localcurrentknobBA[2] = ((keyArray[3] & 0b1100) >> 2);
            localcurrentknobBA[3] = keyArray[3] & 0b11;
            knobs[i].updateknob(localcurrentknobBA[i]);
          }
        }
        //end of setting knob2 and knob3
        if (nrow == 4)
        {
          for (int i = 0; i < 2; i++)
          {
            uint8_t localcurrentknobBA[2];
            localcurrentknobBA[0] = ((keyArray[4] & 0b1100) >> 2);
            localcurrentknobBA[1] = (keyArray[4] & 0b11);
            knobs[i].updateknob( localcurrentknobBA[i]);
          }
        }
        //end of setting knob0 and knob1
        xSemaphoreGive(keyArrayMutex);
      }

    //key change !=0: playing, lock joy stick
    if (key_change != 0) {Local_Lock_JOYX = 1;  tmp_bend_lock=1;}
    else {Local_Lock_JOYX = 0; tmp_bend_lock=0;}

    if (key_change != last_key_change)
    {
      for (int i = C4_Offset+local_shift; i < (C4_Offset+local_shift + 12); i++) //save key map
      {
        if ((((key_change >> (i - C4_Offset-local_shift)) & 1) ^ ((last_key_change >> (i - C4_Offset-local_shift)) & 1)) == 1) //change happened!
        {
          if (((key_change >> (i - C4_Offset-local_shift)) & 1) == 1) //new key pressed
          {
            noteMessage[0] = 'P';
            noteMessage[1] =  (i +48 - C4_Offset)/12 + '0';
            noteMessage[2] = intToHex[(i + 48 - C4_Offset)%12];
            display_note = i;
            xQueueSend(msgOutQ, (char *)noteMessage, portMAX_DELAY);
          }
          else //new key released
          {
            noteMessage[0] = 'R';
            noteMessage[1] = (i + 48 - C4_Offset)/12 + '0';
            noteMessage[2] = intToHex[(i + 48 - C4_Offset)%12];
            display_note = 255;
            xQueueSend(msgOutQ, (char *)noteMessage, portMAX_DELAY);
          }
        }
        //key map stores the notes that are played
        //key map no sound stores the notes that are pressed or received
        if((i>=0) && (i<piano))
        {__atomic_store_n(&key_map[i], LocalKeyMap[i], __ATOMIC_RELAXED);
         __atomic_store_n(&key_map_no_sound[i], LocalKeyMap[i], __ATOMIC_RELAXED);
        }
      }
      last_key_change = key_change;
    }
    //play the sound
    //place info in keymap to queue
    uint8_t shift_tmp;
    bool key_exist = 0;
    uint8_t free_up = 0;
    uint8_t reset_envelope = 0;


    for (int i = 0; i < piano; i++)
    {
      key_exist = 0;
      for (uint8_t j = 0; j < Polyphony_Max; j++) //track key status
      {
        if (Key_Id[j] == 0){continue;}
        if (Key_Id[j] == (i + 1))
        {
          key_exist = 1;
          free_up = j;
        }
      }

      if (key_map[i] == 1)
      {
        reset_envelope |= 0;

        if (key_exist == 0)
        {
          reset_envelope |= 1;
          if (i < C4_Offset)
          {
            shift_tmp = 1 + (C4_Offset - i - 1) / 12;
            localCurrentStepSize = (stepSizes[(uint8_t)((i + 48 - C4_Offset) % 12)]) >> shift_tmp;
          }
          else
          {
            shift_tmp = (i - C4_Offset) / 12;
            localCurrentStepSize = (stepSizes[(uint8_t)((i + 48 - C4_Offset) % 12)]) << shift_tmp;
          }
          Key_Id[Polyphony_RC] = (i + 1);
          //key_id stores
          __atomic_store_n(&currentStepSize[Polyphony_RC], localCurrentStepSize, __ATOMIC_RELAXED);
          //Polyphony_RC is the pointer that decides where to put the new incoming key press in the queue
          Polyphony_RC++;
          if (Polyphony_RC >= Polyphony_Max) {Polyphony_RC = 0;}
        }
      }

      else if (key_map[i] == 0)
      {
        if (key_exist)
        {
          __atomic_store_n(&currentStepSize[free_up], 0, __ATOMIC_RELAXED);
          //free_up records which key has been released
          Polyphony_RC = free_up;
          Key_Id[free_up] = 0; //release from Key_ID
          Polyphony_RC = free_up;
        }
      }
    }
    //if new key pressed, we reset envelop, note that we use envelope to control volume
    if (reset_envelope == 1)
    {
      local_envelope = Max_Volume;
    } //reset envelope;

    JOYX_POS_L = (uint8_t)(pow(2, (float)(1023 - analogRead(JOYX_PIN)) / 512.0 + 6.0)); //to exponential scale
    JOYY_POS_L = (analogRead(JOYY_PIN) >> 4) << 7;                                      //quantize to save
    __atomic_store_n(&JOYX_POS, JOYX_POS_L, __ATOMIC_RELAXED);
    __atomic_store_n(&JOYY_POS, JOYY_POS_L, __ATOMIC_RELAXED);
    __atomic_store_n(&Lock_JOYX, Local_Lock_JOYX, __ATOMIC_RELAXED);
     __atomic_store_n(&bend_lock, tmp_bend_lock, __ATOMIC_RELAXED);
  }//end of while loop
}//end scanKeysTask

void RESET_PARAM()
{
  Enable_LFO=0;
  vTaskDelay(32);
 sin_m_g=0;
 square_m_g=0;
 saw_m_g=1;
 Acco_Mode=1;
 local_shift=0;
 pass_MIX=255;
 pass_freq=22000;
 boost_MIX=0;
 Delay_Gain=0;
 Max_Amp=0xffffffff;
 tone_bend_here=128;
 decay_d=64;
 for(int i=0;i<piano;i++){
   key_map[i]=0;
 }
}

void displayUpdateTask(void *pvParameters)
{ //caution!!!!, there is blocking I2C inside u8g2, therefore vTaskDelay may cause blocking if i2C is not finish.
  const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  static bool sub_effect = 0;
  static bool main_menu = 1;
  static bool sub_filter = 0;
  static bool sub_acco = 0;
  static bool sub_shift = 0;
  static bool sub_gen = 0;
  static bool sub_lfo = 0;
  static bool sub_lshift=0;

  static uint8_t reset_counter=0;

  while (1)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    //Update display
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_5x8_tr); // choose a suitable font
    //u8g2.drawStr(2,10,"Helllo World!");  // write something to the internal memory
    uint8_t keys = readCols();
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    //   Max_Volume = (int)(212 * log10(knobs[3].readknob()));
    Max_Volume = (int)( 255 * pow(1.2,knobs[3].readknob()-16));
    if(knobs[3].readknob()==0)
    {Max_Volume=0;}
    if (main_menu)
    {
      if (knobs[0].readknob() >> 1 == 0)
      {
        u8g2.clearBuffer();
        u8g2.setCursor(40, 10);
        u8g2.print("Smart Play");
        if ((JOYX_POS > 192) && (!Lock_JOYX)) //got sub effect menu
        {
          sub_acco = 1;
          main_menu = 0;
        }
      }
      else if (knobs[0].readknob() >> 1 == 1)
      {
        u8g2.clearBuffer();
        //u8g2.drawFrame(1,0,50,12);
        u8g2.setCursor(40, 10);
        u8g2.print("Set Generator");
        if ((JOYX_POS > 192) && (!Lock_JOYX)) //got sub effect menu
        {
          sub_gen = 1;
          main_menu = 0;
        }
      }
      else if (knobs[0].readknob() >> 1 == 2)
      {
        u8g2.clearBuffer();
        u8g2.setCursor(48, 10);
        u8g2.print("Filters");
        if ((JOYX_POS > 192) && (!Lock_JOYX)) //got sub effect menu
        {
          sub_filter = 1;
          main_menu = 0;
        }
      }
      else if (knobs[0].readknob() >> 1 == 4)
      {
        u8g2.clearBuffer();
        //u8g2.drawFrame(1,0,50,12);
        u8g2.setCursor(40, 10);
        u8g2.print("Global Key Shift");
        if ((JOYX_POS > 192) && (!Lock_JOYX)) //got sub effect menu
        {
          sub_shift = 1;
          main_menu = 0;
        }
      }
      else if(knobs[0].readknob()>>1 == 3)
      { u8g2.clearBuffer();
        //u8g2.drawFrame(1,0,50,12);
        u8g2.setCursor(40, 10);
        u8g2.print("Local Key Shift");
        if ((JOYX_POS > 192) && (!Lock_JOYX)) //got sub effect menu
        {
          sub_lshift = 1;
          main_menu = 0;
        }




      }
      else if (knobs[0].readknob() >> 1 == 5)
      {
        u8g2.clearBuffer();
        //u8g2.drawFrame(1,0,50,12);
        u8g2.setCursor(48, 10);
        u8g2.print("Effect");
        if ((JOYX_POS > 192) && (!Lock_JOYX)) //got sub effect menu
        {
          sub_effect = 1;
          main_menu = 0;
        }
      }
      else if (knobs[0].readknob() >> 1 == 6)
      {
        u8g2.clearBuffer();
        //u8g2.drawFrame(1,0,50,12);
        u8g2.setCursor(48,10);
        u8g2.print("LFO");
        if ((JOYX_POS > 192) && (!Lock_JOYX)) //got sub effect menu
        {
          sub_lfo = 1;
          main_menu = 0;
        }
      }
      else {
        knobs[0].restore();
      }


       u8g2.setCursor(8, 20);
       u8g2.print("note name:");
       u8g2.setCursor(78, 20);
        if (display_note < piano)
        {
          u8g2.print(noteNames[(display_note +48 - C4_Offset) % 12]);
          u8g2.print((display_note + 48 - C4_Offset) / 12);
        }
        u8g2.setCursor(8, 30);
        u8g2.print("volume level:");
        u8g2.setCursor(78, 30);
        u8g2.print(knobs[3].readknob());
        //u8g2.print(((uint16_t)(knobs[3].readknob()*15+15)*100)/255);

        if((JOYY_POS < 2000) && (!Lock_JOYX))
        {reset_counter++;
          if(reset_counter==50)
          {
            RESET_PARAM();
            u8g2.clearBuffer();
            u8g2.setCursor(30, 20);
            u8g2.print("Reseting.....");


          }

        }
        else
        {
          reset_counter=0;
        }
    }
    //below are sub menus for advanced features
     if (sub_lshift)
    {
      int16_t tmp_bend = 0;


      tmp_bend =  ((int8_t)(knobs[1].readknob() >> 1)-4) * 12 + knobs[2].readknob();


      u8g2.clearBuffer();
      //u8g2.drawFrame(1,0,50,12);
      u8g2.setCursor(2, 20);
      u8g2.print("Octave:");
      u8g2.print(tmp_bend / 12);
      u8g2.setCursor(2, 30);
      u8g2.print("Semitone:");
      u8g2.print(tmp_bend % 12);

   if(bend_lock==0)
     { __atomic_store_n(&local_shift, (int8_t)tmp_bend, __ATOMIC_RELAXED);}

      if ((JOYX_POS < 96) && (!Lock_JOYX)) //go back to main menu
      {
        sub_lshift = 0;
        main_menu = 1;
      }
    }

    if (sub_lfo == 1)
    {
      u8g2.clearBuffer();
      //u8g2.drawFrame(1,0,50,12);
      TONE_LFO = (knobs[2].readknob()) * 15;
      B_LFO = (knobs[1].readknob()) * 15 + 15;
      LFO_Freq = (knobs[0].readknob());
      if ((knobs[0].readknob() == 0))
      {
        Enable_LFO = 0;
      }
      else
      {
        Enable_LFO = 1;
      }
      u8g2.setCursor(2, 10);
      u8g2.print("LFO_Freq:");
      u8g2.print(LFO_Freq);
      u8g2.setCursor(2, 20);
      u8g2.print("Band Pass");
      u8g2.print(B_LFO);
      u8g2.setCursor(2, 30);
      u8g2.print("Tone Bend");
      u8g2.print(TONE_LFO);

      if ((JOYX_POS < 96) && (!Lock_JOYX)) //got sub effect menu
      {
        sub_lfo = 0;
        main_menu = 1;
      }
    }

    if (sub_gen == 1)
    {
      sin_m_g = (knobs[1].readknob() >> 1) & 1;
      square_m_g = (knobs[1].readknob() >> 2) & 1;
      saw_m_g = (knobs[1].readknob() >> 3) & 1;

      u8g2.clearBuffer();
      //u8g2.drawFrame(1,0,50,12);
      u8g2.setCursor(2, 10);
      u8g2.print("Sin");
      u8g2.setCursor(2, 20);
      u8g2.print("Square");
      u8g2.setCursor(2, 30);
      u8g2.print("Saw");

      u8g2.setCursor(2, 30);
      if (sin_m_g == 1)
      {
        u8g2.setCursor(64, 10);
        u8g2.print("On");
      }
      if (sin_m_g == 0)
      {
        u8g2.setCursor(64, 10);
        u8g2.print("Off");
      }

      if (square_m_g == 1)
      {
        u8g2.setCursor(64, 20);
        u8g2.print("On");
      }
      if (square_m_g == 0)
      {
        u8g2.setCursor(64, 20);
        u8g2.print("Off");
      }

      if (saw_m_g == 1)
      {
        u8g2.setCursor(64, 30);
        u8g2.print("On");
      }
      if (saw_m_g == 0)
      {
        u8g2.setCursor(64, 30);
        u8g2.print("Off");
      }

      if ((JOYX_POS < 96) && (!Lock_JOYX)) //go back to main menu
      {
        sub_gen = 0;
        main_menu = 1;
      }
    }

    if (sub_shift)
    {
      uint16_t tmp_bend = 0;
      int semi_tone;

      tmp_bend = 80 + (knobs[1].readknob() >> 1) * 12 + knobs[2].readknob();

      if (tmp_bend >= 176)
      {
        tmp_bend = 176;
      }

      semi_tone = (int)tmp_bend - 128;
      u8g2.clearBuffer();
      //u8g2.drawFrame(1,0,50,12);
      u8g2.setCursor(2, 20);
      u8g2.print("Octave:");
      u8g2.print(semi_tone / 12);
      u8g2.setCursor(2, 30);
      u8g2.print("Semitone:");
      u8g2.print(semi_tone % 12);

    //  slave_octave = 4 + semi_tone / 12; no longer used
      __atomic_store_n(&tone_bend_here, (uint8_t)tmp_bend, __ATOMIC_RELAXED);

      if ((JOYX_POS < 96) && (!Lock_JOYX)) //go back to main menu
      {
        sub_shift = 0;
        main_menu = 1;
      }
    }

    if (sub_effect)
    {
      if (knobs[0].readknob() >> 2 == 0)
      {
        Set_Echo();
      }
      if (knobs[0].readknob() >> 2 == 1)
      {
        Set_Dist();
      }
      if (knobs[0].readknob() >> 2 == 2)
      {
        Set_Env();
      }

      if ((JOYX_POS < 96) && (!Lock_JOYX)) //go back to main menu
      {
        sub_effect = 0;
        main_menu = 1;
      }
    }

    if (sub_filter == 1)
    {
      if (knobs[0].readknob() >> 3 == 0)
      {
        Set_LP();
      }
      if (knobs[0].readknob() >> 3 == 1)
      {
        Set_BP();
      }

      if ((JOYX_POS < 96) && (!Lock_JOYX)) //go back to main menu
      {
        sub_filter = 0;
        main_menu = 1;
      }
    }

    if (sub_acco == 1)
    {
      if (knobs[0].readknob()>>1 == 0)
      {
        Acco_Mode = 0;
        u8g2.clearBuffer();
        //u8g2.drawFrame(1,0,50,12);
        u8g2.setCursor(40, 10);
        u8g2.print("Free Play");
        u8g2.setCursor(58, 20);
        if (display_note < piano)
        {
          u8g2.print(noteNames[(display_note + 48 - C4_Offset) % 12]);
          u8g2.print((display_note + 48 - C4_Offset) / 12);
        }

      }
      else if (knobs[0].readknob()>>1 == 1)
      {

        Acco_Mode = 1;
        u8g2.clearBuffer();
        //u8g2.drawFrame(1,0,50,12);
        u8g2.setCursor(40, 10);
        u8g2.print("Music Player");
        u8g2.setCursor(8, 30);
        u8g2.print("Nokia Ringtone");
        //NEW
        u8g2.setCursor(58, 20);
        if (display_note < piano)
        {
          u8g2.print(noteNames[(display_note + 48 - C4_Offset) % 12]);
          u8g2.print((display_note + 48 - C4_Offset) / 12);
        }
      }
      else if (knobs[0].readknob()>>1 == 2)
      {
        tempo = 15 + (knobs[1].readknob()) * 14 + knobs[2].readknob();
        Acco_Mode = 2;
        u8g2.clearBuffer();
        //u8g2.drawFrame(1,0,50,12);
        u8g2.setCursor(40, 10);
        u8g2.print("Arpeggio");
        u8g2.setCursor(58, 20);
        u8g2.print(noteNames[(chord_root + 48 - C4_Offset) % 12]);
        u8g2.print((chord_root + 48 - C4_Offset) / 12);
        if(chord_third-chord_root==3)
        {
          u8g2.print(" Minor");
        }
        else
        {
          u8g2.print(" Major");
        }
        u8g2.setCursor(2, 30);
        u8g2.print("Tempo:");
        u8g2.print(tempo);
        u8g2.print("   Bar:");
        u8g2.print(1 + (bar_count / 2) % 4);
      }
      else if (knobs[0].readknob()>>1 == 3)
      {
        tempo = 15 + (knobs[1].readknob()) * 14 + knobs[2].readknob();
        Acco_Mode = 4;
        u8g2.clearBuffer();
        //u8g2.drawFrame(1,0,50,12);
        u8g2.setCursor(40, 10);
        u8g2.print("Waltz");
        u8g2.setCursor(58, 20);
        u8g2.print(noteNames[(chord_root + 48 - C4_Offset) % 12]);
        u8g2.print((chord_root + 48 - C4_Offset) / 12);
        if(chord_third-chord_root==3)
        {
          u8g2.print(" Minor");
        }
        else
        {
          u8g2.print(" Major");
        }
        u8g2.setCursor(2, 30);
        u8g2.print("Tempo:");
        u8g2.print(tempo);
        u8g2.print("   Bar:");
        u8g2.print(1 + (bar_count) % 3);
      }
      else if (knobs[0].readknob()>>1 == 4)
      {
        tempo = 15 + (knobs[1].readknob()) * 14 + knobs[2].readknob();
        Acco_Mode = 3;
        u8g2.clearBuffer();
        //u8g2.drawFrame(1,0,50,12);
        u8g2.setCursor(40, 10);
        u8g2.print("Folk");
        u8g2.setCursor(58, 20);
        u8g2.print(noteNames[(chord_root + 48 - C4_Offset) % 12]);
        u8g2.print((chord_root + 48 - C4_Offset) / 12);
        if(chord_third-chord_root==3)
        {
          u8g2.print(" Minor");
        }
        else
        {
          u8g2.print(" Major");
        }
        u8g2.setCursor(2, 30);
        u8g2.print("Tempo:");
        u8g2.print(tempo);
        u8g2.print("   Bar:");
        u8g2.print(1 + (bar_count / 2) % 4);
      }
      else
      {
        knobs[0].restore();
      }
      u8g2.setCursor(8, 20);
      u8g2.print("note name:");
       u8g2.setCursor(92, 30);
       u8g2.print("Vol:");
       u8g2.print(knobs[3].readknob());
       //u8g2.print(((uint16_t)(knobs[3].readknob()*15+15)*100)/255);
      if ((JOYX_POS < 96) && (!Lock_JOYX)) //go back to main menu
      {
        sub_acco = 0;
        main_menu = 1;
      }
    }

    xSemaphoreGive(keyArrayMutex);

    u8g2.sendBuffer(); // transfer internal memory to the display
    //Toggle LED
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}//end displayUpdateTask


void msgOutTask(void *pvParameters)
{
  char outMsg[4];
  while (1)
  {
    xQueueReceive(msgOutQ, outMsg, portMAX_DELAY);
    Serial.println(outMsg);
  }
}//end msgOutTask

void msgInTask(void *pvParameters)
{
  const TickType_t xFrequency = 5 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  byte msg;
  int character_counter = 0;
  uint8_t notenumber;
  uint32_t notestep;
  while(1)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    while (Serial.available() > 0)
    {
      msg = Serial.read();
      if (character_counter < 3)
      {
        InMsg[character_counter] = msg;
      }
      character_counter++;
      if (msg == '\n')
      {
        //decoding
        character_counter = 0;
        if ((InMsg[0] == 'R') && (InMsg[1] - '0' < 8))
        {
          if (InMsg[2] <= '9')
          {
            key_map[(InMsg[1] - '0') * 12 - 9 + InMsg[2] - '0'] = 0;
            key_map_no_sound[(InMsg[1] - '0') * 12 - 9 + InMsg[2] - '0'] = 0;
            display_note=255;
          }
          else
          {
            key_map[(InMsg[1] - '0') * 12 - 9 + InMsg[2] - '7'] = 0;
            key_map_no_sound[(InMsg[1] - '0') * 12 - 9 + InMsg[2] - '7'] = 0;
            display_note=255;
          }
        }
        else if ((InMsg[0] == 'P') && (InMsg[1] - '0' < 8))
        {
          if (InMsg[2] <= '9')
          {
            key_map[(InMsg[1] - '0') * 12 - 9 + InMsg[2] - '0'] = 1;
            key_map_no_sound[(InMsg[1] - '0') * 12 - 9 + InMsg[2] - '0'] = 1;
            display_note=(InMsg[1] - '0') * 12 - 9 + InMsg[2] - '0';
          }
          else
          {
            key_map[(InMsg[1] - '0') * 12 - 9 + InMsg[2] - '7'] = 1;
            key_map_no_sound[(InMsg[1] - '0') * 12 - 9 + InMsg[2] - '7'] = 1;
            display_note=(InMsg[1] - '0') * 12 - 9 + InMsg[2] - '7';
          }
        }
      }
    }
  }
}//end msgInTask

void LFOTask(void *pvParameters)
{
  static uint8_t LFO_count = 0;
  static uint8_t LFO_VAL = 0;
  while (1)
  {
    if (Enable_LFO)
    {

      if (LFO_Freq != 0)
      {
        LFO_VAL = sin8_t[LFO_count];
        LFO_VAL = (uint8_t)(pow(2, (float)((uint16_t)LFO_VAL << 2) / 512.0 + 6.0)); //64-255
        LFO_TONE_BEND = (uint16_t)(((int)LFO_VAL - 128) * TONE_LFO / 255 + 128);
        boost_freq = (uint16_t)(((int)LFO_VAL - 128) * B_LFO / 155 + 128) * 16;
        //   LFO_TONE_BEND = (uint8_t) (((int)LFO_VAL - 128) * TONE_LFO / 128 + 128); bad sound
        LFO_count = LFO_count + (LFO_Freq<<2);
        vTaskDelay(16);
      } //8 point LFO
      else{vTaskDelay(100/portTICK_PERIOD_MS);}
    }
    else
    {
      LFO_TONE_BEND = 128;
      vTaskDelay(100/portTICK_PERIOD_MS);
    }
  }
}//end LFOTask

void PolyphonyTask(void *pvParameters)
{ static uint32_t phaseAcc[Polyphony_Max] = {};
  static uint32_t lastphaseAcc[Polyphony_Max] = {};
  static uint32_t tmp_sample = 0;
  static int16_t tmp_freq = 0;
  static uint8_t local_key_count = 1;
  //static uint16_t Poly_Len_L[Polyphony_Max]
  volatile uint32_t lastcurrentStepSize[Polyphony_Max] = {}; //{0x030B5BD9 ,0x048FB801};
  static uint16_t Last_Total_Bend = 0;

  TickType_t xFrequency = 10 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1)
  { xFrequency = 10 / portTICK_PERIOD_MS;
    local_key_count = 0;


    for (int i = 0; i < Polyphony_Max; i++)
    {
      if ((currentStepSize[i]) != 0) //SPEED!! do not waste time on evaluating none pressed key.
      {
        local_key_count++;



        if ((last_bend_here) != (tone_bend_here) || (last_stepsize[i]) != (currentStepSize[i])) //speed up, do not bend the tone when no needed
        {                                                                                       //Wave_Core_Write_Done=0; //update the wave core
          (tune_stepsize[i]) = (uint32_t)((float)(currentStepSize[i]) * powf(1.05946309436, (tone_bend_here)-128));
          (last_bend_here) = tone_bend_here;
          (last_stepsize[i]) = currentStepSize[i];
        }
      }

      phaseAcc[i] = 0;
      lastphaseAcc[i] = 0;

      if ((currentStepSize[i] != 0) && ((lastcurrentStepSize[i] != currentStepSize[i]) || (Last_Total_Bend != (JOYX_POS * LFO_TONE_BEND)))) //generate some cycle sample for each keys
      {
        xFrequency += 5 / portTICK_PERIOD_MS; //add 5ms delay to initiation interval for each process key

        for (int j = 0; j < poly_buff_size; j++)
        {
          phaseAcc[i] += ((tune_stepsize[i]) >> 14) * Last_Total_Bend;
          Get_Samp(phaseAcc[i], tmp_sample, sin_m_g, square_m_g, saw_m_g, 0);
          __atomic_store_n(&Poly_Buff[i][j], tmp_sample >> 24, __ATOMIC_RELAXED);

          if (phaseAcc[i] <= lastphaseAcc[i])
          {
            if (j >= Min_Poly_Len) //store more sample if memory is enough to make tone accurate
            {
              __atomic_store_n(&Poly_Len[i], j, __ATOMIC_RELAXED);
              break;
            }
          }
          lastphaseAcc[i] = phaseAcc[i];
        }
      }
      else if (currentStepSize[i] == 0)
      {
        __atomic_store_n(&Poly_Len[i], 0, __ATOMIC_RELAXED);
      }
      lastcurrentStepSize[i] = currentStepSize[i];
    }

      if (JOYY_POS != JOYY_LAST || boost_freq != last_boost_freq)
        {
          tmp_freq = boost_freq + JOYY_POS - 4096;
          if (tmp_freq < 700)
          {
            tmp_freq = 700;
          }
          Get_B_Filter_Coefficient(B_Filter_Coe, tmp_freq);
          JOYY_LAST = JOYY_POS;
          last_boost_freq = boost_freq;
        }
        else if (JOYY_POS != JOYY_LAST || last_pass_freq != pass_freq) //make one filter at a time, save resource
        {
          tmp_freq = pass_freq + JOYY_POS - 4096;
          if (tmp_freq < 1)
          {
            tmp_freq = 1;
          }

          Get_LP_Filter_Coefficient(LP_Filter_Coe, tmp_freq);
          last_pass_freq = pass_freq;
        }
     Last_Total_Bend = JOYX_POS * LFO_TONE_BEND;
    if (local_key_count == 0)
    {
      local_key_count = 1;
    }
    __atomic_store_n(&key_count, local_key_count, __ATOMIC_RELAXED);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}//end PolyphonyTask

void updateChordTask(void *pvParameters)
{ //scan keys to determine chord

  uint8_t local1 = 39;
  uint8_t local3 = 43;
  uint8_t local5 = 46;
  uint8_t next_root = 39;
  uint8_t next_third = 43;
  uint8_t next_fifth = 46;

  vTaskDelay(1000/portTICK_PERIOD_MS);
  __atomic_store_n(&chord_root, local1, __ATOMIC_RELAXED);
  __atomic_store_n(&chord_third, local3, __ATOMIC_RELAXED);
  __atomic_store_n(&chord_fifth, local5, __ATOMIC_RELAXED);
  while (1)
  {
    if (Acco_Mode != 1 && Acco_Mode != 0)
    {
      for (int j = 0; j < piano; j++)
      {
        if (key_map_no_sound[j] == 1)
        {
          next_root = j;
          next_third = next_root + 4;
          next_fifth = next_root + 7;
          break;
        }
      }
      if (next_root >= 27 && next_root < 63)
      {
        if ((key_map_no_sound[next_root - 3] == 1) && (key_map_no_sound[next_root] == 1))
        {
          next_root = next_root - 3;
          next_third = next_root + 4;
          next_fifth = next_root + 7;
        }
        if (key_map_no_sound[next_root - 4] == 1 && (key_map_no_sound[next_root] == 1))
        {
          next_root = next_root - 4;
          next_third = next_root + 4;
          next_fifth = next_root + 7;
        }

        if (key_map_no_sound[next_root + 3] == 1)
        {
          next_third = next_root + 3;
        }
        else if (key_map_no_sound[next_root + 4] == 1)
        {
          next_third = next_root + 4;
        }

        if (next_root != local1 || next_third != local3)
        {
          for (int j = 0; j < piano; j++)
          {
            if (key_map[j] == 1)
            {
              key_map[j] = 0 || key_map_no_sound[j];
            }
          }
          local1 = next_root;
          local3 = next_third;
          local5 = next_fifth;
          __atomic_store_n(&chord_root, local1, __ATOMIC_RELAXED);
          __atomic_store_n(&chord_third, local3, __ATOMIC_RELAXED);
          __atomic_store_n(&chord_fifth, local5, __ATOMIC_RELAXED);
          //   vTaskDelay((int)((1/(float)tempo)*15000));
        }
      }
    }
    vTaskDelay(100/portTICK_PERIOD_MS);
  }
}//end updateChordTask

void playMusicTask(void *pvParameters)
{ TickType_t xFrequency = 20 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (1)
  {
    if (play_open == 1) //this write all the position of key map respectively to check key sound
    {
      uint8_t stop_up = 60;
      uint8_t play_note_now = 0;

      for (int i = 0; i < 70; i++)
      {
        play_note_now = i;
        if (play_note_now > stop_up)
        {
          play_note_now = stop_up;
        }

        __atomic_store_n(&key_map[play_note_now], 1, __ATOMIC_RELAXED);
        __atomic_store_n(&key_map[play_note_now + 4], 1, __ATOMIC_RELAXED);
        __atomic_store_n(&key_map[play_note_now + 7], 1, __ATOMIC_RELAXED);
        vTaskDelay(50/portTICK_PERIOD_MS); //no precise timing required
        __atomic_store_n(&key_map[play_note_now], 0, __ATOMIC_RELAXED);
        __atomic_store_n(&key_map[play_note_now + 4], 0, __ATOMIC_RELAXED);
        __atomic_store_n(&key_map[play_note_now + 7], 0, __ATOMIC_RELAXED);
      }
      play_open = 0;
    }

    if (Acco_Mode == 0)
    {
      vTaskDelay(500/portTICK_PERIOD_MS);//no precise timing required
    }

    if (Acco_Mode == 2) //auto appagio
    {
      bar_count = 0;
      xFrequency = (int)((1 / (float)tempo) * 30000) / portTICK_PERIOD_MS;
      for (int i = 0; i < 8; i++)
      {

        if (i & 1 == 1)
        {
          key_map[chord_fifth - 12] = 1;

          vTaskDelayUntil(&xLastWakeTime, xFrequency);
        //  vTaskDelay((int)((1 / (float)tempo) * 30000));
          key_map[chord_fifth - 12] = 0 || key_map_no_sound[chord_fifth - 12];
          bar_count++;
        }
        else if (i == 0)
        {
          key_map[chord_root - 12] = 1;
          vTaskDelayUntil(&xLastWakeTime, xFrequency);
     //     vTaskDelay((int)((1 / (float)tempo) * 30000));
          key_map[chord_root - 12] = 0 || key_map_no_sound[chord_root - 12];
          bar_count++;
        }
        else if (i == 4)
        {
          key_map[chord_third] = 1;
          vTaskDelayUntil(&xLastWakeTime, xFrequency);
   //       vTaskDelay((int)((1 / (float)tempo) * 30000));
          key_map[chord_third] = 0 || key_map_no_sound[chord_third];
          bar_count++;
        }
        else
        {
          key_map[chord_root] = 1;
          vTaskDelayUntil(&xLastWakeTime, xFrequency);
    //      vTaskDelay((int)((1 / (float)tempo) * 30000));
          key_map[chord_root] = 0 || key_map_no_sound[chord_root];
          bar_count++;
        }
      }
    }

    if (Acco_Mode == 3) //four beats accordine style
    {
      bar_count = 0;
      xFrequency = (int)((1 / (float)tempo) * 30000) / portTICK_PERIOD_MS;
      for (int i = 0; i < 8; i++)
      {

        if ((i & 0x3) == 0)
        {
          key_map[chord_root - 12] = 1;
          key_map[chord_root - 24] = 1;
          key_map[chord_root] = 1;
               vTaskDelayUntil(&xLastWakeTime, xFrequency);
          key_map[chord_root - 12] = 0 || key_map_no_sound[chord_root - 12];
          key_map[chord_root - 24] = 0 || key_map_no_sound[chord_root - 24];
          key_map[chord_root] = 0 || key_map_no_sound[chord_root];

          bar_count++;
        }
        else if ((i & 0x3) == 2)
        {
          key_map[chord_fifth - 12] = 1;
          key_map[chord_fifth - 24] = 1;
          key_map[chord_fifth] = 1;
              vTaskDelayUntil(&xLastWakeTime, xFrequency);
          key_map[chord_fifth - 12] = 0 || key_map_no_sound[chord_fifth - 12];
          key_map[chord_fifth - 24] = 0 || key_map_no_sound[chord_fifth - 24];
          key_map[chord_fifth] = 0 || key_map_no_sound[chord_fifth];

          bar_count++;
        }
        else
        {
          key_map[chord_third + 12] = 1;
          key_map[chord_third] = 1;
          key_map[chord_fifth + 12] = 1;
          key_map[chord_fifth] = 1;

               vTaskDelayUntil(&xLastWakeTime, xFrequency/3);
          key_map[chord_third + 12] = 0 || key_map_no_sound[chord_third + 12];
          key_map[chord_third] = 0 || key_map_no_sound[chord_third];
          key_map[chord_fifth + 12] = 0 || key_map_no_sound[chord_fifth + 12];
          key_map[chord_fifth] = 0 || key_map_no_sound[chord_fifth];
              vTaskDelayUntil(&xLastWakeTime, xFrequency/3*2);
          bar_count++;
        }
      }
    }

    if (Acco_Mode == 4) //triple
    {

      bar_count = 0;
       xFrequency = (int)((1 / (float)tempo) * 60000) / portTICK_PERIOD_MS;

      for (int i = 0; i < 6; i++)
      {

        if ((i & 0x7) == 0)
        {
          key_map[chord_root - 12] = 1;
          key_map[chord_root - 24] = 1;
          key_map[chord_root] = 1;
          vTaskDelayUntil(&xLastWakeTime, xFrequency);
          key_map[chord_root - 12] = 0 || key_map_no_sound[chord_root - 12];
          key_map[chord_root - 24] = 0 || key_map_no_sound[chord_root - 24];
          key_map[chord_root] = 0 || key_map_no_sound[chord_root];

          bar_count++;
        }
        else if ((i & 0x7) == 3)
        {
          key_map[chord_fifth - 12] = 1;
          key_map[chord_fifth - 24] = 1;
          key_map[chord_fifth] = 1;
         vTaskDelayUntil(&xLastWakeTime, xFrequency);
          key_map[chord_fifth - 12] = 0 || key_map_no_sound[chord_fifth - 12];
          key_map[chord_fifth - 24] = 0 || key_map_no_sound[chord_fifth - 24];
          key_map[chord_fifth] = 0 || key_map_no_sound[chord_fifth];

          bar_count++;
        }
        else
        {
          key_map[chord_third + 12] = 1;
          key_map[chord_third] = 1;
          key_map[chord_fifth + 12] = 1;
          key_map[chord_fifth] = 1;

          vTaskDelayUntil(&xLastWakeTime, xFrequency/3);
          key_map[chord_third + 12] = 0 || key_map_no_sound[chord_third + 12];
          key_map[chord_third] = 0 || key_map_no_sound[chord_third];
          key_map[chord_fifth + 12] = 0 || key_map_no_sound[chord_fifth + 12];
          key_map[chord_fifth] = 0 || key_map_no_sound[chord_fifth];
          vTaskDelayUntil(&xLastWakeTime, xFrequency/3*2);
          bar_count++;
        }
      }
    }

    if (Acco_Mode == 1 )
    {
      for (int i = 0; i < music_size; i++)
      {
        for (int j = 0; j < 32; j++)
        {
          if ((((MusicLowArray[i]) >> j) & 0x1) == 1)
          {
            __atomic_store_n(&key_map[j], 1, __ATOMIC_RELAXED);
          }
          else
          {
            if (key_map_no_sound[j] == 0)
            {
              __atomic_store_n(&key_map[j], 0, __ATOMIC_RELAXED);
            }
          }
        }
        for (int j = 0; j < 32; j++)
        {
          if ((((MusicMidArray[i]) >> j) & 0x1) == 1)
          {
            __atomic_store_n(&key_map[j + 32], 1, __ATOMIC_RELAXED);
          }
          else
          {
            if (key_map_no_sound[j + 32] == 0)
            {
              __atomic_store_n(&key_map[j + 32], 0, __ATOMIC_RELAXED);
            }
          }
        }
        for (int j = 0; j < 24; j++)
        {
          if ((((MusicHighArray[i]) >> j) & 0x1) == 1)
          {
            __atomic_store_n(&key_map[j + 64], 1, __ATOMIC_RELAXED);
          }
          else
          {
            if (key_map_no_sound[j + 64] == 0)
            {
              __atomic_store_n(&key_map[j + 64], 0, __ATOMIC_RELAXED);
            }
          }
        }

         vTaskDelay(MusicTimeArray[i]/portTICK_PERIOD_MS);

      }
      Acco_Mode = 0;
    }
    if(Acco_Mode>4)
    {
      vTaskDelay(500/portTICK_PERIOD_MS);
    }
  }
}//end playMusicTask
