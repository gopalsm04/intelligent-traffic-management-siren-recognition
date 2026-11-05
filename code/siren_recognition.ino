#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Emergency_Siren_Classifier_inferencing.h>
#include "driver/i2s.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ESP32 #2 REMOTE: mic 2, Routes 3/4 and 16x2 LCD.
// Only AMBULANCE may request Route 3. Mic 2 uses stricter confirmation.
const int R3R=13,R3Y=14,R3G=18,R4R=19,R4Y=32,R4G=33;
const int LINK_RX=16,LINK_TX=17;
const bool ACTIVE_LOW=false;
const float AMB_THRESHOLD=0.95f,AMB_MARGIN=0.30f;
const uint8_t CONFIRMATIONS=5;
const unsigned long HOLD_MS=5000;

const i2s_port_t I2S_PORT=I2S_NUM_1;
const int I2S_BCLK=26,I2S_WS=25,I2S_DIN=27;
const int32_t AUDIO_GAIN=64;

LiquidCrystal_I2C lcd(0x27,16,2);
bool request=false,recording=false;
uint8_t evidence=0,warmupSlices=0;
unsigned long lastGood=0,lastPrint=0,lastStatus=0,lastMaster=0;
char shownTime[9]="--:--:--",shownSignal[8]="ALLRED";
char rxLine[40];size_t rxLength=0;
int32_t previousInput=0,previousOutput=0;

struct InferenceBuffer{
  int16_t *buffer[2];volatile uint8_t select,ready;
  volatile uint32_t count;uint32_t samplesPerSlice;
};
InferenceBuffer inference;

bool initI2S();
bool startMicrophone(uint32_t samples);
bool recordSlice();
int getAudioData(size_t offset,size_t length,float *out);
void captureTask(void *parameter);

void writeLed(int pin,bool on){digitalWrite(pin,ACTIVE_LOW?!on:on);}
void r3(bool r,bool y,bool g){writeLed(R3R,r);writeLed(R3Y,y);writeLed(R3G,g);}
void r4(bool r,bool y,bool g){writeLed(R4R,r);writeLed(R4Y,y);writeLed(R4G,g);}
void allRed(){r3(1,0,0);r4(1,0,0);}
void lcdLine(int row,const char *s){char b[17];snprintf(b,sizeof(b),"%-16.16s",s);lcd.setCursor(0,row);lcd.print(b);}
void refreshLcd(){
  char top[17];snprintf(top,sizeof(top),"%s %-7s",shownTime,shownSignal);lcdLine(0,top);
  lcdLine(1,request?"AMBULANCE R3":"SYSTEM NORMAL");
}
void sendStatus(){Serial2.println(request?"AMB2:1":"AMB2:0");lastStatus=millis();}

void processCommand(const char *s){
  if(!strcmp(s,"RTC_ERROR")){strcpy(shownTime,"RTC ERR ");lastMaster=millis();refreshLcd();return;}
  if(!strncmp(s,"TIME:",5)&&strlen(s)>=13){
    strncpy(shownTime,s+5,8);shownTime[8]=0;lastMaster=millis();refreshLcd();return;
  }
  if(!strcmp(s,"ALL_RED"))allRed();
  else if(!strcmp(s,"R1_GREEN")||!strcmp(s,"R1_YELLOW")||
          !strcmp(s,"R2_GREEN")||!strcmp(s,"R2_YELLOW"))allRed();
  else if(!strcmp(s,"R3_GREEN")){r3(0,0,1);r4(1,0,0);}
  else if(!strcmp(s,"R3_YELLOW")){r3(0,1,0);r4(1,0,0);}
  else if(!strcmp(s,"R4_GREEN")){r3(1,0,0);r4(0,0,1);}
  else if(!strcmp(s,"R4_YELLOW")){r3(1,0,0);r4(0,1,0);}
  else return;

  if(!strcmp(s,"R1_GREEN"))strcpy(shownSignal,"R1G");
  else if(!strcmp(s,"R1_YELLOW"))strcpy(shownSignal,"R1Y");
  else if(!strcmp(s,"R2_GREEN"))strcpy(shownSignal,"R2G");
  else if(!strcmp(s,"R2_YELLOW"))strcpy(shownSignal,"R2Y");
  else if(!strcmp(s,"R3_GREEN"))strcpy(shownSignal,"R3G");
  else if(!strcmp(s,"R3_YELLOW"))strcpy(shownSignal,"R3Y");
  else if(!strcmp(s,"R4_GREEN"))strcpy(shownSignal,"R4G");
  else if(!strcmp(s,"R4_YELLOW"))strcpy(shownSignal,"R4Y");
  else strcpy(shownSignal,"ALLRED");
  lastMaster=millis();refreshLcd();
}
void readMaster(){
  while(Serial2.available()){
    char c=(char)Serial2.read();
    if(c=='\n'){rxLine[rxLength]=0;processCommand(rxLine);rxLength=0;}
    else if(c!='\r'&&rxLength<sizeof(rxLine)-1)rxLine[rxLength++]=c;
  }
  if(millis()-lastMaster>3500){allRed();strcpy(shownSignal,"NO LINK");refreshLcd();}
}

void updateMic(){
  if(!recordSlice())return;
  signal_t signal;signal.total_length=EI_CLASSIFIER_SLICE_SIZE;signal.get_data=getAudioData;
  ei_impulse_result_t result={};
  EI_IMPULSE_ERROR err=run_classifier_continuous(&signal,&result,false);
  if(err!=EI_IMPULSE_OK){Serial.printf("Classifier error: %d\n",err);return;}
  if(warmupSlices<EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW){warmupSlices++;return;}

  float ambulance=0,other=0,best=0;const char *bestLabel="unknown";
  for(size_t i=0;i<EI_CLASSIFIER_LABEL_COUNT;i++){
    const char *label=result.classification[i].label;float score=result.classification[i].value;
    if(score>best){best=score;bestLabel=label;}
    if(!strcmp(label,"ambulance"))ambulance=score;else if(score>other)other=score;
  }
  bool good=ambulance>=AMB_THRESHOLD&&(ambulance-other)>=AMB_MARGIN;
  if(good){if(evidence<10)evidence++;}else if(evidence)evidence--;
  if(!request&&evidence>=CONFIRMATIONS){
    request=true;lastGood=millis();sendStatus();refreshLcd();
    Serial.println("MIC 2: AMBULANCE -> REQUEST ROUTE 3");
  }
  if(request&&good)lastGood=millis();
  if(request&&millis()-lastGood>=HOLD_MS){
    request=false;evidence=0;sendStatus();refreshLcd();Serial.println("MIC 2: REQUEST CLEARED");
  }
  if(millis()-lastPrint>=1000){lastPrint=millis();
    Serial.printf("MIC2 best=%s %.3f ambulance=%.3f evidence=%u request=%d\n",
                  bestLabel,best,ambulance,evidence,request);
  }
}

void setup(){
  Serial.begin(115200);Serial2.begin(9600,SERIAL_8N1,LINK_RX,LINK_TX);
  int pins[]={R3R,R3Y,R3G,R4R,R4Y,R4G};for(int p:pins)pinMode(p,OUTPUT);allRed();
  Wire.begin(21,22);lcd.init();lcd.backlight();lcdLine(0,"ESP32 2 STARTING");lcdLine(1,"WAIT FOR MASTER");
  run_classifier_init();
  if(!startMicrophone(EI_CLASSIFIER_SLICE_SIZE)){
    lcdLine(0,"MIC 2 ERROR");lcdLine(1,"CHECK I2S");while(true){allRed();delay(1000);}
  }
  lastMaster=millis();sendStatus();Serial.println("ESP32 #2 READY: mic2->R3");
}
void loop(){readMaster();updateMic();readMaster();if(millis()-lastStatus>=2000)sendStatus();}

bool initI2S(){
  i2s_config_t c={};c.mode=(i2s_mode_t)(I2S_MODE_MASTER|I2S_MODE_RX);
  c.sample_rate=EI_CLASSIFIER_FREQUENCY;c.bits_per_sample=I2S_BITS_PER_SAMPLE_32BIT;
  c.channel_format=I2S_CHANNEL_FMT_RIGHT_LEFT;c.communication_format=I2S_COMM_FORMAT_I2S;
  c.intr_alloc_flags=ESP_INTR_FLAG_LEVEL1;c.dma_buf_count=8;c.dma_buf_len=128;
  i2s_pin_config_t p={};p.bck_io_num=I2S_BCLK;p.ws_io_num=I2S_WS;
  p.data_out_num=I2S_PIN_NO_CHANGE;p.data_in_num=I2S_DIN;
  if(i2s_driver_install(I2S_PORT,&c,0,nullptr)!=ESP_OK)return false;
  if(i2s_set_pin(I2S_PORT,&p)!=ESP_OK){i2s_driver_uninstall(I2S_PORT);return false;}
  i2s_zero_dma_buffer(I2S_PORT);return true;
}
bool startMicrophone(uint32_t n){
  inference.buffer[0]=(int16_t*)malloc(n*sizeof(int16_t));
  inference.buffer[1]=(int16_t*)malloc(n*sizeof(int16_t));
  if(!inference.buffer[0]||!inference.buffer[1])return false;
  inference.select=0;inference.ready=0;inference.count=0;inference.samplesPerSlice=n;
  if(!initI2S())return false;recording=true;
  return xTaskCreatePinnedToCore(captureTask,"mic2_capture",8192,nullptr,10,nullptr,0)==pdPASS;
}
void captureTask(void*){
  int32_t raw[512];
  while(recording){
    size_t bytes=0;if(i2s_read(I2S_PORT,raw,sizeof(raw),&bytes,portMAX_DELAY)!=ESP_OK)continue;
    size_t words=bytes/sizeof(int32_t);
    for(size_t i=1;i<words;i+=2){
      int32_t input=raw[i]>>16;
      int32_t filtered=input-previousInput+(previousOutput*31)/32;
      previousInput=input;previousOutput=filtered;
      int32_t pcm=constrain(filtered*AUDIO_GAIN,-32768L,32767L);
      inference.buffer[inference.select][inference.count++]=(int16_t)pcm;
      if(inference.count>=inference.samplesPerSlice){
        while(inference.ready&&recording)vTaskDelay(1);
        inference.select^=1;inference.count=0;inference.ready=1;
      }
    }
  }
  vTaskDelete(nullptr);
}
bool recordSlice(){while(!inference.ready)delay(1);inference.ready=0;return true;}
int getAudioData(size_t offset,size_t length,float *out){
  numpy::int16_to_float(&inference.buffer[inference.select^1][offset],out,length);return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_MICROPHONE
#error "Edge Impulse model must be an audio model"
#endif
