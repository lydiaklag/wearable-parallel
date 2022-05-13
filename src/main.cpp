#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <SPI.h>
#include <ADXL362.h>
#include "max30105.h"
#include "IIRFilter.h"
#include <Time.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

#define WIFI_NETWORK "secret passage_plus"  //the name of the wifi network //at lab it is ESP_Test
#define WIFI_PASSWORD "Afra!17p89#"    //at lab it is esp8266_test
#define WIFI_TIMEOUT_MS 20000
#define WIFI_SSID "secret passage_plus"

#define API_KEY "AIzaSyAIE_5ozQRoAcZaprySgTDVu_YB7QJycik"
#define DATABASE_URL "https://accel-42dfb-default-rtdb.europe-west1.firebasedatabase.app/"

void setup();
void loop();
void connectToWiFi(void *parameters);
void measurementsAccel(void *parameter);

void iir_ma_filter_for_hr();
int Find_Peak(int p1, int p2, int *x);
int Find_Mean(int p1, int p2, int *x);
int find_min_negative(int p1, int p2, int *x);
void convert_signal_to_positive(int p1, int p2, int *x, int point);
void ComputeHeartRate();
int readSamples();
void loopHR(void * pvParameters);

static int taskCoreHR = 0;
static int taskCoreAccel = 1;
unsigned long t_accel; 
unsigned long t_HR; 

SemaphoreHandle_t baton;

int Sampling_Time = 2400;  //(sampl_time =40ms-->25 samples/sec--> fs=25Hz //ACCEL, 2400 = 4sec
//normally it is 40, now I made it 2400ms which means 4 seconds 
ADXL362 xl;
MAX30105 Sensor;
int16_t temp;
int16_t XValue, YValue, ZValue, Temperature;
int distance;
void *aaa;

//***Define signal parameters for HR
int samp_freq=25; //for each led
const int Num_Samples = 100;  //it stores 4 sec 

uint32_t gr_buffer[Num_Samples];
int filtered_gr_buffer[Num_Samples];
int ma_gr_buffer[Num_Samples];

const int points_pr=4;
float PR[points_pr];
float Pulse_Rate_next=0, Pulse_Rate_previous=70;
int HR;

//***Butterworth band-pass (0.5Hz-5Hz) 2nd order
const double b[] = {0.175087643672101,0,-0.350175287344202,0,0.175087643672101};
const double a[] = {1,-2.299055356038497,1.967497759984451,-0.874805556449481,0.219653983913695};

IIRFilter f(b, a);
double filtered_gr=0;

int Moving_Average_Num = 2;
int Num_Points = 2*Moving_Average_Num+1;  //***5-point moving average filter
int Sum_Points;

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
unsigned long sendDataPrevMillis = 0;
int count_accel = 0;
int count_HR = 0;
bool signupOK = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);           //for serial monitor, baud rate
  xl.begin(5);                   // Setup SPI protocol, issue device soft reset
  xl.beginMeasure();            //these 2 lines are about 362
  baton = xSemaphoreCreateMutex();
  
  connectToWiFi(&aaa);
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  if (Firebase.signUp(&config, &auth, "", "")){
    Serial.println("ok");
    signupOK = true;
  }
  else{
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  
  // Initialize sensor HR
  if (Sensor.begin() == false)  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  byte ledBrightness = 0xDF; //Options: 0=Off to 255=50mA  --> DF=~44mA
  byte sampleAverage = 2;    //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 3;          //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  //byte ledMode = 1;  //playing around
  int sampleRate = 200;      //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411;      //Options: 69, 118, 215, 411
  int adcRange = 2048;       //Options: 2048, 4096, 8192, 16384
  Sensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
  Sensor.setPulseAmplitudeRed(0xFF); //all LEDs are on this way 
  Sensor.setPulseAmplitudeIR(0xFF); //turning on the IR fully 
  //need to put the next 2 lines inside the interrupt 
  // Sensor.setPulseAmplitudeRed(0);
  // Sensor.setPulseAmplitudeIR(0);
  //how to check if IR is on, use phone camera (video)
// Maria's version had the above line as
// Sensor.setPulseAmplitudeRed(0);
//I need to use setPulseAmplitudeGreen, setPulseAmplitudeIR 

  xTaskCreatePinnedToCore(
  //xTaskCreate(  
    measurementsAccel,
    "accel measurements 362",
    10000,
    NULL,
    1,
    NULL,
    taskCoreAccel
  );
  xTaskCreatePinnedToCore(
  //xTaskCreate(  
    loopHR,
    "loop",
    10000, //need to check if that is the ideal stack size 
    NULL,
    1,
    NULL,
    taskCoreHR
  );
}

void connectToWiFi(void *parameters) {
  for(;;){
    Serial.print("Connecting to Wifi");
    WiFi.mode(WIFI_STA);  //station mode, to connect to an existing wifi
    //AP mode to create a wifi network with esp32, to let someone else configure it by connecting to it 
    WiFi.begin(WIFI_NETWORK, WIFI_PASSWORD);
    Serial.print("after t has begun");
    unsigned long startAttemptTime = millis();  //uptime of esp32

    while(WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < WIFI_TIMEOUT_MS){
      Serial.print(".");
      delay(100);
    } 

    if (WiFi.status() != WL_CONNECTED){
      Serial.println("Failed!");
      //break;
    } else {
      Serial.print("Connected!");
      Serial.print(WiFi.localIP());
      break;
    }
  }
}

void measurementsAccel(void * pvParameters) 
{
  for(;;){
    xSemaphoreTake( baton, portMAX_DELAY);
    xSemaphoreGive( baton );
    t_accel = millis();
    //t_accel = t_accel/1000;
    
    xl.readXYZTData(XValue, YValue, ZValue, Temperature); 
    distance = sqrt(pow(XValue, 2) + pow(YValue, 2) + pow(ZValue,2)); //eucledean distance 

    Serial.print("t_accel: ");
    Serial.print(t_accel);
    Serial.print(", ");

    Serial.print("XVALUE="); Serial.print(XValue);	 
    Serial.print("\tYVALUE="); Serial.print(YValue);	 
    Serial.print("\tZVALUE="); Serial.print(ZValue);	 
    //Serial.print("\tTEMPERATURE="); Serial.println(Temperature);

    String taskMessage = "\naccel task running on core ";
    taskMessage = taskMessage + xPortGetCoreID();
    Serial.println(taskMessage);
    Serial.println(" ");
    
    vTaskDelay(Sampling_Time / portTICK_PERIOD_MS);
    
  }
}

void loopHR(void * pvParameters) {
  for(;;){
    xSemaphoreTake( baton, portMAX_DELAY);
    t_HR = millis();
    //t_HR = t_HR/1000;

    int flag=readSamples();
    Serial.println();

    Serial.print("t_HR: ");
    Serial.print(t_HR);
    Serial.print(", ");
    

//    Serial.println("----------------------------------------------------------------------------------------------------");
    if (flag){          //***if sensor reads real data
      
      iir_ma_filter_for_hr();
      int neg=find_min_negative(Moving_Average_Num, Num_Samples - Moving_Average_Num, ma_gr_buffer);
      convert_signal_to_positive(Moving_Average_Num, Num_Samples - Moving_Average_Num, ma_gr_buffer, neg);
      ComputeHeartRate();
      
      Serial.print("NEW DATA--> ");
      Serial.print("HR: ");
      Serial.print(HR);
    }
    else{                 //***else if sensor is unplugged
      Serial.print("unplugged\n"); 
      HR = 0; 
      Serial.println();
    }
    String taskMessageHR = "\nHR task running on core ";
    taskMessageHR = taskMessageHR + xPortGetCoreID();
    Serial.println(taskMessageHR);
    Serial.println(" ");
    Serial.print("time since begining of task: ");
    Serial.println(millis() - t_HR);
    Serial.println("\n----------------------------------------------------------------------------------------------------\n");
    xSemaphoreGive( baton );
    delay(50);
  }
}


void loop() {
  delay(10);
  
  // put your main code here, to run repeatedly:
  if (Firebase.ready() && signupOK && (millis() - sendDataPrevMillis > 4000 || sendDataPrevMillis == 0)){ //it sends a new value to firebase every 15 seconds
    sendDataPrevMillis = millis();
    // Write an Float number on the database path test/float
    if (Firebase.RTDB.setFloat(&fbdo, "accel/XValue accel", XValue)){
    //if (Firebase.RTDB.setFloat(&fbdo, "test/float", 0.01 + random(0,100))){
      // Serial.println("PASSED");
      // Serial.println("PATH: " + fbdo.dataPath());
      // Serial.println("TYPE: " + fbdo.dataType());
      //if the x value has been sent successfully to firebase, then update the count_accel
      if (Firebase.RTDB.setInt(&fbdo, "accel/count", count_accel)){
      // Serial.println("PASSED");
      // Serial.println("PATH: " + fbdo.dataPath());
      // Serial.println("TYPE: " + fbdo.dataType());
      }
      else {
        // Serial.println("FAILED");
        // Serial.println("REASON: " + fbdo.errorReason());
      }
      count_accel++;
    }
    else {
      // Serial.println("FAILED");
      // Serial.println("REASON: " + fbdo.errorReason());
    }
    //now for Y data accel
    if (Firebase.RTDB.setFloat(&fbdo, "accel/YValue accel", YValue)){
      // Serial.println("PASSED");
      // Serial.println("PATH: " + fbdo.dataPath());
      // Serial.println("TYPE: " + fbdo.dataType());
    }
    else {
      // Serial.println("FAILED");
      // Serial.println("REASON: " + fbdo.errorReason());
    }
    //now for Z data accel
    if (Firebase.RTDB.setFloat(&fbdo, "accel/ZValue accel", ZValue)){
      // Serial.println("PASSED");
      // Serial.println("PATH: " + fbdo.dataPath());
      // Serial.println("TYPE: " + fbdo.dataType());
    }
    else {
      // Serial.println("FAILED");
      // Serial.println("REASON: " + fbdo.errorReason());
    }
    //now for time data accel
    if (Firebase.RTDB.setFloat(&fbdo, "accel/t_accel", t_accel)){
      // Serial.println("PASSED");
      // Serial.println("PATH: " + fbdo.dataPath());
      // Serial.println("TYPE: " + fbdo.dataType());
    }
    else {
      // Serial.println("FAILED");
      // Serial.println("REASON: " + fbdo.errorReason());
    }
    //now the same for HR measurements 
    //now for HR data 
    if (Firebase.RTDB.setFloat(&fbdo, "HR/HR", HR)){
      // Serial.println("PASSED");
      // Serial.println("PATH: " + fbdo.dataPath());
      // Serial.println("TYPE: " + fbdo.dataType());
      //if it is all good with sending the HR measurements in firebase, then update the count_HR
      if (Firebase.RTDB.setInt(&fbdo, "HR/count", count_HR)){
        // Serial.println("PASSED");
        // Serial.println("PATH: " + fbdo.dataPath());
        // Serial.println("TYPE: " + fbdo.dataType());
      }
      else {
        // Serial.println("FAILED");
        // Serial.println("REASON: " + fbdo.errorReason());
      }
      count_HR++;
    }
    else {
      // Serial.println("FAILED");
      // Serial.println("REASON: " + fbdo.errorReason());
    }
    //now for time data HR
    if (Firebase.RTDB.setFloat(&fbdo, "HR/t_HR", t_HR)){
      // Serial.println("PASSED");
      // Serial.println("PATH: " + fbdo.dataPath());
      // Serial.println("TYPE: " + fbdo.dataType());
    }
    else {
      // Serial.println("FAILED");
      // Serial.println("REASON: " + fbdo.errorReason());
    }

  }

}

void iir_ma_filter_for_hr(){
  
  for (int i=0;i<Num_Samples;i++){
    
    filtered_gr = f.filter(double(gr_buffer[i]));
    filtered_gr_buffer[i] = round(filtered_gr);

  }

  for (int i= Moving_Average_Num;i<Num_Samples-Moving_Average_Num;i++){
      Sum_Points= 0;
      for( int k =0; k < Num_Points; k++){   
        Sum_Points = Sum_Points + filtered_gr_buffer[i-Moving_Average_Num+k]; 
      }    
      ma_gr_buffer[i] = Sum_Points/Num_Points; 
  }
  
}

int Find_Peak(int p1, int p2, int *x){
  int Peak_Magnitude = 0;
  for (int m = p1; m < p2; m++){
      if(Peak_Magnitude < x[m]){
        Peak_Magnitude = x[m];
      }
  }
  return Peak_Magnitude;
}

int Find_Mean(int p1, int p2, int *x){
  int Mean = 0;
  for (int m = p1; m < p2; m++){
      Mean = Mean+x[m];
  }
  return Mean/(p2-p1);
}

int find_min_negative(int p1, int p2, int *x){
  int min_magnitude = 0;
  for (int m = p1; m < p2; m++)
  {
    if (min_magnitude > x[m])
    {
      min_magnitude = x[m];
    }
  }
  return min_magnitude;
}

void convert_signal_to_positive(int p1, int p2, int *x, int point){
  for (int m = p1; m < p2; m++)
  {
     x[m]=x[m]-point;
  }
}

void ComputeHeartRate(){
  
  int Mean_Magnitude =Find_Mean(Moving_Average_Num, Num_Samples - Moving_Average_Num, ma_gr_buffer);
  
  //***detect successive peaks and compute PR
  for (int i = 0; i < points_pr; i++){
     PR[i] = 0;
  }
  int  Peak = 0;
  int Index = 0;
  int p=0;

    
  for (int j = Moving_Average_Num; j < Num_Samples-Moving_Average_Num; j++){
      //***Find first peak
      
      if(ma_gr_buffer[j] > ma_gr_buffer[j-1] && ma_gr_buffer[j] > ma_gr_buffer[j+1] && ma_gr_buffer[j] > Mean_Magnitude && Peak == 0){
         Peak = ma_gr_buffer[j];
         Index = j; 
      }
      
      //***Search for next peak 
      
      if(Peak > 0 ){
       if(ma_gr_buffer[j] > ma_gr_buffer[j-1] && ma_gr_buffer[j] > ma_gr_buffer[j+1] && ma_gr_buffer[j] > Mean_Magnitude){
        float d=j-Index;
        float pulse=(float)samp_freq*60/d; //bpm for each PEAK interval
        PR[p]=pulse; 
        p++;
        p %= points_pr; //Wrap variable
        Peak = ma_gr_buffer[j];
        Index = j;
       } 
      } 
  } 

  float sum=0;
  int c=0;
  for (int i=0; i< points_pr; i++){
    if (PR[i]!=0 && PR[i]!=INFINITY){
       sum=sum+PR[i];
       c=c+1;
    }      
  }

  if (c!=0){
      Pulse_Rate_next=sum/c;
    if(Pulse_Rate_next > 40 && Pulse_Rate_next < 200){
      if (Pulse_Rate_next-Pulse_Rate_previous>=5){
        Pulse_Rate_next=Pulse_Rate_previous+1;
      }
      if (Pulse_Rate_next-Pulse_Rate_previous<=-5){
        Pulse_Rate_next=Pulse_Rate_previous-1;
      }
      Pulse_Rate_previous=Pulse_Rate_next;
    }
    else{
      Pulse_Rate_next=Pulse_Rate_previous;
    }
  }else{
    Pulse_Rate_next=Pulse_Rate_previous;
  }

  HR= int(round(Pulse_Rate_next));

}

int readSamples(){
 
  int flag=1;
  
  for (uint32_t i=0;i<Num_Samples;i++){ 
    //read max30105
    gr_buffer[i] = Sensor.getGreen();   //using only green LED here
    //gr_buffer[i] = Sensor.getRed();
    //Sensor.nextSample();

    if (gr_buffer[i]<10000){
      flag=0;
    }
    delay(40);
  }

  return flag;
}