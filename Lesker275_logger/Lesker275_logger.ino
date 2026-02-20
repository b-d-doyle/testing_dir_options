// Lesker275_logger v0.0.2
// Brandon Doyle
// 11/21/2025
//
// Purpose:
// Store a time-marked log of pressure, as measured from the analog output of the convectron on GRIT, onto a micro SD card.
// Updated for an OLED, an ADS1115
// Must now be put on something like a mega for memory reasons
//
// Components:
// - Arduino Mega or similar (I'm using too much memory for Uno)
// - Lesker 275 convectron pressure gauge
// - DS3231 RTC module
// - CR2032 holder (usually comes with RTC module)
// - uSD reader/writer module
// - OLED display (128x64) 
// - Status LEDS, 1 each of Red, Green, and Amber (Yellow)
// - Resistive voltage divider. Should either be precision matched or tunable with a pot
// - Various switches and basic components
//
// RTC functionality adapted from John Boxall at http://tronixstuff.com

#include <Wire.h>
#include <SerialCommand.h>
#include <SPI.h>
#include <SD.h>
#include <MultiMap.h>
#include <7semi_ADS1xx5.h>
#define RTC_I2C 0x68 //RTC I2C address
const int SD_CS = 53; //SD chip select pin

float version_no = 0.02;
File logFile;

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Create ADS1xx5_7semi object 
// default address 0x48
ADS1xx5_7semi adc;

/////////////Begin OLED setup stuff///////////////
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define LOGO_HEIGHT   64
#define LOGO_WIDTH    64
static const unsigned char PROGMEM logo_bmp[] =
{
	0xff, 0xff, 0xff, 0xe0, 0x07, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x0f, 0xf0, 0x7f, 0xff, 0xff, 
	0xff, 0xff, 0xf1, 0xff, 0xff, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xcf, 0xff, 0xff, 0xe3, 0xff, 0xff, 
	0xff, 0xff, 0x3f, 0xff, 0xff, 0xfc, 0xff, 0xff, 0xff, 0xfc, 0xff, 0xf1, 0x8f, 0xfe, 0x3f, 0xff, 
	0xff, 0xf9, 0xff, 0xe1, 0x87, 0xff, 0x9f, 0xff, 0xff, 0xf7, 0xff, 0xc1, 0x83, 0xff, 0xcf, 0xff, 
	0xff, 0xcf, 0xff, 0xc1, 0x83, 0xff, 0xf7, 0xff, 0xff, 0x9f, 0xff, 0x81, 0x81, 0xff, 0xfb, 0xff, 
	0xff, 0x3f, 0xff, 0x81, 0x81, 0xff, 0xfd, 0xff, 0xff, 0x7f, 0xff, 0x81, 0x81, 0xff, 0xfe, 0xff, 
	0xfe, 0xff, 0xff, 0x01, 0x80, 0xff, 0xff, 0x7f, 0xfd, 0xff, 0xff, 0x01, 0x80, 0xff, 0xff, 0x3f, 
	0xf9, 0xff, 0xfe, 0x01, 0x80, 0x7f, 0xff, 0xbf, 0xfb, 0xff, 0xfe, 0x01, 0x80, 0x7f, 0xff, 0xdf, 
	0xf7, 0xff, 0xfc, 0x01, 0x80, 0x3f, 0xff, 0xcf, 0xf7, 0xff, 0xfc, 0x01, 0x80, 0x3f, 0xff, 0xef, 
	0xef, 0xff, 0xfc, 0x01, 0x80, 0x3f, 0xff, 0xe7, 0xef, 0xff, 0xf8, 0x01, 0x80, 0x1f, 0xff, 0xf7, 
	0xdf, 0xff, 0xf8, 0x01, 0x80, 0x1f, 0xff, 0xf3, 0xdf, 0xff, 0xf0, 0x01, 0x80, 0x0f, 0xff, 0xfb, 
	0xdf, 0xff, 0xf0, 0x01, 0x80, 0x0f, 0xff, 0xfb, 0xbf, 0xff, 0xf0, 0x01, 0x80, 0x0f, 0xff, 0xfb, 
	0xbf, 0xff, 0xe0, 0x01, 0x80, 0x07, 0xff, 0xfd, 0xbf, 0xff, 0xe0, 0x03, 0xc0, 0x07, 0xff, 0xfd, 
	0xbf, 0xff, 0xe0, 0x07, 0xe0, 0x07, 0xff, 0xfd, 0x3f, 0xff, 0xc0, 0x07, 0xe0, 0x03, 0xff, 0xfd, 
	0x7f, 0xff, 0xc0, 0x0f, 0xf0, 0x03, 0xff, 0xfd, 0x7f, 0xff, 0xc0, 0x1f, 0xf8, 0x03, 0xff, 0xfc, 
	0x7f, 0xff, 0x80, 0x1f, 0xf8, 0x01, 0xff, 0xfc, 0x7f, 0xff, 0x80, 0x3f, 0xfc, 0x01, 0xff, 0xfc, 
	0x7f, 0xff, 0x80, 0x7f, 0xfe, 0x01, 0xff, 0xfc, 0x7f, 0xff, 0x00, 0x7f, 0xfe, 0x00, 0xff, 0xfc, 
	0x7f, 0xff, 0x00, 0xff, 0xff, 0x00, 0xff, 0xfc, 0x7f, 0xff, 0x00, 0xf0, 0x0f, 0x00, 0xff, 0xfd, 
	0x3f, 0xfe, 0x01, 0xef, 0xc7, 0x80, 0x7f, 0xfd, 0xbf, 0xfe, 0x01, 0x9f, 0xf1, 0x80, 0x7f, 0xfd, 
	0xbf, 0xfe, 0x03, 0x87, 0xf9, 0xc0, 0x7f, 0xfd, 0xbf, 0xfe, 0x03, 0x04, 0xf8, 0xc0, 0x7f, 0xfd, 
	0x9f, 0xfc, 0x06, 0x40, 0xfc, 0x60, 0x3f, 0xfb, 0xdf, 0xfc, 0x06, 0xc0, 0xfe, 0x60, 0x3f, 0xfb, 
	0xdf, 0xfc, 0x0e, 0x81, 0xfe, 0x70, 0x3f, 0xfb, 0xcf, 0xfc, 0x0e, 0x81, 0xff, 0x70, 0x3f, 0xf7, 
	0xef, 0xfc, 0x1e, 0xc1, 0xff, 0x78, 0x3f, 0xf7, 0xef, 0xf8, 0x1e, 0xc7, 0xff, 0x78, 0x1f, 0xe7, 
	0xf7, 0xf8, 0x1e, 0xe3, 0x83, 0x78, 0x1f, 0xef, 0xf3, 0xf8, 0x3f, 0x7c, 0x02, 0xfc, 0x1f, 0xcf, 
	0xfb, 0xf8, 0x3f, 0x7f, 0x02, 0xfc, 0x1f, 0xdf, 0xf9, 0xf8, 0x3f, 0xbf, 0x05, 0xfc, 0x1f, 0xbf, 
	0xfc, 0xf8, 0x7f, 0xdf, 0xc3, 0xfe, 0x1f, 0x3f, 0xfe, 0xf0, 0x7f, 0xe7, 0xc7, 0xfe, 0x0e, 0x7f, 
	0xff, 0x70, 0x7f, 0xf8, 0x1f, 0xfe, 0x0e, 0xff, 0xff, 0xbf, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xff, 
	0xff, 0xdf, 0xff, 0xff, 0xff, 0xff, 0xfb, 0xff, 0xff, 0xef, 0xff, 0xff, 0xff, 0xff, 0xe7, 0xff, 
	0xff, 0xf3, 0xff, 0xff, 0xff, 0xff, 0xcf, 0xff, 0xff, 0xf9, 0xff, 0xff, 0xff, 0xff, 0x9f, 0xff, 
	0xff, 0xfe, 0x7f, 0xff, 0xff, 0xfe, 0x7f, 0xff, 0xff, 0xff, 0x1f, 0xff, 0xff, 0xf8, 0xff, 0xff, 
	0xff, 0xff, 0xc7, 0xff, 0xff, 0xe3, 0xff, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 0x1f, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x1f, 0xff, 0xff, 0xff
};
/////////////End OLED setup stuff///////////////

char* msg = new char[50];

void OLED_show(char* msg){
  display.clearDisplay();

  display.setTextSize(3);             // Double the normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);// Draw white text (monochrome board, but doesn't work without this)
  display.setCursor(0,16);            // In pixels (x,y) from top left corner
                                      // The top 16-ish pixels on my OLED are yellow
                                      // and below that, they're blue.

  display.print(msg);
  display.display();
}
//float divider_ratio = 1.717/0.853; //Measure values for your voltage input voltage divider. If no divider, divider_ratio=1
float divider_ratio = 2; //OR, be civilized and use a pot as a divider so you can just adjust it on the fly.

bool status_ready  = false;
bool status_wrn    = true;
bool status_record = false;
const int status_pin_ready  = 2;
const int status_pin_wrn    = 3;
const int status_pin_record = 4;
const int input_pin_record  = 5;
const int input_pin_press   = A0;

void updateStatus(){
  digitalWrite(status_pin_ready , status_ready );
  digitalWrite(status_pin_wrn   , status_wrn   );
  digitalWrite(status_pin_record, status_record);
}

//For timers
unsigned long now = 0;
unsigned long then = 0;
unsigned long interval = 10000; //10 seconds

//////////////////////////////////////////////////////////////////
//SERIAL COMMAND STUFF
SerialCommand sCmd; // The SerialCommand object

//Here are the methods available via Serial interface:

//Display help message
void helpMessage(){
  Serial.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"));
  Serial.println(F("Pressure Logger for GRIT's convectron gauge"));
  Serial.print(F("Version: "));
  Serial.println(version_no,2);
  Serial.println(F("Expected devices: "));
  Serial.println(F(" - mumble brand convectron, model mumblemuble"));
  Serial.println(F(" - DS3231 Real Time Clock (RTC) module or equivalent"));
  Serial.println(F(" - SD reader/writer"));
  Serial.println(' ');
  Serial.println(F("Serial Commands:"));
  Serial.println(F("help"));
  Serial.println(F("   Inputs: None"));
  Serial.println(F("   Desc: Show this text."));
  Serial.println(F("read"));
  Serial.println(F("   Inputs: None"));
  Serial.println(F("   Desc: Read RTC and report to Serial. "));
  Serial.println(F("         Date convention is day/month/year."));
  Serial.println(F("set [sec] [min] [hr] [wkDay] [moDay] [mo] [yr]"));
  Serial.println(F("   Inputs: int int int int int int int"));
  Serial.println(F("   Desc: Sets the RTC module."));
  Serial.println(F("         For example, to set the RTC to 3:02:01 AM on Wednesday, June 5th, 2007,"));
  Serial.println(F("         the command would be, \"set 1 2 3 4 5 6 7\""));
  Serial.println(F("verify"));
  Serial.println(F("   Inputs: None"));
  Serial.println(F("   Desc: Read RTC and report whether it looks like it has been set:"));
  Serial.println(F("         RTC looks unset if it is the year 2000"));
  Serial.println(F("         RTC looks set if it is any other year"));
  Serial.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"));
}

//If the user sends a bad command,
//we insult their family and challenge them to a duel.
void badCommand(){
  Serial.println(F("ERR. Type 'help' for help."));
}

//Call setDS3231time(s,m,h,dw,dm,m,y) using Serial input values
void setRTC(){
  //format: s,m,h,dw,dm,m,y;
  int t[7];
  char* arg;
  for(int i=0;i<7;i++){
    arg = sCmd.next();
    if(arg==NULL){
      badCommand();
      return -1;
    }
    t[i] = atoi(arg);
  }
  DS3231init();
  setDS3231time(t[0],t[1],t[2],t[3],t[4],t[5],t[6]);
}

void readRTC(){
  displayTime(); //temp. implement it here in a better way, then remove their code
}

bool verifyRTC(){
  if(RTCvalid()){
    Serial.println(F("RTC looks set."));
    return true;
  }else{
    Serial.println(F("RTC looks unset."));
    return false;
  }
}

bool RTCvalid(){
  byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
  // retrieve data from DS3231
  readDS3231time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month,
  &year);
  return (int)year>0;
}
//END SERIAL COMMAND STUFF
//////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////
//RTC STUFF

// Convert normal decimal numbers to binary coded decimal
byte decToBcd(byte val){
  return( (val/10*16) + (val%10) );
}
// Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val){
  return( (val/16*10) + (val%16) );
}

//Initialize DS3231 settings. Currently uses default values anyway.
void DS3231init(){
  Wire.beginTransmission(RTC_I2C);
  Wire.write(0x0E);       // select control register
  Wire.write(0b00011100); // These are the default values anyway.
  Wire.endTransmission();
}

//Set the time. For Serial control, use setRTC() instead.
void setDS3231time(byte second, byte minute, byte hour, byte dayOfWeek, byte
dayOfMonth, byte month, byte year){
  // sets time and date data to DS3231
  Wire.beginTransmission(RTC_I2C);
  Wire.write(0); // set DS3231 register pointer to 00h
  Wire.write(decToBcd(second)); // set seconds
  Wire.write(decToBcd(minute)); // set minutes
  Wire.write(decToBcd(hour)); // set hours
  Wire.write(decToBcd(dayOfWeek)); // set day of week (1=Sunday, 7=Saturday)
  Wire.write(decToBcd(dayOfMonth)); // set date (1 to 31)
  Wire.write(decToBcd(month)); // set month
  Wire.write(decToBcd(year)); // set year (0 to 99)
  Wire.endTransmission();
}
void readDS3231time(byte *second,
byte *minute,
byte *hour,
byte *dayOfWeek,
byte *dayOfMonth,
byte *month,
byte *year){
  Wire.beginTransmission(RTC_I2C);
  Wire.write(0); // set DS3231 register pointer to 00h
  Wire.endTransmission();
  Wire.requestFrom(RTC_I2C, 7);
  // request seven bytes of data from DS3231 starting from register 00h
  *second = bcdToDec(Wire.read() & 0x7f);
  *minute = bcdToDec(Wire.read());
  *hour = bcdToDec(Wire.read() & 0x3f);
  *dayOfWeek = bcdToDec(Wire.read());
  *dayOfMonth = bcdToDec(Wire.read());
  *month = bcdToDec(Wire.read());
  *year = bcdToDec(Wire.read());
  Wire.endTransmission();
}
void displayTime(){
  byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
  // retrieve data from DS3231
  readDS3231time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month,
  &year);
  // send it to the serial monitor
  Serial.print(hour, DEC);
  // convert the byte variable to a decimal number when displayed
  Serial.print(F(":"));
  if (minute<10){
    Serial.print(F("0"));
  }
  Serial.print(minute, DEC);
  Serial.print(F(":"));
  if (second<10){
    Serial.print(F("0"));
  }
  Serial.print(second, DEC);
  Serial.print(F(" "));
  Serial.print(dayOfMonth, DEC);
  Serial.print(F("/"));
  Serial.print(month, DEC);
  Serial.print(F("/"));
  Serial.print(year, DEC);
  Serial.print(F(" Day of week: "));
  switch(dayOfWeek){
  case 1:
    Serial.println(F("Sunday"));
    break;
  case 2:
    Serial.println(F("Monday"));
    break;
  case 3:
    Serial.println(F("Tuesday"));
    break;
  case 4:
    Serial.println(F("Wednesday"));
    break;
  case 5:
    Serial.println(F("Thursday"));
    break;
  case 6:
    Serial.println(F("Friday"));
    break;
  case 7:
    Serial.println(F("Saturday"));
    break;
  }
}

//END RTC STUFF
//////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////
//LOGGING STUFF
File dataFile;
char fileName[13]={0};  //8.3 format uses 12 bytes+zero terminator
                        //MAXIMUM for file names

void startLog(){
  Serial.print(F("Start log: "));

  //Check if we think we're ready:
  if(!status_ready){
    Serial.println(F("Log attempted, but device not ready."));
    return;
  }

  //Initialize SD card:
  if(!SD.begin(SD_CS)) {
    Serial.println(F("SD card initialization failed."));
    status_wrn = true;
    status_ready = false;
    updateStatus();
    return;
  }
  
  // retrieve data from DS3231 Real Time Clock
  byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
  readDS3231time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);

  // Create and open file:
  //char fileName[13]={0};  //8.3 format uses 12 bytes+zero terminator //Moved to global variable
  sprintf(fileName,"%002d-%02d-%02d.CSV",year,month,dayOfMonth);  //
  Serial.println(fileName);
  dataFile=SD.open(fileName, FILE_WRITE);

  //if the file opened ok, write to it:
  if (dataFile){
    dataFile.println(F("Time, Pressure (Torr)"));
    status_record = true;
  }else{ //If the file did NOT open we 
    status_wrn = true;
    Serial.println(F("File failed to open."));
    dataFile.close();
    SD.end();
  }
  updateStatus();
}

void log(){
  // retrieve data from DS3231
  byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
  readDS3231time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);

  //retrieve pressure:
  //double P = getP_loglin();
  double P = getP_s();

  Serial.println(P);

  //Attempt to write to file:
  if(dataFile && SD.exists(fileName)){
    dataFile.print(hour);
    dataFile.print(F(":"));
    dataFile.print(minute);
    dataFile.print(F(":"));
    dataFile.print(second);
    dataFile.print(F(","));
    dataFile.println(P,4);
  }else{
    status_wrn = true;
    Serial.println(F("ERR: Attempted to write to non-valid file."));
    endLog();
  }
}

void endLog(){
  Serial.println(F("End log."));
  if(dataFile) dataFile.close();
  status_record=false;
  SD.end();
  updateStatus();
}
//END LOGGING STUFF
//////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////
//PRESSURE STUFF
//For log-linear models (part number ends in "LL"):
double getP_loglin(){
  int sensorValue = 0;
  double v = adc.readCH(0)*2.;
  return pow(10.0,v-5.0);
}
//For S-curve models (part number ends in numbers):
float pTable[] = { 0.0, 0.0001, 0.0002, 0.0005, 0.001, 0.002, 0.005, 0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50, 100, 200, 300, 400, 500, 600, 700, 760, 800, 900, 1000};
float vTable[] = {0.3751,0.3759,0.3768,0.3795,0.3840,0.3927,0.4174,0.4555,0.5226,0.6819,0.878,1.1552,1.6833,2.2168,2.8418,3.6753,4.2056,4.5766,4.8464,4.9449,5.019,5.1111,5.2236,5.3294,5.4194,5.4949,5.5340,5.5581,5.6141,5.6593 };
double getP_s(){
  int sensorValue = 0;
  double v = adc.readCH(0)*2;
  return multiMap<float>(v,vTable,pTable,30);
}
//END PRESSURE STUFF
//////////////////////////////////////////////////////////////////


void setup() {
  Wire.begin();
  Serial.begin(9600);

  //SPI requires begin() and THEN beginTransaction()
  SPI.begin();
  SPI.beginTransaction(SPISettings(30000000, MSBFIRST, SPI_MODE1));

  //Begin I2C communication to the OLED screen:
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  
  // Show splash screen:
  //display.display();
  display.clearDisplay();

  display.drawBitmap(
    (display.width()  - LOGO_WIDTH ) / 2,
    (display.height() - LOGO_HEIGHT) / 2,
    logo_bmp, LOGO_WIDTH, LOGO_HEIGHT, 1);
  display.display();

  //Initialize Status LEDs:
  pinMode(status_pin_ready, OUTPUT);
  pinMode(status_pin_wrn,   OUTPUT);
  pinMode(status_pin_record,OUTPUT);
  pinMode(input_pin_record ,INPUT );
  pinMode(input_pin_press,  INPUT );
  status_ready  = false;
  status_wrn    = true;
  status_record = false;
  updateStatus();

  //Set "all good" status, don't display yet.
  status_wrn   = false;
  status_ready = true;

  //Initialize Serial Commands:
  sCmd.addCommand("help",helpMessage); //help                          //Displays help message
  sCmd.addCommand("read",readRTC);     //read                          //Displays the time and date
  sCmd.addCommand("set",setRTC);       //set sec min hr dow dom mo yr  //Initializes RTC and sets the date/time
  sCmd.addCommand("verify",verifyRTC); //verify                        //Does the RTC appear to have been set?
  sCmd.setDefaultHandler(badCommand);                                  //Insult user's family and challenge them to a duel.

  //Initialize SD card:
  Serial.print(F("Initializing SD card..."));
  if (!SD.begin(SD_CS)) {
    Serial.println(F("initialization failed."));
    status_wrn = true;
    status_ready = false;
  }else{
    Serial.println(F("SD okay."));
  }
  SD.end();

  Serial.print(F("Verifying RTC..."));
  if (!verifyRTC()) status_wrn = true;

    Serial.println("ADS1xx5 Test Begin");

  if (!adc.begin()) {
    Serial.println("Failed to connect to ADS1xx5!");
    while (1)
      ;
  }
  Serial.println("ADS1xx5 Connected");

  // Optional: Set PGA (reference voltage)
  // Default: ±2.048V
  // To change: use
  // PGA_6_144V  6.144V
  // PGA_4_096V  4.096V
  // PGA_1_024V  1.024V
  // PGA_0_512V  0.512V
  // PGA_0_256V  0.256V
  adc.setRefV(PGA_6_144V);  // ±2.048V full scale

  then = millis();
  updateStatus();

  delay(1000); // Pause for 1 second
  // Clear the OLED buffer
  display.clearDisplay();
}

int counter = 0;
int averaging = 5;
float p[5]={0};
float press;
void loop() {
  //Serial inputs:
  sCmd.readSerial();

  //Time for log?:
  now = millis();
  if (status_record && now - then >= interval){
    log();
    then = now;
  }

  //Check for changes in log status:
  if(!status_record && digitalRead(input_pin_record)) startLog();
  if(status_record && !digitalRead(input_pin_record)) endLog();

  
  //p[counter++] = getP_loglin();
  p[counter++] = getP_s();
  if(counter==averaging){
    counter = 0;
  }
  press = 0;
  for(int i=0; i<averaging; i++){
    press+=p[i];
  }
  press/=averaging;
  msg = dtostrf(press,5,2,msg);
  msg = strcat(msg,"\r Torr\0");
  
  // Report via serial connection:
  //Serial.println(msg);

  //Display to OLED screen:
  OLED_show(msg);
  delay(50);
}
