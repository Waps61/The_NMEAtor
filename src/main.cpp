#include <Arduino.h>
/*
  Project:  Yazz_Multiplexer.ino, Copyright 2020, Roy Wassili
  Contact:  waps61 @gmail.com
  URL:      https://www.hackster.io/waps61
  Date:     30-04-2020
  Last
  Update:   29-05-2020
  Achieved: pre-FAT failed due to erroneous implementation of reading input and sending output.
            Functions startListening and Start Talking are overhauled. Function decodeNMEAInput
            added to test and process valid incomming NMEA data.
            
  Purpose:  Build an NMEA0183 manupulator and animator for on board of my sailing boat
            supporting following types of tasks:
            - Reading NMEA0183 v1.5 data without a checksum,
            - Filtering out current heading data causing incorrect course in fo in navigation app
              i.e. HDG, HDM and VHW messages
            - Getting new course and roll & pitch data from a MPU9250
            - Inject this new data into the datastream (aka multiplexing)

            The idea is to make this an event triggered or at least a Task scheduled program
            - Task - Runs repeatedly
            - Timed Task - Runs at a specified rate (100ms, 1000ms, etc.) like reading the MPU9250
            - TriggeredTask - Runs when triggered by an external source, like reading NMEA data


  NOTE:     NMEA encoding conventions in short
            An NMEA sentence consists of a start delimiter, followed by a comma-separated sequence
            of fields, followed by the character '*' (ASCII 42), the checksum and an end-of-line marker.
            i.e. <start delimiter><field 0>,<field 1>,,,<field n>*<checksum><end-of-linemarker>
            The start delimiter is either $ or !. <field 0> contains the tag and the remaining fields
            the values. The tag is normaly a 5 character wide identifier where the 1st 2 characters
            identify the talker ID and the last 3 identify the sentence ID.
            Maximum sentence length, including the $ and <CR><LF> is 82 bytes.

  Source: https://gpsd.gitlab.io/gpsd/NMEA.html#_nmea_0183_physical_protocol_layer


  Minimal Hardware setup MPU9255:
  MPU9255/6500 ------------- Arduino
  VCC ---------------------- 3.3V
  GND ---------------------- GND
  SDA ---------------------- SDA20
  SCL ---------------------- SCL21
  FSYNC--------------------- GND
  OPTIONAL:
  INT ---------------------- ?
 
  Serial is reserved for the MPU9250 communication
        Digital pin 53 (and 51) are reserved for the NMEA listener
        via SoftSerial
        Serial2 is reserved for NMEA talker
  
  Hardware setup:
  Wiring Diagram (for RS-232 to TTL converter MAX3232)
  NMEA-0183 | MAX3232 | ARDUINO
     TX     |  T1 OUT |
            |  R1 OUT | Receive Pin

  Wiring Diagram (for RS-422 / RS-485 shifter)
  NMEA-0183 | RS-422/485 Shifter | ARDUINO
    NMEA+   |     B              |
    NMEA-   |     A              |
            |     VCC            |  5V
            |     GND            |  Ground
            |     RE             |  Ground
            |     RO             |  Receive Pin

Wiring Diagram (for RS-232 to NMEA0183 device)
  Arduino | NMEA device
     TX2  |  RX +   
     GND  |  RX - 
          |  GND (if isolated input available)

Set the pins to the correct ones for your development shield or breakout board.
This program use these 8bit data lines to the LCD,
pin usage as follow:
                  LCD_CS  LCD_CD  LCD_WR  LCD_RD  LCD_RST  SD_SS  SD_DI  SD_DO  SD_SCK 
Arduino Mega2560    A3      A2      A1      A0      A4      10     11     12      13                           

                  LCD_D0  LCD_D1  LCD_D2  LCD_D3  LCD_D4  LCD_D5  LCD_D6  LCD_D7  
Arduino Mega2560    8       9       2       3       4       5       6       7 

*Remember to set the pins to suit your display module!


---------------
Terms of use:
---------------
The software is provided "AS IS", without any warranty of any kind, express or implied,
including but not limited to the warranties of mechantability, fitness for a particular
purpose and noninfringement. In no event shall the authors or copyright holders be liable
for any claim, damages or other liability, whether in an action of contract, tort or
otherwise, arising from, out of or in connection with the software or the use or other
dealings in the software.

-----------
Warning:
-----------
Do NOT use this compass in situations involving safety to life
such as navigation at sea.  
        
TODO: 

Credit:   
*/

/*
    Include the necessary libraries
*/

#include <Wire.h>
//#include <EEPROM.h>
#include "quaternionFilters.h"
#include <MPU9250.h>  

#include <LCDWIKI_GUI.h>
#include <LCDWIKI_KBV.h>
#include <TouchScreen.h>

//*** Since the signal from the RS422-TTL converter is inverted
//*** a digital input is used as a software serial port because
//*** it can invert te signal back to its orignal pulse set
#include <SoftwareSerial.h>
/*
   Definitions go here
*/
#define VESSEL_NAME "YAZZ"
// *** Conditional Debug & Test Info to Serial Monitor
// *** by commenting out the line(s) below the debugger and or test statements will 
// *** be ommitted from the code
//#define DEBUG 1
//#define TEST 1
#define DISPLAY_ATTACHED 1
//#define MPU_ATTACHED 1

#define SAMPLERATE 115200

#define LISTENER_RATE 4800 // Baudrate for the listner
#define LISTENER_PORT 53   // SoftSerial port 
#define TALKER_RATE 4800 // Baudrate for the talker
#define TALKER_PORT 2   // Serial port 2


//*** Some conversion factors
#define FTM  0.3048        // feet to meters
#define MTF  3.28084       // meters to feet
#define NTK  1.852         // nautical mile to km
#define KTN  0.5399569     // km to nautical mile

//*** The NMEA defines in totl 82 characters including the starting 
//*** characters $ or ! and the checksum character *, the checksum
//*** AND last but not least the <CR><LF> chacters.
//*** we define one more for the terminating '\0' character for char buffers
#define NMEA_BUFFER_SIZE 83 // According NEA0183 specs the max char is 82
#define NMEA_TERMINATOR "\r\n"

//*** The maximum number of fields in an NMEA string
//*** The number is based on the largest sentence MDA,
//***  the Meteorological Composite sentence
#define MAX_NMEA_FIELDS 21

#define STACKSIZE 10  // Size of the stack; adjust according use

#define TALKER_ID "AO"
#define VARIATION "1.57,E" //Varition in Lemmer on 12-05-2020, change 0.11 per year
//*** define NMEA tags to be used
//*** make sure you know your Talker ID used in the sentences
//*** In my case next to GP for navigation related sentences
//*** II is used for Integrated Instruments and
//*** PS is used for vendor specific tags like Stowe Marine
//*** AO is used for my Andruino generated sentences
#define _GLL "$GPGLL"   // Geographic Position  Latitude/Longitude
#define _GGA "$GPGGA"   // GPS Fix Data. Time, Position and fix related data for a GPS receiver
#define _GSA "$GPGSA"   // GPS DOP and active satellites
#define _GSV "$GPGSV"   // Satellites in view
#define _DBK "$SDDBK"   // Depth below keel
#define _DBS "$SDDBS"   // Depth below surface
#define _DBT "$SDDBT"   // Depth below transducer
#define _HDG "$IIHDG"   // Heading  Deviation & Variation
#define _HDM "$IIHDM"   // Heading Magnetic
#define _HDT "$IIHDT"  // Heading True
#define _MWD "$IIMWD"  // Wind Direction & Speed
#define _MTW "$IIMTW"  // Water Temperature
#define _MWV "$WIMWV"  // Wind Speed and Angle
#define _RMA "$GPRMA"  // Recommended Minimum Navigation Information
#define _RMB "$GPRMB"  // Recommended Minimum Navigation Information
#define _RMC "$GPRMC"  // Recommended Minimum Navigation Information
#define _ROT "IIROT"  // Rate of Turn
#define _RPM "$IIRPM"  // Revolutions
#define _RSA "$IIRSA"  // Rudder sensor angle
#define _VDR "$IIVDR"  // Set and Drift
#define _VHW "$IIVHW"  // Water Speed and Heading
#define _VLW "$IIVLW"  //  Distance Traveled through Water
#define _VTG "$IIVTG"  //  Track Made Good and Ground Speed
#define _VWR "$IIVWR"  //  Relative Wind Speed and Angle
#define _XDR "$IIXDR"  //  Cross Track Error  Dead Reckoning
#define _XTE "$IIXTE"  //  Cross-Track Error  Measured
#define _XTR "$IIXTR"  //  Cross Track Error  Dead Reckoning
#define _ZDA "$IIZDA"  //  Time & Date - UTC, day, month, year and local time zone
//*** Some specific Robertson / Stowe Marine tags below
#define _TON "$PSTON"  // Distance Nautical since reset
#define _TOE "$PSTOE"  // Engine hours
#define _TOB "$PSTOB"  // Battery voltage
#define _TOD "$PSTOD"  // depth transducer below waterline in feet
//*** Arduino generated TAGS
#define _xDR "$" TALKER_ID "" "XDR" // Arduino Transducer measurement
#define _dBK "$" TALKER_ID "" "DBK" // Arduino Transducer measurement
#define _hDG "$" TALKER_ID "" "HDG" // Arduino Transducer measurement
/* SPECIAL NOTE:
  XDR - Transducer Measurement
        1 2   3 4            n
        | |   | |            |
  $--XDR,a,x.x,a,c--c, ..... *hh<CR><LF>
  Field Number:   1:Transducer Type
                2:Measurement Data
                3:Units of measurement
                4:Name of transducer

  There may be any number of quadruplets like this, each describing a sensor. The last field will be a checksum as usual.
  Example:
  $HCXDR,A,171,D,PITCH,A,-37,D,ROLL,G,367,,MAGX,G,2420,,MAGY,G,-8984,,MAGZ*41
*/

/*
   If there is some special treatment needed for some NMEA sentences then
   add the their definitions to the NMEA_SPECIALTY definition
   The pre-compiler concatenates string literals by using "" in between
*/
#define NMEA_SPECIALTY "" _DBK "" _TOB

//*** A structure to hold the NMEA data
typedef struct {
  String fields[ MAX_NMEA_FIELDS ];
  byte nrOfFields=0;
  String sentence="";

}NMEAData ;

char nmeaBuffer[NMEA_BUFFER_SIZE]={0};

enum NMEAReceiveStatus { INVALID, VALID, RECEIVING, CHECKSUMMING, TERMINATING, NMEA_READY};
byte nmeaStatus = INVALID;
byte nmeaIndex=0;
bool nmeaDataReady = false;

/* the below option false is tested with an NMEAsimulator via an USB to Serial hw 
connection to the Arduino and is working in this setting.
*/
SoftwareSerial nmeaSerial(LISTENER_PORT,51, false);
/*
TFT screen specific definitions go here
*/
#define YP A3  // must be an analog pin, use "An" notation!
#define XM A2  // must be an analog pin, use "An" notation!
#define YM 9   // can be a digital pin
#define XP 8   // can be a digital pin

//param calibration from kbv
/*#define TS_MINX 298
#define TS_MAXX 814

#define TS_MINY 114 
#define TS_MAXY 867
*/
#define TS_MINX 116
#define TS_MAXX 906

#define TS_MINY 92 
#define TS_MAXY 952


// For better pressure precision, we need to know the resistance
// between X+ and X- Use any multimeter to read it
// For the one we're using, its 300 ohms across the X plate
//touch sensitivity for press
#define MINPRESSURE 10
#define MAXPRESSURE 1000

#define BLACK        0x0000  /*   0,   0,   0 */
#define BLUE         0x001F  /*   0,   0, 255 */
#define RED          0xF800  /* 255,   0,   0 */
#define GREEN        0x07E0  /*   0, 255,   0 */
#define CYAN         0x07FF  /*   0, 255, 255 */
#define MAGENTA      0xF81F  /* 255,   0, 255 */
#define YELLOW       0xFFE0  /* 255, 255,   0 */
#define WHITE        0xFFFF  /* 255, 255, 255 */
#define NAVY         0x000F  /*   0,   0, 128 */
#define DARKGREEN    0x03E0  /*   0, 128,   0 */
#define DARKCYAN     0x03EF  /*   0, 128, 128 */
#define MAROON       0x7800  /* 128,   0,   0 */
#define PURPLE       0x780F  /* 128,   0, 128 */
#define OLIVE        0x7BE0  /* 128, 128,   0 */
#define LIGHTGREY    0xC618  /* 192, 192, 192 */
#define DARKGREY     0x7BEF  /* 128, 128, 128 */
#define ORANGE       0xFD20  /* 255, 165,   0 */
#define GREENYELLOW  0xAFE5  /* 173, 255,  47 */
#define LOG_COLOR    0xFD20

#define BUTTON_H  60 //button height
#define BUTTON_W  110 //button wodth
#define BUTTON_X  5 // x position of button column
#define  BUTTON_Y 260 // y position of button column

//if the IC model is known or the modules is unreadable,you can use this constructed function
LCDWIKI_KBV my_lcd(ILI9486,A3,A2,A1,A0,A4); //model,cs,cd,wr,rd,reset
//if the IC model is not known and the modules is readable,you can use this constructed function
//LCDWIKI_KBV my_lcd(320,480,A3,A2,A1,A0,A4);//width,height,cs,cd,wr,rd,reset

TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);


int16_t current_color,flag_colour;
boolean show_flag = true;
int16_t screen_row = 0;

//*** Define the units of measurement in a string pointer array 
//*** and use an enum to get the index for the specific label
const char *screen_units[]={"Kts","NM","Deg","M","C","V"};
enum units { SPEED, DIST, DEGR, MTRS, TEMP, VOLT};
//*** the screen is divided in 4 quadrants
//***   Q1      Q2
//***   Q3      Q4
enum screen_quadrant { Q1, Q2, Q3, Q4 };

//*** a structure to hold the button info
typedef struct _button_info
{
     char button_name[10];
     uint8_t button_name_size;    // the text size i.e. 1,2,...,n based on a 5x8 char
     uint16_t button_name_colour;
     uint16_t button_colour;
     uint16_t button_x;
     uint16_t button_y;     
 }button_info;

//*** define the buttons used with an enumerated tag
enum menu_buttons { SPD, CRS, ALL, LOG};
uint8_t active_menu_button = SPD; //holds the active menu button pressed
//*** the definition of buttons menu
button_info menu_button[4] = 
{
  "Speed",3,BLACK,LIGHTGREY,BUTTON_X,BUTTON_Y,
  "Crs",3,BLACK,LIGHTGREY,BUTTON_X+(1*(BUTTON_W+BUTTON_X)),BUTTON_Y,
  "All",3,BLACK,LIGHTGREY,BUTTON_X+(2*(BUTTON_W+BUTTON_X)),BUTTON_Y,
  "Log",3,BLACK,LIGHTGREY,BUTTON_X+(3*(BUTTON_W+BUTTON_X)),BUTTON_Y, 
};

/* 
  puts a string in a foreground/background color on position x,y
*/
void show_string(char *str,int16_t x,int16_t y,uint8_t csize,uint16_t fc, uint16_t bc,boolean mode)
{
    my_lcd.Set_Text_Mode(mode); // if true background of button is used
    my_lcd.Set_Text_Size(csize);
    my_lcd.Set_Text_colour(fc);
    my_lcd.Set_Text_Back_colour(bc);
    my_lcd.Print_String(str,x,y);
}
/* 
  clears the visible part of the screen above the buttons
*/
void wipe_screen(){
   my_lcd.Set_Draw_color(BLACK);
  my_lcd.Fill_Rectangle(0,0,480,BUTTON_Y);

}

/*
Prints a line on the TFT screen taking into account that there is
a button row from y>BUTTON_Y
*/
void screen_println(char *str,uint8_t csize,uint16_t fc, uint16_t bc,boolean mode)
{
  if(screen_row > (BUTTON_Y-8)){
    screen_row = 0;
    wipe_screen();
  }
    my_lcd.Set_Text_Mode(mode);
    my_lcd.Set_Text_Size(csize);
    my_lcd.Set_Text_colour(fc);
    my_lcd.Set_Text_Back_colour(bc);
    my_lcd.Print_String(str,0,screen_row);
    screen_row += 8*csize;
}

/*
Prints the measured value and it's units + tag combi in one of the quadrants
*/
void update_display(double val,const char *str, const char *tag,int8_t q){
  uint16_t x=0,y=0;
  // which quadrants needs an update
  switch( q ){
    case Q1:
     x = 10;
     y = 10;
    break;
    case Q2:
    x = 240;
     y = 10;
    break;
    case Q3:
    x = 10;
     y = 150;
    break;
    case Q4:
    x = 240;
     y = 150;
    break;
    default:
    break;
  }
    // print the value
    my_lcd.Set_Text_Mode(false);
    my_lcd.Set_Text_Size(6);
    my_lcd.Set_Text_colour(YELLOW);
    my_lcd.Set_Text_Back_colour(BLACK);
    my_lcd.Print_Number_Float(val,1,x,y,'.',5,' ');
    // print the unit and tag
    my_lcd.Set_Text_Size(3);
    my_lcd.Set_Text_colour(WHITE);
    my_lcd.Print_String( str,x+50,y+50);
    my_lcd.Print_String( tag,x+120,y+50);
}

/*
Show the menu as a row of 4 buttons on the lower part of the display
*/
void show_menu(void)
{
    int i;
    uint16_t c=RED;
    wipe_screen();
   for(i = 0;i < (int) (sizeof(menu_button)/sizeof(button_info));i++)
   {
     if( i==active_menu_button) c=RED;
     else c=menu_button[i].button_name_colour;
      my_lcd.Set_Draw_color(menu_button[i].button_colour);
      my_lcd.Fill_Round_Rectangle(menu_button[i].button_x, 
                            menu_button[i].button_y, 
                            menu_button[i].button_x+BUTTON_W, 
                            menu_button[i].button_y+BUTTON_H,
                            3);
      show_string(menu_button[i].button_name,
                  menu_button[i].button_x+5,
                  menu_button[i].button_y+13,
                  menu_button[i].button_name_size,
                  c,
                  menu_button[i].button_colour,
                  true);
                  
   }
}




/* 
 *  MPU specific defenitions go here
 */
#define I2Cclock 400000                                 // I2C clock is 400 kilobits/s
#define I2Cport Wire                                    // I2C using Wire library
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0             // MPU9250 address when ADO = 0 (0x68)  
#define True_North false                                // change this to "true" for True North                
float Declination = +1.57;                              // substitute your magnetic declination 

// ----- software timer
unsigned long Timer1 = 750000L;                         // 500mS loop ... used when sending data to to Processing
unsigned long Stop1;                                    // Timer1 stops when micros() exceeds this value

// ----- NZ Offsets & Scale-factors
float
/*
Mag_x_offset = -34.560013,
Mag_y_offset = 528.885,
Mag_z_offset = -125.259995,
Mag_x_scale = 1.0247924,
Mag_y_scale = 0.99078894,
Mag_z_scale = 0.9853226;
*/
Mag_x_offset = 1.0,
Mag_y_offset = 1.0,
Mag_z_offset = 180,
Mag_x_scale = 1.0,
Mag_y_scale = 1.0,
Mag_z_scale = 1.0;

 
/* EEPROM buffer to mag bias and scale factors */
uint8_t eeprom_buffer[24];
float calValue;

/* MPU 9250 object */
MPU9250 imu(MPU9250_ADDRESS, I2Cport, I2Cclock);      // Create imu instance using I2C at 400 kilobits/s


bool on = true;
byte pin = 22;

//*** flag data on the mpu port is ready
volatile bool mpuDataReady = false;

//*** ISR to set mpuDataReady flag
void mpuReady(){
  mpuDataReady = true;
}

bool mpuNeedsCalibration = true;
String mpuNMEAString = "";

/*
 * End MPU specific definitions
 */

 /*
  * Setting fro Serial interrup
  */
//*** flag data on the listener port is ready
volatile bool listenerDataReady = false;

//*** ISR to set listerDataReady flag
void listenerReady(){
  listenerDataReady = true;
}


/*
debugWrite() <--provides basic debug info from other tasks
takes a String as input parameter
*/
void debugWrite(String debugMsg)
{
  
  #ifdef DISPLAY_ATTACHED
  int strlen = debugMsg.length();
  char charMsg[strlen];
  for(int i; i<strlen; i++){
    charMsg[i] = debugMsg[i];
  }
    screen_println( charMsg,2,flag_colour,BLACK,false);
  #else
    if(debugMsg.length()>1) Serial.println(debugMsg);
    else Serial.print(debugMsg);
    #endif
}

/*
   Class definitions go here
*/

/*
  Purpose:  Helper class stacking NMEA data as a part of the multiplexer application
            - Pushin and popping NMEAData structure on the stack for buffer purposes
 */
class NMEAStack
 {
  public:
  NMEAStack();  // Constructor with the size of the stack
  int push( NMEAData _nmea );   // put an NMEAData struct on the stack and returns the lastIndex or -1
  NMEAData pop();               // get an NMEAData struct from the stack and decreases the lastIndex
  int getIndex();               // returns the position of the next free postion in the stack

  private:
  NMEAData stack[STACKSIZE]; // the array containg the structs
  int lastIndex=0;    // an index pointng to the first free psotiion in the stack
 };
 
  NMEAStack::NMEAStack()
  {
    this->lastIndex = 0;
    for(int i=0; i< STACKSIZE; i++ )
    {
      for(int j=0; j<MAX_NMEA_FIELDS; j++ ){
        stack[i].fields[j]="";
      }
      stack[i].nrOfFields = 0;
      stack[i].sentence = "";
    }
  }
  
  int NMEAStack::push( NMEAData _nmea )
  {
    #ifdef DEBUG
    debugWrite( "Pushing on index:"+ String(this->lastIndex ));
    #endif
    if( this->lastIndex < STACKSIZE )
    {
      stack[ this->lastIndex++ ] = _nmea;
      return this->lastIndex;
    } else
    {
      this->lastIndex = STACKSIZE;
      return -1;    // of stack is full
    }
  }

  NMEAData NMEAStack::pop()
  {
    NMEAData nmeaOut;
    nmeaOut.sentence = "";
    if( this->lastIndex>0)
    {
      this->lastIndex--;
      nmeaOut=stack[ this->lastIndex ];
    }
    #ifdef DEBUG
    debugWrite("Popped from index: "+String(lastIndex ));
    #endif
    
    return nmeaOut;   
  }

  int NMEAStack::getIndex()
  {
    return this->lastIndex;
  }

 /*
    Purpose:  An NMEA0183 parser to convert old to new version NMEA sentences
            - Reading NMEA0183 v1.5 data without a checksum,
            - Filtering out current heading data causing incorrect course in fo in navigation app
              i.e. HDG, HDM and VHW messages
            
          
  NOTE:     NMEA encoding conventions in short
            An NMEA sentence consists of a start delimiter, followed by a comma-separated sequence
            of fields, followed by the character '*' (ASCII 42), the checksum and an end-of-line marker.
            i.e. <start delimiter><field 0>,<field 1>,,,<field n>*<checksum><end-of-linemarker>
            The start delimiter is either $ or !. <field 0> contains the tag and the remaining fields
            the values. The tag is normaly a 5 character wide identifier where the 1st 2 characters
            identify the talker ID and the last 3 identify the sentence ID.
            Maximum sentence length, including the $ and <CR><LF> is 82 bytes.

  Source: https://gpsd.gitlab.io/gpsd/NMEA.html#_nmea_0183_physical_protocol_layer
  */
class NMEAParser 
{
 public:
    NMEAParser(NMEAStack *_ptrNMEAStack);
    void setSentence( String _nmeaSentence);
    void parseNMEASentence(String nmeaIn ); // parse an NMEA sentence with each part stored in the array
    NMEAData getNMEAData(); // getter function to get the NMEADAta struct
    
  private:
    NMEAStack *ptrNMEAStack;
    String NMEAFilter = NMEA_SPECIALTY;
    NMEAData nmeaData;  // self explaining
    String nmeaSentence = "";
    void reset(); // clears the nmeaData struct;
    String checksum( String str ); //calculate the checksum for str
    NMEAData nmeaSpecialty( NMEAData nmeaIn ); // special treatment function
};

// ***
// *** NMEAParser Constructor
// *** input parameters:
// *** reference to the debugger object
NMEAParser::NMEAParser(NMEAStack *_ptrNMEAStack)
  : ptrNMEAStack(_ptrNMEAStack)
{
  //*** initialize the NMEAData struct.
reset();

}

/*
 * Clear the nmeaData attribute
 */
void NMEAParser::reset(){
  nmeaData.nrOfFields = 0;
  nmeaData.sentence = "";
  for(int i=0; i< MAX_NMEA_FIELDS; i++){
    nmeaData.fields[i]="";
  }
  current_color=WHITE;
}

void NMEAParser::setSentence( String _nmeaSentence)
{
  #ifdef DEBUG
  debugWrite( "NMEA received "+String( _nmeaSentence) );
  #endif
  nmeaSentence = _nmeaSentence;
}

/*

*/
NMEAData NMEAParser::nmeaSpecialty( NMEAData nmeaIn )
{
  String filter = NMEA_SPECIALTY;
  String sentence = nmeaIn.sentence;
  String newSentence ="";
  NMEAData nmeaOut ;//= nmeaIn;
  #ifdef DEBUG
  debugWrite( " Specialty found... for filter"+filter);
  #endif
  if ( filter.indexOf( nmeaIn.fields[0]) > -1 )
  {
    /* In my on-board Robertson data network some sentences
       are not NMEA0183 compliant. So these sentences need
       to be converted to compliant sentences
    */
    //*** $IIDBK is not NMEA0183 compliant and needs conversion
    if ( nmeaIn.fields[0] == _DBK ) {
      #ifdef DEBUG
      debugWrite("Found "+String(_DBK));
      #endif
      // a typical non standard DBK message I receive is
      // $IIDBK,A,0017.6,f,,,,
      // Char A can also be a V if invalid and shoul be removed
      // All fields after the tag shift 1 position to the left
      // Since we modify the sentence we'll also put our talker ID in place
      nmeaOut.fields[0]="$AODBK";
      for( int i=1; i<nmeaIn.nrOfFields-2; i++ )
      {
        nmeaOut.fields[i] = nmeaIn.fields[i+1];
      }
      // We need the the feet  to convert to meters and add to string
      float ft = nmeaOut.fields[1].toFloat();
      nmeaOut.fields[3] = String( ft * FTM, 1);
      nmeaOut.fields[4] = "M";
      nmeaOut.fields[5] = "";
      nmeaOut.fields[6] = "";
      nmeaOut.nrOfFields = 7;
      for( int i=0; i< nmeaOut.nrOfFields ; i++)
      {
        #ifdef DEBUG
        debugWrite("Field["+String(i)+"] = "+nmeaOut.fields[i]);
        #endif
        if(i>0) nmeaOut.sentence+=",";
        nmeaOut.sentence += nmeaOut.fields[i];
      }
      nmeaOut.sentence += checksum( nmeaOut.sentence );
      
      #ifdef DEBUG
      debugWrite( " Modified to:"+nmeaOut.sentence);
      #endif
      return nmeaOut;
    }

    //*** current Battery info is in a non NMEA0183 format 
    //*** i.e. $PSTOB,13.2,V
    //*** will be converted to $AOXDR,U,13.2,V,BATT,*CS
    if( nmeaIn.fields[0] == _TOB )
    {
      reset();
      nmeaOut.nrOfFields = 5;
      nmeaOut.fields[0] = "$AOXDR";
      nmeaOut.fields[1] ="U";  // the transducer unit
      nmeaOut.fields[2] = nmeaIn.fields[1];  // the actual measurement value
      nmeaOut.fields[3] = nmeaIn.fields[2]; // unit of measure
      nmeaOut.fields[3].toUpperCase();
      nmeaOut.fields[4] ="BATT";
      for( int i=0; i< nmeaOut.nrOfFields ; i++)
      {
        #ifdef DEBUG
        debugWrite("Field["+String(i)+"] = "+nmeaOut.fields[i]);
        #endif
        if(i>0) nmeaOut.sentence+=",";
        nmeaOut.sentence += nmeaOut.fields[i];
      }
      nmeaOut.sentence += checksum( nmeaOut.sentence );
      return nmeaOut;
    }
  }
  return nmeaOut;
}



// calculate checksum function (thanks to https://mechinations.wordpress.com)
String NMEAParser::checksum( String str )
{
  byte cs = 0;
  for (unsigned int n = 1; n < str.length() - 1; n++)
  {
    if ( str[n] != '&' || str[n] != '!' || str[n] != '*' )
    {
      cs ^= str[n];
    }
  }
  if (cs < 0x10) return "*0" + String(cs, HEX);
  else return "*"+String(cs, HEX);
}

/*
   parse an NMEA sentence into into an NMEAData structure.
*/
void NMEAParser::parseNMEASentence(String nmeaStr)
{
  reset();
  int currentIndex = 0;
  int lastIndex = -1;
  int sentenceLength = nmeaStr.length();
  String newSentence = "";  // used to construct the new senctence
  //*** check for a valid NMEA sentence
  #ifdef DEBUG
    debugWrite(" In te loop to parse for "+String(sentenceLength)+" chars");
    #endif
  if ( nmeaStr[0] == '$' || nmeaStr[0] == '!' )
  {
    
    //*** parse the fields from the NMEA string
    //*** keeping in mind that indeOf() can return -1 if not found!
    currentIndex = nmeaStr.indexOf( ',',0);
    while ( lastIndex < sentenceLength  )
    {
      
      //*** remember to sepatrate fields with the ',' character
      //*** but do not end with one!
      if ( lastIndex>0 ) nmeaData.sentence += ',';

      //*** we want the data without the ',' in fields array
      if( currentIndex-lastIndex >1 ) // check for an empty field
      {
        nmeaData.fields[ nmeaData.nrOfFields ] = nmeaStr.substring(lastIndex+1, currentIndex );
        nmeaData.sentence += nmeaData.fields[ nmeaData.nrOfFields];
      } else nmeaData.fields[ nmeaData.nrOfFields ] = "0";
      nmeaData.nrOfFields++;
      lastIndex = currentIndex; // searching from next char of ',' !
      currentIndex = nmeaStr.indexOf( ',', lastIndex+1);
      //*** check if we found the last seperator
      if( currentIndex < 0 )
      {
        //*** and make sure we parse the last part of the string too!
        currentIndex = sentenceLength;
      }
      
    }
    if ( NMEAFilter.indexOf( nmeaData.fields[0] ) > -1)
    {
      nmeaData = nmeaSpecialty( nmeaData );
    } else if(nmeaData.sentence.indexOf('*')<1)  //Check for checksum in sentence
    {
      nmeaData.sentence += checksum( nmeaData.sentence);
    }

    nmeaData.sentence += NMEA_TERMINATOR;
    #ifdef DEBUG
    debugWrite("Parsed : "+nmeaData.sentence );
    #endif
    ptrNMEAStack->push( nmeaData );   //push the struct to the stack for later use; i.e. buffer it
  }

  return;
}

/*
 * getter function to get the NMEADAta structure
 */
NMEAData NMEAParser::getNMEAData()
{
  return nmeaData;
}


/***********************************************************************************
   Global variables go here
*/
NMEAStack       NmeaStack;
NMEAParser      NmeaParser(&NmeaStack);
NMEAData        NmeaData;

/*
  Initialize the NMEA Talker port and baudrate
  on RX/TX port 2
*/
void initializeTalker(){
  Serial2.begin(TALKER_RATE); 
  #ifdef DEBUG
  debugWrite( "Talker initialized...");
  #endif
}

/*
 * Start reading converted NNMEA sentences from the stack
 * and write them to Serial Port 2 to send them to the 
 * external NMEA device.
 * Update the display with the value(s) send
 */
byte startTalking(){
  NMEAData nmeaOut;
  String outStr = "";
  
  #ifdef DISPLAY_ATTACHED
  double tmpVal=0.0;
  #endif
  
  //*** for all  NMEAData opjects on the stack
  //*** NOTE; the stack has a buffer of NMEA_BUFFER_SIZE objects
  //***       normaly only 1 or 2 should be on the stack
  //***       if the stack is full your timing is out of control

  if( NmeaStack.getIndex()>0 ){
      nmeaOut = NmeaStack.pop();
      outStr =nmeaOut.sentence;
      

      for(int i=0; i< (int) sizeof(outStr); i++){
        //outChar[i]=outStr[i];
        Serial2.write( outStr[i]);
      }
      #ifdef DEBUG
      debugWrite(" Sending :" + outStr );
      #endif
  }
  #ifdef DISPLAY_ATTACHED
  // check which screens is active and update with data
  switch (active_menu_button){
  
    case SPEED:
      if(nmeaOut.fields[0]== _RMC){
        tmpVal=nmeaOut.fields[7].toDouble();
        update_display( tmpVal,screen_units[SPEED],"SOG",Q1);
      }
      if(nmeaOut.fields[0]== _VHW){
        tmpVal=nmeaOut.fields[5].toDouble();
        update_display( tmpVal,screen_units[SPEED],"STW",Q2);
      }
      if(nmeaOut.fields[0]== _VWR){
        tmpVal=nmeaOut.fields[3].toDouble();
        update_display( tmpVal,screen_units[SPEED],"AWS",Q3);
        tmpVal=nmeaOut.fields[1].toDouble();
        char tmpChr[(nmeaOut.fields[2].length())];
        (nmeaOut.fields[2]).toCharArray(tmpChr,nmeaOut.fields[2].length(),0);
      update_display( tmpVal,screen_units[DEGR],"AWA",Q4);
      }
    break;
    case CRS:
        if(nmeaOut.fields[0]== _RMC){
        tmpVal=nmeaOut.fields[8].toDouble();
          update_display( tmpVal,screen_units[DEGR],"TRUE",Q1);
        }
        if(nmeaOut.fields[0]== _hDG){
          tmpVal=nmeaOut.fields[1].toDouble();
          update_display( tmpVal,screen_units[DEGR],"MAG",Q2);
        }
        /*
        if(nmeaOut.fields[0]== _xDR ){
          if(nmeaOut.fields[4]== "PITCH"){
            tmpVal=nmeaOut.fields[2].toDouble();
            update_display( tmpVal,screen_units[DEGR],"PITCH",Q3);
          }
          //*** if we found PITCH we also have ROLL
          if(nmeaOut.fields[8]== "ROLL"){
            tmpVal=nmeaOut.fields[6].toDouble();
            update_display( tmpVal,screen_units[DEGR],"ROLL",Q4);
          }
        }
        */
        if(nmeaOut.fields[0]== _DBS){
          tmpVal=nmeaOut.fields[3].toDouble();
          update_display( tmpVal,screen_units[MTRS],"DPT",Q3);
        }
        if(nmeaOut.fields[0]== _VLW){
          tmpVal=nmeaOut.fields[3].toDouble();
          update_display( tmpVal,screen_units[DIST],"TRIP" ,Q4);
        }
        
    break;
    case ALL:
        if(nmeaOut.fields[0]== _xDR ){
          if(nmeaOut.fields[4]== "BATT"){
            tmpVal=nmeaOut.fields[2].toDouble();
            update_display( tmpVal,screen_units[VOLT],"BAT",Q1);
          }
        }
        if(nmeaOut.fields[0]== _MTW){
            tmpVal=nmeaOut.fields[1].toDouble();
            update_display( tmpVal,screen_units[TEMP],"WTR",Q2);
          }
          if(nmeaOut.fields[0]== _VLW){
            tmpVal=nmeaOut.fields[1].toDouble();
            update_display( tmpVal,screen_units[DIST],"LOG",Q3);
          }
          if(nmeaOut.fields[0]== _VLW){
            tmpVal=nmeaOut.fields[3].toDouble();
            update_display( tmpVal,screen_units[DIST],"TRP",Q4);
          }
    break;
    case LOG:
      default:
      
      debugWrite( outStr );
      
    break;
  }
  #endif
  
  
  return 1;
}
  


 /* sets the sentence in the nmeaData attribute and itself to runnable 
 */
void setNMEASentence( String _nmeaIn ){
  NmeaData.sentence = _nmeaIn;
}

/**********************************************************************************
  Purpose:  Helper class reading NMEA data from the serial port as a part of the multiplexer application
            - Reading NMEA0183 v1.5 data without a checksum,
*/

/*
Cleasr the inputbuffer by reading until empty, since Serial.flush does not this anymore
*/
void clearNMEAInputBuffer(){
  while( nmeaSerial.available()>0){
    nmeaSerial.read();
  }

}

void initializeListener()
{
  nmeaSerial.begin(LISTENER_RATE);
  clearNMEAInputBuffer();
  #ifdef DEBUG
  debugWrite( "Listener initialized...");
  #endif
}

/*
  Decode the incomming character and test if it is valid NMEA data.
  If true than put it the NMEA buffer and call NMEAParser object
  to process incomming and complete MNEA sentence
*/
void decodeNMEAInput(char cIn){
  
  switch( cIn ){
    case '~':
      // reserved by NMEA
    case '!':
    case '$':
      nmeaStatus = RECEIVING;
      break;
    case '*':
      if(nmeaStatus==RECEIVING){
      nmeaStatus = CHECKSUMMING;
      }
      break;
    case '\n':
    case '\r':
      // in old v1.5 version, NMEA Data may not be checksummed!
      if(nmeaStatus== RECEIVING || nmeaStatus==CHECKSUMMING){
        nmeaDataReady = true;
      }
      nmeaStatus = TERMINATING;
      break;
    default:
      
      break;
  }
  switch(nmeaStatus){
    case INVALID:
      // do nothing
      nmeaIndex=0;
      nmeaDataReady = false;
      break;
    case RECEIVING:
    case CHECKSUMMING:
      nmeaBuffer[nmeaIndex] = cIn;
      nmeaIndex++;
      break;
    case TERMINATING:
    
    nmeaStatus = INVALID;
    if( nmeaDataReady){
      nmeaDataReady = false;
      
      //char nmeaOut[nmeaIndex];
      //memcpy(nmeaOut, nmeaBuffer, nmeaIndex);
      for(int y=nmeaIndex+1; y<NMEA_BUFFER_SIZE; y++){
        nmeaBuffer[y]='\0';
      }
      #ifdef DEBUG
      debugWrite( nmeaOut);
      #endif
      NmeaParser.parseNMEASentence( nmeaBuffer );
      memset( nmeaBuffer, 0, NMEA_BUFFER_SIZE);
      nmeaIndex=0;
    }
    //clearNMEAInputBuffer();
      break;
  }
}

/*
 * tart listeneing for incomming NNMEA sentence
 */
void startListening()
{
  #ifdef DEBUG
    debugWrite("Listening....");
    #endif
  
  while(  nmeaSerial.available()>0 && nmeaStatus != TERMINATING){
    decodeNMEAInput( nmeaSerial.read());
  }
  
  
}


int keyboardListener()
{
  int c = 0;
  #ifdef DEBUG
  while(Serial.available()){
    
    c = Serial.read();
    debugWrite("Character(s) received------------------------------");
    debugWrite( String(c));
    debugWrite("################################################");
    
  
  }
  #endif
  return c;
}
/*
 * Below the MPU related functions
 */



/* Bound angle between 0 and 360 */
float constrainAngle360(float dta) {
  dta = fmod(dta, 2.0 * PI);
  if (dta < 0.0)
    dta += 2.0 * PI;
  return dta;
}

void initializeMPU(){
  /* Serial for displaying results */
  #ifdef DEBUG
  Serial.begin(SAMPLERATE);
  while(!Serial) {}
  #endif
  /* 
  * Start the sensor, set the bandwidth the 10 Hz, the output data rate to
  * 50 Hz, and enable the data ready interrupt. 
  */
  Wire.begin();                                         // Start I2C as master
  Wire.setClock(400000);                                // Set I2C clock speed to 400kbs

    // ----- Look for MPU9250|MPU9255
  byte gyroID = imu.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  if (!((gyroID == 0x71) || (gyroID == 0x73))){
    // ------ Failed to connect
    debugWrite(("WHO_AM_I = "));
    debugWrite(String(gyroID, HEX));
    debugWrite(("Could not connect to the MPU9250/MPU9255"));
    debugWrite(("Communication failed ... program aborted !!"));
    abort();
  } else
  {
  // ----- Start by performing self test and reporting values
    imu.MPU9250SelfTest(imu.selfTest);
    debugWrite(("x-axis self test: acceleration trim within : "));
    debugWrite(String(imu.selfTest[0], 1)+" % of factory value");

    debugWrite("y-axis self test: acceleration trim within : ");
    debugWrite(String(imu.selfTest[1], 1)+" % of factory value");

    debugWrite("z-axis self test: acceleration trim within : ");
    debugWrite(String(imu.selfTest[2], 1)+" % of factory value");

    debugWrite("x-axis self test: gyration trim within : ");
    debugWrite(String(imu.selfTest[3], 1)+" % of factory value");

    debugWrite("y-axis self test: gyration trim within : ");
    debugWrite(String(imu.selfTest[4], 1)+" % of factory value");

    debugWrite("z-axis self test: gyration trim within : ");
    debugWrite(String(imu.selfTest[5], 1)+" % of factory value");
    debugWrite("Place the compass on a level surface");
    
    // allow time to place the compass in position
    delay(4000); 

    // ----- Calibrate gyro and accelerometers, load biases in bias registers
    imu.calibrateMPU9250(imu.gyroBias, imu.accelBias);

    // ----- Initialize device for active mode read of accelerometer, gyroscope, and
    //       temperature
    imu.initMPU9250();
    debugWrite("MPU9250 initialized for active data mode....");
    

    // ----- Look for the AK8963 magnetometer
    byte MagID = imu.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    if (!(MagID == 0x48))
    {
      // ----- Communication failed, stop here
      debugWrite("WHO_AM_I = ");
      debugWrite(String(MagID, HEX));
      debugWrite("Could not connect to the AK8963");
      debugWrite("Communication failed ... program aborted !!");
      abort();
    } else
    {
      // ----- AK8963 found
      debugWrite("WHO_AM_I = 0x48\nAK8963 is online ...");
      
      // ----- Get factory ASA calibration values
      imu.initAK8963(imu.factoryMagCalibration);

      // ----- Initialize device for active mode read of magnetometer
      debugWrite("AK8963 initialized for active data mode....");
      #ifdef DEBUG
      // ----- Display AK8963 fuse rom values
      debugWrite("AK8963 Fuse ROM values: ");
      debugWrite("ASAX: ");
      debugWrite(String(imu.factoryMagCalibration[0], 2));
      debugWrite("ASAY: ");
      debugWrite(String(imu.factoryMagCalibration[1], 2));
      debugWrite("ASAZ: ");
      debugWrite(String(imu.factoryMagCalibration[2], 2));
      debugWrite("");
      #endif
      // ----- Get correct sensor resolutions (You only need to do this once)
      imu.getAres();      // milli-gravity
      imu.getGres();      // dps
      imu.getMres();      // milli-Gauss 14-bit|16-bit

      #ifdef DEBUG
      // ---- display sensor scale multipliers
      debugWrite("Sensor-scale multipliers");
      debugWrite("accel: ");
      debugWrite(String(imu.aRes, 6));
      debugWrite(" gyro: ");
      debugWrite(String(imu.gRes, 6));
      debugWrite("  mag: ");
      debugWrite(String(imu.mRes, 6));
      debugWrite("");
      #endif
        // ----- Copy the hard-iron offsets
      imu.magBias[0] = Mag_x_offset;
      imu.magBias[1] = Mag_y_offset;
      imu.magBias[2] = Mag_z_offset;

      // ----- Copy the soft-iron scalefactors
      imu.magScale[0] = Mag_x_scale;
      imu.magScale[1] = Mag_y_scale;
      imu.magScale[2] = Mag_z_scale;

      mpuNeedsCalibration=false;
      #ifdef DEBUG
      // ----- Display offsets & scale-factors
      debugWrite("Mag_x_offset = ");
      debugWrite(String(imu.magBias[0]));
      debugWrite("Mag_y_offset = ");
      debugWrite(String(imu.magBias[1]));
      debugWrite("Mag_z_offset = ");
      debugWrite(String(imu.magBias[2]));
      debugWrite("Mag_x_scale = ");
      debugWrite(String(imu.magScale[0]));
      debugWrite("Mag_y_scale = ");
      debugWrite(String(imu.magScale[1]));
      debugWrite("Mag_z_scale = ");
      debugWrite(String(imu.magScale[2]));
      debugWrite("");
      #endif
      // ----- start software timer
      Stop1 = micros() + Timer1;                            // Used by send_data() function

    }    
  }
  /*** old mpu code
  imu.begin();
  imu.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_10HZ);
  imu.setSrd(19);
  //imu.enableDataReadyInterrupt();
  
  uint8_t eeprom_buffer[24];
  for (unsigned int i = 0; i < sizeof(eeprom_buffer); i++ ) {
    eeprom_buffer[i] = EEPROM.read(i);
  }
  memcpy(&mpu_hxb, eeprom_buffer, sizeof(mpu_hxb));
  memcpy(&mpu_hyb, eeprom_buffer + 4, sizeof(mpu_hyb));
  memcpy(&mpu_hzb, eeprom_buffer + 8, sizeof(mpu_hzb));
  memcpy(&mpu_hxs, eeprom_buffer + 12, sizeof(mpu_hxs));
  memcpy(&mpu_hys, eeprom_buffer + 16, sizeof(mpu_hys));
  memcpy(&mpu_hzs, eeprom_buffer + 20, sizeof(mpu_hzs));
  
  if( (String(mpu_hxb)).indexOf("NAN")>0){
    mpuNeedsCalibration = true;
  } else mpuNeedsCalibration = false;
  if( !mpuNeedsCalibration  ){
    #ifdef DEBUG
    debugWrite(" is calibrated = "+ mpuNeedsCalibration );
    #endif
  imu.setMagCalX(mpu_hxb, mpu_hxs);
  imu.setMagCalY(mpu_hyb, mpu_hys);
  imu.setMagCalZ(mpu_hzb, mpu_hzs);
  #ifdef DEBUG
  debugWrite( "Magnetometer Calibration values:");
  debugWrite("mpu_hxb:"+String( mpu_hxb,3 )+'\t'+"mpu_hxs:"+String(mpu_hxs,3));
  debugWrite("mpu_hyb:"+String( mpu_hyb,3 )+'\t'+"mpu_hys:"+String(mpu_hys,3));
  debugWrite("mpu_hzb:"+String( mpu_hzb,3 )+'\t'+"mpu_hzs:"+String(mpu_hzs,3));
  #endif
  // Attach the data ready interrupt to the data ready ISR 
  // With Touchscreen attached no interrupt pin free
  pinMode(kMpu9250DrdyPin, INPUT);
  attachInterrupt(kMpu9250DrdyPin, mpuReady, RISING);  
 
  mpuDataReady= true;
  debugWrite( "MPU9250 initialized and ready to go...");
  } else{
      
 debugWrite(" MPU needs calibration before use...");
 
  debugWrite( "Magnetometer Calibration values:");
  debugWrite("mpu_hxb:"+String( mpu_hxb,3 )+'\t'+"mpu_hxs:"+String(mpu_hxs,3));
  debugWrite("mpu_hyb:"+String( mpu_hyb,3 )+'\t'+"mpu_hys:"+String(mpu_hys,3));
  debugWrite("mpu_hzb:"+String( mpu_hzb,3 )+'\t'+"mpu_hzs:"+String(mpu_hzs,3));
  }
  */
 #ifdef DEBUG
 debugWrite("Inititializing done. MPU ready to go...");
 #endif
}

void sampleMPU(){
  // ----- Perform these tasks every 500mS
  // ----- Poll the MPU9250 interrupt status in I2C mode
  #ifdef DEBUG
    debugWrite("Sampling MPU....");
    #endif
  if (imu.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    #ifdef DEBUG
    debugWrite("Reading MPU....");
    #endif
    // ----- Read the accelerometer x|y|z register values
    imu.readAccelData(imu.accelCount);                                  // Read accelerometer register values

    // ----- Now we'll calculate the acceleration value into actual g's
    //       This depends on scale being set
    imu.ax = (float)imu.accelCount[0] * imu.aRes;                     // Convert raw register value to milli-Gauss
    imu.ay = (float)imu.accelCount[1] * imu.aRes;                     // Convert raw register value to milli-Gauss
    imu.az = (float)imu.accelCount[2] * imu.aRes;                     // Convert raw register value to milli-Gauss

    // ----- Read the gyro x|y|z register values
    imu.readGyroData(imu.gyroCount);                                    // Read gyro register values

    // ----- Calculate the gyro value into actual degrees per second
    //       This depends on scale being set
    imu.gx = (float)imu.gyroCount[0] * imu.gRes;                      // Convert raw register value to dps  <-+   plus -ve sign for positive pitch
    imu.gy = (float)imu.gyroCount[1] * imu.gRes;                      // Convert raw register value to dps  <-+--- gx & gy interchanged
    imu.gz = (float)imu.gyroCount[2] * imu.gRes;                      // Convert raw register value to dps <----- applied -ve sign for CW rotation

    // Read the magnetometer x|y|z register values
    imu.readMagData(imu.magCount);                                      // Read magnetometer register values

    // ----- Calculate the magnetometer values in milliGauss and  apply
    //       the ASA fuse ROM values and milli-Gauss scale corrections
    //       The MPU92590 magnetometer uses the 14-bit scale-correction of 0.6
    imu.mx = (float)imu.magCount[0] * imu.mRes * imu.factoryMagCalibration[0] - imu.magBias[0];   // Convert/correct raw register value to milli-Gauss
    imu.my = (float)imu.magCount[1] * imu.mRes * imu.factoryMagCalibration[1] - imu.magBias[1];   // Convert/correct raw register value to milli-Gauss
    imu.mz = (float)imu.magCount[2] * imu.mRes * imu.factoryMagCalibration[2] - imu.magBias[2];   // Convert/correct raw register value to milli-Gauss
  }

  // ----- This library function MUST be called before updating the Mahoney quaternions!
  imu.updateTime();

  /*
    The following quaternion values assume that the MPU-9250 gyro X-axis
    is pointing North and that the gyro Z-axis is pointing upwards.

    These values produce:
      - a clockwise heading of 0..360 degrees if we use the formula "Heading = atan2(imu.mx, imu.my;"
      - q0,q1,q2,q3 values of 1,0,0,0 when the compass is pointing north
  */

  MahonyQuaternionUpdate(  imu.ax,              -imu.ay,              imu.az,
                          imu.gx * DEG_TO_RAD, -imu.gy * DEG_TO_RAD, imu.gz * DEG_TO_RAD,
                          imu.my,              -imu.mx,              -imu.mz,
                          imu.deltat);

  //  MadgwickQuaternionUpdate( imu.ax,              -imu.ay,              imu.az,
  //                            imu.gx * DEG_TO_RAD, -imu.gy * DEG_TO_RAD, imu.gz * DEG_TO_RAD,
  //                            imu.my,              -imu.mx,              -imu.mz,
  //                            imu.deltat);

  // ----- calculate the pitch in degrees using Magwick quaternions
  imu.pitch = asin(2.0f * (*(getQ() + 1) * *(getQ() + 3) - *getQ() * *(getQ() + 2)));

  // ----- calculate the roll in degrees using Magwick quaternions
  imu.roll = -atan2(2.0f * (*getQ() * *(getQ() + 1) + * (getQ() + 2) * *(getQ() + 3)), *getQ() * *getQ() - * (getQ() + 1)
                      * *(getQ() + 1) - * (getQ() + 2) * *(getQ() + 2) + * (getQ() + 3) * *(getQ() + 3));

  // ----- calculate the yaw in degrees using Magwick quaternions
  imu.yaw = atan2(2.0f * (*(getQ() + 1) * *(getQ() + 2) + *getQ() * *(getQ() + 3)), *getQ() * *getQ() + * (getQ() + 1)
                    * *(getQ() + 1) - * (getQ() + 2) * *(getQ() + 2) - * (getQ() + 3) * *(getQ() + 3));
  
  
  
  
}

void calibrateMPU(){

  
  /* Serial for displaying instructions */
  #ifdef DEBUG
  Serial.end();
  delay(15);
  Serial.begin(SAMPLERATE);
  while(!Serial) {}
  /* Start communication with IMU */
  #endif
  
  /* Calibrate magnetometer */
  debugWrite("Calibrating magnetometer, slowly move in a figure 8 until done...");
  imu.magCalMPU9250(imu.magBias, imu.magScale);

  // -----Read the magnetometer x|y|z register values
  imu.readMagData(imu.magCount);

  
  debugWrite("Done!");

  /* TO DO
  debugWrite("Saving results to EEPROM...");

  // Save to EEPROM 
  
  calValue = imu.getMagBiasX_uT();
  memcpy(eeprom_buffer, &calValue, sizeof(calValue));
  calValue = imu.getMagBiasY_uT();
  memcpy(eeprom_buffer + 4, &calValue, sizeof(calValue));
  calValue = imu.getMagBiasZ_uT();
  memcpy(eeprom_buffer + 8, &calValue, sizeof(calValue));
  calValue = imu.getMagScaleFactorX();
  memcpy(eeprom_buffer + 12, &calValue, sizeof(calValue));
  calValue = imu.getMagScaleFactorY();
  memcpy(eeprom_buffer + 16, &calValue, sizeof(calValue));
  calValue = imu.getMagScaleFactorZ();
  memcpy(eeprom_buffer + 20, &calValue, sizeof(calValue));
  for (unsigned int i = 0; i < sizeof(eeprom_buffer); i++) {
    EEPROM.write(i, eeprom_buffer[i]);
  }
  
  debugWrite("Done! You may power off your board.");
  while(1){}
*/
  
}

void readIMUSensor(){
  /*
    For all practical purposes the compass yaw and compass heading are the same
  */
 #ifdef DEBUG
  debugWrite("reading MPU sensordata...");
  #endif
  // ----- display the heading
  float heading = imu.yaw * RAD_TO_DEG;
  if (heading < 0) heading += 360.0;              // Yaw goes negative between 180 amd 360 degrees
  if (True_North == true) heading += Declination; // calculate True North
  if (heading < 0) heading += 360.0; 
  if (heading > 360) heading -= 360.0;
  
  mpuNMEAString = "$"+String(TALKER_ID)+"HDG,"+String(heading,1)+",,,"+String(VARIATION);
  NmeaParser.parseNMEASentence( mpuNMEAString );
  mpuNMEAString = "$"+String(TALKER_ID)+"XDR,,"+String((imu.pitch * RAD_TO_DEG),1)+",D,PITCH,,"+String((imu.roll * RAD_TO_DEG),1)+",D,ROLL";
  NmeaParser.parseNMEASentence( mpuNMEAString );
  startTalking();
}
/*
 * End MPU related functions
 */

#ifdef TEST

String NmeaStream[10] ={
  "$IIVWR,151,R,02.4,N,,,,\r\n",
  "$IIMTW,12.2,C\r\n",
  "!AIVDM,1,1,,A,13aL<mhP000J9:PN?<jf4?vLP88B,0*2B\r\n",
  "$IIDBK,A,0014.4,f,,,,\r\n",
  "$IIVLW,1149.1,N,001.07,N\r\n",
  "$GPGLL,5251.3091,N,00541.8037,E,151314.000,A,D*5B\r\n",
  "$GPRMC,095218.000,A,5251.5621,N,00540.8482,E,4.25,201.77,120420,,,D*6D\r\n",
  "$PSTOB,13.0,v\r\n",
  "$IIVWR,151,R,02.3,N,,,,\r\n",
  "$IIVHW,,,000,M,01.57,N,,\r\n"
  };

 int softIndex = 0;
 long softTimerOld = 0;
 long softTimerNow;


void runSoftGenerator()
{
  softTimerNow = millis();
  //if( softTimerNow - softTimerOld >250 ){
    if (on) {
      digitalWrite(pin, LOW);
      on = false;
      //debugWrite("BLINKER: OFF");
      // If the LED is off, turn it on and remember the state.
    } else {
      digitalWrite(pin, HIGH);
      on = true;
      //Send output to Serial Monitor via debugger
      //debugWrite("BLINKER: ON");
    }
  
    if(softIndex < 10){
      
        
      NmeaParser.parseNMEASentence(NmeaStream[softIndex++]);
      /*
      if(Serial2.availableForWrite()){
      for(uint16_t c=0; c< (NmeaStream[softIndex]).length(); c++){
        Serial2.write((NmeaStream[softIndex]).charAt(c));
      }
      }
      */
      delay(20);
      softIndex++;
      } else softIndex=0;
  
 // }
 // softTimerOld=softTimerNow;
}

#endif

void buttonPressed(){
  TSPoint pt, p = ts.getPoint();
  uint8_t prev_button = active_menu_button;
  pinMode(XM, OUTPUT);
  pinMode(YP, OUTPUT);
  //if (p.z > MINPRESSURE && p.z < MAXPRESSURE)
  
  if (p.z > ts.pressureThreshhold) {
  
    //p.x = my_lcd.Get_Display_Width()-map(p.x, TS_MINX, TS_MAXX, my_lcd.Get_Display_Width(), 0);
    //p.y = my_lcd.Get_Display_Height()-map(p.y, TS_MINY, TS_MAXY, my_lcd.Get_Display_Height(), 0);
    pt = p;
    p.y = map(pt.x, TS_MINX, TS_MAXX, my_lcd.Get_Display_Height(),0);
    p.x = map(pt.y, TS_MINY, TS_MAXY, my_lcd.Get_Display_Width(),0);
  //debugWrite( "x= "+String(p.x)+"y = "+String(p.y)+" z= "+String(p.z)+" treshold= "+String(ts.pressureThreshhold));
    if(p.y> BUTTON_Y && p.y<(BUTTON_Y+BUTTON_H)){
      if(p.x>(2*(BUTTON_X+BUTTON_W))){
        //button ALL or LOG pressed
        if(p.x>(3*(BUTTON_X+BUTTON_W))) active_menu_button = LOG;
        else active_menu_button = ALL;
      }else if( p.x<(BUTTON_X+BUTTON_W)) active_menu_button=SPD;
      else active_menu_button = CRS;
      
      show_string(menu_button[prev_button].button_name,
                  menu_button[prev_button].button_x+5,
                  menu_button[prev_button].button_y+13,
                  menu_button[prev_button].button_name_size,
                  menu_button[prev_button].button_name_colour,
                  menu_button[prev_button].button_colour,
                  true);
       show_string(menu_button[active_menu_button].button_name,
                  menu_button[active_menu_button].button_x+5,
                  menu_button[active_menu_button].button_y+13,
                  menu_button[active_menu_button].button_name_size,
                  RED,
                  menu_button[active_menu_button].button_colour,
                  true);     
      wipe_screen();      
    }
  }
  
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(SAMPLERATE);
  #ifdef DISPLAY_ATTACHED
  my_lcd.Set_Rotation(1); //Landscape
  my_lcd.Init_LCD();
  my_lcd.Fill_Screen(BLACK); 
  char vessel_name[] = VESSEL_NAME;
  show_string(  vessel_name,CENTER,132,8,RED,BLACK,false);
  flag_colour = YELLOW;
  #endif

  #ifdef MPU_ATTACHED
  initializeMPU();
  
  if( mpuNeedsCalibration ){
    calibrateMPU();
  } 
  #endif

  initializeListener();
  initializeTalker();
    
  #ifdef DISPLAY_ATTACHED
  show_menu();
  #endif
  // ----- start software timer
  Stop1 = micros() + Timer1; 
}

void loop() {
  // put your main code here, to run repeatedly:
  //sampleMPU(); 
  #ifdef TEST
  runSoftGenerator();
  #endif

  //*** peridically read the MPU sensor and
  //*** generate NMEA data
  #ifdef MPU_ATTACHED
 if (micros() > Stop1)
  {
    Stop1 += Timer1;                                    // Reset timer

    //readIMUSensor(); 
  }
  #endif
  startListening();

  #ifdef DISPLAY_ATTACHED
  buttonPressed();
  #endif

  startTalking();
}

