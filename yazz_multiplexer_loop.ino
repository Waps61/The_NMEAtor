/*
  Project:  Yazz_Multiplexer.ino, Copyright 2020, Roy Wassili
  Contact:  waps61 @gmail.com
  URL:      https://www.hackster.io/waps61
  Date:     30-04-2020
  Last
  Update:   10-05-2020
  Achieved: NMEAParser is working correctly
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


  Hardware setup MPU9250:
  MPU9250 Breakout --------- Arduino
  VCC ---------------------- 3.3V
  GND ---------------------- GND
  SDA ---------------------- SDA20
  SCL ---------------------- SCL21
  
  Serial is reserved for the MPU9250 communication
        Serial1 is reserved for NMEA listener
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

        
  TODO: 

   Credit:   Based on Alan Burlison's Task Scheduler Library
              The following is Copyright Alan Burlison, 2011
              Original Source Code:  http://bleaklow.com/files/2010/Task.tar.gz
              Original Reference:   http://bleaklow.com/2010/07/20/a_very_simple_arduino_task_manager.html

              Source: https://github.com/gadgetstogrow/TaskScheduler
              Tutorial: https://www.hackster.io/GadgetsToGrow/don-t-delay-use-an-arduino-task-scheduler-today-215cfe

*/

/*
    Include the necessary libraries
*/

#include <Wire.h>
#include <I2Cdev.h>
#include <CalLib.h>
#include <EEPROM.h>
#include <MPU9250.h>  // MPU9250 from the MPU9250-master folder

/*
   Definitions go here
*/
// *** Conditional Debug & Test Info to Serial Monitor
// *** by commenting out the line(s) below the debugger and or test statements will 
// *** be ommitted from the code

//#define DEBUG 1
#define TEST 1
#define SAMPLERATE 115200

#define LISTENER_RATE 4800 // Baudrate for the listner
#define LISTENER_PORT 1   // Serial port 1
#define NMEA_BUFFER_SIZE 83 // According NEA0183 specs the max char is 82


//*** Some conversion factors
#define FTM  0.3048        // feet to meters
#define MTF  3.28084       // meters to feet
#define NTK  1.852         // nautical mile to km
#define KTN  0.5399569     // km to nautical mile

//*** The maximum number of fields in an NMEA string
//*** The number is based on the largest sentence MDA,
//***  the Meteorological Composite sentence
#define MAX_NMEA_FIELDS 21
#define TALKER_RATE 38400 // Baudrate for the listner
#define TALKER_PORT 2   // Serial port 1

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
#define _DBK "$IIDBK"   // Depth below keel
#define _DBS "$IIDBS"   // Depth below surface
#define _DBT "$IIDBT"   // Depth below transducer
#define _HDG "$IIHDG"   // Heading  Deviation & Variation
#define _HDM "$IIHDM"   // Heading Magnetic
#define _HDT "$IIHDT"  // Heading True
#define _MWD "$IIMWD"  // Wind Direction & Speed
#define _MTW "$IIMTW"  // Water Temperature
#define _MWV "$IIMVW"  // Wind Speed and Angle
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
#define _xDR "$"TALKER_ID"""XDR" // Transducer measurement
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
#define NMEA_SPECIALTY ""_DBK""_TOB

/* MPU specific defenitions go here
 *  
 */
 /* accelerometer and magnetometer data */
float a, ax, ay, az, h, hx, hy, hz;
/* magnetometer calibration data */
float hxb, hxs, hyb, hys, hzb, hzs;
/* euler angles */
float pitch_rad, roll_rad, yaw_rad, heading_rad;
/* filtered heading */
float filtered_heading_rad;
float window_size = 20;
/* conversion radians to degrees */
const float R2D = 180.0f / PI;
/* MPU 9250 data ready pin */
const uint8_t kMpu9250DrdyPin = 2;

/* EEPROM buffer to mag bias and scale factors */
uint8_t eeprom_buffer[24];
float calValue;

/* MPU 9250 object */
MPU9250 imu(Wire, 0x68);

bool on = true;
byte pin = 13;
long mpuTimerNow;
long mpuTimerOld=0;

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
   Structures go here
*/
//*** A structure to hold the NMEA data

typedef struct {
  String fields[ MAX_NMEA_FIELDS ];
  byte nrOfFields;
  String sentence;

}NMEAData ;

// ***
// *** debugWrite() <--provides basic debug info from other tasks
// *** takes a String as input parameter
void debugWrite(String debugMsg)
{
  Serial.println(debugMsg);
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
    void setSentence( char _nmeaSentence);
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
}

void NMEAParser::setSentence( char _nmeaSentence)
{
  #ifdef DEBUG
  Serial.println( "NMEA received "+ _nmeaSentence );
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
  Serial.println( " Specialty found... for filter"+filter);
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
      Serial.println("Found "+String(_DBK));
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
      float ft = nmeaIn.fields[1].toFloat();
      nmeaOut.fields[3] = String( ft * FTM, 1);
      nmeaOut.fields[4] = "M";
      nmeaOut.fields[5] = "";
      nmeaOut.fields[6] = "";
      nmeaOut.nrOfFields = 7;
      for( int i=0; i< nmeaOut.nrOfFields ; i++)
      {
        #ifdef DEBUG
        Serial.println("Field["+String(i)+"] = "+nmeaOut.fields[i]);
        #endif
        if(i>0) nmeaOut.sentence+=",";
        nmeaOut.sentence += nmeaOut.fields[i];
      }
      nmeaOut.sentence += checksum( nmeaOut.sentence );
      
      #ifdef DEBUG
      Serial.println( " Modified to:"+nmeaOut.sentence);
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
        Serial.println("Field["+String(i)+"] = "+nmeaOut.fields[i]);
        #endif
        if(i>0) nmeaOut.sentence+=",";
        nmeaOut.sentence += nmeaOut.fields[i];
      }
      nmeaOut.sentence += checksum( nmeaOut.sentence );
      return nmeaOut;
    }
  }
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
  if ( nmeaStr[0] == '$' || nmeaStr[0] == '!' )
  {
    #ifdef DEBUG
    Serial.println(" In te loop to parse for "+String(sentenceLength)+" chars");
    #endif
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
    
    #ifdef TEST
    Serial.println("Parsed : "+nmeaData.sentence );
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



void initializeTalker(){
  Serial2.begin(TALKER_RATE); 
}
/*
 * Start writin converted NNMEA sentences frm the stack
 */
byte startTalking(){
  NMEAData nmeaOut;
  String outStr = "";
  
  
  while( NmeaStack.getIndex() ){
      nmeaOut = NmeaStack.pop();
      outStr =nmeaOut.sentence;
      for(int i=0; i< outStr.length(); i++){
        Serial2.write( outStr[i]);
      }
      #ifdef DEBUG
      debugWrite(" Sending :" + outStr );
      #endif
  }
  return 1;
}

 /* sets the sentence in the nmeaData attrubure and itself to runnable 
 */
void setNMEASentence( String _nmeaIn ){
  NmeaData.sentence = _nmeaIn;
}

/**********************************************************************************
  Purpose:  Helper class reading NMEA data from the serial port as a part of the multiplexer application
            - Reading NMEA0183 v1.5 data without a checksum,
*/


void initializeListener()
{
  Serial1.begin(LISTENER_RATE);
}
/*
 * tart listeneing for incomming NNMEA sentence
 */
bool startListening()
{
  if( Serial1.available() ){
    String nmeaBuffer="";
    nmeaBuffer = Serial1.readStringUntil( '\n' );
    NmeaParser.parseNMEASentence( nmeaBuffer );
  return true;
  } else return false;
}

/*
 * Below the MPU related functions
 */
/*


/* Bound angle between 0 and 360 */
float constrainAngle360(float dta) {
  dta = fmod(dta, 2.0 * PI);
  if (dta < 0.0)
    dta += 2.0 * PI;
  return dta;
}

void initializeMPU(){
/* Serial for displaying results */
  Serial.begin(115200);
  while(!Serial) {}
  /* 
  * Start the sensor, set the bandwidth the 10 Hz, the output data rate to
  * 50 Hz, and enable the data ready interrupt. 
  */
  imu.begin();
  imu.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_10HZ);
  imu.setSrd(19);
  imu.enableDataReadyInterrupt();
  /*
  * Load the magnetometer calibration
  */
  uint8_t eeprom_buffer[24];
  for (unsigned int i = 0; i < sizeof(eeprom_buffer); i++ ) {
    eeprom_buffer[i] = EEPROM.read(i);
  }
  memcpy(&hxb, eeprom_buffer, sizeof(hxb));
  memcpy(&hyb, eeprom_buffer + 4, sizeof(hyb));
  memcpy(&hzb, eeprom_buffer + 8, sizeof(hzb));
  memcpy(&hxs, eeprom_buffer + 12, sizeof(hxs));
  memcpy(&hys, eeprom_buffer + 16, sizeof(hys));
  memcpy(&hzs, eeprom_buffer + 20, sizeof(hzs));
  if( (String(hxb)).indexOf("NAN")>0){
    mpuNeedsCalibration = true;
  } else mpuNeedsCalibration = false;
  if( !mpuNeedsCalibration  ){
    #ifdef DEBUG
    debugWrite(" is calibrated = "+ mpuNeedsCalibration );
    #endif
  imu.setMagCalX(hxb, hxs);
  imu.setMagCalY(hyb, hys);
  imu.setMagCalZ(hzb, hzs);
  #ifdef DEBUG
  debugWrite( "Magnetometer Calibration values:");
  debugWrite("hxb:"+String( hxb,3 )+'\t'+"hxs:"+String(hxs,3));
  debugWrite("hyb:"+String( hyb,3 )+'\t'+"hys:"+String(hys,3));
  debugWrite("hzb:"+String( hzb,3 )+'\t'+"hzs:"+String(hzs,3));
  #endif
  /* Attach the data ready interrupt to the data ready ISR */
  pinMode(kMpu9250DrdyPin, INPUT);
  attachInterrupt(kMpu9250DrdyPin, mpuReady, RISING);  
  debugWrite( "MPU9250 initialized and ready to go...");
  } else{
    debugWrite(" MPU needs calibration before use...");
    debugWrite(" is calibrated = "+ mpuNeedsCalibration );
 #ifdef DEBUG
  debugWrite( "Magnetometer Calibration values:");
  debugWrite("hxb:"+String( hxb,3 )+'\t'+"hxs:"+String(hxs,3));
  debugWrite("hyb:"+String( hyb,3 )+'\t'+"hys:"+String(hys,3));
  debugWrite("hzb:"+String( hzb,3 )+'\t'+"hzs:"+String(hzs,3));
  #endif
  }
}

void sampleMPU(){
  mpuDataReady = false;
  /* Read the MPU 9250 data */
  imu.readSensor();
  ax = imu.getAccelX_mss();
  ay = imu.getAccelY_mss();
  az = imu.getAccelZ_mss();
  hx = imu.getMagX_uT();
  hy = imu.getMagY_uT();
  hz = imu.getMagZ_uT();
  /* Normalize accelerometer and magnetometer data */
  a = sqrtf(ax * ax + ay * ay + az * az);
  ax /= a;
  ay /= a;
  az /= a;
  h = sqrtf(hx * hx + hy * hy + hz * hz);
  hx /= h;
  hy /= h;
  hz /= h;
  /* Compute euler angles */
  pitch_rad = asinf(ax);
  roll_rad = asinf(-ay / cosf(pitch_rad));
  yaw_rad = atan2f(hz * sinf(roll_rad) - hy * cosf(roll_rad), hx * cosf(pitch_rad) + hy * sinf(pitch_rad) * sinf(roll_rad) + hz * sinf(pitch_rad) * cosf(roll_rad));
  heading_rad = constrainAngle360(yaw_rad);
  /* Filtering heading */
  filtered_heading_rad = (filtered_heading_rad * (window_size - 1.0f) + heading_rad) / window_size;
  /* Display the results */
  #ifdef DEBUG
  Serial.print(pitch_rad * R2D);
  Serial.print("\t");
  Serial.print(roll_rad * R2D);
  Serial.print("\t");
  Serial.print(yaw_rad * R2D);
  Serial.print("\t");
  Serial.print(heading_rad * R2D);
  Serial.print("\t");
  Serial.println(filtered_heading_rad * R2D);
  #endif  
  mpuNMEAString = "$"+String(TALKER_ID)+"HDG,"+String(float(filtered_heading_rad*R2D),2)+",,,"+String(VARIATION);
  NmeaParser.parseNMEASentence( mpuNMEAString );
}

void calibrateMPU(){
  /* Serial for displaying instructions */
  Serial.end();
  delay(15);
  Serial.begin(115200);
  while(!Serial) {}
  /* Start communication with IMU */
  imu.begin();
  /* Calibrate magnetometer */
  Serial.print("Calibrating magnetometer, please slowly move in a figure 8 until complete...");
  imu.calibrateMag();
  Serial.println("Done!");
  Serial.print("Saving results to EEPROM...");
  /* Save to EEPROM */
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
  Serial.println("Done! You may power off your board.");
  while(1){}
}

/*
 * End MPU related functions
 */

#ifdef TEST
String NmeaStream[10] ={
  "$IIVWR,151,R,02.4,N,,,,",
  "!AIVDM,1,1,,B,E>jMjb1WT;9h19Q00000000000006jDo>k9<0D2RRSv000,4*30",
  "!AIVDM,1,1,,A,13aL<mhP000J9:PN?<jf4?vLP88B,0*2B",
  "$IIDBK,A,0014.4,f,,,,",
  "$GPGGA,151314.000,5251.3091,N,00541.8037,E,2,8,1.09,1.6,M,46.8,M,0000,0000*55",
  "$GPGLL,5251.3091,N,00541.8037,E,151314.000,A,D*5B",
  "$GPRMC,151314.000,A,5251.3091,N,00541.8037,E,0.09,0.00,070520,,,D*65",
  "$PSTOB,13.0,v",
  "$IIVWR,151,R,02.3,N,,,,",
  "$IIVHW,,,000,M,01.57,N,,"
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
      } else softIndex=0;
  
 // }
 // softTimerOld=softTimerNow;
}

#endif


void setup() {
  // put your setup code here, to run once:
  initializeMPU();
  if( mpuNeedsCalibration ){
    calibrateMPU();
  } else{
    initializeListener();
    initializeTalker();
  }
}

void loop() {
  // put your main code here, to run repeatedly:

#ifdef TEST
  runSoftGenerator();
  delay(100);
  startTalking();
#endif
  //if( startListening() ) startTalking();
  delay(250);
  if (mpuDataReady){
    sampleMPU();
    startTalking();
  }
  delay(250);
  
}
