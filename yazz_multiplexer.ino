/**********************************************************************************
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




  TODO: Serial is reserved for the MPU9250 communication
        Serial1 is reserved for NMEA communication
        Pinout Arduino Mega 2560 R3:
          PIN     PURPOSE                 TASK TYPE
          --------------------------------------------------------------------
          A0:
          Pin 3:
          Pin 4:
          Pin 5:
          Pin 13:

          USB Port: TX/RX Debugger data to Serial Monitor -Task

   Credit:   Based on Alan Burlison's Task Scheduler Library
              The following is Copyright Alan Burlison, 2011
              Original Source Code:  http://bleaklow.com/files/2010/Task.tar.gz
              Original Reference:   http://bleaklow.com/2010/07/20/a_very_simple_arduino_task_manager.html

              Source: https://github.com/gadgetstogrow/TaskScheduler
              Tutorial: https://www.hackster.io/GadgetsToGrow/don-t-delay-use-an-arduino-task-scheduler-today-215cfe

*/

/**********************************************************************************
    Include the necessary libraries
*/

#include <Wire.h>
#include <I2Cdev.h>
//#include <RTIMUSettings.h>
//#include <RTIMU.h>
//#include <RTFusionRTQF.h>
#include <CalLib.h>
#include <EEPROM.h>
//#include <RTIMUMPU9250.h>
#include <PString.h>
#include <nmea.h>
#include <Task.h>
#include <TaskScheduler.h>
#include <quaternionFilters.h>
#include <MPU9250.h>

/***********************************************************************************
   Definitions go here
*/
// *** Conditional Debug & Test Info to Serial Monitor
// *** by commenting out the line(s) below the debugger and or test statements will 
// *** be ommitted from the code

#define DEBUG 1
#define TEST 1

//*** The maximum number of fields in an NMEA string
//*** The number is based on the largest sentence MDA,
//***  the Meteorological Composite sentence
#define MAX_NMEA_FIELDS 21

#define STACKSIZE 15  // Size of the stack; adjust according use

//*** define NMEA tags to be used
//*** make sure you know your Talker ID used in the sentences
//*** In my case next to GP for navigation related sentences
//*** II is used for Integrated Instruments and
//*** PS is used for vendor specific tags like Stowe Marine
//*** AO is used for my Andruino generated sentences
#define _GLL "$GPGLL"   // Geographic Position – Latitude/Longitude
#define _GGA "$GPGGA"   // GPS Fix Data. Time, Position and fix related data for a GPS receiver
#define _GSA "$GPGSA"   // GPS DOP and active satellites
#define _GSV "$GPGSV"   // Satellites in view
#define _DBK "$IIDBK"   // Depth below keel
#define _DBS "$IIDBS"   // Depth below surface
#define _DBT "$IIDBT"   // Depth below transducer
#define _HDG "$IIHDG"   // Heading – Deviation & Variation
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
#define _XDR "$IIXDR"  //  Cross Track Error – Dead Reckoning
#define _XTE "$IIXTE"  //  Cross-Track Error – Measured
#define _XTR "$IIXTR"  //  Cross Track Error – Dead Reckoning
#define _ZDA "$IIZDA"  //  Time & Date - UTC, day, month, year and local time zone
//*** Some specific Robertson / Stowe Marine tags below
#define _TON "$PSTON"  // Distance Nautical
#define _TOE "$PSTOE"  // Engine hours
#define _TOB "$PSTOB"  // Battery voltage
#define _TOD "$PSTOD"  // depth transducer below waterline in feet
//*** Arduino generated TAGS
#define _xDR "$AOXDR" // Transducer measurement
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

//*** Some conversion factors
#define FTM  0.3048        // feet to meters
#define MTF  3.28084       // meters to feet
#define NTK  1.852         // nautical mile to km
#define KTN  0.5399569     // km to nautical mile

/***********************************************************************************
   Structures go here
*/
//*** A structure to hold the NMEA data

typedef struct {
  String fields[ MAX_NMEA_FIELDS ];
  byte nrOfFields;
  String sentence;

}NMEAData ;



/***********************************************************************************
   Class definitions go here
*/



/*****************************************************************************************
   Class:    NMEADebugger
  Task Type:  Task (always runs)
  Purpose:  This expands on Kevin Gagnon who expanded on Alan Burlison's original example
        code which demonstrates a task that reads from the serial port and echoes to
        the Serial Monitor.
        I've expanded it so that other classes use a pointer to the debugger object
        to output simple debug messages while this example executes.
        Classes that use the debugger object are passed a reference to &debugger
        in their respective constructors.

        For example: Blinker(uint8_t _pin, uint32_t _rate, Debugger *_ptrDebugger);

        To output debug information use: ptrDebugger->debugWrite("debug info");

  Notes:    Yeah, I lazily used the String() function in this demonstration. Sue me.
******************************************************************************************/
// ***
// *** Define the NMEADebugger Class as type Task
// ***
class NMEADebugger : public Task
{
  public:
    NMEADebugger();
    void debugWrite(String debugMsg); //Used for simple debugging of other tasks
    void debugWrite( float debugValue, int decimals = 2); //Used for simple debugging of other tasks, based on float values
    virtual void run(uint32_t now);   //Override the run() method
    virtual bool canRun(uint32_t now);  //Override the canRun() method
};

// ***
// *** Debugger Constructor
// ***
NMEADebugger::NMEADebugger()
  : Task()
{
  Serial.begin(4800);
}

// ***
// *** NMEADebugger::canRun() <--checked by TaskScheduler
// ***
bool NMEADebugger::canRun(uint32_t now)
{
  return Serial.available() > 0;
}

// ***
// *** NMEADebugger::run() <--executed by TaskScheduler as a result of canRun() returning true.
// ***
void NMEADebugger::run(uint32_t now)
{
  uint16_t byteCount = 0;

#ifdef DEBUG
  Serial.println("-----------------");
  Serial.println("NMEA Input Received...");
  Serial.println("-----------------");
  while (Serial.available() > 0) {
    int byte = Serial.read();
    Serial.print("'") ;
    Serial.print(char(byte));
    Serial.print("' = ");
    Serial.print(byte, DEC);
    Serial.println(" ");
    if (byte == '\r') {
      Serial.print('\n', DEC);
    }

    byteCount++;
  }

  Serial.println("-----------------");
  Serial.print("Bytes Received: "); Serial.println(String(byteCount));
  Serial.println("-----------------");
  
#endif


}

// ***
// *** NMEADebugger::debugWrite() <--provides basic debug info from other tasks
// *** takes a String as input parameter
void NMEADebugger::debugWrite(String debugMsg)
{
  Serial.println(debugMsg);
}

// ***
// *** NMEADebugger::debugWrite() <--provides basic debug info from other tasks
// *** takes a float and nr of decimals as input parameters
void NMEADebugger::debugWrite(float debugValue, int decimals = 2)
{
  Serial.println(debugValue, decimals);
}

/*
 * Class NMEAStack
 * A class to push and pop NMEA structs on a stack and can act as a buffer
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
  int lastIndex;    // an index pointng to the first free psotiion in the stack
 };
 
  NMEAStack::NMEAStack()
  {
    lastIndex = 0;
  }
  
  int NMEAStack::push( NMEAData _nmea )
  {
    if( lastIndex < STACKSIZE )
    {
 /*     for(int i=0; i<MAX_NMEA_FIELDS; i++ )
      {
        stack[ lastIndex ].fields[i] = _nmea.fields[i];
      }
      stack[ lastIndex ].nrOfFields = _nmea.nrOfFields;
      stack[ lastIndex ].sentence = _nmea.sentence;
      */
      stack[ lastIndex ] = _nmea;
      return ++lastIndex;
    } else
    {
      lastIndex = STACKSIZE;
      return -1;    // of stack is full
    }
  }

  NMEAData NMEAStack::pop()
  {
    NMEAData nmeaOut;
    nmeaOut.sentence = "";
    if( lastIndex>0)
    {
      lastIndex--;
 /*     
  *      for( int i=0; i<MAX_NMEA_FIELDS; i++ )
      {
        nmeaOut.fields[i] = stack[lastIndex].fields[i];
      }
      nmeaOut.nrOfFields = stack[lastIndex].nrOfFields;
      nmeaOut.sentence = stack[lastIndex].sentence;
*/
      nmeaOut=stack[ lastIndex ];
    }
    
    return nmeaOut;   
  }

  int NMEAStack::getIndex()
  {
    return lastIndex;
  }
 

/*****************************************************************************************
  Define the NMEAtor Class as type TriggeredTask

  Demonstrate basic serial communication functionality reading NMEA data from the register.
  An RS232 to TTL  cnverter or RS484 shifter should be used to prevent frying the Arduino board

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


*****************************************************************************************/
class NMEAtor : public TriggeredTask
{
  public:
    NMEAtor(uint32_t _rate, NMEADebugger *_ptrDebugger, NMEAStack *_ptrNMEAStack);
    virtual void run(uint32_t now);
    void setSentence( String _nmeaSentence);
    void parseNMEA(String nmeaIn ); // parse an NMEA sentence with each part stored in the array
    NMEAData getNMEAData(); // getter function to get the NMEADAta struct
    
  private:
    uint32_t rate;
    NMEADebugger *ptrDebugger;
    NMEAStack *ptrNMEAStack;
    MPU9250 mpu9250;
    String NMEAFilter = NMEA_SPECIALTY;
    NMEAData nmeaData;  // self explaining
    String nmeaSentence = "";

    String checksum( String str ); //calculate the checksum for str
    String nmeaSpecialty( NMEAData nmeaIn ); // special treatment function
};

// ***
// *** NMEAtor Constructor
// *** input parameters:
// *** reference to the debugger object
NMEAtor::NMEAtor(uint32_t _rate, NMEADebugger *_ptrDebugger, NMEAStack *_ptrNMEAStack)
  : TriggeredTask(),
    rate( _rate),
    ptrDebugger(_ptrDebugger),
    ptrNMEAStack(_ptrNMEAStack)
{
  Wire.begin();

  Serial1.begin(_rate);


}

// ***
// *** NMEAtor::run() <--executed by TaskScheduler as a result of canRun() returning true.
// ***
void NMEAtor::run(uint32_t now)
{
  // *** proces the nmeaSentence here

#ifdef DEBUG
  // Print raw NMEA sentences
  ptrDebugger->debugWrite("-----Raw NMEA Data received on serial port 1-----");
  ptrDebugger->debugWrite( nmeaSentence);
#endif

  parseNMEA(nmeaSentence );


#ifdef DEBUG
  ptrDebugger->debugWrite("-----End Data processed on serial port 1-----");
  #endif


  // ***
  // *** resetRunnable() IMPORTANT! IMPORTANT!
  // *** It's important to resetRunnable() after executing a TriggeredTask.
  // *** If bool runFlag in a TriggeredTask is not reset, the TriggeredTask will
  // *** continue to run indefinitely which defeats its purpose. It will stay dormant
  // *** and be ignored by the TaskScheduler until triggered again.
  // ***
  resetRunnable();
}

void NMEAtor::setSentence( String _nmeaSentence)
{
  nmeaSentence = _nmeaSentence;
}

/*

*/
String NMEAtor::nmeaSpecialty( NMEAData nmeaIn )
{
  String filter = NMEA_SPECIALTY;
  String sentence = nmeaIn.sentence;
  String newSentence ="";
  #ifdef TEST
  ptrDebugger->debugWrite( " Specialty found... for filter"+filter);
  #endif
  if ( filter.indexOf( nmeaIn.fields[0]) > -1 )
  {
    /* In my on-board Robertson data network some sentences
       are not NMEA0183 compliant. So these sentences need
       to be converted to compliant sentences
    */
    //*** $IIDBK is not NMEA0183 compliant and needs conversion
    if ( nmeaIn.fields[0] == _DBK ) {
      #ifdef TEST
      ptrDebugger->debugWrite("Found "+String(_DBK));
      #endif
      // a typical non standard DBK message I receive is
      // $IIDBK,A,0017.6,f,,,,
      // Char A can also be a V if invalid and shoul be removed
      // All fields after the tag shift 1 position to the left
      // Since we modify the sentence we'll also put our talker ID in place
      nmeaIn.fields[0]="$AODBK";
      for( int i=1; i<nmeaIn.nrOfFields-2; i++ )
      {
        nmeaIn.fields[i] = nmeaIn.fields[i+1];
      }
      // We need the the feet  to convert to meters and add to string
      float ft = nmeaIn.fields[1].toFloat();
      nmeaIn.fields[3] = String( ft * FTM, 1);
      nmeaIn.fields[4] = "M";
      nmeaIn.fields[5] = "";
      nmeaIn.fields[6] = "";

      for( int i=0; i< nmeaIn.nrOfFields -1; i++)
      {
        #ifdef TEST
        ptrDebugger->debugWrite("Field["+String(i)+"] = "+nmeaIn.fields[i]);
        #endif
        if(i>0) newSentence+=",";
        newSentence += nmeaIn.fields[i];
      }
      newSentence += ",*"+checksum( newSentence );

      #ifdef TEST
      ptrDebugger->debugWrite( " Modified to:"+newSentence);
      #endif
      return newSentence;
    }
    //*** current $IIHDG,$IIHDM & $IIVHW sentences are causing issues in my navigational app
    //*** so its been filtered out here
    if ( nmeaIn.fields[0] == _HDM || nmeaIn.fields[0] == _HDG || nmeaIn.fields[0] == _VHW) {
      #ifdef TEST
      ptrDebugger->debugWrite("Found "+nmeaIn.fields[0]);
      #endif
      newSentence = "";
      return newSentence;
    }

    //*** current Battery info is in a non NMEA0183 format 
    //*** i.e. $PSTOB,13.2,V
    //*** will be converted to $AOXDR,U,13.2,V,BATT,*CS
    if( nmeaIn.fields[0] == _TOB )
    {
      nmeaIn.nrOfFields =5;
      nmeaIn.fields[0] = "$AOXDR";
      //*** by working backwards we can shift some values to the right index
      nmeaIn.fields[4] ="BATT";
      nmeaIn.fields[3] = nmeaIn.fields[2]; // unit of measure
      nmeaIn.fields[2] = nmeaIn.fields[1];  // the actual measurement value
      nmeaIn.fields[1] ="U";  // the transducer unit
      nmeaIn.fields[3].toUpperCase();
      for( int i=0; i< nmeaIn.nrOfFields -1; i++)
      {
        #ifdef TEST
        ptrDebugger->debugWrite("Field["+String(i)+"] = "+nmeaIn.fields[i]);
        #endif
        if(i>0) newSentence+=",";
        newSentence += nmeaIn.fields[i];
      }
      newSentence += checksum( newSentence );

    }
  }
}



// calculate checksum function (thanks to https://mechinations.wordpress.com)
String NMEAtor::checksum( String str )
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
void NMEAtor::parseNMEA(String nmeaStr)
{
  nmeaData.nrOfFields = 0;
  nmeaData.sentence="";
  for(int i=0; i<MAX_NMEA_FIELDS; i++)
  {
    nmeaData.fields[i]="";
  }
  int currentIndex = 0;
  int lastIndex = -1;
  int sentenceLength = nmeaStr.length();
  String newSentence = "";  // used to construct the new senctence
  //*** check for a valid NMEA sentence
  if ( nmeaStr[0] == '$' || nmeaStr[0] == '!' )
  {
    #ifdef TEST
    ptrDebugger->debugWrite(" In te loop to parse for "+String(sentenceLength)+" chars");
    #endif
    //*** parse the fields from the NMEA string
    //*** keeping in mind that indeOf() can return -1 if not found!
    currentIndex = nmeaStr.indexOf( ',',0);
    while ( lastIndex < sentenceLength  )
    {
      
      //*** remember to sepatrate fields with the ',' character
      //*** but do not end with one!
      if ( lastIndex>0 ) newSentence += ',';

      //*** we want the data without the ',' in fields array
      if( currentIndex-lastIndex >1 ) // check for an empty field
      {
        nmeaData.fields[ nmeaData.nrOfFields ] = nmeaStr.substring(lastIndex+1, currentIndex );
        newSentence += nmeaData.fields[ nmeaData.nrOfFields];
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
      newSentence = nmeaSpecialty( nmeaData );
    } else if(newSentence.indexOf('*')<1)  //Check for checksum in sentence
    {
      newSentence += checksum( newSentence);
    }
    nmeaData.sentence = newSentence;
    #ifdef DEBUG
    ptrDebugger->debugWrite( "new sentence is: ");
    ptrDebugger->debugWrite(nmeaData.sentence );
    #endif
    ptrNMEAStack->push( nmeaData );   //push the struct to the stack for later use; i.e. buffer it
  }

  return;
}

/*
 * getter function to get the NMEADAta structure
 */
NMEAData NMEAtor::getNMEAData()
{
  return nmeaData;
}

/*****************************************************************************************
  /*
  /* Define the MPU Class as type TriggeredTask

   This class is basically a wrapper class fro the real MPU9250, which is
   not a virtual class and is instantiated as an object within this wrapper.
  Demonstrate basic MPU-9250 functionality including parameterizing the register
  addresses, initializing the sensor, getting properly scaled accelerometer,
  gyroscope, and magnetometer data out. Added display functions to allow display
  to on breadboard monitor. Addition of 9 DoF sensor fusion using open source
  Madgwick and Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini
  and the Teensy 3.1.
  SDA and SCL should have external pull-up resistors (to 3.3V).
  10k resistors are on the EMSENSR-9250 breakout board.
  Hardware setup:
  MPU9250 Breakout --------- Arduino
  VDD ---------------------- 3.3V
  VDDI --------------------- 3.3V
  SDA ----------------------- A4
  SCL ----------------------- A5
  GND ---------------------- GND

******************************************************************************************/

class MPU : public TriggeredTask
{
  public:
    MPU(NMEADebugger *_ptrDebugger);
    virtual void run(uint32_t now);
  private:
    uint32_t rate;
    NMEADebugger *ptrDebugger;
    MPU9250 mpu9250;
};

// ***
// *** MPU Constructor
// *** input parameters:
// *** initial samplerate for the MPU9250 i.e. 19200
MPU::MPU(NMEADebugger *_ptrDebugger)
  : TriggeredTask(),
    ptrDebugger(_ptrDebugger)
{
  Wire.begin();
  // TWBR = 12;  // 400 kbit/sec I2C speed
  Serial.begin(19200);


  // Read the WHO_AM_I register, this is a good test of communication
  byte c = mpu9250.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  /*Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
    Serial.print(" I should be "); Serial.println(0x71, HEX);*/
  ptrDebugger->debugWrite( "MPU9250 ");
  ptrDebugger->debugWrite( "I AM " + String( c, HEX) );
  ptrDebugger->debugWrite( " I should be " + String( 0x71, HEX) );

  if (c == 0x73) // WHO_AM_I should always be 0x68
  {
    //Serial.println("MPU9250 is online...");
    ptrDebugger->debugWrite("MPU9250 is online...");

    // Start by performing self test and reporting values
    mpu9250.MPU9250SelfTest(mpu9250.SelfTest);
    ptrDebugger->debugWrite("Performing self test...");

    // Calibrate gyro and accelerometers, load biases in bias registers
    mpu9250.calibrateMPU9250(mpu9250.gyroBias, mpu9250.accelBias);
    ptrDebugger->debugWrite("Calibrating gyro and accelerometers, load biases in bias registers...");

    mpu9250.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    //Serial.println("MPU9250 initialized for active data mode....");
    ptrDebugger->debugWrite("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = mpu9250.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    //Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX);
    //Serial.print(" I should be "); Serial.println(0x48, HEX);
    ptrDebugger->debugWrite("AK8963 magnetometer");
    ptrDebugger->debugWrite("I AM " + String(d, HEX));
    ptrDebugger->debugWrite(" I should be ");
    ptrDebugger->debugWrite("I AM " + String(0x48, HEX));


    // Get magnetometer calibration from AK8963 ROM
    mpu9250.initAK8963(mpu9250.magCalibration);
    // Initialize device for active mode read of magnetometer
    //Serial.println("AK8963 initialized for active data mode....");
    ptrDebugger->debugWrite("AK8963 magnetometer initialized for active data mode....");

#ifdef DEBUG
    ptrDebugger->debugWrite("Calibration values: ");
#endif

  } // if (c == 0x71)
  else
  {
    //Serial.print("Could not connect to MPU9250: 0x");
    //Serial.println(c, HEX);
    ptrDebugger->debugWrite("Could not connect to MPU9250: 0x");
    ptrDebugger->debugWrite(String(c, HEX));

    ptrDebugger->debugWrite("Waiting for ever.... or restart");
    //while (1) ; // Loop forever if communication doesn't happen
  }

}

// ***
// *** MPU::run() <--executed by TaskScheduler as a result of canRun() returning true.
// ***
void MPU::run(uint32_t now)
{
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (mpu9250.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    mpu9250.readAccelData(mpu9250.accelCount);  // Read the x/y/z adc values
    mpu9250.getAres();

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    mpu9250.ax = (float)mpu9250.accelCount[0] * mpu9250.aRes; // - accelBias[0];
    mpu9250.ay = (float)mpu9250.accelCount[1] * mpu9250.aRes; // - accelBias[1];
    mpu9250.az = (float)mpu9250.accelCount[2] * mpu9250.aRes; // - accelBias[2];

    mpu9250.readGyroData(mpu9250.gyroCount);  // Read the x/y/z adc values
    mpu9250.getGres();

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    mpu9250.gx = (float)mpu9250.gyroCount[0] * mpu9250.gRes;
    mpu9250.gy = (float)mpu9250.gyroCount[1] * mpu9250.gRes;
    mpu9250.gz = (float)mpu9250.gyroCount[2] * mpu9250.gRes;

    mpu9250.readMagData(mpu9250.magCount);  // Read the x/y/z adc values
    mpu9250.getMres();
    // User environmental x-axis correction in milliGauss, should be
    // automatically calculated
    mpu9250.magbias[0] = +470.;
    // User environmental x-axis correction in milliGauss TODO axis??
    mpu9250.magbias[1] = +120.;
    // User environmental x-axis correction in milliGauss
    mpu9250.magbias[2] = +125.;

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    mpu9250.mx = (float)mpu9250.magCount[0] * mpu9250.mRes * mpu9250.magCalibration[0] -
                 mpu9250.magbias[0];
    mpu9250.my = (float)mpu9250.magCount[1] * mpu9250.mRes * mpu9250.magCalibration[1] -
                 mpu9250.magbias[1];
    mpu9250.mz = (float)mpu9250.magCount[2] * mpu9250.mRes * mpu9250.magCalibration[2] -
                 mpu9250.magbias[2];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions!
  mpu9250.updateTime();

  //digitalWrite(myLed, !digitalRead(myLed));  // toggle led
  mpu9250.delt_t = 600; //millis() - mpu9250.count;
  if (mpu9250.delt_t > 500)
  {

#ifdef DEBUG
    // Print acceleration values in milligs!
    ptrDebugger->debugWrite(1000 * mpu9250.ax); ptrDebugger->debugWrite("\t");
    ptrDebugger->debugWrite(1000 * mpu9250.ay); ptrDebugger->debugWrite("\t");
    ptrDebugger->debugWrite(1000 * mpu9250.az); ptrDebugger->debugWrite("\t");
    /*ptrDebugger->debugWrite("X-acceleration: "); ptrDebugger->debugWrite(1000*mpu9250.ax);
      ptrDebugger->debugWrite(" mg ");
      ptrDebugger->debugWrite("Y-acceleration: "); ptrDebugger->debugWrite(1000*mpu9250.ay);
      ptrDebugger->debugWrite(" mg ");
      ptrDebugger->debugWrite("Z-acceleration: "); ptrDebugger->debugWrite(1000*mpu9250.az);
      Serial.println(" mg ");*/

    // Print gyro values in degree/sec
    ptrDebugger->debugWrite(mpu9250.gx, 3); ptrDebugger->debugWrite("\t");
    ptrDebugger->debugWrite(mpu9250.gy, 3); ptrDebugger->debugWrite("\t");
    ptrDebugger->debugWrite(mpu9250.gz, 3); ptrDebugger->debugWrite("\t");
    /*ptrDebugger->debugWrite("X-gyro rate: "); ptrDebugger->debugWrite(mpu9250.gx, 3);
      ptrDebugger->debugWrite(" degrees/sec ");
      ptrDebugger->debugWrite("Y-gyro rate: "); ptrDebugger->debugWrite(mpu9250.gy, 3);
      ptrDebugger->debugWrite(" degrees/sec ");
      ptrDebugger->debugWrite("Z-gyro rate: "); ptrDebugger->debugWrite(mpu9250.gz, 3);
      Serial.println(" degrees/sec");*/

    // Print mag values in degree/sec
    ptrDebugger->debugWrite(mpu9250.mx); ptrDebugger->debugWrite("\t");
    ptrDebugger->debugWrite(mpu9250.my); ptrDebugger->debugWrite("\t");
    ptrDebugger->debugWrite(mpu9250.mz); ptrDebugger->debugWrite("\n");

    ptrDebugger->debugWrite("X-mag field: "); ptrDebugger->debugWrite(mpu9250.mx);
    ptrDebugger->debugWrite(" mG ");
    ptrDebugger->debugWrite("Y-mag field: "); ptrDebugger->debugWrite(mpu9250.my);
    ptrDebugger->debugWrite(" mG ");
    ptrDebugger->debugWrite("Z-mag field: "); ptrDebugger->debugWrite(mpu9250.mz);
    ptrDebugger->debugWrite(" mG");

#endif
    mpu9250.tempCount = mpu9250.readTempData();  // Read the adc values
    // Temperature in degrees Centigrade
    mpu9250.temperature = ((float) mpu9250.tempCount) / 333.87 + 21.0;

#ifdef DEBUG
    // Print temperature in degrees Centigrade
    ptrDebugger->debugWrite("Temperature is ");  ptrDebugger->debugWrite(mpu9250.temperature, 1);
    ptrDebugger->debugWrite(" degrees C");
#endif
  }

  // ***
  // *** resetRunnable() IMPORTANT! IMPORTANT!
  // *** It's important to resetRunnable() after executing a TriggeredTask.
  // *** If bool runFlag in a TriggeredTask is not reset, the TriggeredTask will
  // *** continue to run indefinitely which defeats its purpose. It will stay dormant
  // *** and be ignored by the TaskScheduler until triggered again.
  // ***
  resetRunnable();
}




#ifdef TEST
class SoftGenerator : public TimedTask
{
  public:
  SoftGenerator( uint32_t _pin, uint32_t _rate, NMEADebugger *_ptrDebugger, NMEAtor *_ptrNMEAtor);
  virtual void run( uint32_t now);

  private:
  uint32_t pin;
  uint32_t rate;
  bool on;
  NMEADebugger *ptrDebugger;
  NMEAtor *ptrNMEAtor;
  int index;
  
  String nmeaStream[10] ={
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

 
};

SoftGenerator::SoftGenerator( uint32_t _pin, uint32_t _rate, NMEADebugger *_ptrDebugger, NMEAtor *_ptrNMEAtor)
      : TimedTask(millis()),
      pin( _pin ),
      rate(_rate),
      on(false),
      ptrDebugger(_ptrDebugger),
      ptrNMEAtor(_ptrNMEAtor)
      {
        pinMode( 13, OUTPUT );

        index = 0;
      }

void SoftGenerator::run( uint32_t now ){
  if (on) {
    digitalWrite(pin, LOW);
    on = false;
    ptrDebugger->debugWrite("BLINKER: OFF");
    // If the LED is off, turn it on and remember the state.
  } else {
    digitalWrite(pin, HIGH);
    on = true;
    //Send output to Serial Monitor via debugger
    ptrDebugger->debugWrite("BLINKER: ON");
  }

  if(index < 10){
    ptrNMEAtor->setSentence(nmeaStream[index++]);
   } else index=0;
   ptrNMEAtor->setRunnable();
    
        // Run again in the specified number of milliseconds.
  incRunTime(rate);
  
  
}


#endif
void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
/***********************************************************************************
   Global variables go here
*/

NMEAStack       NmeaStack;

NMEADebugger    Debugger;
//MPU             Mpu(&Debugger);
NMEAtor         Nmeator( 4800, &Debugger, &NmeaStack);
SoftGenerator TestNMEA( 13, 100, &Debugger, &Nmeator);

  Task *tasks[] = {

    &Debugger,
//    &Mpu,
    &Nmeator
    #ifdef TEST
    ,&TestNMEA
    #endif
  };

  // ***
  // *** Instantiate the TaskScheduler and fill it with tasks.
  // ***
  TaskScheduler scheduler(tasks, NUM_TASKS(tasks));

  // GO! Run the scheduler - it never returns.
  scheduler.runTasks();

}

/*
   SerialEvent is linked to RX to read MPU9250 data
   SerialEvent occurs whenever a new data comes in the hardware serial RX. This
   routine is run between each time loop() runs, so using delay inside loop can
   delay response. Multiple bytes of data may be available.
*/

/*
void serialEvent() {
  if (Serial.available()) {

    Mpu.setRunnable();
  }

}
*/
/*
   SerialEvet1 is linked to RX1 to read NMEA data
   SerialEvent1 occurs whenever a new data comes in the hardware serial RX1. This
   routine is run between each time loop() runs, so using delay inside loop can
   delay response. Multiple bytes of data may be available.
*/
/*
void serialEvent1() {
  while (Serial1.available()) {
    // get the new byte:
    char inChar = (char)Serial1.read();
    // add it to the inputString:
    Nmeator.nmeaSentence += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      //stringComplete = true;
      Nmeator.setRunnable();
    }
  }

}
*/
