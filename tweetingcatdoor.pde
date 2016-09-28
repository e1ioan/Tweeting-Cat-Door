/*
  [1] GND (Ground)
 [2] /RST (Reset Reader) Connect to +5vDC     
 [3] ANT (NC)  
 [4] ANT (NC)      
 [5] CP  (NC) (Card Present)     
 [6] NC
 [7] FS (Format select) (ground this for ASCII)
 [8] D1 (NC)
 [9] D0 (TTL Serial) to arduino RX             
 [10] Buzzer/LED (Card read indicator) (Connect to transistor + buzzer/led to indicate card was read)
 [11] 5v+ ( +5vDC power) 
 
 On Arduino...
 [RX0] (Serial IN) to [9] D0 TTL serial out on ID-20
 
 Remark: disconnect the rx serial wire to the ID-12 when uploading the sketch
 -------------------------------------------------------------------------------------------
 */


#include <Servo.h> 
#include <EEPROM.h> // Needed to write to EEPROM storage
#include <Time.h>
#include <TimeAlarms.h>
#include <IRremote.h>

const int servoPin = 12; 
const int irLedPin = 3;
const int irDetectPin = 2;
const int buttonPin = 4;
int ledRGBOne[] = {
  6, 7, 8}; //the three digital pins of the first digital LED 6 = redPin, 7 = greenPin, 8 = bluePin

const boolean ON = LOW;  //Define on as LOW (this is because we use a common Anode RGB LED (common pin is connected to +5 volts)
const boolean OFF = HIGH; //Define off as HIGH

//Predefined Colors
const boolean RED[] = {
  ON, OFF, OFF};    
const boolean GREEN[] = {
  OFF, ON, OFF}; 
const boolean BLUE[] = {
  OFF, OFF, ON}; 
const boolean YELLOW[] = {
  ON, ON, OFF}; 
const boolean CYAN[] = {
  OFF, ON, ON}; 
const boolean MAGENTA[] = {
  ON, OFF, ON}; 
const boolean WHITE[] = {
  ON, ON, ON}; 
const boolean BLACK[] = {
  OFF, OFF, OFF}; 

#define OPEN_SECONDS 5 // do it 15 for production
#define IN_OUT 0
#define ONLY_IN 1
#define LOCKED 2

int objectPresent = LOW;

boolean programMode = false;
boolean match = false;
boolean openbyir = false;
boolean openbycard = false;
boolean doorIsOpen = false;
boolean isTimerActive = false;
boolean buttonPressed = false;
boolean isRFIDParallax = false;

byte storedCard[10];  // Stores an ID read from EEPROM
byte readCard[10];    // Sotres an ID read from the RFID reader
byte first_rfid_byte;
int lockMode = 0;  // lock mode: 0 (green) open by rfid/ir; 1 (blue) only in rfid; 2 (red) locked

Servo myservo;  // create servo object to control a servo 

IRsend irsend; // create IR object

int servopos = 20; // variable to store the servo position

/***********************************************************************************************/
void setup() 
{
  //  for (int i = 0; i < 512; i++) // Uncoment to wipe the EEPROM
  //    EEPROM.write(i, 0);

  for(int i = 0; i < 3; i++)
  {   
    pinMode(ledRGBOne[i], OUTPUT);  //Set the three LED pins as outputs  
  } 

  pinMode(buttonPin, INPUT);  // initialize the pushbutton pin as an input:   

  myservo.attach(servoPin);  // attaches the servo on pin 9 to the servo object

  if(isRFIDParallax)
  {
    first_rfid_byte = 10; // Parallax : First Byte should be 10
    Serial.begin(2400); // for for parallax
  }
  else
  {
    first_rfid_byte = 2; // ID-12 First Byte should be 2, STX byte
    Serial.begin(9600); // for id-12 Connect to the serial port
  }
  Serial.flush();

  irsend.enableIROut(38); // setup the IR object on 38 kHz
  irsend.mark(0);  

  setTime(14,23,40,2,12,10); // set time to 14:23:40 feb 12, 2010
  Alarm.timerOnce(2, onceOnlyCloseDoor); 

  servoClose(); // test the servo
  failedWrite(); // test the LED
}

/***********************************************************************************************/
void loop () 
{
  byte val = 0;       // Temp variable to hold the current byte 
  normalModeOn();    

  if (programMode)   // Program mode to add a new ID card
  {
    programModeOn();  // Program Mode cycles through RGB waiting to read a new card

    if(Serial.available() > 0)  // Waits for something to come on the serial line
    {
      if((val = Serial.read()) == first_rfid_byte)
      {  
        getID();                      // Get the ID, sets readCard = to the read ID
        if ( !isMaster(readCard) )    // Check to see if it is the master programing card
        {
          writeID(readCard);          // If not, write the card to the EEPROM sotrage
          programMode = false;        // Turn off programing mode
        }
      }
    }
  }
  // Normal Operation...
  else 
  {
    // check if the pushbutton is pressed.
    // if it is, the buttonState is HIGH:
    if (digitalRead(buttonPin) == HIGH) 
    {         
      if (!buttonPressed)
      {
        lockMode +=1;
        if (lockMode > 2)
        {
          lockMode = 0;
        }
        buttonPressed = true;
      }
    } 
    else 
    {
      buttonPressed = false; 
    }

    if ((lockMode != ONLY_IN) & (lockMode != LOCKED) )
    {
      // read the state of the IRDetector value:
      objectPresent = digitalRead(irDetectPin);

      // check if the pushbutton is pressed.
      // if it is, the buttonState is HIGH:
      if (objectPresent == HIGH) 
      {     
        if ( !openbycard )
        {
          openbyir = true;
          openDoor();              //open the door lock 
        }
        else
        {
          openbyir = false;
          openbycard = false;
        }     
      } 
    }

    if (lockMode != LOCKED )
    {
      if(Serial.available() > 0)          // If the serial port is available and sending data...
      {
        if((val = Serial.read()) == first_rfid_byte)
        {                  
          getID();          // Get the ID, sets readCard = to the read ID
          byte bytesread = 0;
          {                                     // matches the checksum caculated 
            if ( isMaster( readCard ) )         // Check to see if the card is the master programing card
            {
              programMode = true;               // If so, enable programing mode
            }
            else
            {
              if ( findID(readCard) )           // If not, see if the card is in the EEPROM
              {
                for(int k=0; k<10; k++)
                  Serial.print(readCard[k]); // write the tag to the serial
                if ( openbyir )
                {
                  openbyir = false;
                  openbycard = false;
                  Alarm.delay(500); // wait to make sure the cat is out
                  Serial.println("-out.txt");
                }
                else
                {
                  openbycard = true;
                  openbyir = false;
                  Serial.println("-in.txt");
                  openDoor(); // If it is, open the door lock
                } 
              }
              else
              {
                Serial.println("unauthorized.txt");
                failed();                       // If not, show that the ID was not valid
              }
            }
          }
        }
      }
    }
  }
  Alarm.delay(500);
}


/***********************************************************************************************/
// If the serial port is ready and we received the STX BYTE (2) then this function is called 
// to get the 4 BYTE ID + 1 BYTE checksum. The ID+checksum is stored in readCard[6]
// Bytes 0-4 are the 5 ID bytes, byte 5 is the checksum

void getID()
{
  byte bytesread = 0;
  byte i = 0;
  byte val = 0;
  while ( bytesread < 10 ) // Read 10 digit code + 2 digit checksum
  {                        
    if( Serial.available() > 0)   // Check to make sure data is coming on the serial line
    { 
      val = Serial.read();        // Store the current ASCII byte in val
      if((val == 0x0D)||(val == 0x0A)||(val == 0x03)||(val == 0x02))
      {                           // If header or stop bytes before the 10 digit reading
        break;                    // Stop reading                                 
      }
      readCard[bytesread] = val;
      bytesread++;           // Increment the counter to keep track
    }
  }
  Alarm.delay(1000);
  Serial.flush();
  bytesread = 0;
}

/***********************************************************************************************/
// Read an ID from EEPROM and save it to the storedCard[6] array
void readID( int number )  // Number = position in EEPROM to get the 5 Bytes from 
{
  int start = (number * 10 ) - 9;  // Figure out starting position   
  for ( int i = 0; i < 10; i++ )  // Loop 5 times to get the 5 Bytes
  {
    storedCard[i] = EEPROM.read(start+i);  // Assign values read from EEPROM to array
  }   
}

/***********************************************************************************************/
// Write an array to the EEPROM in the next available slot
void writeID( byte a[] )
{
  if ( !findID( a ) )          // Before we write to the EEPROM, check to see if we have seen this card before!
  {
    int num = EEPROM.read(0);  // Get the numer of used spaces, position 0 stores the number of ID cards
    int start = ( num * 10 ) + 1;   // Figure out where the next slot starts
    num++;                         // Increment the counter by one
    EEPROM.write( 0, num );        // Write the new count to the counter

    for ( int j = 0; j < 10; j++ )  // Loop 5 times
    {
      EEPROM.write( start+j, a[j] );  // Write the array values to EEPROM in the right position
    }
    Serial.println();
    successWrite();
  }
  else
  {
    failedWrite();
  }
}

/***********************************************************************************************/
// Check two arrays of bytes to see if they are exact matches
boolean checkTwo ( byte a[], byte b[] )
{
  if ( a[0] != NULL )             // Make sure there is something in the array first
    match = true;                 // Assume they match at first

  for ( int k = 0;  k < 10; k++ )  // Loop 5 times
  {
    if ( a[k] != b[k] )           // IF a != b then set match = false, one fails, all fail
      match = false;
  }
  if ( match )                    // Check to see if if match is still true
    return true;                  // Return true
  else 
    return false;                 // Return false
}

/***********************************************************************************************/
// Looks in the EEPROM to try to match any of the EEPROM ID's with the passed ID
boolean findID( byte find[] )
{
  int count = EEPROM.read(0);             // Read the first Byte of EEPROM that
  for ( int i = 1; i <= count; i++ )      // Loop once for each EEPROM entry
  {
    readID(i);                            // Read an ID from EEPROM, it is stored in storedCard[6]
    if( checkTwo( find, storedCard ) )    // Check to see if the storedCard read from EEPROM 
    {                                     // is the same as the find[] ID card passed
      return true;
      break;                              // Stop looking we found it
    }
  }
  return false;
}

/***********************************************************************************************/
// Flashes Red LED if failed login
void failed()
{
  setColor(ledRGBOne, RED);
  Alarm.delay(2000);
}

/***********************************************************************************************/
// Check to see if the ID passed is the master programing card
boolean isMaster( byte test[] ) 
{
  byte val[10] = {
    '0','1','0','6','8','E','2','0','8','1'  };   // <-- The MASTER card
  if ( checkTwo( test, val ) ) // Check to see if the master = the test ID
    return true;
  else
    return false;
}
/***********************************************************************************************/
// Opens door and turns on the green LED for setDelay seconds
void openDoor()
{
  if (!doorIsOpen)
  {
    doorIsOpen = true;
    servoOpen();
    if (!isTimerActive)
    {  
      isTimerActive = true;
      Alarm.timerOnce(OPEN_SECONDS, onceOnlyCloseDoor);            // called once after "setDelay" seconds 
    }
  }
}

/***********************************************************************************************/
void onceOnlyCloseDoor()
{
  if (objectPresent == HIGH) 
  {
    isTimerActive = true;
    Alarm.timerOnce(OPEN_SECONDS, onceOnlyCloseDoor);            // called once after "setDelay" seconds
  } 
  else
  {  
    digitalWrite(9, HIGH);
    Serial.flush();
    digitalWrite(9, LOW);
    servoClose(); // will stop the door
    Alarm.delay(500);
    servoOpen(); // open lock to make sure the door is in the right position
    Alarm.delay(500);
    servoClose(); // lock the door
    isTimerActive = false;
    doorIsOpen = false;
  }
}

/***********************************************************************************************/
void servoOpen()
{
  for(; servopos < 60; servopos += 1)  // goes from 0 degrees to 180 degrees 
  {                                    // in steps of 1 degree 
    myservo.write(servopos);           // tell servo to go to position in variable 'pos' 
    Alarm.delay(3);                    // waits 15ms for the servo to reach the position 
  }
}

/***********************************************************************************************/
void servoClose()
{
  for(; servopos>=20; servopos-=1) // goes from 180 degrees to 0 degrees 
  {                                
    myservo.write(servopos);       // tell servo to go to position in variable 'pos' 
    Alarm.delay(3);                // waits 15ms for the servo to reach the position 
  }    
}

/***********************************************************************************************/
// Controls LED's for Normal mode, Blue on, all others off
void normalModeOn()
{
  switch (lockMode)
  {
  case IN_OUT:
    setColor(ledRGBOne, GREEN);  
    break;
  case ONLY_IN:
    setColor(ledRGBOne, BLUE);
    break;
  case LOCKED:
    digitalWrite(9, HIGH);
    Serial.flush();
    digitalWrite(9, LOW);
    setColor(ledRGBOne, RED);
    break;
  }
}

/***********************************************************************************************/
// Controls LED's for program mode, cycles through RGB
void programModeOn()
{
  setColor(ledRGBOne, RED);
  Alarm.delay(200);
  setColor(ledRGBOne, BLUE);
  Alarm.delay(200);
}

/***********************************************************************************************/
// Flashes the green LED 3 times to indicate a successful write to EEPROM
void successWrite()
{
  setColor(ledRGBOne, BLACK);
  Alarm.delay(300);
  setColor(ledRGBOne, GREEN);
  Alarm.delay(300);
  setColor(ledRGBOne, BLACK); 
  Alarm.delay(300);
  setColor(ledRGBOne, GREEN);
  Alarm.delay(300);
  setColor(ledRGBOne, BLACK);
  Alarm.delay(300); 
  setColor(ledRGBOne, GREEN);
  Alarm.delay(300);
  setColor(ledRGBOne, BLACK);
  Alarm.delay(500); 
}

/***********************************************************************************************/
// Flashes the red LED 3 times to indicate a failed write to EEPROM
void failedWrite()
{
  setColor(ledRGBOne, RED);
  Alarm.delay(300);
  setColor(ledRGBOne, BLACK);  
  Alarm.delay(300);
  setColor(ledRGBOne, RED);
  Alarm.delay(300);
  setColor(ledRGBOne, BLACK);  
  Alarm.delay(300);
  setColor(ledRGBOne, RED);
  Alarm.delay(300);
  setColor(ledRGBOne, BLACK);  
  Alarm.delay(300);
  setColor(ledRGBOne, RED);
  Alarm.delay(300);
  setColor(ledRGBOne, BLACK);  
}

/* 
 Sets an led to any color   led - a three element array defining the three color pins 
 (led[0] = redPin, led[1] = greenPin, led[2] = bluePin)   
 color - a three element boolean array (color[0] = red value (LOW = on, HIGH = off), 
 color[1] = green value, color[2] =blue value)
 */

void setColor(int* led, boolean* color)
{ 
  for(int i = 0; i < 3; i++)
  {   
    digitalWrite(led[i], color[i]); 
  }
}

/* A version of setColor that allows for using const boolean colors*/
void setColor(int* led, const boolean* color)
{  
  boolean tempColor[] = {
    color[0], color[1], color[2]        };  
  setColor(led, tempColor);
}






