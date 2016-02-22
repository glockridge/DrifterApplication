//#include <LiquidCrystal.h>     //Use this library if you are using a liquid crystal display
#include <Adafruit_GPS.h>        //Load the GPS Library. Make sure you have installed the library form the adafruit site
#include <SD.h>                  // Load the SD card library
#include <SPI.h>                 // Load the serial library
#include <Wire.h>                //Load the I2C library
#include <SoftwareSerial.h>      //Load the Software Serial Library. This library in effect gives the arduino additional serial ports
#include <elapsedMillis.h>       //Load the timer library


//SoftwareSerial myserial(8, 7);   //THIS IS FOR UNO//define how the soft serial port is going to work.
SoftwareSerial mySerial(52, 7); //Initialize SoftwareSerial, and tell it you will be connecting through pins (x,y)
//Apparently, the MEGA does not support softwareSerail on pins 8,7 like the UNO, RX on the MEGA is supported on different pins
// I chose 52, because I'm badass like that!
//#define mySerial Serial1         // Use this line if you are using an Arduino Mega
Adafruit_GPS GPS(&mySerial);     //Create GPS object

String NMEA1;                    //We will use this variable to hold our first NMEA sentence
String NMEA2;                    //We will use this variable to hold our second NMEA sentence
char c;                          //Used to read the characters spewing from the GPS module
float degLon;                    //will hold position data in simple degree format
float degWholeLon;               //variable for whole part of position
float degDecLon;                 //variable for the decimal part of degree
float degLat;                    //will hold position data in simple degree format
float degWholeLat;               //variable for whole part of position
float degDecLat;                 //variable for the decimal part of degree

int chipSelect = 10;             //chipSelect pin is set to 10 to keep the SD card reader happy... 
unsigned long timeout;

 

//elapsedMillis timer0;            //Attempting to make headers in logfile
#define interval 1000
boolean timer0Fired;
File logfile;


int cond_address = 100;          //address of the atlas EC microchip on the I2C bus... On the MEGA SCL is pin 21 connects to RX on the chip, SDA is pin 20 and connects to TX on the chip...Make sure to connect 10k resistors from TX and RX on the chip to +5v in addition to pins 20 and 21  
byte code=0;                     //used to hold the I2C response code. 
char ec_data[48];                //we make a 48 byte character array to hold incoming data from the EC circuit. 
byte in_char=0;                  //used as a 1 byte buffer to store in bound bytes from the EC Circuit.   
byte i=0;                        //counter used for ec_data array. 
int time=1400;                   //used to change the delay needed depending on the command sent to the EZO Class EC Circuit. 

char *ec;                        //char pointer used in string parsing. 
char *tds;                       //char pointer used in string parsing.
char *sal;                       //char pointer used in string parsing.
char *sg;                        //char pointer used in string parsing.

float ec_float;                  //float var used to hold the float value of the conductivity. 
float tds_float;                 //float var used to hold the float value of the TDS.
float sal_float;                 //float var used to hold the float value of the salinity.
float sg_float;                  //float var used to hold the float value of the specific gravity.

float tempC;                     //float var used to hold the temperatre reading.
float temp_data;


void setup()  
{
  Serial.begin(9600);
  Wire.begin();
  
  pinMode(10, OUTPUT);  //setting the digital pin 10 as an output to keep the SD card reader happy
  pinMode(50, OUTPUT);  //setting the temperatre probe voltage supply as an output.
  
  timeout = millis();   //  This is keeping track of the elapsed time to use for logging
  
  if (!SD.begin(chipSelect, 11, 12,13)) {                   //This is helpful for debuging, I recommend leaving this in the code...
    Serial.println("Card init. failed!");
    }
  //if                                                     //Uncomment these lines to create a new file each time the device cycles power.
 // char filename[15];                             
//strcpy(filename, "1Drift.csv");  
//  for (uint8_t i = 0; i < 100; i++) {
//    filename[6] = '0' + i/10;
//    filename[7] = '0' + i%10;
//    if (! SD.exists(filename)) {                // create if does not exist, do not open existing, write, sync after write
//    break;
//  }}

 // logfile = SD.open(filename, FILE_WRITE);     //open a file and label the columns
 
 char filename[15];                             
strcpy(filename, "1Drift.csv");  //CHANGE THE DRIFTER NUMBER HERE, DO NOT CHANGE THE FORMAT, THE PROGRAM GETS PISSY IF  YOU START ALTERING FILNAMES!!


 if (! SD.exists(filename)){  //if this filename does not already exist, then add headers, else, don't add headers, open the file and append data.
      Serial.println ("adding headers");
      
     logfile= SD.open(filename, FILE_WRITE);  //This prevents multiple files from being written, instead, it creates a single file and writes to it over and over.
          logfile.print("Temperature C");
          logfile.print(",");
          logfile.print("Latitude");
          logfile.print(",");
          logfile.print("Longitude");
          logfile.print(",");
          logfile.print("Speed (Knots)");
          logfile.print(",");
          logfile.print("Satellites");
          logfile.print(",");
          logfile.print("Date");
          logfile.print(",");
          logfile.print("Time");
          logfile.print(",");
         
          logfile.print("Electrical Conductivity");
          logfile.print(",");
          logfile.print("Total Dissolved Solids");
          logfile.print(",");
          logfile.print("Salinity");
          logfile.print(",");
          logfile.print("Specific Gravity");
          logfile.println(","); 
          }
else{
logfile= SD.open(filename, FILE_WRITE);
    }  
 
 
  GPS.begin(9600);                            //Turn GPS on at baud rate of 9600
  GPS.sendCommand("$PGCMD,33,0*6D");          // Turn Off GPS Antenna Update
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //Tell GPS we want only $GPRMC and $GPGGA NMEA sentences
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);  // 1 Hz update rate

}//End of the void setup function!


void loop () 
           {
            readGPS ();                        //This is a function we define below which reads two NMEA sentences from GPS
}  //end of void loop function!!!



void readGPS() 
             {  
             clearGPS();                      //this is a function that is defined below... basically clearing the serial buff before trying to read again.
        while(!GPS.newNMEAreceived()){        //Loop until you have a good NMEA sentence
             c=GPS.read();
             }
             GPS.parse(GPS.lastNMEA());       //Parse the last good NMEA sentence
             NMEA1=GPS.lastNMEA();            //THIS IS NMEA 1
         while(!GPS.newNMEAreceived()){       //Loop until you have a good NMEA sentence
             c=GPS.read();
             }
             GPS.parse(GPS.lastNMEA());       //Parse that last good NMEA sentence
             NMEA2=GPS.lastNMEA();            //THIS IS NMEA 2

         Serial.println(NMEA1);              //Serial print the two NMEA sentenses
         Serial.println(NMEA2);
         Serial.println("");
 
         GPS.read();                         //Read the GPS again and this time convert as described below...
 
                  degWholeLon=float(int(GPS.longitude/100));              // Converting lat/lon to google earth friendly format
                  degDecLon=(GPS.longitude-degWholeLon*100)/60;
                  degLon=degWholeLon+degDecLon;
              if(GPS.lon=='W') {
                  degLon=(-1)*degLon;
                  }
                  
                  degWholeLat=float(int(GPS.latitude/100));
                  degDecLat=(GPS.latitude-degWholeLat*100)/60;
                  degLat=degWholeLat+degDecLat;
              if (GPS.lat=='S') {
                  degLat=(-1)*degLat; 
                  }
  
        logGPS();                             //This function is defined below and includes a reading from the atlas EC microchip
  
        }  //End of void readGPS function

                   
void clearGPS() {
                {                            //Since between GPS reads, we still have data streaming in, we need to clear
        while(!GPS.newNMEAreceived())        // the old data by reading a few sentences, and discarding these
            c=GPS.read();
        }
            GPS.parse(GPS.lastNMEA());
        while(!GPS.newNMEAreceived()) {
            c=GPS.read();
        }
            GPS.parse(GPS.lastNMEA());
            
}//End void clearGPS function

  
void logGPS()                                //This function loggs the readings from void readGPS... also creates headers and reads/logs conductivity data
  {
      if (millis() > timeout)                //This is keeping track of the time between logged samples!!!
        { 
        timeout += 10000;                    //INTERVAL BETWEEN LOGGED SAMPLES in milliseconds!!!!  
   //Serial.println("Log GPS");              //Use this line for debuging
   
//          if (!timer0Fired)                  //&& (timer0 > interval)) //This is creating headers... 'if no time has passed, then add the following to the SD card.../
//          {
//          timer0Fired = true;                //don't execute this again command (boolean)
//           
//     Serial.println ("adding headers");
//     
//          logfile.print("Temperature C");
//          logfile.print(",");
//          logfile.print("Latitude");
//          logfile.print(",");
//          logfile.print("Longitude");
//          logfile.print(",");
//          logfile.print("Speed (Knots)");
//          logfile.print(",");
//          logfile.print("Satellites");
//          logfile.print(",");
//          logfile.print("Date");
//          logfile.print(",");
//          logfile.print("Time");
//          logfile.print(",");
//         
//          logfile.print("Electrical Conductivity");
//          logfile.print(",");
//          logfile.print("Total Dissolved Solids");
//          logfile.print(",");
//          logfile.print("Salinity");
//          logfile.print(",");
//          logfile.print("Specific Gravity");
//          logfile.println(",");     
//          }  
          
     readEC();                                // this function is defined below... reads the values from the atlas EC microchip
 //    temp();
     
          logfile.print (degLat, 6);         // Printing values to the SD card.
          logfile.print (",");
          logfile.print (degLon, 6);
          logfile.print (",");
          logfile.print (GPS.speed);
          logfile.print (",");
          logfile.print (GPS.satellites);
          logfile.print (",");
          logfile.print(GPS.day, DEC); logfile.print('/');
          logfile.print(GPS.month, DEC); logfile.print("/20");
          logfile.print(GPS.year, DEC); 
      logfile.print(",");    
          logfile.print (GPS.hour);
          logfile.print (":");
          logfile.print (GPS.minute);
          logfile.print (":");
          logfile.print (GPS.seconds);
          logfile.print (",");
         
          logfile.println(ec_data);            //printing data from the atlas EC microchip!!
          logfile.flush();                     //this command helps protect the data on the SD card if someone removes the card w/out cutting power first!
  
      // Serial.println(ec_data);              //use this line for debuging
      // Serial.println("data logged");        //use this line for debuging
 
         }                                    //END OF THE "IF" STATEMENT THAT IS CONTROLLED BY THE SAMPLING INTERVAL!!!!!
  }// end of void logGPS function
         
         

void readEC() {
  float v_out;
float tempC;
char tempData[20];  //holding the value of tempC as a character.
digitalWrite(A8, LOW);  //White wire from the temp sensor

digitalWrite(50, HIGH);  //Red wire from the temp sensor
delay(2);
v_out=analogRead(A8);
digitalWrite(50, LOW);

v_out*=0.0048;

v_out*=1000;
tempC=0.0512*v_out-18.4128;  // CALIBRATE TEMP BY CHANGING END INTERGER HERE...  factory reset calibration 20.5128

     
      Serial.println(tempC);
      logfile.print(tempC);
      logfile.print(",");
      dtostrf(tempC, 4, 3, tempData);  //This is the magic line of code that converts the float TempC into a char, that can then be sent to the EC chip via I2C.  4 is the number of width, and three is the precision or number of decimal places.  This was really fucking hard to figure out... but it's my bitch now.
     // tempData=tempC;
  
           Wire.beginTransmission(cond_address);  // Begin communication via I2C with the atlas EC microchip
           delay(15);            // This delay is necessary after initiating I2C communication  
      Wire.write("T,");
     Wire.write(tempData);  //SENDING TEMP C IS NOT A VALUE THAT WORKS>>> YOU NEED TO SEND ACTUAL READING
      delay(300);
       Wire.requestFrom(cond_address,6,1);
       while(Wire.available()){
         char c = Wire.read();
         Serial.print(c);}
         
      Wire.endTransmission(); 
 
//delay(200);


           Wire.beginTransmission(cond_address);  // Begin communication via I2C with the atlas EC microchip
           delay(15); 
           Wire.write('r');                       //Sending the atlas EC microchip a command requesting a single reading
           Wire.endTransmission();                //end I2C transmission and wait for response from atlas EC microchip
           delay(1400);                           //This delay is the amount of time required by the atlas EC microchip to take a single EC reading  
           Wire.requestFrom(cond_address,48,1);   //cond_address is definded at the begining of the program as 100.  48 is the number of bytes that are requested, actual byte count will be smaller

       while(Wire.available()) {
           in_char = Wire.read();                 //code speak for breaking the four readings that are recieved from the atlas EC microchip into a form that can be understood via serial/SD 
           ec_data[i]=in_char;
           i+=1;
       if (in_char==0) {
           i=0;
           Wire.endTransmission();
           break; 
           }
           Wire.beginTransmission(cond_address);
           delay(15);
           Wire.write("Sleep");
           delay(300);
           Wire.endTransmission();
           }
       Serial.println(ec_data);    
       // Serial.println("Log EC");               //use this line for debuging
  }//end of the void readEC function
  
//  void temp() {
//     float v_out;
//float temp_data;
//digitalWrite(A0, LOW);  //White wire from the temp sensor
//
//digitalWrite(50, HIGH);  //Red wire from the temp sensor
//delay(2);
//v_out=analogRead(0);
//digitalWrite(50, LOW);
//
//v_out*=0.0048;
//
//v_out*=1000;
//temp_data=0.0512*v_out-20.5128;

//} 
           
  /* 
  Add a sleep command to the atlas EC microchip after the reading is transmitted so that it doesn't use battery.
  Add a temperature compensation value, need to debug the thermocouple readings first.
 */
 



