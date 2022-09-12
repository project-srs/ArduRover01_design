/////////////////////////////////////////
// connection 
// Arduino Uno -- gy-us42v2
// VIN -- VCC
// GND -- GND
// A4  -- DT
// A5  -- CR
/////////////////////////////////////////

//Default SDA is Pin5 PORTC for the UNO
#define SCL_PIN 5
#define SCL_PORT PORTC
//Default SCL is Pin4 PORTC for the UNO
#define SDA_PIN 4
#define SDA_PORT PORTC
//Define a timeout of 100 ms
#define I2C_TIMEOUT 100

// connet sonor with I2C
#include <SoftI2CMaster.h>

// -1: invalid
// 1~127: connected address
int connected_device_address = -1;

bool disntace_read = false;

void setup() {
  // Initialize both the serial and I2C bus
  Serial.begin(9600);
  i2c_init();

  Serial.println("");
  Serial.println("");
  Serial.println("###############################");
  Serial.println("Start I2C Sonor address chager.");
  Serial.println("###############################");
  Serial.println("");

  connected_device_address = search_adress();

  Serial.println("");
  Serial.print("detect address [[ ");
  Serial.print(connected_device_address);
  Serial.println(" ]]");
  Serial.println("1~127: set address");
  Serial.println("negative(<0): get distance value");
}

void loop()
{
  if (disntace_read) {
    int address2 = connected_device_address*2;
    boolean error = start_sensor(address2);    //Start the sensor and collect any error codes.
    if (!error) {                 //If you had an error starting the sensor there is little point in reading it as you will get old data.
      int range = read_sensor(address2);   //reading the sensor will return an integer value -- if this value is 0 there was an error
      Serial.println(range);
    } else {
      Serial.print("error ");  
      Serial.println(address2);  
    }
    delay(100);
  } else {
    int in = Serial.parseInt();
    if (in < 0) {
      disntace_read = true;
      Serial.println("start read distance");
    } else if (1 <= in && in <= 127) {
      Serial.print("change address ");
      Serial.print(connected_device_address);
      Serial.print(" -> ");
      Serial.println(in);
      boolean error = change_address(connected_device_address * 2, in * 2);
      if (error) {
        Serial.println("fail");        
      } else {
        Serial.println("success, please power off (reset is not enouth)");
      }
    }
  }
}


/////////////////////////////////////////
// Function: Search address
/////////////////////////////////////////
int search_adress(void) {
  Serial.println("Polling addresses... 1 to 127");
  Serial.println("If no I2C device connected, it takes long time");

  for (byte i = 1; i < 128; i++) {
    int address2 = i * 2;
    boolean error = start_sensor(address2);    //Start the sensor and collect any error codes.
    if (!error) {               //If you had an error starting the sensor there is little point in reading it.
      delay(100);
      int range = read_sensor(address2);   //reading the sensor will return an integer value -- if this value is 0 there was an error
      if (range != 0) {
        Serial.print("Device found at:");
        Serial.print(i);
        Serial.print(" Reported value of:");
        Serial.println(range);
        return i;
      }
    }
  }
  Serial.println("Address Fail.");
  return -1;
}

/////////////////////////////////////////
// Function: get input mode
// 1~127: taraget address(1~127)
// other: other(0)
// no input: no input(-1)
/////////////////////////////////////////
int get_input_mode(void) {
  int input_length = Serial.available();
  if (0 < input_length) {
    int value = 0;
    for (size_t i = 0; i < input_length; i++) {
      char inkey = Serial.read();
      if ('0' <= inkey && inkey <= '9') {
        value += (inkey - '0') * pow(10, (input_length - i));
      }
      else {
        return 0;
      }
    }
    Serial.print( "input is ");
    Serial.print( value );
  }
  return -1;
}


///////////////////////////////////////////////////
// Function: Start a range reading on the sensor //
///////////////////////////////////////////////////
//Uses the I2C library to start a sensor at the given address
//Collects and reports an error bit where: 1 = there was an error or 0 = there was no error.
//INPUTS: byte bit8address = the address of the sensor that we want to command a range reading
//OUPUTS: bit  errorlevel = reports if the function was successful in taking a range reading: 1 = the function
//  had an error, 0 = the function was successful
boolean start_sensor(byte bit8address) {
  boolean errorlevel = 0;
  bit8address = bit8address & B11111110;               //Do a bitwise 'and' operation to force the last bit to be zero -- we are writing to the address.
  errorlevel = !i2c_start(bit8address) | errorlevel;   //Run i2c_start(address) while doing so, collect any errors where 1 = there was an error.
  errorlevel = !i2c_write(81) | errorlevel;            //Send the 'take range reading' command. (notice how the library has error = 0 so I had to use "!" (not) to invert the error)
  i2c_stop();
  return errorlevel;
}



///////////////////////////////////////////////////////////////////////
// Function: Read the range from the sensor at the specified address //
///////////////////////////////////////////////////////////////////////
//Uses the I2C library to read a sensor at the given address
//Collects errors and reports an invalid range of "0" if there was a problem.
//INPUTS: byte  bit8address = the address of the sensor to read from
//OUPUTS: int   range = the distance in cm that the sensor reported; if "0" there was a communication error
int read_sensor(byte bit8address) {
  boolean errorlevel = 0;
  int range = 0;
  byte range_highbyte = 0;
  byte range_lowbyte = 0;
  bit8address = bit8address | B00000001;  //Do a bitwise 'or' operation to force the last bit to be 'one' -- we are reading from the address.
  errorlevel = !i2c_start(bit8address) | errorlevel;
  range_highbyte = i2c_read(0);           //Read a byte and send an ACK (acknowledge)
  range_lowbyte  = i2c_read(1);           //Read a byte and send a NACK to terminate the transmission
  i2c_stop();
  range = (range_highbyte * 256) + range_lowbyte;  //compile the range integer from the two bytes received.
  if (errorlevel) {
    return 0;
  }
  else {
    return range;
  }
}

/////////////////////////////////////////
// Function: Change the sensor address //
/////////////////////////////////////////
//Uses the I2C library to change the address of a sensor at a given address
//Collects and reports an error bit where: 1 = there was an error or 0 = there was no error.
//INPUTS: byte oldaddress = the current address of the sensor that we want to change
//INPUTS: byte newddress  = the address that we want to change the sensor to
//OUPUTS: bit  errorlevel = reports if the function was successful in changing the address: 1 = the function had an
//      error, 0 = the function was successful
boolean change_address(byte oldaddress, byte newaddress) {
  //note that the new address will only work as an even number (odd numbers will round down)
  boolean errorlevel = 0;
  oldaddress = oldaddress & B11111110;  //Do a bitwise 'and' operation to force the last bit to be zero -- we are writing to the address.
  errorlevel = !i2c_start(oldaddress) | errorlevel; //Start communication at the new address and track error codes
  errorlevel = !i2c_write(170) | errorlevel;        //Send the unlock code and track the error codes
  errorlevel = !i2c_write(165) | errorlevel;        //Send the unlock code and track the error codes
  errorlevel = !i2c_write(newaddress) | errorlevel; //Send the new address
  i2c_stop();
  return errorlevel;
}
