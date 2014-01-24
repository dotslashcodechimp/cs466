
/*******************************************************************************
 Programmer: Jesse Waite
 Class: EE 234
 Programming Assignment: Lab 2.
 Date: January 29, 2013

  Notes:

  -attempting (and failing) to read the accelerometer (for many different invalid dev addresses) returns 0xc3: is this an i2c error code or pre-initialized master read-register value?

  -UART use Andy's legacy code. Likely need to figure out why my code was failing (configureUART1() and EnableUART() calls)
  -How can we get I2C to recover after initial failure?  I program the board but must restart it for I2C to work correctly, else
  the bus is busy and cannot be used due to bus collisions etc.  I tried disabling then enabling the bus after failure, but this
  did not work.  Need to figure out something short of powering off the board that will allow I2C recovery.
  -Continue with attempt to get board to receive data, with and without interrupts
  -Most .h notes indicate that peripheral interrupts (OC, TMR, etc) must be configured BEFORE configuring interrupts

  -Andy stuff: syntax for writing assembly in C, stepper motors, i2c pull up resistors???, why does reconnecting UART(ie, hyperterminal connection)
   reset the program to beginning?, differences between microcontrollers (duino, atmel, etc),
   Does adxl345 output high Z value just because of gravity?  This doesn't actually make sense: if z is inverted, it should decrease from 511
   Does adxl345 update registers between reads, or are registers usually latched/locked?

   -We can prove spi by implementing the adxl345; may be useful checkpoint for wifi device

   5V ping sensor appears not to work with the 3.3V cerebot:
     -workarounds
     -popularity of one voltage vs. another


  -IMU mounting considerations: From adxl345 manual "Locating the accelerometer
    near a hard mounting point ensures that any PCB vibration at
    the accelerometer is above the accelerometer’s mechanical sensor
    resonant frequency and, therefore, effectively invisible to the
    accelerometer."
    
  -IMPLEMENT AUTOINCREMENT!  It is possible for at least the adxl345 and the magnetometer.
     -adxl345 requires fifo mode and 8 i2c-clk delay between reads
     -gyro requires fifo configuration (see manual "read_burst")
     -magnetometer: supports auoincrement by default

  -precise, assembly-based delay function
  
  -ExtINT prototypes are in PORTS.h

  SDA, SCL are bidirectional
  I2C gyro 0-100kHz range

  Gyro:  110100xb   "The SDO pin can be used to modify the least significant bit (LSb) of the device address.
            If the SDO pin is connected to the voltage supply, LSb is ?1? (address 1101001b). Otherwise,
            if the SDO pin is connected to ground, the LSb value is ?0? (address 1101000b)." (refman)
  ADX345 (acc) 7-bit address is 0x1D -> becomes 0x3A for a write and 0x3B for a read
    0x1D = 11101  ->  0x3A = 111010(w)  0x3B = 111011(r)      --> use 1110111
  BMP085  addr: 111011(1/0)(r/w)                       --> use 1110111
  Magneto addr: 0x1E (7bit slave)   0x3D (read address)    0x3C (write addr)

  64 byte error string format shall be: "Error [src function name]: [description]"
    Example: "ERROR i2cGets(): i2cGetc returned 0x1234"

  linux lib locs:
  /opt/microchip/xc32/v1.30/pic32-libs/peripheral


 ******************************************************************************/

#include <peripheral/ports.h>    // PORTSetPinsDigitalIn (), PORTSetPinsDigitalOut (), PORTRead (), PORTWrite (), BIT_XX, IOPORT_X
#include <peripheral/uart.h>    // UART lib:
#include <peripheral/timer.h>    // Timer lib:
#include <peripheral/outcompare.h>  // Output Compare lib:
#include <peripheral/int.h>      // Interrupt lib:
#include <peripheral/incap.h>    // Input capture lib:
#include <peripheral/system.h>    // Set up the system and perihperal clocks for best performance
#include <proc/p32mx460f512l.h>   // Vector table constants
#include <sys/attribs.h>       // for ISR...???
#include <peripheral/i2c.h>      // I2C functions for IMU
#include <peripheral/spi.h>      // spi functions for IMU
#include <stdlib.h>
// SYSCLK = 80 MHz (8 MHz Crystal/ FPLLIDIV * FPLLMUL / FPLLODIV)
// PBCLK = 40 MHz
// Primary Osc w/PLL (XT+,HS+,EC+PLL)
// WDT OFF
// Other options are don't care

/* Oscillator Settings
*/
#pragma config FNOSC = PRIPLL // Oscillator selection
#pragma config POSCMOD = EC // Primary oscillator mode
#pragma config FPLLIDIV = DIV_2 // PLL input divider
#pragma config FPLLMUL = MUL_20 // PLL multiplier
#pragma config FPLLODIV = DIV_1 // PLL output divider
#pragma config FPBDIV = DIV_2 // Peripheral bus clock divider
#pragma config FSOSCEN = OFF // Secondary oscillator enable

#define SYSTEM_CLOCK        80000000
#define PERIPHERAL_CLOCK    40000000
#define I2C_CLOCK_FREQ      100000
#define DESIRED_DATA_RATE   57600
#define U1_INT_PRI          6
#define PR_BASE             256
#define TINY_DLY            128
#define VERYSHORT_DLY       4096
#define SHORT_DLY           8192
#define MEDIUM_DLY          32768
#define LONG_DLY            65000
#define INT_DISABLE         0
#define DC_ZeroPct      0
#define DC_50Pct      128
#define DC_60Pct      154
#define DC_65Pct      166
#define DC_70Pct      179
#define DC_75Pct      192
#define DC_80Pct      205
#define DC_90Pct      230
#define I2C_CONFIG_FLAGS  0    //no I2C config bits seem to apply to normal usage

/*******************************************************************************************************/
/*
   Device variables for IMU GY-80 9DOF Arduino sensor. 
   Compiler warning: addresses must be sent as bytes: using #define does not guarantee byte size.
   
*/
#define L3G4200D_ADDR    0b1101001  //alternate address: 0b1101000
#define L3G4200D_ADDR_W  0b11010010  //110100xb Two possible address permutations: LSB is one or zero depending on SDO pin to allow two Gyros on same I2C bus
#define L3G4200D_ADDR_R  0b11010011

#define L3G4200D_X_L      0x28
#define L3G4200D_X_H      0x29
#define L3G4200D_Y_L      0x2A
#define L3G4200D_Y_H      0x2B
#define L3G4200D_Z_H      0x2C
#define L3G4200D_Z_L      0x2D
#define L3G4200D_TEMP     0x26
#define L3G4200D_FIFO_CTL 0x2E
#define L3G4200D_CTL_REG1 0x20
#define L3G4200D_CTL_REG2 0x21
#define L3G4200D_CTL_REG3 0x22
#define L3G4200D_CTL_REG4 0x23
#define L3G4200D_CTL_REG5 0x24
#define L3G4200D_STATUS   0x27

#define L3G4200D_ID_ADDR 0x0F
#define L3G4200D_ID_VAL  0xD3

#define BMP085_ADDR   0x77
#define BMP085_ADDR_W 0bEE //oxEE write
#define BMP085_ADDR_R 0bEF //oxEF read
#define BMP085_ID_ADDR 0xD0
#define BMP085_ID_VAL 0x55
#define BMP085_GET_TEMP 0x2E
#define BMP085_GET_PRESS 0x34
#define BMP085_CTRL   0xF4
#define BMP085_DO1    0xF6  //Data output regs 1 and 2: pressure and temperature are output from these.
#define BMP085_DO2    0xF7  //The sequence is start/select measurement of temp or press, then read the vals here.
#define BMP085_AC1_H  0xAA  //ERROR OUT if reading any of these calib. reg's returns 0x0 or 0xFF
#define BMP085_AC1_L  0xAB  
#define BMP085_AC2_H  0xAC  
#define BMP085_AC2_L  0xAD  
#define BMP085_AC3_H  0xAE  
#define BMP085_AC3_L  0xAF  
#define BMP085_AC4_H  0xB0  
#define BMP085_AC4_L  0xB1  
#define BMP085_AC5_H  0xB2  
#define BMP085_AC5_L  0xB3  //ERROR OUT if reading any of these calib. reg's returns 0x0 or 0xFF
#define BMP085_AC6_H  0xB4  
#define BMP085_AC6_L  0xB5  
#define BMP085_B1_H   0xB6 
#define BMP085_B1_L   0xB7 
#define BMP085_B2_H   0xB8 
#define BMP085_B2_L   0xB9 
#define BMP085_MB_H   0xBA 
#define BMP085_MB_L   0xBB  //ERROR OUT if reading any of these calib. reg's returns 0x0 or 0xFF
#define BMP085_MC_H   0xBC
#define BMP085_MC_L   0xBD     
#define BMP085_MD_H   0xBE 
#define BMP085_MD_L   0xBF     


#define HMC5883L_ADDR   0x1E  //0b0011110
#define HMC5883L_ADDR_R 0x3D
#define HMC5883L_ADDR_W 0x3C
#define HMC5883L_CFIG_A  0x0
#define HMC5883L_CFIG_B  0x1
#define HMC5883L_MODE  0x2
#define HMC5883L_X_H  0x3  //data vals are signed, in range 0xF800 to 0x07FF
#define HMC5883L_X_L  0x4
#define HMC5883L_Y_H  0x5
#define HMC5883L_Y_L  0x6
#define HMC5883L_Z_H  0x7
#define HMC5883L_Z_L  0x8
#define HMC5883L_STATUS  0x9
#define HMC5883L_ID_ADDR  0xA
#define HMC5883L_ID_VAL  0x48

// Accelerometer: The output data is twos complement, with DATAx0 as the
// least significant byte and DATAx1 as the most significant byte, where x represent X, Y, or Z.
//search adxl345 pdf for this register's usage. Device is in standby on start measurement enabled by writing bit D3 (BIT_3, starting from 0)
//which should be done only after dev is configured.
#define ADXL345_POWER_CTL  0x2D  
#define ADXL345_PCTL_AUTOSLEEP_BIT  4
#define ADXL345_ADDR_W     0xA6 // 0b10100110    0x3A, 0b00111010 alternate addr
#define ADXL345_ADDR_R     0xA7 // 0b10100111    0x3B, 0b00111011 alternate addr
#define ADXL345_ADDR       0x53
#define ADXL345_ID_ADDR    0x0  // yup, zero
#define ADXL345_ID_VAL     0xE5
#define ADXL345_DATA_FRMT  0x31
#define ADXL345_X_L        0x32
#define ADXL345_X_H        0x33
#define ADXL345_Y_L        0x34
#define ADXL345_Y_H        0x35
#define ADXL345_Z_L        0x36
#define ADXL345_Z_H        0x37
/******************************************* end I2C IMU devices ********************************************************/

/******************************************* HC-SR04 defs and stuff ********************************************************/


/* online example of precision us. delay necessary for ping(), but the math doesnt add up...
// 32HMz -> Nop() == 1/8us
#define WAIT10US() Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();\
                   Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();\
                   Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();\
                   Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();\
                   Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();\
                   Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();\
                   Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();\
                   Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();
*/

/******************************************* HC-SR04 end ********************************************************/

//compact struct for communicating with I2C devices
typedef struct i2cDev{
  unsigned char devAddr;
  unsigned char subAddr;
} i2cDevice;

// IMU data packet
typedef struct imuVector{
  short int accX;  //accelerometer vals
  short int accY;
  short int accZ;
  short int gyroRoll; //gyroscope vals
  short int gyroPitch;
  short int gyroYaw;
  short int gyroTemp;
  short int magX;  //magnetometer vals
  short int magY;
  short int magZ;
  short int bmpPa; //barometer / temp vals
  short int bmpTemp;
}imuVec;

//all the calibration constants needed for reading temp and pressure from the BMP085 (see manual)
typedef struct bmp085Vector{
    short int ac1;
    short int ac2;
    short int ac3;
    short int ac4;
    short int ac5;
    short int ac6;
    short int b1;
    short int b2;
    short int mb;
    short int mc;
    short int md;
    short int oss;
    short int rawTemp;
    short int rawPress;
    short int trueTemp;
    short int truePress;
}bmp085Vec;

void configurePortIO(void);
void configOCModule2(void);
void configOCModule3(void);
void configureUART1(void);
void enableUART1(void);
void configureTimer2(int Period);
void configureTimer4(int Period);
void configureInterrupts(void);
void configureInputCaptureMods(void);
unsigned int ping(void);

void i2cConfigModule1(void);
void putI2CResult(I2C_RESULT resultVal);
BOOL I2CTransmitByte( I2C_MODULE i2c_mod, unsigned char data );
BOOL i2cStartTransfer( BOOL restart );
void i2cStopTransfer( void );
BOOL I2CReceiveByte( unsigned char* data );
void i2cSendAck(I2C_MODULE i2c_mod, BOOL ack);

void delayMS(int ms);
void delay(unsigned int ms);
void writeOC2(unsigned int newOC2RValue, unsigned int newOC2RSValue);
void writeOC3(unsigned int newOC3RValue, unsigned int newOC3RSValue);
void setMotorsForward(void);
void setMotorsReverse(void);
void clearStr(unsigned char str[], int len);

BOOL i2cGetc(struct i2cDev* dev, unsigned char *rxData); //my functions
BOOL i2cPutc(struct i2cDev* dev, unsigned char* txData);
BOOL i2cWriteBit(struct i2cDev* dev, unsigned char bitNum, BOOL enable);
BOOL i2cPuts(struct i2cDev* dev, unsigned char txData[], int n);
BOOL i2cGets(struct i2cDev* dev, unsigned char rxData[], int nBytes);
BOOL i2cGetsManualIncrement(struct i2cDev* dev, unsigned char rxData[], unsigned int nRegs);

BOOL ADXL345TestConnection(void);
BOOL L3G4200DTestConnection(void);
BOOL HMC5883LTestConnection(void);

BOOL HMC5883LGetMag(unsigned char magBytes[]);
BOOL L3G4200DGetGyro(unsigned char gyroBytes[]);

int readIMU(struct imuVector* imuVec, struct bmp085Vector* bmpVec);
int InitializeIMU(struct bmp085Vector *barVec);
BOOL ADXL345GetAccel(unsigned char accBytes[]);
void ADXL345SelfTest(void);

BOOL InitializeADXL345(void);
BOOL InitializeHMC5883L(void);
BOOL InitializeL3G4200D(void);
BOOL InitializeBMP085(struct bmp085Vector *barVec);
BOOL BMP085GetCalCoefficients(struct bmp085Vector *barVec);
BOOL BMP085SetCalCoefficients(void);
BOOL BMP085TestConnection(void);
void BMP085CalcTempAndPressure(struct bmp085Vector *barVec);
BOOL BMP085GetTempAndPressure(struct bmp085Vector *bmpVec);

// spi and pmod wifi methods
void spiInitialize(void);
char spiTransfer (char c);
void spiPuts (char *s);
char spiPutc (char c);
char spiGetc(void);
void MRF24WB0MAinitialize();


void clearLEDs(void);
void output_led (short index);
void winSequence(void);
void loseSequence(void);
void setupUART1 (unsigned int pb_clock); //Andy version of UART setup
void sigError(void);

/*
  This demonstrates a working I2C communications read sequence.

>>> http://www.i2cdevlib.com/ <<< (all arduino, but methods can be rewritten)

 To do: check top of this file as well.
  -implement device self-tests and calibration
  -implement device sensitivity/correction (requires physical testing)
  -multibyte read
  -fix my uart code, find out why it fails.
  -spi / wifi
  -fix bluetooth
  -exercise other peripherals: motors, bluetooth, re-org code into include files

  -sending a pointer output parameter to i2cPutc is non-sensical; convert this parameter to a simple unsigned char
  
  Sometimes reading after the first device fails (infinite loop at i2cStartTransfer().  I am now printing the i2c status over uart:
    -when failure occurs, reading the status register gives 0x408 (0x400 === I2C_ARBITRATION_LOSS, 0x008 == I2C_START)
  -when successful, status is 0x10  (0x10 == I2C_STOP)

  -helpful: i2cgetstatus() appears to return the status after the last possible transaction. It returned 0x8008 when i was sending NACK NACK STOP before correction.
      After correction (sending only one NACK), getStatus returned 0x10 (STOP), which was the desired state, and reflected the last call to Stop().
 
 
 */

int main (void)
{
  unsigned int RX_data = 0;
  unsigned int j = 0;

  struct i2cDev dev; // = {0b1101001, 0b0001111};
    dev.devAddr = 0b1101001;
    dev.subAddr = 0b0001111;
  struct bmp085Vector bmpVec = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  struct imuVector imuVec =  {0,0,0,0,0,0,0,0,0,0,0,0};
  unsigned char testByte = NULL;

  //I2C examples vars
  unsigned char       i2cData[10] = {'\0'};
  I2C_7_BIT_ADDRESS   SlaveAddress;
  int                 Index = 0;
  int         i = 0;
  int                 DataSz = 0;
  unsigned int    actualClock = 0;
  BOOL                Acknowledged = FALSE;
  int                Success = 0b1111;
  unsigned char       i2cbyte = '\0';
  int test = 0;
  int XL = 0, YL = 0, ZL = 0;
  int XH = 0, YH = 0, ZH = 0;
  int X = 0, Y = 0, Z = 0;
  I2C_STATUS stat;

  unsigned char TxByte = '?';
  unsigned char RxByte = '?';
  I2C_RESULT result;  //I2C enum: 0 = success, ERROR = 1, MASTERBUS_COLL = 2, I2C_RECEIVEOVERFLOW = 3
  I2C_STATUS  status;
  char magStr[64] = {'\0'};
  char accStr[64] = {'\0'};
  char gyroStr[64] = {'\0'};
  char barStr[64] = {'\0'};
  
  configurePortIO();
  //configOCModule2();
  //configOCModule3();
  //configureUART1();
  //enableUART1();
  setupUART1(PERIPHERAL_CLOCK);
  configureTimer2(PR_BASE);
  //configureTimer4(32); //for ping sensor
  //spiInitialize();
  //MRF24WB0MAinitialize();  //does nothing; it would be nice to check the dev id or something, as with the i2c devs

  //configureInterrupts();
  //configureInputCaptureMods();
  i2cConfigModule1();
  InitializeIMU( &bmpVec );

  putsUART1("\033[2J"); //clear hyperterminal screen
  putsUART1("Beginning test\n\r"); //clear hyperterminal screen
  sigError();

  dev.devAddr = ADXL345_ADDR;
  dev.subAddr = ADXL345_X_L;
  TxByte = 0x0;
  
  //putsUART1("attempt\n\r");
  
  while(1){
    
    //PING test code for HC-SR04
    //test = ping();
    //delay(SHORT_DLY);
    
    //spiPutc('$');    

    readIMU(&imuVec, &bmpVec);
    //putsUART1("here1\n\r");
    
    //this output is just for debugging. Data packet is defined by imuvector, not char strings.
    sprintf(accStr,"Acc(X,Y,Z)  %40X %40X %40X\n\r",imuVec.accX,imuVec.accY,imuVec.accZ);
    putsUART1(accStr);
    
    //sprintf(gyroStr,"Gyro(X,Y,Z,tmp) %6d %6d %6d %4d\n\r",imuVec.gyroRoll,imuVec.gyroPitch,imuVec.gyroYaw,imuVec.gyroTemp);
    //putsUART1(gyroStr);
    
    //sprintf(magStr,"Mag(X,Y,Z)  %6d %6d %6d\n\r",imuVec.magX,imuVec.magY,imuVec.magZ);
    //putsUART1(magStr);
    
    //sprintf(barStr,"Bar(tmp,Pa) %6d %6d\n\r",imuVec.bmpTemp,imuVec.bmpPa);
    //putsUART1(barStr);
    
    /*
    for(i = 0; i < 4; i++){
      delay(SHORT_DLY >> 1);
    }
    */
    
    //putsUART1("\033[2J"); //clear hyperterminal screen
  }

  return 0;
}

/*
  Fail proof, but devices must be initialized before calling this.
  There's actually not much reason to format the unsigned chars to ints, since master
  could do this.  This version is just for debugging.
  
  Returns: an integer bit string representing devices failures.  0b0000 indicates all succeeded.
    0b0001 = 1 = ADXL345 FAIL
    0b0010 = 2 = L3G4200D FAIL
    0b0100 = 4 = HMC5883L FAIL
    0b1000 = 8 = BMP085 FAIL
*/
int readIMU(struct imuVector* imuVec, struct bmp085Vector* bmpVec)
{
  int i = 0, result = 0;
  int rawVals[6];
  unsigned char dataBytes[20] = {'\0'};
  unsigned char* iter = dataBytes;
 
  //putsUART1("debugA\n\r");
 
  //get all the imu Vals
  if( ADXL345TestConnection() ){
    ADXL345GetAccel(iter);
  }
  else{
    result |= 1;
  }
  iter += 6;
  
  //putsUART1("debugB\n\r");
  if( L3G4200DTestConnection() ){
    L3G4200DGetGyro(iter);
  }
  else{
    result |= 2;
  }
  iter += 7;  //seventh byte is for temp reading (one byte size)
  
  if( HMC5883LTestConnection() ){
    HMC5883LGetMag(iter);
  }
  else{
    result |= 4;
  }
  iter += 6;  //seventh byte is for temp reading (one byte size)
  
  if( BMP085TestConnection() ){
    BMP085GetTempAndPressure(bmpVec);
  }
  else{
    result |= 8;
  }
  
  imuVec->accX = 0x0000FFFF & (((unsigned int)dataBytes[1] << 8) | (dataBytes[0]));  //accelerometer vals
  imuVec->accY = 0x0000FFFF & (((unsigned int)dataBytes[3] << 8) | (dataBytes[2]));
  imuVec->accZ = 0x0000FFFF & (((unsigned int)dataBytes[5] << 8) | (dataBytes[4]));
  imuVec->gyroRoll =  0x0000FFFF & (((unsigned int)dataBytes[7] << 8) | (dataBytes[6])); //gyroscope vals
  imuVec->gyroPitch = 0x0000FFFF & (((unsigned int)dataBytes[9] << 8) | (dataBytes[8]));
  imuVec->gyroYaw = 0x0000FFFF & (((unsigned int)dataBytes[11] << 8) | (dataBytes[10]));
  imuVec->gyroTemp = 0x000000FF & dataBytes[12];  //the solo additional temp val from the gyro
  imuVec->magX = 0x0000FFFF & (((unsigned int)dataBytes[13] << 8) | (dataBytes[14]));  //magnetometer vals: notice that MSB/LSB is reversed for the HMC5883L
  imuVec->magY = 0x0000FFFF & (((unsigned int)dataBytes[15] << 8) | (dataBytes[16]));
  imuVec->magZ = 0x0000FFFF & (((unsigned int)dataBytes[17] << 8) | (dataBytes[18]));
  imuVec->bmpPa =  bmpVec->truePress; //barometer / temp vals
  imuVec->bmpTemp =  bmpVec->trueTemp;
  
  return result;
}

/*
  bmpVec param is required to store the cal data of the BMP085 on start up.

  Returns 0 for Success. Non-zero returns:
    0b0001 = 1 = ADXL345 FAIL
    0b0010 = 2 = L3G4200D FAIL
    0b0100 = 4 = HMC5883L FAIL
    0b1000 = 8 = BMP085 FAIL
    
*/
int InitializeIMU(struct bmp085Vector *bmpVec)
{
  int status = 0;

 if( !InitializeADXL345() ){
    //putsUART1("ADXL345 Failed to initialize\n\r");
    status |= 0b0001;
  }
  //putsUART1("success 1\n\r");
  if( !InitializeL3G4200D() ){
    //putsUART1("L3G4200D Failed to initialize\n\r");
    status |= 0b0010;
  }
  //putsUART1("success 2\n\r");
  if( !InitializeHMC5883L() ){
    //putsUART1("HMC5883L Failed to initialize\n\r");
    status |= 0b0100;
  }
  //putsUART1("success 3\n\r");
  if( !InitializeBMP085(bmpVec) ){
    //putsUART1("BMP085 Failed to initialize\n\r");
    status |= 0b1000;
  }

  if(status != 0){
    putsUART1("IMU failed to initialize completely\n\r");
  }
  
  return status;  
}


/////////////////////////////////////////////////////////////////////////////
/////////////////BMP085 code/////////////////////////////////////////////////
/*
 The BMP085 includes a temp and atmospheric pressure sensor, but the
 raw temp and pressure outputs must be fed to calculations which also require
 factory-set calibration values.  Here we test the connection by verifying a read
 of the BMP085 id reg, then verify reading the calibration coefficients so that we have
 them in memory.
*/
BOOL InitializeBMP085(struct bmp085Vector *barVec)
{

  if( !BMP085TestConnection() ){
    //putsUART1("ERROR BMP085 init() failed\n\r");  
    return FALSE;
  }
  
  return BMP085GetCalCoefficients(barVec);
}

// Must return 0x55 for successful connection
BOOL BMP085TestConnection(void)
{
  unsigned char testByte = 0;
  //char debugStr[64] = {'\0'};
  struct i2cDev dev;  
    dev.devAddr = BMP085_ADDR;
    dev.subAddr = BMP085_ID_ADDR;
  
  i2cGetc(&dev, &testByte);
  //sprintf(debugStr,"Id of reg[%d]: 0x%x\n\r",dev.subAddr,(int)testByte);
  //putsUART1(debugStr);
  
  return testByte == BMP085_ID_VAL;
}

//Gets the calibration coefficients from BMP085 EEPROM; needed for calculating pres and temp data (see manual)
BOOL BMP085GetCalCoefficients(struct bmp085Vector *barVec)
{
  unsigned char byteStr[32] = {'\0'};
  BOOL success = FALSE;
  //unsigned char debugStr[256] = {'\0'};
  short int shints[11] = {0};
  short int* nP = (short int*)barVec;  //point nP at first short entry in barVec struct; allows sequential iteration over struct
  int i = 0;
  struct i2cDev dev;
    dev.devAddr = BMP085_ADDR;
    dev.subAddr = BMP085_AC1_H; // 0xAA, address of first EEPROM register for reading Calibration regs sequentially
  
  //putsUART1("buff\n\r");
  //read the 22 hi/lo bytes of cal regs
  if( i2cGetsManualIncrement(&dev, byteStr, 22) ){
    //putsUART1("here2\n\r");
    //convert the bytes to short ints and store in barVec
    for(i = 0; i < 22; i+=2, nP++){
      *nP = ((short int)byteStr[i] << 8) | byteStr[i + 1];
    }

    /*
    //debug
    sprintf(debugStr,"cal vals: %d %d %d %d %d\n\r\t%d %d %d %d %d %d\n\r\0",
            barVec->ac1,barVec->ac2,barVec->ac3,barVec->ac4,barVec->ac5,
            barVec->ac6,barVec->b1,barVec->b2,barVec->mb,barVec->mc,barVec->md);
    putsUART1(debugStr);
    */
    success = TRUE;
  }
  else{
    success = FALSE;
    putsUART1("ERROR i2cGets() failed in BMP085GetCalCoefficients()\n\r");
  }

  return success;
}

/*
   Read and calculate temp value. Getting pressure depends on getting temp, hence 2 in 1.
   GetTemp() could be its own function, but getPRess() would require getTemp().
   See page 12 of the BMP085 manual.
*/
BOOL BMP085GetTempAndPressure(struct bmp085Vector *barVec)
{
  int i = 0;
  unsigned char txByte = '\0', d0 = '\0', d1 = '\0';
  struct i2cDev dev;
    dev.devAddr = BMP085_ADDR;
    dev.subAddr = BMP085_GET_TEMP;
 
 /*
  //debug
  char debugStr[256] = {'\0'};
  sprintf(debugStr,"cal vals: %d %d %d %d %d\n\r\t%d %d %d %d %d %d\n\r\0",
            barVec->ac1,barVec->ac2,barVec->ac3,barVec->ac4,barVec->ac5,
            barVec->ac6,barVec->b1,barVec->b2,barVec->mb,barVec->mc,barVec->md);
  putsUART1(debugStr);
*/
 
  //putsUART1("debug getTemp()\n\r");
  barVec->oss = 0;  //set oversampling to zero; this may need to be 3 (??), if you look at manual's formula and vals

  if( !BMP085TestConnection() ){
    putsUART1("Connection failed in BMP085getTempAndPress()\n\r");
    return FALSE;
  }

  dev.subAddr = BMP085_CTRL;
  txByte = BMP085_GET_TEMP;
  if( !i2cPutc(&dev, &txByte) ){
    putsUART1("Selecting temp reg failed in BMP085getTempAndPress()\n\r");
    return FALSE;
  }
  
  //delay at least 4.5 ms: 80mhz clk * 0.0045 = 360000 clks
  for(i = 0; i < 2; i++){ delay(16000); }
  
  dev.subAddr = BMP085_DO1; //0xF6 data-out MSB reg
  if( !i2cGetc(&dev, &d0) ){
    putsUART1("Read DO1 failed in BMP085getTempAndPress()\n\r");
    return FALSE;
  }
   
  dev.subAddr = BMP085_DO2; //0xF7 data-out LSB reg
  if( !i2cGetc(&dev, &d1)){
    putsUART1("Read D02 failed in BMP085getTempAndPress()\n\r");
    return FALSE;
  }

  barVec->rawTemp = ((unsigned int)d0 << 8) | d1;
  
  //now fetch the raw pressure val
  dev.subAddr = BMP085_CTRL;  // control reg 0xF4
  txByte = BMP085_GET_PRESS;  // 0x34 (OR'ed with OSS selection; 0 default OK; see manual)
  if( !i2cPutc(&dev, &txByte) ){
    putsUART1("Selecting pressure reg failed in BMP085getTempAndPress()\n\r");
    return FALSE;
  }
  
  //delay at least 4.5 ms: 80mhz clk * 0.0045 = 360000 clks
  for(i = 0; i < 2; i++){ delay(16000); }
  
  dev.subAddr = BMP085_DO1; //0xF6 data-out MSB reg
  if( !i2cGetc(&dev, &d0) ){
    putsUART1("Read DO1 press failed in BMP085getTempAndPress()\n\r");
    return FALSE;
  }
  
  dev.subAddr = BMP085_DO2; //0xF7 data-out LSB reg
  if( !i2cGetc(&dev, &d1)){
    putsUART1("Read DO2 press failed in BMP085getTempAndPress()\n\r");
    return FALSE;
  }
  
  barVec->rawPress = ((unsigned int)d0 << 8) | d1;
  
  BMP085CalcTempAndPressure( barVec );
  
  return TRUE;
}

//All based on manual formulae. barVec must be loaded with cal coefficients before call.
void BMP085CalcTempAndPressure(struct bmp085Vector *barVec)
{
	int  tval = 0, pval = 0;
	int  x1 = 0, x2 = 0, x3 = 0, b3 = 0, b5 = 0, b6 = 0, p = 0;
	int  b4 = 0, b7 = 0;
	//unsigned char oss = 3;

	x1 = (barVec->rawTemp - barVec->ac6) * barVec->ac5 >> 15;
  x2 = ((int)barVec->mc << 11) / (x1 + barVec->md);
	b5 = x1 + x2;
	barVec->trueTemp = (b5 + 8) >> 4;
  
	b6 = b5 - 4000;
	x1 = (barVec->b2 * (b6 * b6 >> 12)) >> 11; 
	x2 = barVec->ac2 * b6 >> 11;
	x3 = x1 + x2;
	b3 = (((int)barVec->ac1 * 4 + x3)<< barVec->oss + 2) >> 2;
	x1 = barVec->ac3 * b6 >> 13;
	x2 = (barVec->b1 * (b6 * b6 >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	b4 = (barVec->ac4 * (unsigned int ) (x3 + 32768)) >> 15;
	b7 = ((unsigned int ) barVec->rawPress - b3) * (50000 >> barVec->oss);
	p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;
	x1 = (p >> 8) * (p >> 8);
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	barVec->truePress = p + ((x1 + x2 + 3791) >> 4);
}


/* Cal coefficients are set at the factory; I dont believe we need to set these. Ever.
//Vals are fixed (see manual), and must be loaded at configuration time.
BOOL BMP085SetCalCoefficients(void)
{
  int i = 0;
  struct i2cDev dev;
  struct bmp085Vector barVec;
  unsigned char regVals[22] = {'\0'};
  char debugStr[128] = {'\0'};
  
  dev.devAddr = BMP085_ADDR;
  dev.subAddr = BMP085_AC1_H;
  
  regVals[0] = (unsigned char)(0x000000FF & (408 >> 8));  // MSB comes first,
	regVals[1] = (unsigned char)(0x000000FF & 408);         // then LSB, for each pair of lines.
  
  regVals[2] = (unsigned char)(0x000000FF & (-72 >> 8));
	regVals[3] = (unsigned char)(0x000000FF & -72);
  
  regVals[4] = (unsigned char)(0x000000FF & (-14383 >> 8));
	regVals[5] = (unsigned char)(0x000000FF & -14383);
                
  regVals[6] = (unsigned char)(0x000000FF & (32741 >> 8));
	regVals[7] = (unsigned char)(0x000000FF & 32741);
  
  regVals[8] = (unsigned char)(0x000000FF & (32757 >> 8));
	regVals[9] = (unsigned char)(0x000000FF & 32757);
  
  regVals[10] = (unsigned char)(0x000000FF & (23153 >> 8));
	regVals[11] = (unsigned char)(0x000000FF & 23153);
                 
  regVals[12] = (unsigned char)(0x000000FF & (6190 >> 8));
	regVals[13] = (unsigned char)(0x000000FF & 6190);
                 
  regVals[14] = (unsigned char)(0x000000FF & (4 >> 8));
	regVals[15] = (unsigned char)(0x000000FF & 4);
                 
  regVals[16] = (unsigned char)(0x000000FF & (-32767 >> 8));
	regVals[17] = (unsigned char)(0x000000FF & -32767);
                 
  regVals[18] = (unsigned char)(0x000000FF & (-8711 >> 8));
	regVals[19] = (unsigned char)(0x000000FF & -8711);
                 
  regVals[20] = (unsigned char)(0x000000FF & (2868 >> 8));
  regVals[21] = (unsigned char)(0x000000FF & 2868);
  
  
  sprintf(debugStr,"Vals: %d %d %d %d %d %d\n\r\0",(int)((regVals[0] << 8) |regVals[1]), (int)((regVals[2] << 8) |regVals[3]), (int)((regVals[4] << 8) |regVals[5]), (int)((regVals[6] << 8) |regVals[7]), (int)((regVals[8] << 8) |regVals[9]),(int)((regVals[10] << 8) |regVals[11]));
  putsUART1(debugStr);
  clearStr(debugStr,128);
  sprintf(debugStr,"Vals: %d %d %d %d %d %d\n\r\0",(int)((regVals[12] << 8) |regVals[13]), (int)((regVals[14] << 8) |regVals[15]), (int)((regVals[16] << 8) |regVals[17]), (int)((regVals[18] << 8) |regVals[19]), (int)((regVals[20] << 8) |regVals[21]),(int)((regVals[22] << 8) |regVals[23]));
  putsUART1(debugStr);
  
  
  for(i = 0; i < 22; i++, dev.subAddr++){
    if( !i2cPutc(&dev, regVals+i) ){
      return FALSE;
    }
  }
  
  return TRUE;
}
*/


/////////////////////////////////////////////////////////////////////////////
/////////////////ADXL345 code////////////////////////////////////////////////
BOOL ADXL345TestConnection(void)
{
  unsigned char testByte = 0;
  //char debugStr[64] = {'\0'};
  struct i2cDev dev;  
    dev.devAddr = ADXL345_ADDR;
    dev.subAddr = ADXL345_ID_ADDR;
  
  i2cGetc(&dev, &testByte);
  //sprintf(debugStr,"Id of reg[%d]: 0x%x\n\r",dev.subAddr,(int)testByte);
  //putsUART1(debugStr);
  
  return testByte == ADXL345_ID_VAL;
}

/*
  ADXL345 self test: from manual "It is recommended that the
part be set to full-resolution, 16 g mode to ensure that there is
sufficient dynamic range for the entire self-test shift."

*/
void ADXL345SelfTest(void)
{
  return;
}

BOOL InitializeADXL345(void)
{
  unsigned char byte = 0;
  struct i2cDev dev;

  dev.devAddr = ADXL345_ADDR;  // gives 0xA6 for write, 0xA7 for read
  dev.subAddr = ADXL345_POWER_CTL;
  byte = 0;
  
  if( i2cPutc(&dev, &byte) ){ // reset all power settings to 0
    delay(TINY_DLY);
    putsUART1("debug1\n\r");
    //set autosleep enabled bit (bit 4) in pwr_ctl reg 0x2d
      //setAutoSleepEnabled(TRUE); I2Cdev::writeBit(devAddr, ADXL345_RA_POWER_CTL, ADXL345_PCTL_AUTOSLEEP_BIT, enabled);
    if(i2cWriteBit(&dev, (unsigned char)4, TRUE) ){ //ADXL345_PCTL_AUTOSLEEP_BIT == 4
    
      //enable measurement
      if(i2cWriteBit(&dev, (unsigned char)3, TRUE)){  //ADXL345_PCTL_MEASURE_BIT = bit_3
        putsUART1("debug3\n\r");
        return TRUE;
      }
    }
  }
  
  putsUART1("ERROR ADXL345initialize() FAIL\n\r");
  return FALSE;
}

/*
  The following code was tested successfully in main:
    //Z value is a high value due to gravity (?)
    dev.subAddr = ADXL345_X_L;
    i2cGetc(&dev, &(testStr[0]) );
    dev.subAddr = ADXL345_X_H;
    i2cGetc(&dev, &(testStr[1]) );
    dev.subAddr = ADXL345_Y_L;
    i2cGetc(&dev, &(testStr[2]) );
    dev.subAddr = ADXL345_Y_H;
    i2cGetc(&dev, &(testStr[3]) );
    dev.subAddr = ADXL345_Z_L;
    i2cGetc(&dev, &(testStr[4]) );
    dev.subAddr = ADXL345_Z_H;
    i2cGetc(&dev, &(testStr[5]) );

    //format the vals
    XL = 0x000000FF & testStr[0];
    XH = 0x000000FF & testStr[1];
    YL = 0x000000FF & testStr[2];
    YH = 0x000000FF & testStr[3];
    ZL = 0x000000FF & testStr[4];
    ZH = 0x000000FF & testStr[5];
    X = (XH << 8) | XL;
    Y = (YH << 8) | YL;
    Z = (ZH << 8) | ZL;
    
    sprintf(accStr,"Acc(X,Y,X)  0x%04X  0x%04X  0x%04X\n\r",X,Y,Z);
    putsUART1(accStr);
    
    X = 0;   Y = 0;  Z = 0;
    XL = 0; YL = 0; ZL = 0;
    XH = 0; YH = 0; ZH = 0;
    clearStr(accStr,64);
    clearStr(testStr,64);

Read in XL,XH,YL,YH,ZL,ZH accel vals

  From the manual : "After the register
    addressing and the first byte of data, each subsequent set of
    clock pulses (eight clock pulses) causes the ADXL345 to point
    to the next register for a read or write."



*/
BOOL ADXL345GetAccel(unsigned char accBytes[])
{
  int i = 0;
  char accStr[64] = {'\0'};
  struct i2cDev adxl345;
    adxl345.devAddr = ADXL345_ADDR;
    adxl345.subAddr = ADXL345_X_L;
  
  //ADXL345 autoincrement not yet implemented, so loop and increment the subAddr manually
  //multibyte read is suggested so we get the consistent readings between reading hi/lo byte register
  /*
  while((success = TRUE) && (i < 6)){
    adxl345.subAddr += i;
    sprintf(str,"subAddr 0x%x\n\r",(int)adxl345.subAddr);
    putsUART1(str);
    success = i2cGetc(&adxl345, inChars[i] );
    i++;
  }
  */
  
  if( !i2cGetsManualIncrement(&adxl345,accBytes,6) ){
    putsUART1("ERROR i2cGets() failed in ADXL345GetAccel()\n\r");
    return FALSE;
  }
  
  //sprintf(accStr,"getAcc(X,Y,X):  0x%04X  0x%04X  0x%04X\n\r", (0x0000FFFF & ((int)accBytes[1] << 8)) | accBytes[0],(0x0000FFFF & ((int)accBytes[3] << 8)) | accBytes[2],(0x0000FFFF & ((int)accBytes[5] << 8)) | accBytes[4] );
  //putsUART1(accStr);
  
  return TRUE;
}

/////////////////////////////////////////////////////////////////////////////
/////////////////L3G4200D code///////////////////////////////////////////////
BOOL L3G4200DTestConnection(void)
{
  unsigned char testByte = 0;
  //char debugStr[64] = {'\0'};
  struct i2cDev dev;  
    dev.devAddr = L3G4200D_ADDR;
    dev.subAddr = L3G4200D_ID_ADDR;
  
  i2cGetc(&dev, &testByte);
  //sprintf(debugStr,"Id of reg[%d]: 0x%x\n\r",dev.subAddr,(int)testByte);
  //putsUART1(debugStr);
  
  return testByte == L3G4200D_ID_VAL;
}

//turns on gyro, 500 deg/sec sensitivity, BDU mode (ctrl_reg5)
BOOL InitializeL3G4200D(void)
{
  unsigned char byte = 0;
  struct i2cDev dev;

  dev.devAddr = L3G4200D_ADDR;
  dev.subAddr = L3G4200D_CTL_REG1;
  byte = 0x1E;
  
  if( i2cPutc(&dev, &byte) ){ // set all power settings to 0x1F (power on, some bandwidth selection or other)
    //putsUART1("debug1\n\r");
    
    // Set scale (500 deg/sec), BDU on (doesnt update output registers until MSB and LSB are read)
    dev.devAddr = L3G4200D_ADDR;
    dev.subAddr = L3G4200D_CTL_REG5;
    byte = 0x90;
    if( i2cPutc(&dev, &byte) ){ //ADXL345_PCTL_AUTOSLEEP_BIT == 4
      //putsUART1("debug2\n\r");
      return TRUE;
    }
  }
  
  putsUART1("ERROR L3G4200Dinitialize() FAIL\n\r");
  return FALSE;
}

//Get gyro x, y, and Z angular velocities, as well as the gyro's temp sensor reading.
BOOL L3G4200DGetGyro(unsigned char gyroBytes[])
{
  int i = 0;
  //char accStr[64] = {'\0'};
  struct i2cDev gyro;
    gyro.devAddr = L3G4200D_ADDR;
    gyro.subAddr = L3G4200D_X_L;
  
  //ADXL345 autoincrement not yet implemented, so loop and increment the subAddr manually
  //multibyte read is suggested so we get the consistent readings between reading hi/lo byte register
  /*
  while((success = TRUE) && (i < 6)){
    adxl345.subAddr += i;
    sprintf(str,"subAddr 0x%x\n\r",(int)adxl345.subAddr);
    putsUART1(str);
    success = i2cGetc(&adxl345, inChars[i] );
    i++;
  }
  */
  
  if( !i2cGetsManualIncrement(&gyro,gyroBytes,6) ){
    putsUART1("ERROR i2cGetsManualIncrement() failed in L3G4200DGetGyro()\n\r");
    return FALSE;
  }
  
  //now get the temperature
  gyro.subAddr = L3G4200D_TEMP;
  i2cGetc(&gyro,gyroBytes+6);

  //debug
  //sprintf(gyroStr,"getGyro(X,Y,X, temp): 0x%04X  0x%04X  0x%04X  0x%04X\n\r", (0x0000FFFF & ((int)gyroBytes[1] << 8)) | gyroBytes[0],(0x0000FFFF & ((int)gyroBytes[3] << 8)) | gyroBytes[2],(0x0000FFFF & ((int)gyroBytes[5] << 8)) | gyroBytes[4], (0x000000FF & gyroBytes[6]) );
  //putsUART1(gyroStr);
  
  return TRUE;
}

/////////////////////////////////////////////////////////////////////////////
/////////////////HMC5883L code///////////////////////////////////////////////
// Must return 0x48 for successful connection
BOOL HMC5883LTestConnection(void)
{
  unsigned char testByte = 0;
  //char debugStr[64] = {'\0'};
  struct i2cDev dev;  
    dev.devAddr = HMC5883L_ADDR;
    dev.subAddr = HMC5883L_ID_ADDR;
  
  i2cGetc(&dev, &testByte);
  //sprintf(debugStr,"Id of reg[%d]: 0x%x\n\r",dev.subAddr,(int)testByte);
  //putsUART1(debugStr);
  
  return testByte == HMC5883L_ID_VAL;
}

/*
  See manual. Clear BIT_1 in the STATUS register after reading all registers. To preserve measurement state
  for a given read sequence, BIT_1 locks data measurement if not all regs have been read. BIT_7 in the MODE
  register is also set every time a read occurs in single-measurement mode.
*/
BOOL InitializeHMC5883L(void)
{
  unsigned char byte = 0;
  struct i2cDev dev;

  dev.devAddr = HMC5883L_ADDR;  // 0x1E
  dev.subAddr = HMC5883L_CFIG_A; //0x0
  byte = 0x70; // 0b01110000 Clear BIT_7 (manual specs this is necessity), 8 samples averaged per output (default), data output rate 15 hz, Normal measurement mode
  
  if( i2cPutc(&dev, &byte) ){ // reset all power settings to 0  
    //putsUART1("debug1\n\r");
    dev.subAddr = HMC5883L_MODE;
    byte = 0;
    
    if( i2cPutc(&dev, &byte) ){ // reset all power settings to 0; from manual example
      //putsUART1("debug2\n\r");
      
      //clear BIT_1 in the STATUS register (measurement lock bit): this is just a precondition
      dev.subAddr = HMC5883L_STATUS;
      if(i2cWriteBit(&dev, (unsigned char)1, FALSE) ){ //ADXL345_PCTL_AUTOSLEEP_BIT == 4
        //putsUART1("HMC5883L init success\n\r");
        return TRUE;
      }
    }
  }
  
  putsUART1("ERROR ADXL345initialize() FAIL\n\r");
  return FALSE;
}

/*
  Get the X, Y, and Z magnetometer vectors.
*/
BOOL HMC5883LGetMag(unsigned char magBytes[])
{
  int i = 0;
  //char accStr[64] = {'\0'};
  struct i2cDev mag;
    mag.devAddr = HMC5883L_ADDR;
    mag.subAddr = HMC5883L_X_H;
  
  //ADXL345 autoincrement not yet implemented, so loop and increment the subAddr manually
  //multibyte read is suggested so we get the consistent readings between reading hi/lo byte register
  /*
  while((success = TRUE) && (i < 6)){
    adxl345.subAddr += i;
    sprintf(str,"subAddr 0x%x\n\r",(int)adxl345.subAddr);
    putsUART1(str);
    success = i2cGetc(&adxl345, inChars[i] );
    i++;
  }
  */
  
  if( !i2cGetsManualIncrement(&mag,magBytes,6) ){
    putsUART1("ERROR i2cGetsManualIncrement() failed in HMC5883LGetMag()\n\r");
    return FALSE;
  }
  
  //debug
  //sprintf(gyroStr,"getGyro(X,Y,X): 0x%04X  0x%04X  0x%04X\n\r", (0x0000FFFF & ((int)gyroBytes[1] << 8)) | gyroBytes[0],(0x0000FFFF & ((int)gyroBytes[3] << 8)) | gyroBytes[2],(0x0000FFFF & ((int)gyroBytes[5] << 8)) | gyroBytes[4] );
  //putsUART1(gyroStr);
  
  return TRUE;
}


///////////////////////////////////////////////////////////////////////////////
/////////////////I2C Module Code///////////////////////////////////////////////
// Write or clear a bit.  To clear, pass FALSE for third parameter.
BOOL i2cWriteBit(struct i2cDev* dev, unsigned char bitNum, BOOL enable)
{
    unsigned char byte = '\0';
    i2cGetc(dev, &byte);
    
    byte = (enable != FALSE) ? (byte | (1 << bitNum)) : (byte & ~(1 << bitNum));
    return i2cPutc(dev, &byte);
}

//params: device address and subaddress, and a pointer to a data byte mem location
//cascade changes to i2cPuts() below
BOOL i2cPutc(struct i2cDev* dev, unsigned char* txData)
{
  //debug output
  int test = 0;
  
  /*
  //debug
  I2C_STATUS status;
  char str[64] = {'\0'};
  status = I2CGetStatus(I2C1);
  test = (int)status | test;
  sprintf(str,"init i2C status: 0x%x\n\r",test);
  putsUART1(str);
  //end debug
  */

  if( !i2cStartTransfer(FALSE) ){
    putsUART1("ERROR i2cPutc failed to initiate START cond\n\r");
    return FALSE;
  }

  //Write byte to slave:
  //  Transmit sequence a byte at a time: Slave write address, slave sub address (the id register), then data to write
  if( !I2CTransmitByte(I2C1, ( dev->devAddr << 1 | 0) ) ){ //send write address
    putsUART1("ERROR i2cPutc() failed to send dev addr\n\r");
    return FALSE;
  }

  if( !I2CTransmitByte(I2C1, dev->subAddr ) ){ //send subaddress to write to
    putsUART1("ERROR i2cPutc() failed to send dev subaddr\n\r");
    return FALSE;
  }
  
  //putsUART1("here\n\r");
  if( !I2CTransmitByte(I2C1, *txData ) ){ //send data byte to write to
    putsUART1("ERROR i2cPutc() failed to send txData byte\n\r");
    return FALSE;
  }

  i2cStopTransfer();
  delay(128);
  
  //putsUART1("post stop\n\r");
  //I2CClearStatus ( I2C1, I2C_ARBITRATION_LOSS | I2C_SUCCESS | I2C_ERROR | I2C_START | I2C_STOP | I2C_MASTER_BUS_COLLISION | I2C_RECEIVE_OVERFLOW | I2C_TRANSMITTER_OVERFLOW ); //second param selects flags to clear (all of them)

   return TRUE;
}

//just an iterative modification of i2cPutc
BOOL i2cPuts(struct i2cDev* dev, unsigned char txData[], int nBytes)
{
  int i = 0;
  
  /*
  //debug output
  int test = 0;
  I2C_STATUS status;
  char str[64] = {'\0'};
  status = I2CGetStatus(I2C1);
  test = (int)status | test;
  sprintf(str,"init i2C status: 0x%x\n\r",test);
  putsUART1(str);
  //end debug
  */

  if( !i2cStartTransfer(FALSE) ){
    putsUART1("ERROR i2cPuts failed to initiate START cond\n\r");
    return FALSE;
  }

  //Write byte to slave:
  //  Transmit sequence a byte at a time: Slave write address, slave sub address (the id register), then data to write
  if( !I2CTransmitByte(I2C1, ( dev->devAddr << 1 | 0) ) ){ //send write address
    putsUART1("ERROR i2cPuts() failed sending devAddr\n\r");
    return FALSE;
  }

  if( !I2CTransmitByte(I2C1, dev->subAddr ) ){ //send subaddress to write to
    putsUART1("ERROR i2cPuts() failed sending dev subAddr\n\r");
    return FALSE;
  }

  while(i < nBytes){
    if( !I2CTransmitByte(I2C1, txData[i++] ) ){ //send subaddress to write to
      putsUART1("ERROR i2cPuts() failed to send txData byte\n\r");
      return FALSE;
    }
  }

  i2cStopTransfer();
  
  //I2CClearStatus ( I2C1, I2C_ARBITRATION_LOSS | I2C_SUCCESS | I2C_ERROR | I2C_START | I2C_STOP | I2C_MASTER_BUS_COLLISION | I2C_RECEIVE_OVERFLOW | I2C_TRANSMITTER_OVERFLOW ); //second param selects flags to clear (all of them)

   return TRUE;
 }

//params: device address and subaddress, and a pointer to a data byte mem location
BOOL i2cGetc(struct i2cDev* dev, unsigned char *rxData)
{
  int i = 0;
  I2C_RESULT result = I2C_SUCCESS;
  char errorStr[64] = {'\0'};

  /*
  //debug output
  int test = 0;
  int Success = 0b1111;
  I2C_STATUS status;
  char str[64] = {'\0'};
  status = I2CGetStatus(I2C1);
  test = (int)status | test;
  sprintf(str,"init i2C status: 0x%x\n\r",test);
  putsUART1(str);
  //end debug
  */
  
  if( !i2cStartTransfer(FALSE) ){
    putsUART1("i2cStart xfer failed in i2cGetc()\n\r");
    return FALSE;
  }

  //Read data from slave:
  //  Transmit sequence a byte at a time: Slave write address, slave sub address (the id register), but NOT the read register slave address (comes below, after repeated start command)
  if ( !I2CTransmitByte(I2C1, ( dev->devAddr << 1 | 0) )){ //send write address
    putsUART1("ERROR devAddr could not be sent in i2cGetc()\n\r");
    return FALSE;
  }

  if( !I2CTransmitByte(I2C1, dev->subAddr ) ){ //send subaddress to read from
    putsUART1("Error: devSubAddr could not be sent in i2cGetc()\n\r");
    return FALSE;
  }

  i2cStartTransfer(TRUE);  //now send repeated start signal followed by slave read address + read bit
  ////putsUART1("made it to repeat start\n\r");
  
  if( !I2CTransmitByte(I2C1, (dev->devAddr << 1 | 1) ) ){  //send the read address
    putsUART1("ERROR Failed to send devSubAddr_R in i2cGetc()\n\r");
    i2cStopTransfer();
    return FALSE;
  }
    
  result = I2CReceiverEnable(I2C1, TRUE);
  if(result != I2C_SUCCESS ){
    sprintf(errorStr,"ERROR I2CRxEnable returned 0x%x in i2cGetc()\n\r",(int)result);
    putsUART1(errorStr);
    I2CReceiverEnable(I2C1, FALSE);
    i2cStopTransfer();
    return FALSE;
  }

  //while( !I2CReceivedDataIsAvailable( I2C1 ));
  //wait for data to arrive: optimistically if we made it this far, the wait should succeed
  while( (I2CReceivedDataIsAvailable( I2C1 ) == FALSE) && (i < 10) ){
    i++;
    delay(TINY_DLY); //wait about 128+ clks
  }
  
  if(i >= 10){
    putsUART1("ERROR Data not available in i2cGetc()\n\r");
    i2cSendAck(I2C1, FALSE);
    I2CReceiverEnable(I2C1, FALSE);
    i2cStopTransfer();
    return FALSE;
  }
  
  *rxData = I2CGetByte(I2C1);
  i2cSendAck(I2C1, FALSE); // IF LOOP-READING MULTIPLE BYTES, THIS SHOULD PASS *TRUE* (false is for ending transmission: master sends NACK, then STOP
  I2CReceiverEnable(I2C1, FALSE);  //dont forget to disable receiver once Rx completed!  Not known if this is necessary or not
  i2cStopTransfer();

  // End the transfer
  //putsUART1("pre\n\r");  succeeds / prints
  //i2cSendAck(I2C1, FALSE);
  // putsUART1("post\n\r"); succeeds / print
  //I2CClearStatus ( I2C1, I2C_ARBITRATION_LOSS | I2C_SUCCESS | I2C_ERROR | I2C_START | I2C_STOP | I2C_MASTER_BUS_COLLISION | I2C_RECEIVE_OVERFLOW | I2C_TRANSMITTER_OVERFLOW ); //second param selects flags to clear (all of them)
  
  return TRUE;
}

/*
  read a sequence of bytes from a slave.  Devices typically offer reg address auto-incrementing.
  Reading sequential registers entails reading from the lowest reg addr, the device then increments
  the subaddress on successive reads.
  THIS FUNCTION NOT TESTED

  See i2cGetsManualIncrement().  The behavior of this function is purely device specific.  Some devices support
  autoincrementing, some do not.  Those that do may only support it under specific conditions.  For example, the
  adxl345 support autoincrementing only when each read is buffered by 8 clock pulses.

  adxl345: supports autoincrement, but only if reads are buffered by 8 clock pulses.
  mag: supports autoincrement by default, "without further master intervention".  Manual gives no further info.
  gyro: supports autoincrement depending on usage of read ops and value of FIFO_CTRL_REG (see manual for "read_burst")
  barometer: 
  

  PRECON: Device must support or be configured-for address auto-incrementing.

 */
BOOL i2cGets(struct i2cDev* dev, unsigned char rxData[], int nBytes)
{
  int i = 0, readCt = 0;
  BOOL ack = TRUE;
  I2C_RESULT result = I2C_SUCCESS;
  char errorStr[64] = {'\0'};
  
  /*
  //debug output
  int test = 0;
  int Success = 0b1111;
  I2C_STATUS status;
  char str[64] = {'\0'};
  status = I2CGetStatus(I2C1);
  test = (int)status | test;
  sprintf(str,"init i2C status: 0x%2x\n\r",test);
  //putsUART1(str);
  //end debug
  */

  if( !i2cStartTransfer(FALSE) ){
    putsUART1("ERROR i2cGets(): i2cStart xfer failed\n\r");
    return FALSE;
  }

  //Read data from slave:
  //  Transmit sequence a byte at a time: Slave write address, slave sub address (the id register), but NOT the read register slave address (comes below, after repeated start command)
  if ( !I2CTransmitByte(I2C1, ( dev->devAddr << 1 | 0)) ){ //send write address
    sprintf(errorStr,"ERROR i2cGets(): devAddr >0x%x< not sent\n\r",dev->devAddr);
    putsUART1(errorStr);
    return FALSE;
  }

  if( !I2CTransmitByte(I2C1, dev->subAddr ) ){ //send subaddress to read from
    sprintf(errorStr,"ERROR i2cGets(): subAddr >0x%x< not sent\n\r",dev->subAddr);
    putsUART1(errorStr);
    return FALSE;
  }

  i2cStartTransfer(TRUE);  //now send repeated start signal followed by slave read address + read bit
  ////putsUART1("made it to repeat start\n\r");
  
  if( I2CTransmitByte(I2C1, (dev->devAddr << 1 | 1) ) ){ //send the read address
    
    result = I2CReceiverEnable(I2C1, TRUE);
    if(result != I2C_SUCCESS ){
      sprintf(errorStr,"ERROR i2cGets(): I2CRxEnable() returned 0x%x\n\r",(int)result);
      putsUART1(errorStr);
      I2CReceiverEnable(I2C1, FALSE);
      i2cStopTransfer();
      return FALSE;
    }

    //iteratively read bytes from the bus
    for(readCt = 0; readCt < nBytes; readCt++){

      //wait ~128 clks for data to arrive: optimistically if we made it this far, the wait should succeed, some devices just need time to catch up...
      for(i = 0; !I2CReceivedDataIsAvailable(I2C1) && (i < 10); i++){ delay(TINY_DLY); }
      
      if(i < 10){  //outer error check if > 10, then something went wrong...
        ack = (readCt < nBytes) ? TRUE : FALSE; //this check is necessary so we don't send TRUE on the last read
        i2cSendAck(I2C1, ack); // IF LOOP-READING MULTIPLE BYTES: THIS MUST PASS *TRUE* (FALSE is for ending transmission: master sends NACK, then STOP)
        rxData[readCt] = I2CGetByte(I2C1);

        /* 
           Wait for 6600 sys-clks (3 ops * 2200: < comparison, ++, and Nop()).
           This is specifically for the adxl345, which requires 8 i2c-clk pulses for autoincrement to occur.
           80 MHz sysclk / 100 KHz i2c-clk = 800 sysClk/i2cClk * 8 == 6400 sysClks
        */
        for(i = 0; i < 2200; i++){ Nop(); }


        //sprintf(compass_X,"Rx'ed: 0x%x\n\r", *rxData );
      }
      else{
        sprintf(errorStr,"ERROR i2cGets(): Data not available >> rdCt=0x%x i=0x%x\n\r",readCt, i);
        putsUART1(errorStr);
        i2cSendAck(I2C1, FALSE);
        I2CReceiverEnable(I2C1, FALSE);
        i2cStopTransfer();
        return FALSE;
      }
    }
    
    //putsUART1(compass_X);
    I2CReceiverEnable(I2C1, FALSE);  //dont forget to disable receiver once Rx completed!  Not known if this is necessary or not
  }
  else{
    putsUART1("ERROR i2cGets(): Failed to send devSubAddr_R\n\r");
    i2cStopTransfer();
    return FALSE;
  }

  // End the transfer
  //putsUART1("pre\n\r");  succeeds / prints
  //i2cSendAck(I2C1, FALSE);
  i2cStopTransfer();
  // putsUART1("post\n\r"); succeeds / print
  //I2CClearStatus ( I2C1, I2C_ARBITRATION_LOSS | I2C_SUCCESS | I2C_ERROR | I2C_START | I2C_STOP | I2C_MASTER_BUS_COLLISION | I2C_RECEIVE_OVERFLOW | I2C_TRANSMITTER_OVERFLOW ); //second param selects flags to clear (all of them)
}

/*
  i2cGets() only supports devices that support auto-incrementing register read.  This
  function instead calls i2cGetc() iteratively, manually getting each byte in a
  complete i2c transmission (start, W, Sub, restart, R, byte, stop).
  Since the target registers are almost always in sequential blocks of 8 bit registers,
  this function takes in a device struct containing the starting register address,
  then reads up to n sequential registers.

  WARNINGS: Expected returns of this function may be device specific.  Some devices (adxl345)
  require a discrete pause of n-clk sigs before a new register may be addressed, and likewise
  may update/not-update registers as expected for a given sequential register read.
  
  Using i2cGets() is always preferred, not just for speed, but also because this function
  risks updating MSB/LSB registers between reads operations, such that when MSB byte is read,
  its associated LSB is updated with new data before being read, as follows:
  
     MSB     LSB       Op
  1  0x12    0x34      Master reads MSB 0x12>>
  2  0x56    0x78      <<Slave updates regs
  3  0x56    0x78      Master reads LSB 0x78>>
    
    Result:          Master reads 0x1278
    Intended result: Master reads 0x1234
    
  This is extremely likely to occur, and update will almost always occur while the master is busy
  initiating a new i2c transmission between the MSB and LSB read.
  Some devices offer blocking bits or other measures to prevent this result, but not all.
  
  Returns: SUCCESS or FAIL.  We could instead return the number of bytes read, along with some
  failure/success output parameter.
*/
BOOL i2cGetsManualIncrement(struct i2cDev* dev,unsigned char rxData[], unsigned int nRegs)
{
  int i = 0;
  
  //loop through the sequential register addresses
  for(i = 0; i < nRegs; i++, dev->subAddr++){
    if( !i2cGetc(dev, rxData+i) ){
      return FALSE;
    }
  }

  return TRUE;
}



/*****************************************************************************
I2C configuration:

-necessary I2C freq is unclear

reference: I2C1
void I2CConfigure ( i2c_modULE id, I2C_CONFIGURATION flags )
  precon/usage:
unsigned int I2CSetFrequency ( i2c_modULE id, unsigned int sourceClock, unsigned int i2cClock );
  precon/usage: peripheral clock must already be configured (running?)
void I2CSetSlaveAddress ( i2c_modULE id, UINT16 address, UINT16 mask,
                              I2C_ADDRESS_MODE flags )

void I2CEnable( i2c_modULE id, BOOL enable )
  precondition: The module should be appropriately configured (using I2CConfigure and
    I2CSetFrequency, and possibly I2CSetSlaveAddress) before being enabled.

BOOL I2CBusIsIdle( i2c_modULE id ) This is a routine to determine if the I2C bus is idle or busy.
I2C_RESULT I2CStart( i2c_modULE id ) This routine sends the "start" signal (a falling edge on SDA while SCL is high) to start a transfer on the I2C bus.
I2C_RESULT I2CRepeatStart ( i2c_modULE id ) This routine supports sending a repeated Start condition to change slave targets or transfer direction to support certain I2C transfer formats.
void I2CStop ( i2c_modULE id ) This is a routine to send an I2C Stop condition to terminate a transfer.
BOOL I2CTransmitterIsReady ( i2c_modULE id ) This is a routine to detect if the transmitter is ready to accept data to transmit.

BOOL I2CTransmissionHasCompleted ( i2c_modULE id ) The module must have been configured appropriately and enabled, a transfer
                          must have been previously started, and a data or address byte must have
                          been sent.
BOOL I2CByteWasAcknowledged ( i2c_modULE id )    This routine allows a transmitter to determine if the byte just sent was
                          positively acknowledged (ACK'd) or negatively acknowledged (NAK'd) by the
                          receiver. Byte must have been sent.
I2C_RESULT I2CReceiverEnable ( i2c_modULE id, BOOL enable ) This is a routine to enable the module to receive data from the I2C bus.
                        This routine enables the module to receive data from the I2C bus.
                        The module must have been configured appropriately and enabled, a transfer
                        must have been previously started (either by the I2C module or by an
                        external master), and module must be the intended receiver of the next byte
                        of data.
BOOL I2CReceivedDataIsAvailable ( i2c_modULE id );  The I2C module must have been configured appropriately and enabled, and a
                          transfer must have been previously started.
BOOL I2CReceivedByteIsAnAddress ( i2c_modULE id )  The module must have been configured appropriately and enabled, and a
                          transfer must have been previously started.
BYTE I2CGetByte ( i2c_modULE id )  The module must have been configured appropriately and enable, a transfer
                  must have been previously started, and a byte of data must have been
                  received from the I2C bus.
 void I2CAcknowledgeByte ( i2c_modULE id, BOOL ack )The module must have been configured appropriately and enabled, a transfer
                          must have been previously started, and a byte of data must have been
                          received from the I2C bus.
BOOL I2CAcknowledgeHasCompleted ( i2c_modULE id )
I2C_STATUS I2CGetStatus ( i2c_modULE id )   This routine provides a bitmask of the current status flags for the I2C module.
void I2CClearStatus ( i2c_modULE id, I2C_STATUS status )

Write:
1. Send a start sequence
2. Send the I2C address of the slave with the R/W bit low (even address)
3. Send the internal register number you want to write to
4. Send the data byte
5. [Optionally, send any further data bytes]
6. Send the stop sequence.
Read:
1. Send a start sequence
2. Send 0xC0 ( I2C address of the CMPS03 with the R/W bit low (even address)
3. Send 0x01 (Internal address of the bearing register)
4. Send a start sequence again (repeated start)
5. Send 0xC1 ( I2C address of the CMPS03 with the R/W bit high (odd address)
6. Read data byte from CMPS03
7. Send the stop sequence

Look up: OpenI2C1() function (other one-liners?)

*****************************************************************************/
void i2cConfigModule1(void)
{
  unsigned int actualClock = 0;

  I2CConfigure( I2C1, 0 );  //I2C_CONFIG_FLAGS should be zero; no config bits seem to apply to normal use (test)
  actualClock = I2CSetFrequency(I2C1, PERIPHERAL_CLOCK, I2C_CLOCK_FREQ);
    /*if ( abs(actualClock-I2C_CLOCK_FREQ) > I2C_CLOCK_FREQ/10 )
    {
      putsUART1("I2C1 clock frequency (%ld) error exceeds 10%%\n", actualClock);
    }*/

  I2CClearStatus ( I2C1, I2C_ARBITRATION_LOSS | I2C_SUCCESS | I2C_ERROR | I2C_MASTER_BUS_COLLISION | I2C_RECEIVE_OVERFLOW | I2C_TRANSMITTER_OVERFLOW ); //second param selects flags to clear (all of them)
  I2CEnable( I2C1, TRUE );    /*can only enable I2C connection after timer is configured:
                  The module should be appropriately configured (using I2CConfigure and
                  I2CSetFrequency, and possibly I2CSetSlaveAddress) before being enabled.*/
  //i2cStopTransfer();

  //should clear the i/o buffers here
  I2CReceiverEnable(I2C1, FALSE);
}

/*******************************************************************************
  Function:
    BOOL i2cStartTransfer( BOOL restart )
  Summary:
    Starts (or restarts) a transfer to/from the device.
  Description:
    This routine starts (or restarts) a transfer to/from the device, waiting (in
    a blocking loop) until the start (or re-start) condition has completed.
  Precondition:
    The I2C module must have been initialized.
  Parameters:
    restart - If FALSE, send a "Start" condition
            - If TRUE, send a "Restart" condition\
  Returns:
    TRUE    - If successful
    FALSE   - If a collision occured during Start signaling
  Example:
    <code>
    i2cStartTransfer(FALSE);
    </code>
  Remarks:
    This is a blocking routine that waits for the bus to be idle and the Start
    (or Restart) signal to complete.
  *****************************************************************************/
BOOL i2cStartTransfer( BOOL restart )
{
    I2C_STATUS  result;
  int i = 0;

  // Send the Start (or Restart) signal
  if(restart){
    //I2CBusIsIdle(I2C1) <--calling this is inappropriate here; this call is for initiating transfers only

    result = I2CRepeatStart(I2C1);

    //this error condition occurs frequently in single-master mode when
    //the MCU is not turned off and back on before resetting the program.
    //Only the caller can resolve this failure for repeatStart, since they
    //must restart the communication/device-addressing sequence.
        if(result == I2C_MASTER_BUS_COLLISION){
          putsUART1("i2cRepeatStart() failed due to I2C_MASTER_BUS_COLLISION\r\n");
          return FALSE;
        }
  }
  else{  
    // Wait for the bus to be idle, then start the transfer
    while( !I2CBusIsIdle(I2C1) && (i < 10)){
      //putsUART1("Waiting for idle i2c bus, before i2cstart()\r\n");
      //putsUART1("Waiting...\n\r");
      i++;
    }

    result = I2CStart(I2C1);

    //this error condition occurs frequently in single-master mode when
    //the MCU is not turned off and back on before reseting the program.
    //I dont know how to handle such collisions, so this is brute force:
    if(result == I2C_MASTER_BUS_COLLISION){
      I2CStop(I2C1);  //send STOP and restart the system
      I2CEnable(I2C1, FALSE);
      for(i = 0; i < 256; i++){
        asm("nop");
      }
      I2CClearStatus ( I2C1, I2C_ARBITRATION_LOSS | I2C_SUCCESS | I2C_ERROR | I2C_MASTER_BUS_COLLISION | I2C_RECEIVE_OVERFLOW | I2C_TRANSMITTER_OVERFLOW ); //second param selects flags to clear (all of them)
      I2CEnable(I2C1, TRUE);

      result = I2CStart(I2C1);
      if(result == I2C_MASTER_BUS_COLLISION){
        putsUART1("I2C_START FAILURE");
        return FALSE;
      }
    }
  }

  //hastily pulled out a lot of these wait-loops, some of which appeared infinite at times.
  //this one was necessary, else a master bus collision would occur in ALL cases, not just the common one when the board not power-cycled
  do { 
    result = I2CGetStatus(I2C1);  //Wait for the signal to complete
  } while ( !(result & I2C_START) );

  return TRUE;
}

void i2cSendAck(I2C_MODULE i2c_mod, BOOL ack)
{
  //I2C_STATUS  status;

  I2CAcknowledgeByte(i2c_mod,ack);

  while(!I2CTransmissionHasCompleted(i2c_mod));
}

void i2cStopTransfer( void )
{
    I2C_STATUS status = 0;

    // Send the Stop signal
    I2CStop(I2C1);
    //delay(10);
    
    /*  This check was problematic. Sometimes this function would never return.
    // Wait for the signal to complete
  do
    { status = I2CGetStatus(I2C1);
    } while ( !(status & I2C_STOP) ); <--I suspect I2C_STOP. This bit *detects* a stop condition, which doesn't seem like something a master would receive.
    why would master want to detect its own stop condition? I2C_STOP does not appear to detect a *stop* state on the bus, but rather the stop signal itself.
    ...Which is also relative to the receiver, which means recevier_enabled must be TRUE.
  */
}

BOOL I2CTransmitByte( I2C_MODULE i2c_mod, unsigned char data )
{
  int i = 0;

  while(!I2CTransmitterIsReady(i2c_mod));

  if(I2CSendByte(i2c_mod, data) == I2C_MASTER_BUS_COLLISION){
    putsUART1("ERROR I2C Master Bus Collision\n\r");
    return FALSE;
  }

  while(!I2CTransmissionHasCompleted(i2c_mod));

  while( !I2CByteWasAcknowledged(I2C1) && (i < 5) ){
    putsUART1("Waiting for ACK in TxByte()\n\r");
    i++;
    delay(TINY_DLY);
  }

  if(i >= 5){
    putsUART1("ERROR I2CTransmitByte failed\n\r");
    return FALSE;
  }

  return TRUE;
}

void putI2CResult(I2C_RESULT resultVal)
{
  switch(resultVal)
  {
    case I2C_SUCCESS:
      putsUART1("I2C_SUCCESS\n\r\0");
      break;
    case I2C_ERROR:
      putsUART1("I2C_ERROR\n\r\0");
      break;
    case I2C_MASTER_BUS_COLLISION:
      putsUART1("I2C_MASTER_BUS_COLLISION\n\r\0");
      I2C1STAT &= 0xFBFF;
      break;
    case I2C_RECEIVE_OVERFLOW:
      putsUART1("I2C_RECEIVE_OVERFLOW\n\r\0");
      break;
    default:
      putsUART1("WARN: I2C_RESULT error code unknown!\n\r\0");
      break;
  }
}

/*******************************************************************************
  Function:
    BOOL I2CReceiveByte( unsigned char* data )

  Summary:
     Get one byte of data.

  Description:
    This routine checks for recieve overflow if not if data is available get
 *  byte.

  Precondition:
    The I2C module must have been initialized & a transfer started.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    if(!RecieveOneByte(data))
 *    Success = 0;
    </code>

  Remarks:
    This is a blocking routine that waits for the Stop signal to complete. FROM ONLINE AT http://www.edaboard.com/thread248565.html
  *****************************************************************************/
BOOL I2CReceiveByte(unsigned char* data)
{
    if (!I2CReceiverEnable(I2C1, TRUE) == I2C_RECEIVE_OVERFLOW)
    {    return FALSE;  }
    else
    {
        while(!I2CReceivedDataIsAvailable(I2C1));
        *(data) = I2CGetByte(I2C1);
    }

    return TRUE;
}

void clearStr(unsigned char str[], int len)
{
  int i = 0;

  while( i < len ){
    str[i++] = '\0';
  }
}

/****************************************************************
Function: UART1Handler


****************************************************************/
void __ISR(_UART_1_VECTOR, IPL6AUTO) UART1Handler(void) // SW1
{

   if (INTGetFlag(INT_SOURCE_UART_RX(UART1)))
    {
      // Echo what we just received.
    //UARTSendDataByte(UART1, UARTGetDataByte(UART1));
    //UARTSendDataByte(UART1, '<-dR ');

      // Clear the RX interrupt Flag
    INTClearFlag(INT_SOURCE_UART_RX(UART1));

      // Toggle LED to indicate UART activity
      //mPORTBToggleBits(BIT_0 | BIT_1 |BIT_2 | BIT_3);
    PORTToggleBits(IOPORT_B, BIT_0);
    }
  // We don't care about TX interrupt
  else if ( INTGetFlag(INT_SOURCE_UART_TX(UART1)) )      //if-else logic should also define/include UI_ERR interrupt handling
    {
    INTClearFlag(INT_SOURCE_UART_TX(UART1));
    PORTSetBits(IOPORT_B, BIT_3);
    }
  else
  {
    INTClearFlag(INT_SOURCE_UART_TX(UART1));    //need case to clear all U1 interrupts
    INTClearFlag(INT_SOURCE_UART_RX(UART1));
    //PORTSetBits(IOPORT_B, BIT_1 | BIT_3 );
  }
}

//Vector numbers for handlers are from p32mx460f512l.h file
void __ISR(_INPUT_CAPTURE_2_VECTOR, IPL5AUTO) IC2Handler(void) // SW1
{
  int myData = 0;

  /*Reading from buffers is how we clear them. As such, this loop is necessary
  to clear the IC buffer.  Otherwise, the buffer will always be non-empty, so
  the interrupt will continually fire! */
  while( mIC2CaptureReady() )
  {
    myData = mIC2ReadCapture();
  }

  PORTToggleBits(IOPORT_B, BIT_1);
  INTClearFlag(INT_IC2);
  //mIC2ClearIntFlag();
}


//Vector numbers for handlers are from p32mx460f512l.h file
void __ISR(_INPUT_CAPTURE_3_VECTOR, IPL5AUTO) IC3Handler(void) // SW1
{
  int myData = 0;

  /*Reading from buffers is how we clear them. As such, this loop is necessary
  to clear the IC buffer.  Otherwise, the buffer will always be non-empty, so
  the interrupt will continually fire! */
  while( mIC3CaptureReady() )
  {
    myData = mIC3ReadCapture();
  }

  PORTToggleBits(IOPORT_B, BIT_2);
  INTClearFlag(INT_IC3);
  //mIC3ClearIntFlag();
}

/*
//MPLab example.  Single interrupt for UART, which internally maps Tx/Rx/Err interrupt source
void __ISR(_UART1_VECTOR, IPL2AUTO) IntUART1Handler(void)
{
  // Is this an RX interrupt?
  if (INTGetFlag(INT_SOURCE_UART_RX(UART1)))
    {
      // Echo what we just received.
    UARTSendDataByte(UART1, UARTGetDataByte(UART1));
    UARTSendDataByte(UART1, 'R');

      // Clear the RX interrupt Flag
    INTClearFlag(INT_SOURCE_UART_RX(UART1));

      // Toggle LED to indicate UART activity
      //mPORTBToggleBits(BIT_0 | BIT_1 |BIT_2 | BIT_3);
    PORTToggleBits(IOPORT_B, BIT_0);
    }
  // We don't care about TX interrupt
  else if ( INTGetFlag(INT_SOURCE_UART_TX(UART1)) )      //if-else logic should also define/include UI_ERR interrupt handling
    {
    INTClearFlag(INT_SOURCE_UART_TX(UART1));
    PORTWrite(IOPORT_B, 0x0802);
    }
  else
  {
    INTClearFlag(INT_SOURCE_UART_TX(UART1));    //need case to clear all U1 interrupts
    INTClearFlag(INT_SOURCE_UART_RX(UART1));
    PORTWrite(IOPORT_B, 0x280A);
  }
}
*/

/*
void __attribute__((interrupt,no_auto_psv)) _U1RXInterrupt(void)
{
  IFS0bits.U1RXIF = 0;       // clear TX interrupt flag

  unsigned int LED_state = 0;

  LED_state = PORTRead(IOPORT_B);

  LED_state &= 0x3C00;

  if(LED_state == 0)
    PORTWrite(IOPORT_B, 0x3C00);
}
*/

/*
 void __attribute__((interrupt, no_auto_psv)) __U1RXInterrupt(void){
 IFS0bits.U1RXIF=0;                                                     //      Clear flag
 mLED_5_Toggle();
 }
*/
/*
void __ISR(_EXTERNAL_1_VECTOR, ipl7) INT1Handler(void) // U1Rx interrupt
{

  if (LED_value == 1) // // Is LED1 on?
  {
    flag = 1;
    player1Score++;
  }

  mINT1ClearIntFlag ();
}
*/

/*
    Left motor HB5:   (PIC)  (Cerebot)
              OC3  ->  RD2  (output waveform toggles the H-bridge enable)
              Dir  ->  RD6  (direction)
              RD10->   IC3
              Vdd  ->  Vdd
              Gnd  ->  Gnd

    Right motor HB5:   (PIC)  (Cerebot)
              OC2  ->  RD1
              Dir  ->  RD7  (direction)
              RD9 ->   IC2
              Vdd  ->  Vdd
              Gnd  ->  Gnd
*/
//Common bug: writing direction to direction register; remember the logic state of the pins determines direction, so write to PORTx reg, not TRISx.
void setMotorsReverse(void)
{
  SetPulseOC2(DC_ZeroPct, DC_ZeroPct);
  SetPulseOC3(DC_ZeroPct, DC_ZeroPct);
  delay(TINY_DLY);

  PORTWrite(IOPORT_D, BIT_7);
  PORTClearBits(IOPORT_D, BIT_6);

  /*
  //PORTSetPinsDigitalOut(IOPORT_D, BIT_6);    //Invert bits 6 and 7 on RD port
  PORTWrite(IOPORT_D, BIT_7 | BIT_6);
*/
  }

void setMotorsForward(void)
{
  SetPulseOC2(DC_ZeroPct, DC_ZeroPct);
  SetPulseOC3(DC_ZeroPct, DC_ZeroPct);
  delay(TINY_DLY);
  PORTWrite(IOPORT_D, BIT_6);
  PORTClearBits(IOPORT_D, BIT_7);

  /*
  PORTSetPinsDigitalOut (IOPORT_D, BIT_7 | BIT_6);
  //PORTSetPinsDigitalIn (IOPORT_D, BIT_6);
  */
}

/*******************************************************************
 Function: void winSequence(void)
 * Returns: void
 * Input Parameters: void
 * OutParams: void
 * Description: Outputs a friggin awesome win sequence.
 ********************************************************************/
void winSequence(void)
{
    short count = 0;

    while(count < 12)
    {
        PORTClearBits(IOPORT_B, BIT_1 | BIT_3);
        PORTSetBits(IOPORT_B, BIT_0 | BIT_2);
        delay(LONG_DLY);
        clearLEDs();
        PORTClearBits(IOPORT_B, BIT_0 | BIT_2);
        PORTSetBits(IOPORT_B, BIT_1 | BIT_3);
        delay(LONG_DLY);
        clearLEDs();
        count++;
    }
}

/*
  Delay for some matter of milliseconds based on system clock.
  Needs calculation.
*/
void delayMS(int ms)
{
  long int i = 0, j = 0, clkPerMs;

  clkPerMs =  0.001 * SYSTEM_CLOCK;

  //really this should execute for ms * 5 times, due to comparison and increment ops in the loops
  for(i = 0; i < ms; i++){  //outer loop count ms
    for(j = 0; j < clkPerMs; j++){  //inner loop iterates (0.001 seconds) number of times
      Nop();
    }
  }
}


/*******************************************************************
 Function: void loseSequence(void)
 * Returns: void
 * Input Parameters: void
 * OutParams: void
 * Description: This function displays a lose sequence for big losers.
 ********************************************************************/
void loseSequence(void)
{
    unsigned int new_state = 0;
    short i = 0, j = 0;

    while(i < 4)
    {
        while(j < 4)
        {
            output_led(j % 4);
            delay(SHORT_DLY);
            j++;
        }

        clearLEDs();
        delay(TINY_DLY);
        j = 3;

        while(j >= 0)
        {
        //scroll left
            output_led(j % 4);
            delay(SHORT_DLY);
            j--;
        }
        clearLEDs();
        delay(TINY_DLY);
        i++;
    }

    clearLEDs();
}

/****************************************************************************************
 Function: void delay(unsigned int ms)
 Returns: void
 Input Parameters: u_int ms (amount of delay)
 Description: This function delays operations for a specified number of instruction cycles, by passing the assembly instruction
 NOP (no-operation) to the system. This causes the CPU to execute no-operation for the number of cycles specified by the
 input parameter "ms", an unsigned integer type.  Recall the max size of an integer is 65,535 and the PIC CPU runs at around
 80 MHz, or 80 million cycles per second.  So if we pass 65535 into this function, the maximum delay is about 65535 / 80 million,
 or 0.8 milliseconds.  So how do we get a longer delay?  Simply nest another loop inside the main loop of this function to get
 ms^2 (ms squared) performance, or likewise you can call the function itself from within a loop.
 Usages: delaying, debouncing (see wikipedia "debouncing")
 Preconditions: None
 Postconditions: A timed delay occurs
 ****************************************************************************************/
void delay(unsigned int ms)
{
    unsigned int count = 0;

    for(count = 0; count < ms; count++)
    {
        asm("NOP");
    }

    /* Alternative counter loop template for longer delays: nested loops can square the number of overall loop iterations,
       so this loop will execute a maximum of 65535 * 65535 times, or about 4 billion times.  Dividing this value by our
       clock frequency of 80 MHz gives a delay of around 53 seconds, clearly a very long (maximum) delay.

    unsigned int count = 0;
    unsigned int inner_count = 0;

    for(count = 0; count < ms; count++)
    {
      for(inner_count = 0; inner_count < ms; inner_count++)
      {
        asm("NOP");
      }
    }
    */
}


/*****************************************************************************************
 Function: configurePortIO(void)
 Description: This function sets up the pins to the on-board LEDs as output pins

 Notes: may need to call AD1CFG, as in original assembly, to set analog ports to digital???

    Mapping: Arrows indicate software/PIC mapping to physical ports on Cerebot board.
    OCx (output compare) port-locations drive the motors.

    Left motor HB5:   (PIC)  (Cerebot)
              OC3  ->  RD2  (output waveform toggles the H-bridge enable)
              Dir  ->  RD6  (direction)
              IC3 ->   RD10
              Vdd  ->  Vdd
              Gnd  ->  Gnd

    Right motor HB5:   (PIC)  (Cerebot)
              OC2  ->  RD1
              Dir  ->  RD7  (direction)
              IC2 ->  RD9
              Vdd  ->  Vdd
              Gnd  ->  Gnd

    UART(B-Tooth):  (PIC)  (Cerebot)
              U1TX ->  RF02
              U1RX ->  RF08

    I2C1:     SCL1 -> RA14
              SDA1 -> RA15
              From the PIC32 IC2 manual (sect 24):
              In I2C mode, pin SCL is clock and pin SDA is data. The module will override the data direction
              bits (TRIS bits) for these pins.

 *****************************************************************************************/
void configurePortIO (void)
{
  unsigned int portState = 0;

  //Ping Sensor configuration. Uses Peripheral ports RE02 and RE03 just as basic ports (no special PIC internal peripherals)
    PORTSetPinsDigitalOut (IOPORT_E, BIT_3);
    PORTSetPinsDigitalIn (IOPORT_E, ~BIT_3);  //set all remaining RE pins as inputs
    PORTWrite(IOPORT_E, ~BIT_3);
    
  //Motor configuration:   SET: BIT_7 (RD7)    CLEAR: BIT_6 (RD6), BIT_2 (OC3/RD2), BIT_1 (OC2/RD1)
    PORTSetPinsDigitalOut (IOPORT_D, BIT_7 | BIT_6 | BIT_2 | BIT_1);
    PORTSetPinsDigitalIn (IOPORT_D, BIT_10 | BIT_9);

  //set BIT_6 to 1 (left motor direction), BIT_7 (and all others) to 0 (right motor direction)
    PORTWrite(IOPORT_D, BIT_6);
    PORTClearBits(IOPORT_D, BIT_7);

  //UART configuration: Set BIT_8 to 1 (RF02), clear BIT_2 to 0 (RF08)
    PORTSetPinsDigitalOut (IOPORT_F, BIT_8);
    PORTSetPinsDigitalIn (IOPORT_F, BIT_2);

  //LED signal configuration: set all Port_B as output. BIT_3 output is for HC-SR04 sensor TRIG signal.
    PORTSetPinsDigitalOut (IOPORT_B, BIT_0 | BIT_1 | BIT_2 | BIT_3 | BIT_13);
    PORTSetPinsDigitalIn (IOPORT_B, BIT_12 );  //BIT_12 input is for ECHO signal from HC-SR04

  //I2C1 setup    SCL1 = RA14    SDA1 = RA15
    PORTSetPinsDigitalIn(IOPORT_A, BIT_14 | BIT_15);  //see header: the manual says this I/O will be appropriately overridden by the IC2 module itself

  //SPI setup for digilent pmod wifi
    PORTSetPinsDigitalOut(IOPORT_G, BIT_6 | BIT_8 | BIT_9 );  //Set SDO1, SCLK, SS lines output
    PORTSetPinsDigitalOut(IOPORT_B, BIT_14 | BIT_15 );        //Set all the rest of em to outputs to ignore...
    PORTSetPinsDigitalOut(IOPORT_D, BIT_4 | BIT_5 );        // these too.
    PORTSetPinsDigitalIn(IOPORT_G, BIT_7 );  //Set SDI1 to input at RG07
    //now initialize the spi port values, make sure they're low
    PORTGbits.RG6       = 0;  //port G
    PORTGbits.RG8       = 0;
    PORTGbits.RG9       = 0;
    PORTBbits.RB14       = 0; //port B
    PORTBbits.RB15       = 0;
    PORTDbits.RD4       = 0;  //port D
    PORTDbits.RD5       = 0;

  //Also set RB15, RD05, RD4, and RB14 to inputs.  These are four additional wifi mod signals.

}

/*
  Line mode config: Enabling UART transmissions will immediately cause a TX interrupt to
    indicate that the transmitter needs data, unless the transmitter FIFO/buffer
    was pre-loaded with data.

  #define PERIPHERAL_CLOCK    10000000
  #define DESIRED_DATA_RATE   19200

  unsigned int    actualDataRate;

    actualDataRate = UARTSetDataRate(UART1, PERIPHERAL_CLOCK, DESIRED_DATA_RATE);

  Also useful:
  unsigned int UARTSetDataRate ( UART_MODULE id, unsigned int sourceClock, unsigned int dataRate );
  unsigned int UARTGetDataRate ( UART_MODULE id, unsigned int sourceClock );
  BOOL UARTTransmitterIsReady ( UART_MODULE id );
  void UARTSendDataByte ( UART_MODULE id, BYTE data )
  void UARTSendData ( UART_MODULE id, UART_DATA data )
      UART_DATA data = 0x1ff; //can be 8-bit or 9-bit word, depending on how UART is configured

      if (UARTTransmitterIsReady(UART1))
      {
        UARTSendData(UART1, data);
      }
  BOOL UARTReceivedDataIsAvailable ( UART_MODULE id )
  BOOL UARTTransmissionHasCompleted ( UART_MODULE id )
  BYTE UARTGetDataByte ( UART_MODULE id )
  UART_DATA UARTGetData ( UART_MODULE id )
  void UARTSendBreak ( UART_MODULE id )
  void UARTStartAutoDataRateDetect ( UART_MODULE id )

*/
void configureUART1(void)
{
  unsigned int    actualDataRate;

  //void UARTConfigure ( UART_MODULE id, UART_CONFIGURATION flags );
    UARTConfigure ( UART1, UART_ENABLE_PINS_CTS_RTS | UART_RTS_WHEN_RX_NOT_FULL);

  //UART_INTERRUPT_ON_TX_NOT_FULL is default mode
    UARTSetFifoMode( UART1, UART_INTERRUPT_ON_RX_NOT_EMPTY | UART_INTERRUPT_ON_TX_NOT_FULL );

    UARTSetLineControl( UART1, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1 );

  //unsigned int UARTSetDataRate ( UART_MODULE id, unsigned int sourceClock, unsigned int dataRate )
    actualDataRate = UARTSetDataRate ( UART1, PERIPHERAL_CLOCK, DESIRED_DATA_RATE );

  //ConfigIntUART1(UART_RX_INT_EN | UART_TX_INT_DIS | UART_ERR_INT_EN | UART_INT_PR0 | UART_INT_SUB_PR0);
    ConfigIntUART1(UART_RX_INT_EN | UART_TX_INT_DIS | UART_ERR_INT_DIS | UART_INT_PR6 | UART_INT_SUB_PR3 | UART_TX_INT_PR0);
}

//Andy version
void setupUART1 (unsigned int pb_clock)
{
  // OpenUART1 (config1, config2, ubrg)
  OpenUART1 (UART_EN | UART_IDLE_CON | UART_RX_TX | UART_DIS_WAKE | UART_DIS_LOOPBACK | UART_DIS_ABAUD | UART_NO_PAR_8BIT | UART_1STOPBIT | UART_IRDA_DIS |
               UART_MODE_FLOWCTRL | UART_DIS_BCLK_CTS_RTS | UART_NORMAL_RX | UART_BRGH_SIXTEEN,
               UART_TX_PIN_LOW | UART_RX_ENABLE | UART_TX_ENABLE | UART_INT_TX | UART_INT_RX_CHAR | UART_ADR_DETECT_DIS  | UART_RX_OVERRUN_CLEAR,
         mUARTBRG(pb_clock, DESIRED_DATA_RATE));
}

/*
  Remarks:
    Enabling the UART trasnmitter may cause an immediate UART TX interrupt
    request (if the UART TX interrupt is enabled), unless the transmit buffer
    has been pre-loaded with data.
*/
void enableUART1(void)
{
  UARTEnable( UART1, UART_ENABLE_FLAGS(UART_ENABLE | UART_PERIPHERAL | UART_RX | UART_TX));
}

void configureTimer2(int Period)
{
  OpenTimer2(T2_ON | T2_PS_1_2, Period);
  WriteTimer2(0);
}

void configureTimer4(int Period)
{
  OpenTimer4(T4_ON | T4_IDLE_CON | T4_32BIT_MODE_OFF | T4_PS_1_8, Period);
  ConfigIntTimer4( T4_INT_OFF );
  WriteTimer4(0);
}

//void OpenOC1( config, value1, value2)
//Ex.: OpenOC1( OC_IDLE_STOP | OC_TIMER_MODE32 | OC_PWM_FAULT_PIN_DISABLE, OC_SINGLE_PULSE , 0x0000, 0xFFFF )
void configOCModule2(void)
{
  OpenOC2(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);
}

void configOCModule3(void)
{
  OpenOC3(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);
}

//unsigned int myInt = ReadDCOC3PWM(); <--gets duty cycle
//unisgned int myInt = ReadRegOC3()  preferable
// SetPulseOC1(0xFF, 0xFFFF)  I believe first val is desired OCxRS, second is desire PR
void writeOC2(unsigned int newOC2RValue, unsigned int newOC2RSValue)
{
  SetPulseOC2(newOC2RValue, newOC2RSValue);  //returns OCxR or OCxRS as u_int
}

void writeOC3(unsigned int newOC3RValue, unsigned int newOC3RSValue)
{
  SetPulseOC3(newOC3RValue, newOC3RSValue);  //returns OCxR or OCxRS as u_int
}
/*
INT_IC2  //from int_3xx_4xx.h file -> all INT_SOURCE values
INT_IC3
INT_U1RX
INT_U1TX

void INTGetFlag(INT_SOURCE source);  int.h
void INTSetFlag(INT_SOURCE source);
void INTClearFlag(INT_SOURCE source);
unsigned int INTGetInterruptVectorNumber(void)
void INTEnable(INT_SOURCE source, INT_EN_DIS enable);
void INTSetVectorPriority(INT_VECTOR vector, INT_PRIORITY priority);
INT_PRIORITY INTGetVectorPriority(INT_VECTOR vector);

C:\Program Files (x86)\Microchip\xc32\v1.20\pic32mx\include\peripheral\int.h
*most macros and types are in int_3xx_4xx.h; priority levels are in int.h;
*vector numbers/mapping is in C:\Program Files (x86)\Microchip\xc32\v1.20\pic32mx\include\proc\p32mx460f512l.h

*/
//configure IC2, IC3, and U1RX interrupts
//elected instead to do interrupt configuration within its associated module (IC, UARt, etc) config
void configureInterrupts(void)
{
  INTDisableInterrupts();
  //mClearAllIECRegister();

  INTEnableSystemMultiVectoredInt();

  INTSetVectorPriority(INT_UART_1_VECTOR, INT_PRIORITY_LEVEL_6);    //set priority to 6, all ints
  INTEnable(INT_UART_1_VECTOR, INT_ENABLED);

  //mU1SetIntPriority(INT_PRIORITY_LEVEL_6);
  //mU1TXIntEnable(INT_DISABLE); //disable the TX interrupt   Old Crap

  INTClearFlag(INT_U1RX);
  INTClearFlag(INT_U1TX);
  INTClearFlag(INT_U1E);
  INTEnableInterrupts();
}

/*
interrupt on every 4th capture, where 4 sigs needed for each capture, so 16 signals = 16/19 turns of the wheel
*/
void configureInputCaptureMods(void)
{
  INTDisableInterrupts();

  OpenCapture2( IC_ON | IC_IDLE_STOP | IC_FEDGE_RISE | IC_TIMER2_SRC | IC_INT_4CAPTURE | IC_EVERY_4_RISE_EDGE);
  OpenCapture3( IC_ON | IC_IDLE_STOP | IC_FEDGE_RISE | IC_TIMER2_SRC | IC_INT_4CAPTURE | IC_EVERY_4_RISE_EDGE);

  ConfigIntCapture2(IC_INT_ON | IC_INT_PRIOR_5 | IC_INT_SUB_PRIOR_3);
  ConfigIntCapture3(IC_INT_ON | IC_INT_PRIOR_5 | IC_INT_SUB_PRIOR_2);

  INTClearFlag(INT_IC2);
  INTClearFlag(INT_IC3);

  INTEnableInterrupts();
}

void sigError(void)
{
  int i = 0, j = 0;

  for(i = 0; i < 5; i++)
  {
    PORTSetBits(IOPORT_B, BIT_0 | BIT_1 | BIT_2 | BIT_3);
    for(j = 0; j < 1; j++){
      delay(LONG_DLY);
    }
    PORTClearBits(IOPORT_B, BIT_0 | BIT_1 | BIT_2 | BIT_3);
    for(j = 0; j < 1; j++){
      delay(LONG_DLY);
    }
  }
}

void spiInitialize(void)
{
  unsigned char dummy = 0;

  SPI2CONbits.ON      = 0;        // disable SPI to reset any previous state
  dummy               = SPI2BUF;  // clear receive buffer
  
  /*
    See PIC32 peripheral ref man: spiClkFreq = PBCLK / (2 * (SPIxBRG + 1))
    So for 1 MHz operation, SPIxBRG == 19
  */
  SPI2BRG             = 0x10011;  // 0x10011 == 19

  SPI2CONbits.FRMEN   = 1;        // disable frame mode 
  SPI2CONbits.MSTEN   = 1;        // enable master mode
  SPI2CONbits.CKE     = 1;        // set clock-to-data timing
  SPI2CONbits.SMP     = 1;        // input data is sampled at end of data output time
  SPI2CONbits.ON      = 1;        // turn SPI on
  SPI2CONbits.MODE32     = 1;        // set clock-to-data timing

  //May choose to ignore this: selectable 8, 16, or 32 bit data mode. 16 bit may be preferrable for IMU packet (?).
  //SPI2CONbits.MODE     = 1;        // set clock-to-data timing

  // Signal G9 is on the same pin as SPI2SS (Slave Select) signal.
  // Since slave select is not used in this configuration
  // the pin is used as a reset signal for the external SPI slave.

  TRISGbits.TRISG9    = 0;
  PORTGbits.RG9       = 0;
  delayMS(300);
  PORTGbits.RG9       = 1;
  delayMS(300);
}

//clear on-board and peripheral LEDs
void clearLEDs(void)
{
  PORTClearBits(IOPORT_B, BIT_0 | BIT_1 | BIT_2 | BIT_3);
}

/*******************************************************************
 Function: void output_led (short index)
 * Returns: void
 * Input Parameters: short index
 * OutParams: void
 * Description: Thsi function outputs a single led based on an integer
 * index.  The index is simply to make the program more readable and
 * traceable than using bytes to store the state of four leds.
 ********************************************************************/
//NOTE: ouput was reversed (currently not), 0->bit4...3->bit0
void output_led (short index)
{
  unsigned int led_states = 0x00000000;

        switch(index){
        case 0:
            led_states = BIT_0;
            break;
        case 1:
            led_states = BIT_1;
            break;
        case 2:
            led_states = BIT_2;
            break;
        case 3:
            led_states = BIT_3;
            break;
        case 4:
            led_states = BIT_0 | BIT_1 | BIT_2 | BIT_3;
            break;
        default:
            led_states = BIT_0 | BIT_1 | BIT_2 | BIT_3;
            break;
        }

        PORTSetBits(IOPORT_B, led_states);
}


/*
  MRF24WB0MA Notes:
  -The antenna must be mounted away from other copper material.  Apparently
   external antennas are available for greater range.
  -Device can accept clk sped up to 25 mHz.
  -Otherwise the data sheet suggests no device specific configuration, just use it
   as a regular SPI device.

*/
void MRF24WB0MAinitialize()
{
  //device can accept clk signal of up to 25 mHz



}

//The driver for both sending and receiving, since master/slave data line is a ring / circular buffer
char spiTransfer (char c)
{

/*
  Example of sending a command to a slave and reading its response:

  LATBbits.LATB4 = 0;                  // select slave device
  SPI1_transfer( tx_data);             // transmit byte tx_data
  char response = SPI1_transfer( 0);   // send dummy byte and read response
  LATBbits.LATB4 = 1;                  // release slave device
*/

  LATBbits.LATB4 = 0;            // select slave device
  SPI2BUF = c;                   // send data to slave
  while (SPI2STATbits.SPIBUSY);  // wait until SPI transmission complete
  LATBbits.LATB4 = 1;            // release slave device
  return SPI2BUF;
}

char spiGetc(void)
{
    return spiTransfer(0);
}

char spiPutc (char c)
{
    return spiTransfer(c);
}

void spiPuts (char *s)
{
  for( ; *s != '\0'; s++){
    spiPutc (*s);
  }
}
////////////////////////// end spi code /////////////////////////////



/*
  Unfinished.  This code was intended for an ultrasonic ping sensor, but it was
  found the 5v sensor was incompatible with the 3.3V cerebot board.

  PBClock runs at 40mhz, and timer 4 at 40 / 8 (prescaler), or 5Mhz.
  BIT_13 of IOPORT_B is output / TRIG; BIT_12 as input / ECHO-in.
*/
unsigned int ping(void)
{
  unsigned int pingClk = 0, dt = 0, echo = 0, i = 0, j = 0,k = 0;
  char debugStr[128] = {'\0'};
  unsigned int t[16] = {0};
  unsigned int period = 65000, lastVal = 0, currVal = 0;
  
  //start timer
  configureTimer4(period);
  
  //precon: send short low pulse so high pulse is clean/discrete
  PORTClearBits(IOPORT_B, BIT_13 );
  delay(SHORT_DLY);
  
  //read current clock val and trigger a ping
  //pingClk = ReadTimer4();
  PORTSetBits(IOPORT_B, BIT_13 ); //ping!
  
  // Delay 10 microsec while BIT_3 high. At 80Mhz, 10 microseconds == 800 clocks
  //for(i = 0; !(PORTRead(IOPORT_E) & 0x00000004); i++){ // 267 = 800 / 3, since loop contains three ops per iteration: <, ++, and nop
  for(i = 0; (i < 4096) && !(PORTRead(IOPORT_B) & BIT_12); i++){ // 267 = 800 / 3, since loop contains three ops per iteration: <, ++, and nop
    asm("nop");
  }
  
  PORTClearBits(IOPORT_B, BIT_13); //end ping
  
  //wait for echo segment to begin
  while( !(PORTRead(IOPORT_B) & 0x00001000) ){k++;}
  while( PORTRead(IOPORT_B) & 0x00001000 ){
    //currVal = ReadTimer4();
    //dt += currVal - lastVal;
    //lastVal = currVal;
    j++;
  }
  
  /*
  //wait until echo line goes high; it stays high until echo is received, or 30 ms, whichever is first
  while( !(PORTRead(IOPORT_E) & 0x00000004) && (i < 65000) ){
    //putsUART1("Waiting...\n\r");
    i++;
  }
  */
  
/*
  //loop while ping-echo falling edge not detected.  This is low resolution.
  do{
  
    currVal = ReadTimer4();
    if(currVal < lastVal){
      dt = currVal + period - lastVal;
    }
    else{
      dt += currVal - lastVal;
    }
    lastVal = currVal;
  
    echo = PORTRead(IOPORT_B) & BIT_12;
    j++;
  }while( echo && (j < 10000) );
  */
  sprintf(debugStr,"dt: %08X T4: %08X i: %d j: %d k: %d\n\r",dt,ReadTimer4(),i,j,k);
  putsUART1(debugStr);
  
  /*
  i = 0;
  while(i < 10){
    t[i++] = ReadTimer4();
  }
  sprintf(debugStr,"%6X %6X %6X %6X\n\r%6X %6X %6X %6X\n\r",t[0],t[1],t[2],t[3],t[4],t[5],t[6],t[7]);
  putsUART1(debugStr);
  */
  
  CloseTimer4();
  
  return dt;
}



/* 
  Uses polling to detect distance, using the HC-SR04 ping sensor.
  Returns the positive time delay between ping and ping-echo
*/
/*
unsigned int ping(void)
{
  unsigned int portState = PORTRead(IOPORT_E); //set BIT_3
  unsigned int pingClk = 0, dt = 0, echo = 0, i = 0;
  char debugStr[128] = {'\0'};
  unsigned int t[16] = {0};
  
  configureTimer4(1024);
  
  //read current clock val and trigger a ping
  pingClk = ReadTimer4();
  PORTWrite(IOPORT_E, (portState | BIT_3) ); //ping!
  delay(TINY_DLY); //ping duration
  portState &= 0xFFFFFFF7; //clear BIT_3
  PORTWrite (IOPORT_E, portState);  //end ping
  
  //while ping-echo not detected AND counter val is less than greatest detection distance.
  //This prevents inifinite loop if view in front of sensor is unobstructed.
  do{
    echo = PORTRead(IOPORT_E) & BIT_3;
    dt = ReadTimer4() - pingClk;
  } while(  !echo && (dt < 512) );
  
  sprintf(debugStr,"dt: %08X  pingClk: %08X  T2: %08X\n\r",dt,pingClk,ReadTimer2());
  putsUART1(debugStr);
  
  i = 0;
  while(i < 10){
    t[i++] = ReadTimer4();
  }
  sprintf(debugStr,"%6X %6X %6X %6X\n\r%6X %6X %6X %6X\n\r",t[0],t[1],t[2],t[3],t[4],t[5],t[6],t[7]);
  putsUART1(debugStr);
  
    i = 0;
  while(i < 10){
    t[i++] = ReadTimer4();
  }
  sprintf(debugStr,"%6X %6X %6X %6X\n\r%6X %6X %6X %6X\n\r",t[0],t[1],t[2],t[3],t[4],t[5],t[6],t[7]);
  putsUART1(debugStr);
  
  CloseTimer4();
  
  return dt;
}
*/

/*
OLD i2cGets function:

BOOL i2cGets(struct i2cDev* dev, unsigned char rxData[], int nBytes)
{
  int i = 0;

  //debug output
  int test = 0;
  int Success = 0b1111;
  I2C_STATUS status;
  char str[64] = {'\0'};
  status = I2CGetStatus(I2C1);
  test = (int)status | test;
  sprintf(str,"init i2C status: 0x%2x\n\r",test);
  //putsUART1(str);
  //end debug

  if( !StartTransfer(FALSE) ){
    putsUART1("i2cStart xfer failed in i2cReadNBytes()\n\r");
    return FALSE;
  }

  //Read data from slave:
  //  Transmit sequence a byte at a time: Slave write address, slave sub address (the id register), but NOT the read register slave address (comes below, after repeated start command)
  if ( !I2CTransmitByte(I2C1, ( dev->devAddr << 1 | 0) )){ //send write address
    putsUART1("ERROR devAddr could not be sent in i2cGetc()\n\r");
    return FALSE;
  }

  if( !I2CTransmitByte(I2C1, dev->subAddr ) ){ //send subaddress to read from
    putsUART1("Error: devSubAddr could not be sent in i2cGetc()\n\r");
    return FALSE;
  }

  StartTransfer(TRUE);  //now send repeated start signal followed by slave read address + read bit
  ////putsUART1("made it to repeat start\n\r");
  
  if( I2CTransmitByte(I2C1, (dev->devAddr << 1 | 1) ) ){ //send the read address
    if(I2CReceiverEnable(I2C1, TRUE) == I2C_RECEIVE_OVERFLOW){
      putsUART1("ERROR I2C Rx Overflow in i2cGetc()\n\r");
      I2CReceiverEnable(I2C1, FALSE);
      return FALSE;
    }

    //putsUART1("Ready to Rx\n\r");

    //loop terminates before reading the last byte; this is because on the last byte we send NACK instead of ACK to the slave.
    for(i = 0; i < (nBytes - 1); i++){
      rxData[i] = I2CGetByte(I2C1);
      I2CSendAck(I2C1, TRUE); // IF LOOPING, THIS SHOULD PASS *TRUE* (false is for ending transmission: master sends NACK, then STOP at EOT)
    }

    rxData[i] = I2CGetByte(I2C1);
    I2CSendAck(I2C1, FALSE); // IF LOOPING, THIS SHOULD PASS *TRUE* (false is for ending transmission: master sends NACK, then STOP    
    I2CReceiverEnable(I2C1, FALSE);  //dont forget to disable receiver once Rx completed!  Not known if this is necessary or not
  }
  else{
    putsUART1("ERROR Failed to send devSubAddr_R in i2cGetc()\n\r");
    return FALSE;
  }

  //putsUART1("pre\n\r");  succeeds / prints
  StopTransfer();
  // putsUART1("post\n\r"); succeeds / print
  //I2CClearStatus ( I2C1, I2C_ARBITRATION_LOSS | I2C_SUCCESS | I2C_ERROR | I2C_START | I2C_STOP | I2C_MASTER_BUS_COLLISION | I2C_RECEIVE_OVERFLOW | I2C_TRANSMITTER_OVERFLOW ); //second param selects flags to clear (all of them)
}

*/
