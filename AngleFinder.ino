/**
******************************************************************************
*   @file     main.c
*   @brief    Project main source file
*   @version  V0.1
*   @author   ADI
*   @date     May 2016
*  @par Revision History:
*  - V0.1, May 2016: initial version.
*
*******************************************************************************
* Copyright 2016(c) Analog Devices, Inc.
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*  - Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*  - Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in
*    the documentation and/or other materials provided with the
*    distribution.
*  - Neither the name of Analog Devices, Inc. nor the names of its
*    contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*  - The use of this software may or may not infringe the patent rights
*    of one or more patent holders.  This license does not release you
*    from the requirement that you obtain separate licenses from these
*    patent holders to use this software.
*  - Use of the software either in source or binary form, must be run
*    on or directly connected to an Analog Devices Inc. component.
*
* THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT, MERCHANTABILITY
* AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
* INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*******************************************************************************
**/

/***************************** Include Files **********************************/
#include <stdio.h>
#include <stdlib.h>
#include <SPI.h>
#include <Wire.h>

#include "SSD1306Wire.h"
#include "ADXL355.h"

#define __PLIOT__  1

/*******************************************************************************
**************************** Internal types ************************************
********************************************************************************/

/* Write data mode */
typedef enum {
   SPI_WRITE_ONE_REG = 1,         /* Write 1 ACC register */
   SPI_WRITE_TWO_REG,             /* Write 2 ACC register */
} enWriteData;

typedef enum {
   SPI_READ_ONE_REG = 1,            /* Read one ACC register */
   SPI_READ_TWO_REG,                /* Read two ACC registers */
   SPI_READ_THREE_REG,              /* Read X,Y,Z ACC registers */
} enRegsNum;

/*******************************************************************************
**************************** Internal definitions ******************************
********************************************************************************/
#define int8_t    char
#define uint8_t   unsigned char
#define int32_t   int
#define uint32_t  unsigned int

/* Accelerometer write command */
#define ADXL355_WRITE         0x0

/* Accelerometer read command */
#define ADXL355_READ          0x1

#define CSACC_PIN           15   /* CSADXL355 - output */
#define INT1ACC_PIN         0   /* INT1 - input */
#define INT2ACC_PIN         0   /* INT2 - input */
#define DATARDYACC_PIN      2   /* DATA RDY - input, was 4 */  

#define OLED_DC      2
#define OLED_RESET   10
#define OLED_CS      0

#define INIT_COUNT 100

int32_t volatile i32SensorX;
int32_t volatile i32SensorY;
int32_t volatile i32SensorZ;
int32_t volatile i32SensorT;

int32_t volatile i32SensorX_init;
int32_t volatile i32SensorY_init;
int32_t volatile i32SensorZ_init;

uint32_t volatile ui32SensorX;
uint32_t volatile ui32SensorY;
uint32_t volatile ui32SensorZ;
uint32_t volatile ui32SensorT;

float volatile f32temp = 0.0f;
float adxl355Scale;

// Initialize the OLED display using Wire library
SSD1306Wire  display(0x3c, D3, D4);

/**************************** Function Definitions ****************************/

/**
   @brief SPI initialization

   @return none

**/
void SPI_Init(void)
{
  SPI.begin();
//  SPI.setClockDivider(SPI_CLOCK_DIV128);

  digitalWrite(CSACC_PIN, HIGH);         /* Deselect accelerometer */
}

/**
   @brief Writes a data, a command or a register to the LCD or to ACC via SPI.

   @param ui8address - ACC register address
   @param ui8Data - value to be written in 1 register write
   @param ui8Data2 - 2nd value to be written in 2 register write
   @enMode enWriteData - write mode

   @return none

**/
void SPI_Write(uint8_t ui8address, uint8_t ui8Data, uint8_t ui8Data2, enWriteData enMode)
{
  uint8_t ui8writeAddress;
  ui8writeAddress = ((ui8address <<1)|ADXL355_WRITE);
  
  if(enMode == SPI_WRITE_ONE_REG) {
     digitalWrite(CSACC_PIN, LOW);         /* Select accelerometer */
  
     SPI.transfer(ui8writeAddress);     /* Send register address */
     SPI.transfer(ui8Data);             /* Send value to be written */
  
    digitalWrite(CSACC_PIN, HIGH);         /* Deselect accelerometer */  
  }
  
  if(enMode == SPI_WRITE_TWO_REG) {
		digitalWrite(CSACC_PIN, LOW);         /* Select accelerometer */
  
     SPI.transfer(ui8writeAddress);     /* Send register address */
     SPI.transfer(ui8Data);             /* Send 1st value to be written */
     SPI.transfer(ui8Data2);             /* Send 2nd value to be written */
  
  	digitalWrite(CSACC_PIN, HIGH);         /* Deselect accelerometer */
  }
}

/**
   @brief Reads a specified register or two registers address in the accelerometer via SPI.

   @param ui8address - register address
   @param enRegs - register number

   @return reading result

**/
uint32_t SPI_Read(uint8_t ui8address, enRegsNum enRegs)
{
   uint32_t ui32Result = 0;

   uint32_t ui32valueL = 0;
   uint32_t ui32valueM = 0;
   uint32_t ui32valueH = 0;

   uint8_t ui8writeAddress;
   ui8writeAddress = ((ui8address <<1)|ADXL355_READ);

   digitalWrite(CSACC_PIN, LOW);         /* Select accelerometer */

   SPI.transfer(ui8writeAddress);       /* Send register address */

   if (enRegs == SPI_READ_ONE_REG) {
      ui32Result = SPI.transfer(0x00);            /* Set read result*/
   }
   
   if (enRegs == SPI_READ_TWO_REG) {          /* Only used for Temp & X,Y,Z offset and threshold registers*/
      ui32valueH = SPI.transfer(0x00);             /* Read the register value */
      ui32valueL = SPI.transfer(0x00);

      ui32Result = ((ui32valueH << 8) | ui32valueL); /* Set read result*/
   }
   
   if (enRegs == SPI_READ_THREE_REG) {          /* Only used for X,Y,Z axis data registers*/    
      ui32valueH = SPI.transfer(0x00);             /* Read the register value */
      ui32valueM = SPI.transfer(0x00);
      ui32valueL = SPI.transfer(0x00);

      ui32Result = ((ui32valueH << 16) | (ui32valueL << 8) | ui32valueL); /* Set read result*/
   }

	 digitalWrite(CSACC_PIN, HIGH);         /* Deselect accelerometer */
	 
   return ui32Result;
}

/**
   @brief Initialization the accelerometer sensor

   @return none

**/
void ADXL355_Init(void)
{
   pinMode(CSACC_PIN, OUTPUT);                  /* Set CSACC pin as output */
   pinMode(DATARDYACC_PIN, INPUT);              /* Set DRDY pin as input */
//   pinMode(INT1ACC_PORT, INPUT);         /* Set INT1ACC pin as input */
//   pinMode(INT2ACC_PORT, INPUT);         /* Set INT2ACC pin as input */

   /* Quick verification test for boards */
//   uint32_t volatile ui32test = SPI_Read(DEVID_AD, SPI_READ_ONE_REG);                  /* Read the ID register */
//   uint32_t volatile ui32test2 = SPI_Read(DEVID_MST, SPI_READ_ONE_REG);                  /* Read the ID register */
//   uint32_t volatile ui32test3 = SPI_Read(PARTID, SPI_READ_ONE_REG);                  /* Read the ID register */
//   uint32_t volatile ui32test4 = SPI_Read(REVID, SPI_READ_ONE_REG);                 /* Read the ID register */
}

/**
   @brief Turns on accelerometer measurement mode.

   @return none

**/
void ADXL355_Start_Sensor(void)
{
   uint8_t ui8temp;

   ui8temp = (uint8_t)SPI_Read(POWER_CTL, SPI_READ_ONE_REG);       /* Read POWER_CTL register, before modifying it */
   
   ui8temp = ui8temp & 0xFE;                                       /* Set measurement bit in POWER_CTL register */
   SPI_Write(POWER_CTL, ui8temp, 0x00, SPI_WRITE_ONE_REG);         /* Write the new value to POWER_CTL register */  
}

/**
   @brief Puts the accelerometer into standby mode.

   @return none

**/
void ADXL355_Stop_Sensor(void)
{
   uint8_t ui8temp;

   ui8temp = (uint8_t)SPI_Read(POWER_CTL, SPI_READ_ONE_REG);        /*Read POWER_CTL register, before modifying it */
   ui8temp = ui8temp | 0x01;                                      /* Clear measurement bit in POWER_CTL register */
   SPI_Write(POWER_CTL, ui8temp, 0x00, SPI_WRITE_ONE_REG);                 /* Write the new value to POWER_CTL register */
}

/**
   @brief Reads the accelerometer data.

   @return none

**/
void ADXL355_Data_Scan(void)
{
      ui32SensorT = SPI_Read(TEMP2, SPI_READ_TWO_REG);
      
      ui32SensorX = SPI_Read(XDATA3, SPI_READ_THREE_REG);
      ui32SensorY = SPI_Read(YDATA3, SPI_READ_THREE_REG);
      ui32SensorZ = SPI_Read(ZDATA3, SPI_READ_THREE_REG);
      
      i32SensorX = ADXL355_Acceleration_Data_Conversion(ui32SensorX);
      i32SensorY = ADXL355_Acceleration_Data_Conversion(ui32SensorY);
      i32SensorZ = ADXL355_Acceleration_Data_Conversion(ui32SensorZ);
}


/**
   @brief Convert the two's complement data in X,Y,Z registers to signed integers

   @param ui32SensorData - raw data from register

   @return int32_t - signed integer data

**/
int32_t ADXL355_Acceleration_Data_Conversion (uint32_t ui32SensorData)
{
   int32_t volatile i32Conversion = 0;

   ui32SensorData = (ui32SensorData  >> 4);
   ui32SensorData = (ui32SensorData & 0x000FFFFF);

   if((ui32SensorData & 0x00080000)  == 0x00080000){
         i32Conversion = (ui32SensorData | 0xFFF00000);
   }else{
         i32Conversion = ui32SensorData;
   }

   return i32Conversion;
}

void ADXL355_Data_Init()
{
  for(char i=0 ; i<=INIT_COUNT ; i++)
  {
    delay(10);
    ADXL355_Data_Scan();
      
    i32SensorX_init = (i32SensorX_init*i/INIT_COUNT) + (i32SensorX*(INIT_COUNT-i)/INIT_COUNT);
    i32SensorY_init = (i32SensorY_init*i/INIT_COUNT) + (i32SensorY*(INIT_COUNT-i)/INIT_COUNT);
    i32SensorZ_init = (i32SensorZ_init*i/INIT_COUNT) + (i32SensorZ*(INIT_COUNT-i)/INIT_COUNT);
  }  
}

/**
   @brief The main application function

   @return the function contains infinite loop and never returns/

**/
void setup()
{
   volatile uint32_t ui32test=0x00;                  /* Read the ID register */
   volatile uint32_t ui32test2;                  /* Read the Manufacturer register */
   volatile uint32_t ui32test3;                  /* Read the Part ID register */
   
   /* Initialize UART */
   Serial.begin(115200);
   delay(500);
   
  // Initialising the UI will init the display too.
  display.init();

  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);

   
#if(!__PLIOT__)
   Serial.println("***** ADXL355 Simple Test *****");
#endif

   /* Initialize accelerometer */
   ADXL355_Init();
   
   /* Initialize SPI */
   SPI_Init();  
   delay(500);
  
#if ADXL_RANGE == 2
   SPI_Write(RANGE, 0x81, 0x00, SPI_WRITE_ONE_REG);          /* Set sensor range within RANGE register */
   adxl355Scale = 256.0f;
#endif

#if ADXL_RANGE == 4
   SPI_Write(RANGE, 0x82, 0x00, SPI_WRITE_ONE_REG);          /* Set sensor range within RANGE register */
   adxl355Scale = 128.0f;
#endif 

#if ADXL_RANGE == 8
   SPI_Write(RANGE, 0x83, 0x00, SPI_WRITE_ONE_REG);          /* Set sensor range within RANGE register */
   adxl355Scale = 64.0f;
#endif

  do{
     /* Start accelerometer measurement mode */
     ADXL355_Start_Sensor();
  
     ui32test = SPI_Read(DEVID_AD, SPI_READ_ONE_REG);                  /* Read the ID register */
     ui32test2 = SPI_Read(DEVID_MST, SPI_READ_ONE_REG);                  /* Read the Manufacturer register */
     ui32test3 = SPI_Read(PARTID, SPI_READ_ONE_REG);                  /* Read the Part ID register */

#if(!__PLIOT__) 
     Serial.print("DEVID_AD  = 0x");                /* Print the ID register */
     Serial.println(ui32test, HEX);
     Serial.print("DEVID_MST = 0x");                /* Print the Manufacturer register */
     Serial.println(ui32test2, HEX);
     Serial.print("PART_ID = 0x");                  /* Print the Part ID register */
     Serial.println(ui32test3, HEX);
     Serial.println("");
#endif
     delay(1000);
  }while( ui32test == 0x00 );

  ADXL355_Data_Init();
//  SPI_Write(FILTER, 0x05, 0x00, SPI_WRITE_ONE_REG); 
}

char loop_count;
char sensorX[10], sensorY[10], sensorZ[10];

void loop() 
{
  display.clear();
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  
  if( digitalRead(DATARDYACC_PIN) == HIGH )
  {
    ADXL355_Data_Scan();

    f32temp = ((((float)ui32SensorT - ADXL355_TEMP_BIAS)) / ADXL355_TEMP_SLOPE) + 25.0;

  #if(__PLIOT__)
      float Xdata = ((float)(i32SensorX-i32SensorX_init) / adxl355Scale);
      float Ydata = ((float)(i32SensorY-i32SensorY_init) / adxl355Scale);
      float Zdata = ((float)(i32SensorZ-i32SensorZ_init) / adxl355Scale);

      dtostrf(Xdata, 6, 3, sensorX);
      dtostrf(Ydata, 6, 3, sensorY);
      dtostrf(Zdata, 6, 3, sensorZ);
      
      String sXdata = ("X: ") + String(Xdata, 3) + String("mg");
      String sYdata = ("Y: ") + String(Ydata, 3) + String("mg");
      String sZdata = ("Z: ") + String(Zdata, 3) + String("mg");      
  
      Serial.print(Xdata, 3);
      Serial.print(", ");
      Serial.print(Ydata, 3);
      Serial.print(", ");
      Serial.print(Zdata, 3);        
      Serial.println("");
      
      Serial.print("The temperature data is: " );      /* Print the Temperature data */
      Serial.print(f32temp, 2);        
      Serial.println("[C]");
      Serial.println();

  
  #else
  #if (1)   
      Serial.print("X acceleration data [G]: ");        /* Print the X-axis data */
      Serial.print((float)(i32SensorX-i32SensorX_init) / adxl355Scale, 3);
      Serial.println("[mg]");
      
      Serial.print("Y acceleration data [G]: ");        /* Print the Y-axis data */
      Serial.print((float)(i32SensorY-i32SensorY_init) / adxl355Scale, 3);
      Serial.println("[mg]");
      
      Serial.print("Z acceleration data [G]: ");        /* Print the Z-axis data */
      Serial.print((float)(i32SensorZ-i32SensorZ_init) / adxl355Scale, 3);        
      Serial.println("[mg]");
  #else
  
      Serial.print("X acceleration data [G]: ");        /* Print the X-axis data */
      Serial.print((float)i32SensorX / adxl355Scale, 3);
      Serial.println("[mg]");
      
      Serial.print("Y acceleration data [G]: ");        /* Print the Y-axis data */
      Serial.print((float)i32SensorY / adxl355Scale, 3);
      Serial.println("[mg]");
      
      Serial.print("Z acceleration data [G]: ");        /* Print the Z-axis data */
      Serial.print((float)i32SensorZ / adxl355Scale, 3);        
      Serial.println("[mg]");
  #endif    
      Serial.print("The temperature data is: " );      /* Print the Temperature data */
      Serial.print(f32temp, 2);        
      Serial.println("[C]");
      Serial.println();
  #endif
      if( loop_count%0x1F == 0)
      {
        display.clear();
        display.drawString(10, 15, sXdata);
        display.drawString(10, 30, sYdata);
        display.drawString(10, 45, sZdata);
        display.display();
      }
      loop_count++;
    }  
    delay(1000);  
  }

/* End Of File *///
