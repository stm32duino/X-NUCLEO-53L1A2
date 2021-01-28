/**
 ******************************************************************************
 * @file    X_NUCLEO_53L1A2_HelloWorld.ino
 * @author  SRA
 * @version V1.0.0
 * @date    30 July 2020
 * @brief   Arduino test application for the STMicrolectronics X-NUCLEO-53L1A2
 *          proximity sensor expansion board based on FlightSense.
 *          This application makes use of C++ classes obtained from the C
 *          components' drivers.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2020 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <Arduino.h>
#include <Wire.h>
#include <vl53l1_x_nucleo_53l1a2_class.h>
#include <stmpe1600_class.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>

#define DEV_I2C Wire
#define SerialPort Serial

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif
#define LedPin LED_BUILTIN

/* Please uncomment the line below if you want also to use the satellites */
//#define SATELLITES_MOUNTED

// Components.
STMPE1600DigiOut xshutdown_top(&DEV_I2C, GPIO_15, (0x42 * 2));
VL53L1_X_NUCLEO_53L1A2 sensor_vl53l1_top(&DEV_I2C, &xshutdown_top);
#ifdef SATELLITES_MOUNTED
STMPE1600DigiOut xshutdown_left(&DEV_I2C, GPIO_14, (0x43 * 2));
VL53L1_X_NUCLEO_53L1A2 sensor_vl53l1_left(&DEV_I2C, &xshutdown_left);
STMPE1600DigiOut xshutdown_right(&DEV_I2C, GPIO_15, (0x43 * 2));
VL53L1_X_NUCLEO_53L1A2 sensor_vl53l1_right(&DEV_I2C, &xshutdown_right);
#endif

/* Setup ---------------------------------------------------------------------*/

void setup()
{
   // Led.
   pinMode(LedPin, OUTPUT);

   // Initialize serial for output.
   SerialPort.begin(115200);
   SerialPort.println("Starting...");

   // Initialize I2C bus.
   DEV_I2C.begin();

   // Configure VL53L1 top component.
   sensor_vl53l1_top.begin();

   // Switch off VL53L1 top component.
   sensor_vl53l1_top.VL53L1_Off();

#ifdef SATELLITES_MOUNTED
   // Configure (if present) VL53L1 left component.
   sensor_vl53l1_left.begin();

   //Switch off (if present) VL53L1 left component.
   sensor_vl53l1_left.VL53L1_Off();

   // Configure (if present) VL53L1 right component.
   sensor_vl53l1_right.begin();

   // Switch off (if present) VL53L1 right component.
   sensor_vl53l1_right.VL53L1_Off();
#endif

   //Initialize all the sensors
   sensor_vl53l1_top.InitSensor(0x10);
#ifdef SATELLITES_MOUNTED
   sensor_vl53l1_left.InitSensor(0x12);
   sensor_vl53l1_right.InitSensor(0x14);
#endif

   // Start Measurements
   sensor_vl53l1_top.VL53L1_SetPresetMode(VL53L1_PRESETMODE_RANGING);
   sensor_vl53l1_top.VL53L1_StartMeasurement();
#ifdef SATELLITES_MOUNTED
   sensor_vl53l1_left.VL53L1_SetPresetMode(VL53L1_PRESETMODE_RANGING);
   sensor_vl53l1_left.VL53L1_StartMeasurement();
   sensor_vl53l1_right.VL53L1_SetPresetMode(VL53L1_PRESETMODE_RANGING);
   sensor_vl53l1_right.VL53L1_StartMeasurement();
#endif
}

void loop()
{
   VL53L1_MultiRangingData_t MultiRangingData;
   VL53L1_MultiRangingData_t *pMultiRangingData = &MultiRangingData;
   uint8_t NewDataReady = 0;
   int no_of_object_found = 0, j;
   char report[64];
   int status;

   do
   {
      status = sensor_vl53l1_top.VL53L1_GetMeasurementDataReady(&NewDataReady);
   } while (!NewDataReady);

   //Led on
   digitalWrite(LedPin, HIGH);

   if((!status)&&(NewDataReady!=0))
   {
      status = sensor_vl53l1_top.VL53L1_GetMultiRangingData(pMultiRangingData);
      no_of_object_found=pMultiRangingData->NumberOfObjectsFound;
      snprintf(report, sizeof(report), "VL53L1 Top: Count=%d, #Objs=%1d ", pMultiRangingData->StreamCount, no_of_object_found);
      SerialPort.print(report);
      for(j=0;j<no_of_object_found;j++)
      {
         if(j!=0)SerialPort.print("\r\n                               ");
         SerialPort.print("status=");
         SerialPort.print(pMultiRangingData->RangeData[j].RangeStatus);
         SerialPort.print(", D=");
         SerialPort.print(pMultiRangingData->RangeData[j].RangeMilliMeter);
         SerialPort.print("mm");
         SerialPort.print(", Signal=");
         SerialPort.print((float)pMultiRangingData->RangeData[j].SignalRateRtnMegaCps/65536.0);
         SerialPort.print(" Mcps, Ambient=");
         SerialPort.print((float)pMultiRangingData->RangeData[j].AmbientRateRtnMegaCps/65536.0);
         SerialPort.print(" Mcps");
      }
      SerialPort.println("");
      if (status==0)
      {
         status = sensor_vl53l1_top.VL53L1_ClearInterruptAndStartMeasurement();
      }
   }

   digitalWrite(LedPin, LOW);

#ifdef SATELLITES_MOUNTED
   NewDataReady = 0;
   no_of_object_found = 0;

   do
   {
      status = sensor_vl53l1_left.VL53L1_GetMeasurementDataReady(&NewDataReady);
   } while (!NewDataReady);

   //Led on
   digitalWrite(LedPin, HIGH);

   if((!status)&&(NewDataReady!=0))
   {
      status = sensor_vl53l1_left.VL53L1_GetMultiRangingData(pMultiRangingData);
      no_of_object_found=pMultiRangingData->NumberOfObjectsFound;
      snprintf(report, sizeof(report), "VL53L1 Left: Count=%d, #Objs=%1d ", pMultiRangingData->StreamCount, no_of_object_found);
      SerialPort.print(report);
      for(j=0;j<no_of_object_found;j++)
      {
         if(j!=0)SerialPort.print("\r\n                                ");
         SerialPort.print("status=");
         SerialPort.print(pMultiRangingData->RangeData[j].RangeStatus);
         SerialPort.print(", D=");
         SerialPort.print(pMultiRangingData->RangeData[j].RangeMilliMeter);
         SerialPort.print("mm");
         SerialPort.print(", Signal=");
         SerialPort.print((float)pMultiRangingData->RangeData[j].SignalRateRtnMegaCps/65536.0);
         SerialPort.print(" Mcps, Ambient=");
         SerialPort.print((float)pMultiRangingData->RangeData[j].AmbientRateRtnMegaCps/65536.0);
         SerialPort.print(" Mcps");
      }
      SerialPort.println("");
      if (status==0)
      {
         status = sensor_vl53l1_left.VL53L1_ClearInterruptAndStartMeasurement();
      }
   }

   digitalWrite(LedPin, LOW);

   NewDataReady = 0;
   no_of_object_found = 0;

   do
   {
      status = sensor_vl53l1_right.VL53L1_GetMeasurementDataReady(&NewDataReady);
   } while (!NewDataReady);

   //Led on
   digitalWrite(LedPin, HIGH);

   if((!status)&&(NewDataReady!=0))
   {
      status = sensor_vl53l1_right.VL53L1_GetMultiRangingData(pMultiRangingData);
      no_of_object_found=pMultiRangingData->NumberOfObjectsFound;
      snprintf(report, sizeof(report), "VL53L1 Right: Count=%d, #Objs=%1d ", pMultiRangingData->StreamCount, no_of_object_found);
      SerialPort.print(report);
      for(j=0;j<no_of_object_found;j++)
      {
         if(j!=0)SerialPort.print("\r\n                                 ");
         SerialPort.print("status=");
         SerialPort.print(pMultiRangingData->RangeData[j].RangeStatus);
         SerialPort.print(", D=");
         SerialPort.print(pMultiRangingData->RangeData[j].RangeMilliMeter);
         SerialPort.print("mm");
         SerialPort.print(", Signal=");
         SerialPort.print((float)pMultiRangingData->RangeData[j].SignalRateRtnMegaCps/65536.0);
         SerialPort.print(" Mcps, Ambient=");
         SerialPort.print((float)pMultiRangingData->RangeData[j].AmbientRateRtnMegaCps/65536.0);
         SerialPort.print(" Mcps");
      }
      SerialPort.println("");
      if (status==0)
      {
         status = sensor_vl53l1_right.VL53L1_ClearInterruptAndStartMeasurement();
      }
   }

   digitalWrite(LedPin, LOW);
#endif
}
