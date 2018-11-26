
/*
 * This source code is for the purpose of Introduction to Embedded Engineering Design
 * FreeRTOS Library
 * dorobo32 library
 * Copyright
 *
 * Tien Quang Tran
 * Tuan Duc Nguyen
 * */

#include <stdlib.h>
#include <stdbool.h>
#include "dorobo32.h"
#include "FreeRTOS.h"
#include "task.h"
#include "trace.h"
#include "adc.h"
#include "motor.h"
#include "fft.h"
#include "digital.h"
#include <stdlib.h>

static void read_Switch(void *pvParameters);
static void read_Distance(void *pvParameters);
static void read_IR(void *pvParameters);
static void motor(void *pvParameters);

void move(char c);


int drive_motor(int m1,int m2,int m3);
#define minDistance 15
#define maxDistance 60
#define maxSpeed 100
#define accelerationInterval 3.8

static int leftEye, rightEye;
static int run = 0;
static int IR_read = minDistance;
static int distanceL,distanceR;
static int RW, LW;
static float accel = 0;
static int flag = 0;
static int flag1 = 0,flag3= 0;

int main()
{
  //Initialization
  dorobo_init();			//Call dorobo_init() function to initialize HAL, Clocks, Timers etc.
  adc_init();
  digital_init();
  motor_init();

  // Task initialization
   xTaskCreate(motor,"ControlMotor", 128, NULL , 2, NULL);
   xTaskCreate(read_IR,"READIR", 128, NULL, 2, NULL);
   xTaskCreate(read_Distance, "READDistance", 128, NULL, 2, NULL);
   xTaskCreate(read_Switch, "READSWITCH", 128, NULL, 2, NULL);

   vTaskStartScheduler();	 //start the freertos scheduler

	return 0;				//should not be reached!
}
/* Read Switch sensors*/
static void read_Switch(void *pvParameters)
{
  while (1)
  {
    RW = LW = 0;
    if(!digital_get_pin(DD_PIN_PC8))
    {
//      traces("Right button is pressed\r\n");
      RW = 1;
    }
    if(!digital_get_pin(DD_PIN_PD15))
     {
//       traces("Left button is pressed\r\n");
       LW = 1;
     }
    if(!digital_get_pin(DD_PIN_PC9))
        {
      if(run == 0) run = 1;
      else run = 0;
//      tracef("%d \r\n",run);
        }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
/* Read Distance sensors*/
static void read_Distance(void *pvParameters)
{
  while (1)
  {
    int value,value1,delta;
    float voltage;


    value = adc_get_value(DA_ADC_CHANNEL9);
    value1 = adc_get_value(DA_ADC_CHANNEL8);
    voltage = value* 0.00087;


    distanceL = (34700.16/(value- 111.786))- 3.25;
    distanceR = (34700.16/(value1- 111.786))- 3.25;
    delta = abs(distanceL-distanceR);

    if (distanceL >= 0)   tracef("DistanceL (cm): %d \r\n",distanceL);
   else                 distanceL  = 80;
    if (distanceR >= 0)  tracef("DistanceR (cm): %d \r\n",distanceR);
    else                 distanceR = 80;

    IR_read= minDistance;
    if (delta < 15 && distanceL >= 0 && distanceR >= 0)
    {
    IR_read =(distanceL+ distanceR)/2;
//    tracef("Collision Warning in (cm): %d \r\n",IR_read);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
/* Read IREye sensors*/
static void read_IR(void *pvParameters)
{
  while (1)
  {
    rightEye = 0;
    leftEye = 0;
    if(!digital_get_pin(DD_PIN_PC13))
        {
//          tracef("R \r\n");
            rightEye = 1;
        }
     if(!digital_get_pin(DD_PIN_PD14))
        {
//       tracef("L \r\n");
          leftEye = 1;
        }
//    if( leftEye == 0 && rightEye == 0) tracef("No signal \r\n");
   vTaskDelay(pdMS_TO_TICKS(10));
   }
}
/* Robot main behavior*/
static void motor(void *pvParameters)
{
  while (1) {
//    Just Obstacle avoidance
    if(LW == 1 && RW == 1) {
      move('b'); // Both Switches Touch
      flag1 = 0;
    }
    else if(LW == 1 && RW == 0){
      move('r'); // Switch Left Touch
      flag1 = 0;
    }
    else if(LW == 0 && RW == 1)
    {
      move('l');
      flag1 = 0;// Switch Right Touch
    }
    else if (distanceR <= 40  &&  distanceL <= 40)
      {
      if(flag1 == 0)
      {
        if(rand() % 2 == 0)
          {
          move('r');
          flag3 =0;
          }
        else
          {
          move ('l');
          flag3 =1;
          }
       flag1 = 1;
       }
      if(flag3 == 0 ) move ('r');
      else move ('l');
      }
    else if (distanceL < minDistance )
      {
      move('r');
      flag1 = 0;
      }
    else if (distanceR < minDistance) {
      move('l');
      flag1 = 0;
    }
// Source Finding
    else{
          flag1 = 0;
         if (leftEye == 1 && rightEye == 1) move('f');
         else if (leftEye == 0 && rightEye == 1)  move('r');
         else if (leftEye == 1 && rightEye == 0)  move('l');
          else move('f');

    }
  vTaskDelay(pdMS_TO_TICKS(20));
  }
}

/* Drive motor with acceleration*/
void move(char c){
  if(run == 1){
    switch (c){
    case 'l':
      if (accel > 0 && flag != 1) {
      accel = 0;
      drive_motor(0,0,0);
      flag = 1;
      }
      accel += accelerationInterval;
      if (accel >= maxSpeed) accel = maxSpeed;
      drive_motor(-(int) accel,(int) accel,-(int) accel);
    break;
    case 'r':
      if (accel > 0 && flag != 2) {
          accel = 0;
          drive_motor(0,0,0);
          flag = 2;
          }
      accel += accelerationInterval;
      if (accel >= maxSpeed) accel = maxSpeed;
      drive_motor((int) accel,-(int) accel,(int) accel);
    break;
    case 'b':
      if (accel > 0 && flag != 3) {
              accel = 0;
              drive_motor(0,0,0);
              flag = 3;
              }

      accel += accelerationInterval;
      if (accel >= maxSpeed) accel = maxSpeed;
      drive_motor(0,-(int) accel,-(int) accel);
    break;
    case 'f':
      if (accel > 0 && flag != 4) {
              accel = 0;
              drive_motor(0,0,0);
              flag = 4;
              }

      accel += accelerationInterval;
      if (accel >= maxSpeed) accel = maxSpeed;
      drive_motor(0,(int) accel,(int) accel);
    break;
    default: break;
    }
    tracef("m1, m2, m3: %d %d %d, moving %c \r\n",motor_get_speed(DM_MOTOR1),motor_get_speed(DM_MOTOR2),motor_get_speed(DM_MOTOR0),c);
  }
  else {
    drive_motor(0,0,0);
  }
}
/* Drive signal to motor*/
int drive_motor(int m1,int m2,int m3)
{
  motor_set (DM_MOTOR1, m1);
  motor_set (DM_MOTOR2, m2);
  motor_set (DM_MOTOR0, m3);
  return 0;
}


