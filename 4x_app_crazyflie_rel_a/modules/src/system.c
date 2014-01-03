/*
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * system.c - Top level module implementation
 */
#define DEBUG_MODULE "SYS"

#include <stdbool.h>

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "debug.h"
#include "config.h"
#include "param.h"
#include "led.h"

#include "system.h"
#include "freeRTOSdebug.h"
#include "uart.h"
#include "comm.h"
#include "stabilizer.h"
#include "commander.h"

#include "console.h"

/* Private variable */
static bool canFly;

static bool isInit;

/* System wide synchronisation */
xSemaphoreHandle canStartMutex;

/* Private functions */
static void systemTask(void *arg);

/* Public functions */
void systemLaunch(void)
{
  xTaskCreate(systemTask, (const signed char *)"SYSTEM",
              2*configMINIMAL_STACK_SIZE, NULL, /*Piority*/2, NULL);

}

//This must be the first module to be initialized!
void systemInit(void)
{
  if(isInit)
    return;

  canStartMutex = xSemaphoreCreateMutex();
  xSemaphoreTake(canStartMutex, portMAX_DELAY);

//  configblockInit();
//  workerInit();
//  adcInit();
//  ledseqInit();
//  pmInit();
    
  isInit = true;
}

bool systemTest()
{
  bool pass=isInit;
  
//  pass &= adcTest();
//  pass &= ledseqTest();
//  pass &= pmTest();
//  pass &= workerTest();
  
  return pass;
}

/* Private functions implementation */

extern int paramsLen;

void systemTask(void *arg)
{
  bool pass = true,i=1;
  portTickType lastWakeTime2;
	lastWakeTime2 = xTaskGetTickCount ();
  //Init the high-levels modules
  systemInit();

#ifndef USE_UART_CRTP
#ifdef UART_OUTPUT_TRACE_DATA
  debugInitTrace();
#endif
#ifdef HAS_UART
  uartInit();
#endif
#endif //ndef USE_UART_CRTP

  commInit();

  DEBUG_PRINT("Crazyflie is up and running!\n");
 // DEBUG_PRINT("Build %s:%s (%s) %s\n", V_SLOCAL_REVISION,
 //             V_SREVISION, V_STAG, (V_MODIFIED)?"MODIFIED":"CLEAN");
 // DEBUG_PRINT("I am 0x%X%X%X and I have %dKB of flash!\n",
 //             *((int*)(0x1FFFF7E8+8)), *((int*)(0x1FFFF7E8+4)),
 //             *((int*)(0x1FFFF7E8+0)), *((short*)(0x1FFFF7E0)));

  commanderInit();
  stabilizerInit();
  
  //Test the modules
  pass &= systemTest();
  pass &= commTest();
  pass &= commanderTest();
  pass &= stabilizerTest();
  ledInit();
  //Start the firmware
  if(pass)
  {
    systemStart();
//    ledseqRun(LED_RED, seq_alive);
//    ledseqRun(LED_GREEN, seq_testPassed);
  }
  else
  {
    if (systemTest())
    {
			while(1)
      {
        ledSet(LED_RED, i);
				i=!i;
				//ledseqRun(LED_RED, seq_testPassed); //Red passed == not passed!
        vTaskDelayUntil(&lastWakeTime2, M2T(500));
      }
    }
    else
    {
      ledInit();
      ledSet(LED_RED, true);
    }
  }
  while(1)
	{
		ledSet(LED_RED, i);
		i=!i;
		vTaskDelayUntil(&lastWakeTime2, M2T(100));
	}
}


/* Global system variables */
void systemStart()
{
  xSemaphoreGive(canStartMutex);
}

void systemWaitStart(void)
{
  //This permits to guarantee that the system task is initialized before other
  //tasks waits for the start event.
  while(!isInit)
    vTaskDelay(2);

  xSemaphoreTake(canStartMutex, portMAX_DELAY);
  xSemaphoreGive(canStartMutex);
}

void systemSetCanFly(bool val)
{
  canFly = val;
}

bool systemCanFly(void)
{
  return canFly;
}

/*System parameters (mostly for test, should be removed from here) */
/***************************************************************
PARAM_GROUP_START(cpu)
PARAM_ADD(PARAM_UINT16 | PARAM_RONLY, flash, 0x1FFFF7E0)
PARAM_ADD(PARAM_UINT32 | PARAM_RONLY, id0, 0x1FFFF7E8+0)
PARAM_ADD(PARAM_UINT32 | PARAM_RONLY, id1, 0x1FFFF7E8+4)
PARAM_ADD(PARAM_UINT32 | PARAM_RONLY, id2, 0x1FFFF7E8+8)
PARAM_GROUP_STOP(cpu)
***************************************************************/

void paramscpuinit(paramsvar *pparamsint)
{
	pparamsint->params_cpu[0].type=PARAM_GROUP | PARAM_START;
	pparamsint->params_cpu[0].name="cpu";
	pparamsint->params_cpu[0].address=0x0;
	pparamsint->params_cpu[1].type=PARAM_UINT16 | PARAM_RONLY;
	pparamsint->params_cpu[1].name="flash";
	pparamsint->params_cpu[1].address=(int *)0x1FFFF7E0;
	pparamsint->params_cpu[2].type=PARAM_UINT32 | PARAM_RONLY;
	pparamsint->params_cpu[2].name="id0";
	pparamsint->params_cpu[2].address=(int *)0x1FFFF7E8+0;
	pparamsint->params_cpu[3].type=PARAM_UINT32 | PARAM_RONLY;
	pparamsint->params_cpu[3].name="id1";
	pparamsint->params_cpu[3].address=(int *)0x1FFFF7E8+4;
	pparamsint->params_cpu[4].type=PARAM_UINT32 | PARAM_RONLY;
	pparamsint->params_cpu[4].name="id2";
	pparamsint->params_cpu[4].address=(int *)0x1FFFF7E8+8;
	pparamsint->params_cpu[5].type=PARAM_GROUP | PARAM_STOP;
	pparamsint->params_cpu[5].name="stop_cpu";
	pparamsint->params_cpu[5].address=0x0;
}


