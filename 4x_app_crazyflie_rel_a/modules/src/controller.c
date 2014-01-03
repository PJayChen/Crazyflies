/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
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
 *
 */
#include <stdbool.h>
 
#include "stm32f10x_conf.h"
#include "FreeRTOS.h"
#include "task.h"

#include "controller.h"
#include "pid.h"
#include "param.h"

/*
#define TRUNCATE_SINT16(out, in) \
  {\
    if (in > INT16_MAX) out = (int16_t)INT16_MAX;\
    else if (in < INT16_MIN) out = (int16_t)INT16_MIN;\
    else out = (int16_t)in;\
  }
*/

//Fancier version
#define TRUNCATE_SINT16(out, in) (out = (in<INT16_MIN)?INT16_MIN:((in>INT16_MAX)?INT16_MAX:in) )

//Better semantic
#define SATURATE_SINT16(in) ( (in<INT16_MIN)?INT16_MIN:((in>INT16_MAX)?INT16_MAX:in) )

PidObject pidRollRate;
PidObject pidPitchRate;
PidObject pidYawRate;
PidObject pidRoll;
PidObject pidPitch;
PidObject pidYaw;

int16_t rollOutput;
int16_t pitchOutput;
int16_t yawOutput;

static bool isInit;

void controllerInit()
{
  if(isInit)
    return;
  
  //TODO: get parameters from configuration manager instead
  pidInit(&pidRollRate, 0, PID_ROLL_RATE_KP, PID_ROLL_RATE_KI, PID_ROLL_RATE_KD);
  pidInit(&pidPitchRate, 0, PID_PITCH_RATE_KP, PID_PITCH_RATE_KI, PID_PITCH_RATE_KD);
  pidInit(&pidYawRate, 0, PID_YAW_RATE_KP, PID_YAW_RATE_KI, PID_YAW_RATE_KD);
  pidSetIntegralLimit(&pidRollRate, PID_ROLL_RATE_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidPitchRate, PID_PITCH_RATE_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidYawRate, PID_YAW_RATE_INTEGRATION_LIMIT);

  pidInit(&pidRoll, 0, PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD);
  pidInit(&pidPitch, 0, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD);
  pidInit(&pidYaw, 0, PID_YAW_KP, PID_YAW_KI, PID_YAW_KD);
  pidSetIntegralLimit(&pidRoll, PID_ROLL_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidPitch, PID_PITCH_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidYaw, PID_YAW_INTEGRATION_LIMIT);
  
  isInit = true;
}

bool controllerTest()
{
  return isInit;
}

void controllerCorrectRatePID(
       float rollRateActual, float pitchRateActual, float yawRateActual,
       float rollRateDesired, float pitchRateDesired, float yawRateDesired)
{
  pidSetDesired(&pidRollRate, rollRateDesired);
  TRUNCATE_SINT16(rollOutput, pidUpdate(&pidRollRate, rollRateActual, TRUE));

  pidSetDesired(&pidPitchRate, pitchRateDesired);
  TRUNCATE_SINT16(pitchOutput, pidUpdate(&pidPitchRate, pitchRateActual, TRUE));

  pidSetDesired(&pidYawRate, yawRateDesired);
  TRUNCATE_SINT16(yawOutput, pidUpdate(&pidYawRate, yawRateActual, TRUE));
}

void controllerCorrectAttitudePID(
       float eulerRollActual, float eulerPitchActual, float eulerYawActual,
       float eulerRollDesired, float eulerPitchDesired, float eulerYawDesired,
       float* rollRateDesired, float* pitchRateDesired, float* yawRateDesired)
{
  pidSetDesired(&pidRoll, eulerRollDesired);
  *rollRateDesired = pidUpdate(&pidRoll, eulerRollActual, TRUE);

  // Update PID for pitch axis
  pidSetDesired(&pidPitch, eulerPitchDesired);
  *pitchRateDesired = pidUpdate(&pidPitch, eulerPitchActual, TRUE);

  // Update PID for yaw axis
  float yawError;
  yawError = eulerYawDesired - eulerYawActual;
  if (yawError > 180.0)
    yawError -= 360.0;
  else if (yawError < -180.0)
    yawError += 360.0;
  pidSetError(&pidYaw, yawError);
  *yawRateDesired = pidUpdate(&pidYaw, eulerYawActual, FALSE);
}

void controllerResetAllPID(void)
{
  pidReset(&pidRoll);
  pidReset(&pidPitch);
  pidReset(&pidYaw);
  pidReset(&pidRollRate);
  pidReset(&pidPitchRate);
  pidReset(&pidYawRate);
}

void controllerGetActuatorOutput(int16_t* roll, int16_t* pitch, int16_t* yaw)
{
  *roll = rollOutput;
  *pitch = pitchOutput;
  *yaw = yawOutput;
}

/****************************************************************
PARAM_GROUP_START(pid_attitude)
PARAM_ADD(PARAM_FLOAT, roll_kp, &pidRoll.kp)
PARAM_ADD(PARAM_FLOAT, roll_ki, &pidRoll.ki)
PARAM_ADD(PARAM_FLOAT, roll_kd, &pidRoll.kd)
PARAM_ADD(PARAM_FLOAT, pitch_kp, &pidPitch.kp)
PARAM_ADD(PARAM_FLOAT, pitch_ki, &pidPitch.ki)
PARAM_ADD(PARAM_FLOAT, pitch_kd, &pidPitch.kd)
PARAM_ADD(PARAM_FLOAT, yaw_kp, &pidYaw.kp)
PARAM_ADD(PARAM_FLOAT, yaw_ki, &pidYaw.ki)
PARAM_ADD(PARAM_FLOAT, yaw_kd, &pidYaw.kd)
PARAM_GROUP_STOP(pid_attitude)

PARAM_GROUP_START(pid_rate)
PARAM_ADD(PARAM_FLOAT, roll_kp, &pidRollRate.kp)
PARAM_ADD(PARAM_FLOAT, roll_ki, &pidRollRate.ki)
PARAM_ADD(PARAM_FLOAT, roll_kd, &pidRollRate.kd)
PARAM_ADD(PARAM_FLOAT, pitch_kp, &pidPitchRate.kp)
PARAM_ADD(PARAM_FLOAT, pitch_ki, &pidPitchRate.ki)
PARAM_ADD(PARAM_FLOAT, pitch_kd, &pidPitchRate.kd)
PARAM_ADD(PARAM_FLOAT, yaw_kp, &pidYawRate.kp)
PARAM_ADD(PARAM_FLOAT, yaw_ki, &pidYawRate.ki)
PARAM_ADD(PARAM_FLOAT, yaw_kd, &pidYawRate.kd)
PARAM_GROUP_STOP(pid_rate)
***************************************************************/

void paramscontrinit(paramsvar *pparamsint)
{
	pparamsint->params_pid_attitude[0].type=PARAM_GROUP | PARAM_START;
	pparamsint->params_pid_attitude[0].name="pid_attitude";
	pparamsint->params_pid_attitude[0].address=0x0;
	pparamsint->params_pid_attitude[1].type=PARAM_FLOAT;
	pparamsint->params_pid_attitude[1].name="roll_kp";
	pparamsint->params_pid_attitude[1].address=&pidRoll.kp;
	pparamsint->params_pid_attitude[2].type=PARAM_FLOAT;
	pparamsint->params_pid_attitude[2].name="roll_ki";
	pparamsint->params_pid_attitude[2].address=&pidRoll.ki;
	pparamsint->params_pid_attitude[3].type=PARAM_FLOAT;
	pparamsint->params_pid_attitude[3].name="roll_kd";
	pparamsint->params_pid_attitude[3].address=&pidRoll.kd;
	pparamsint->params_pid_attitude[4].type=PARAM_FLOAT;
	pparamsint->params_pid_attitude[4].name="pitch_kp";
	pparamsint->params_pid_attitude[4].address=&pidPitch.kp;
	pparamsint->params_pid_attitude[5].type=PARAM_FLOAT;
	pparamsint->params_pid_attitude[5].name="pitch_ki";
	pparamsint->params_pid_attitude[5].address=&pidPitch.ki;
	pparamsint->params_pid_attitude[6].type=PARAM_FLOAT;
	pparamsint->params_pid_attitude[6].name="pitch_kd";
	pparamsint->params_pid_attitude[6].address=&pidPitch.kd;
	pparamsint->params_pid_attitude[7].type=PARAM_FLOAT;
	pparamsint->params_pid_attitude[7].name="yaw_kp";
	pparamsint->params_pid_attitude[7].address=&pidYaw.kp;
	pparamsint->params_pid_attitude[8].type=PARAM_FLOAT;
	pparamsint->params_pid_attitude[8].name="yaw_ki";
	pparamsint->params_pid_attitude[8].address=&pidYaw.ki;
	pparamsint->params_pid_attitude[9].type=PARAM_FLOAT;
	pparamsint->params_pid_attitude[9].name="yaw_kd";
	pparamsint->params_pid_attitude[9].address=&pidYaw.kd;
	pparamsint->params_pid_attitude[10].type=PARAM_GROUP | PARAM_STOP;
	pparamsint->params_pid_attitude[10].name="stop_pid_attitude";
	pparamsint->params_pid_attitude[10].address=0x0;
	
	pparamsint->params_pid_rate[0].type=PARAM_GROUP | PARAM_START;
	pparamsint->params_pid_rate[0].name="pid_rate";
	pparamsint->params_pid_rate[0].address=0x0;
	pparamsint->params_pid_rate[1].type=PARAM_FLOAT;
	pparamsint->params_pid_rate[1].name="roll_kp";
	pparamsint->params_pid_rate[1].address=&pidRollRate.kp;
	pparamsint->params_pid_rate[2].type=PARAM_FLOAT;
	pparamsint->params_pid_rate[2].name="roll_ki";
	pparamsint->params_pid_rate[2].address=&pidRollRate.ki;
	pparamsint->params_pid_rate[3].type=PARAM_FLOAT;
	pparamsint->params_pid_rate[3].name="roll_kd";
	pparamsint->params_pid_rate[3].address=&pidRollRate.kd;
	pparamsint->params_pid_rate[4].type=PARAM_FLOAT;
	pparamsint->params_pid_rate[4].name="pitch_kp";
	pparamsint->params_pid_rate[4].address=&pidPitchRate.kp;
	pparamsint->params_pid_rate[5].type=PARAM_FLOAT;
	pparamsint->params_pid_rate[5].name="pitch_ki";
	pparamsint->params_pid_rate[5].address=&pidPitchRate.ki;
	pparamsint->params_pid_rate[6].type=PARAM_FLOAT;
	pparamsint->params_pid_rate[6].name="pitch_kd";
	pparamsint->params_pid_rate[6].address=&pidPitchRate.kd;
	pparamsint->params_pid_rate[7].type=PARAM_FLOAT;
	pparamsint->params_pid_rate[7].name="yaw_kp";
	pparamsint->params_pid_rate[7].address=&pidYawRate.kp;
	pparamsint->params_pid_rate[8].type=PARAM_FLOAT;
	pparamsint->params_pid_rate[8].name="yaw_ki";
	pparamsint->params_pid_rate[8].address=&pidYawRate.ki;
	pparamsint->params_pid_rate[9].type=PARAM_FLOAT;
	pparamsint->params_pid_rate[9].name="yaw_kd";
	pparamsint->params_pid_rate[9].address=&pidYawRate.kd;
	pparamsint->params_pid_rate[10].type=PARAM_GROUP | PARAM_STOP;
	pparamsint->params_pid_rate[10].name="stop_pid_rate";
	pparamsint->params_pid_rate[10].address=0x0;
}
