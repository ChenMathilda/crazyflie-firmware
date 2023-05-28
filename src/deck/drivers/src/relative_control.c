#include "system.h"
#include "FreeRTOS.h"
#include "task.h"
#include "commander.h"
#include "relative_control.h"
#include "relative_localization.h"
#include "crtp_commander_high_level.h"
#include "ranging_struct.h"
#include "num.h"
#include "param.h"
#include "debug.h"
#include <stdlib.h> // random
#include "configblock.h"
#include "uart1_dma_task.h"
#include "uart2.h"
#include "log.h"
#include "math.h"
#include "adhocdeck.h"
#include "led.h"
// #include "lighthouse_position_est.h"
#define DEBUG_MODULE "CTRL"
#include "debug.h"

static uint16_t MY_UWB_ADDRESS;
static bool isInit;
static bool onGround = true;               // 无人机当前是否在地面上?
static bool isCompleteTaskAndLand = false; // 无人机是否已经执行了飞行任务并落地?
static setpoint_t setpoint;
static float_t relaVarInCtrl[RANGING_TABLE_SIZE + 1][STATE_DIM_rl];
static currentNeighborAddressInfo_t currentNeighborAddressInfo;
static float_t height = 0.5;
static uint32_t takeoff_tick;
static uint32_t tickInterval;
static float relaCtrl_p = 2.0f;
// static float relaCtrl_i = 0.0001f;
static float relaCtrl_i = 0.01f;
static float relaCtrl_d = 0.01f;
// static float NDI_k = 2.0f;

#define DEBUG_FLY
#ifndef DEBUG_FLY
static bool keepFlying = false;
#else
static bool keepFlying = true;
#endif
static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawrate)
{
#ifndef DEBUG_FLY
  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;
  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = yawrate;
  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;
  setpoint->velocity.x = vx;
  setpoint->velocity.y = vy;
  setpoint->velocity_body = true;
  commanderSetSetpoint(setpoint, 3);
#endif
}
static float steerAngle;
static float collision;
static float signFromation;
static float_t Yaw;
static float_t Vel;

static void flyRandomIn1meter(float_t randomVel)
{
  float_t randomYaw = (rand() / (float)RAND_MAX) * 3.14f; // 0-2pi rad
  // float_t randomVel = (rand() / (float)RAND_MAX) * 1;     // 0-1 m/s
  float_t vxBody = randomVel * cosf(randomYaw); // 速度分解
  float_t vyBody = randomVel * sinf(randomYaw);
  for (int i = 1; i < 100; i++)
  {
    setHoverSetpoint(&setpoint, vxBody, vyBody, height, 0);
    vTaskDelay(M2T(10));
  }
  setHoverSetpoint(&setpoint, 0, 0, height, 0);
  vTaskDelay(M2T(10));
  for (int i = 1; i < 100; i++)
  {
    setHoverSetpoint(&setpoint, -vxBody, -vyBody, height, 0);
    vTaskDelay(M2T(10));
  }
}

static void flyRandomByAIResult(float steerAngle, float collision, float ai_height)
{

  Yaw = ((1 - beta) * Yaw + beta * (PI / 2) * steerAngle);    // deg
  Vel = (1 - alpha) * Vel + alpha * (1 - collision) * velMax; // m/s
  for (int i = 0; i < 100; i++)                               // 1s
  {
    setHoverSetpoint(&setpoint, Vel, 0, ai_height, Yaw); // deg/s
    vTaskDelay(M2T(10));
  }
}

#define SIGN(a) ((a >= 0) ? 1 : -1)
static float_t targetX;
static float_t targetY;
static float_t targetYaw;
static float PreErr_x = 0;
static float PreErr_y = 0;
static float IntErr_x = 0;
static float IntErr_y = 0;
static uint32_t PreTime;
static void formation0asCenter(float_t tarX, float_t tarY, float_t targetYaw, float_t ai_height)
{
  float dt = (float)(xTaskGetTickCount() - PreTime) / configTICK_RATE_HZ;
  PreTime = xTaskGetTickCount();
  if (dt > 1) // skip the first run of the EKF
    return;
  // pid control for formation flight 当前是1号无人机
  // relaVarInCtrl[0][STATE_rlX]:0号无人机在本机坐标系下的位置
  // tarX:坐标系变化后本机在0号无人机下的目标位置
  float err_x = -(tarX - relaVarInCtrl[0][STATE_rlX]); // 是相反的关系，并且应该误差很小
  float err_y = -(tarY - relaVarInCtrl[0][STATE_rlY]);
  float pid_vx = relaCtrl_p * err_x;  // 2.0*err_x 基于距离差进行一个速度控制
  float pid_vy = relaCtrl_p * err_y;  // 2.0*err_y
  float dx = (err_x - PreErr_x) / dt; // 先前的速度
  float dy = (err_y - PreErr_y) / dt;
  PreErr_x = err_x;
  PreErr_y = err_y;
  pid_vx += relaCtrl_d * dx; // 0.01*dx 先前速度*比例系数
  pid_vy += relaCtrl_d * dy; // 0.01*dy
  IntErr_x += err_x * dt;
  IntErr_y += err_y * dt;
  pid_vx += relaCtrl_i * constrain(IntErr_x, -0.5, 0.5); // += (+-)0.00005
  pid_vy += relaCtrl_i * constrain(IntErr_y, -0.5, 0.5);
  pid_vx = constrain(pid_vx, -1.5f, 1.5f);
  pid_vy = constrain(pid_vy, -1.5f, 1.5f);

  // float rep_x = 0.0f;
  // float rep_y = 0.0f;
  // for(uint8_t i=0; i<NumUWB; i++){
  //   if(i!=selfID){
  //     float dist = relaVarInCtrl[i][STATE_rlX]*relaVarInCtrl[i][STATE_rlX] + relaVarInCtrl[i][STATE_rlY]*relaVarInCtrl[i][STATE_rlY];
  //     dist = sqrtf(dist);
  //     rep_x += -0.5f * (SIGN(0.5f - dist) + 1) / (abs(relaVarInCtrl[i][STATE_rlX]) + 0.001f) * SIGN(relaVarInCtrl[i][STATE_rlX]);
  //     rep_y += -0.5f * (SIGN(0.5f - dist) + 1) / (abs(relaVarInCtrl[i][STATE_rlY]) + 0.001f) * SIGN(relaVarInCtrl[i][STATE_rlY]);
  //   }
  // }
  // rep_x = constrain(rep_x, -1.5f, 1.5f);
  // rep_y = constrain(rep_y, -1.5f, 1.5f);

  // pid_vx = constrain(pid_vx + rep_x, -1.5f, 1.5f);
  // pid_vy = constrain(pid_vy + rep_y, -1.5f, 1.5f);
  setHoverSetpoint(&setpoint, 0, 0, ai_height, targetYaw);
  for (int i = 1; i < 10; i++)
  {
    setHoverSetpoint(&setpoint, pid_vx, pid_vy, ai_height, 0);
    vTaskDelay(M2T(10));
  }
}

void take_off()
{
  for (int i = 0; i < 5; i++)
  {
    setHoverSetpoint(&setpoint, 0, 0, height, 0);
    vTaskDelay(M2T(100));
  }
  // for (int i = 0; i < 10 * MY_UWB_ADDRESS; i++)
  // {
  //   setHoverSetpoint(&setpoint, 0, 0, height, 0);
  //   vTaskDelay(M2T(100));
  // }
  onGround = false;
}

void land()
{
  // landing procedure
  if (!onGround)
  {
    int i = 0;
    float land_height_per_100ms = 0.02;            // 每秒下降的高度为该变量的值*10
    while (height - i * land_height_per_100ms > 0) // 1s下降0.2s
    {
      i++;
      setHoverSetpoint(&setpoint, 0, 0, height - (float)i * land_height_per_100ms, 0);
      vTaskDelay(M2T(100));
    }
    isCompleteTaskAndLand = true;
  }
  onGround = true;
}

float get_min(float *var_history, int len_history)
{
  float res = var_history[0];
  for (size_t i = 1; i < len_history; i++)
  {
    res = var_history[i] < res ? var_history[i] : res;
  }
  return res;
}

float get_max(float *var_history, int len_history)
{
  float res = var_history[0];
  for (size_t i = 1; i < len_history; i++)
  {
    res = var_history[i] > res ? var_history[i] : res;
  }
  return res;
}

void reset_estimators()
{

  int len_history = 10;
  float var_x_history[len_history];
  float var_y_history[len_history];
  float var_z_history[len_history];
  for (size_t i = 0; i < len_history; i++)
  {
    var_x_history[i] = 1000.0;
    var_y_history[i] = 1000.0;
    var_z_history[i] = 1000.0;
  }
  float threshold = 0.001;
  int i = 0;
  while (true)
  {
    /* PX,PY,PZ log variable id */
    float idVelocityX = logGetVarId("kalman", "varPX");
    float idVelocityY = logGetVarId("kalman", "varPY");
    float idVelocityZ = logGetVarId("kalman", "varPZ");
    float velocityX = logGetFloat(idVelocityX);
    float velocityY = logGetFloat(idVelocityY);
    float velocityZ = logGetFloat(idVelocityZ);
    var_x_history[i] = velocityX;
    var_y_history[i] = velocityY;
    var_z_history[i] = velocityZ;

    float min_x = get_min(var_x_history, len_history);
    float max_x = get_max(var_x_history, len_history);
    float min_y = get_min(var_y_history, len_history);
    float max_y = get_max(var_y_history, len_history);
    float min_z = get_min(var_z_history, len_history);
    float max_z = get_max(var_z_history, len_history);
    if (((max_x - min_x) < threshold) &&
        ((max_y - min_y) < threshold) &&
        ((max_z - min_z) < threshold))
    {
      break;
    }
    i = (i + 1) % len_history;
  }
}

void relativeControlTask(void *arg)
{
  static const float_t targetList[7][STATE_DIM_rl] = {{0.0f, 0.0f, 0.0f}, {-0.5f, -0.5f, 0.0f}, {0.5f, 0.5f, 0.0f}};
  static const float_t changeFormationList1[3][STATE_DIM_rl] = {{0.0f, 0.0f, 0.0f}, {-0.5f, 0.5f, 0.0f}, {-1.0f, 0.0f, 0.0f}};
  static const float_t changeFormationList2[3][STATE_DIM_rl] = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.5f, 0.0f}, {-1.0f, 0.0f, 0.0f}};
  ledInit();
  systemWaitStart();
  reset_estimators(); // 判断无人机数值是否收敛
  uint8_t i = 0;
  while (1)
  {
    vTaskDelay(10);
    keepFlying = getOrSetKeepflying(MY_UWB_ADDRESS, keepFlying);
    bool is_connect = relativeInfoRead((float_t *)relaVarInCtrl, &currentNeighborAddressInfo);
    if (is_connect && keepFlying && !isCompleteTaskAndLand)
    {
      // take off
      if (onGround)
      {
        take_off();
        takeoff_tick = xTaskGetTickCount();
      }
      // // control loop
      tickInterval = xTaskGetTickCount() - takeoff_tick;
      // DEBUG_PRINT("tick:%d,rlx:%f,rly:%f,rlraw:%f\n", tickInterval, relaVarInCtrl[0][STATE_rlX], relaVarInCtrl[0][STATE_rlY], relaVarInCtrl[0][STATE_rlYaw]);
      if (tickInterval <= 5000) // stage_0
      {
#ifdef DEBUG_FLY
        DEBUG_PRINT("STAGE_0\n");
#endif
        float_t randomVel = 0.3;      // 0-1 m/s
        flyRandomIn1meter(randomVel); // random flight within first 10 seconds
        targetX = relaVarInCtrl[0][STATE_rlX];
        targetY = relaVarInCtrl[0][STATE_rlY];
      }
      else if ((tickInterval > 5000) && (tickInterval <= 30000)) // stage_1
      {
#ifdef DEBUG_FLY
        DEBUG_PRINT("STAGE_1\n");
#endif
        uint8_t times = get0AiStateInfo(&steerAngle, &collision, &signFromation);
        targetX = -cosf(relaVarInCtrl[0][STATE_rlYaw]) * targetList[MY_UWB_ADDRESS][STATE_rlX] + sinf(relaVarInCtrl[0][STATE_rlYaw]) * targetList[MY_UWB_ADDRESS][STATE_rlY];
        targetY = -sinf(relaVarInCtrl[0][STATE_rlYaw]) * targetList[MY_UWB_ADDRESS][STATE_rlX] - cosf(relaVarInCtrl[0][STATE_rlYaw]) * targetList[MY_UWB_ADDRESS][STATE_rlY];
        if (MY_UWB_ADDRESS == 0)
        {
          flyRandomByAIResult(steerAngle, collision, height);
        }
        else
        {
          formation0asCenter(targetX, targetY, 0, height);
        }
        // NDI_formation0asCenter(targetX, targetY);
      }
      else if ((tickInterval > 30000) && (tickInterval <= 38000)) // stage_2
      {
#ifdef DEBUG_FLY
        DEBUG_PRINT("STAGE_2\n");
#endif
        // for (int i = 0; i < 5; i++)
        // {
        //   ledSet(LED_GREEN_L, 1);
        //   ledSet(LED_GREEN_L, 0);
        //   vTaskDelay(M2T(250));
        //   ledSet(LED_RED_L, 1);
        //   ledSet(LED_RED_L, 0);
        uint8_t times = get0AiStateInfo(&steerAngle, &collision, &signFromation);
        DEBUG_PRINT("signal:%d\n", signFromation);
        // }
        if (MY_UWB_ADDRESS == 0)
        {
          setHoverSetpoint(&setpoint, 0, 0, height, 0);
        }
        else
        {
          formation0asCenter(targetX, targetY, 0, height);
        }
      }
      else if ((tickInterval > 38000) && (tickInterval <= 90000)) // stage_3
      {
#ifdef DEBUG_FLY
        DEBUG_PRINT("STAGE_3\n");
#endif
        if (signFromation == 1)
        {
          targetX = -cosf(relaVarInCtrl[0][STATE_rlYaw]) * changeFormationList1[MY_UWB_ADDRESS][STATE_rlX] + sinf(relaVarInCtrl[0][STATE_rlYaw]) * changeFormationList1[MY_UWB_ADDRESS][STATE_rlY];
          targetY = -sinf(relaVarInCtrl[0][STATE_rlYaw]) * changeFormationList1[MY_UWB_ADDRESS][STATE_rlX] - cosf(relaVarInCtrl[0][STATE_rlYaw]) * changeFormationList1[MY_UWB_ADDRESS][STATE_rlY];
        }
        else if (signFromation == -1)
        {
          targetX = -cosf(relaVarInCtrl[0][STATE_rlYaw]) * changeFormationList2[MY_UWB_ADDRESS][STATE_rlX] + sinf(relaVarInCtrl[0][STATE_rlYaw]) * changeFormationList2[MY_UWB_ADDRESS][STATE_rlY];
          targetY = -sinf(relaVarInCtrl[0][STATE_rlYaw]) * changeFormationList2[MY_UWB_ADDRESS][STATE_rlX] - cosf(relaVarInCtrl[0][STATE_rlYaw]) * changeFormationList2[MY_UWB_ADDRESS][STATE_rlY];
        }
        else
        {
          targetX = -cosf(relaVarInCtrl[0][STATE_rlYaw]) * targetList[MY_UWB_ADDRESS][STATE_rlX] + sinf(relaVarInCtrl[0][STATE_rlYaw]) * targetList[MY_UWB_ADDRESS][STATE_rlY];
          targetY = -sinf(relaVarInCtrl[0][STATE_rlYaw]) * targetList[MY_UWB_ADDRESS][STATE_rlX] - cosf(relaVarInCtrl[0][STATE_rlYaw]) * targetList[MY_UWB_ADDRESS][STATE_rlY];
        }
        if (MY_UWB_ADDRESS == 0)
        {
          setHoverSetpoint(&setpoint, 0, 0, height + 0.2 * signFromation, 0);
        }
        else
        {
          // DEBUG_PRINT("flyheight:%.2f,targetX:%.2f,targetY:%.2f\n", height + 0.2 * signFromation, targetX, targetY);
          formation0asCenter(targetX, targetY, 0, height + 0.2 * signFromation);
        }
      }
      else if (tickInterval > 90000 && tickInterval <= 91000) // stage_4
      {
#ifdef DEBUG_FLY
        DEBUG_PRINT("STAGE_4\n");
#endif
        if (MY_UWB_ADDRESS == 0)
        {
          setHoverSetpoint(&setpoint, 0, 0, height, 0);
        }
        else
        {
          formation0asCenter(targetX, targetY, 0, height);
        }
      }
      else
      {
        // 运行90s之后，落地
        land();
      }
    }
    else
    {
      land();
    }
  }
}

void relativeControlInit(void)
{
  if (isInit)
    return;
  MY_UWB_ADDRESS = getUWBAddress();
  xTaskCreate(relativeControlTask, "relative_Control", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
  isInit = true;
}

PARAM_GROUP_START(relative_ctrl)
PARAM_ADD(PARAM_UINT8, keepFlying, &keepFlying)
PARAM_ADD(PARAM_FLOAT, relaCtrl_p, &relaCtrl_p)
PARAM_ADD(PARAM_FLOAT, relaCtrl_i, &relaCtrl_i)
PARAM_ADD(PARAM_FLOAT, relaCtrl_d, &relaCtrl_d)
PARAM_GROUP_STOP(relative_ctrl)