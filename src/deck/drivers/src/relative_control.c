#include "system.h"
#include "FreeRTOS.h"
#include "task.h"
#include "commander.h"
#include "relative_control.h"
#include "relative_localization.h"
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
#define DEBUG_MODULE "CTRL"
#include "debug.h"

static uint16_t MY_UWB_ADDRESS;
static bool isInit;
static bool onGround = true;               // 无人机当前是否在地面上?
static bool isCompleteTaskAndLand = false; // 无人机是否已经执行了飞行任务并落地?
static bool keepFlying = false;
static setpoint_t setpoint;
static float_t relaVarInCtrl[RANGING_TABLE_SIZE + 1][STATE_DIM_rl];
static currentNeighborAddressInfo_t currentNeighborAddressInfo;
static float_t height = 0.5;
static uint32_t takeoff_tick;
static uint32_t tickInterval;

static float relaCtrl_p = 2.0f;
static float relaCtrl_i = 0.0001f;
static float relaCtrl_d = 0.01f;
// static float NDI_k = 2.0f;

static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawrate)
{
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
}
static float steerAngle;
static float collision;
float signFromation = 0.5f;
static float_t Yaw;
static float_t Vel;
latest3_data_t latest3_data = {{0.0f, 0.0f, 0.0f,0.0f, 0.0f}, {0.0f, 0.0f, 0.0f,0.0f, 0.0f}, {0.5f, 0.5f, 0.5f,0.5f, 0.5f}, 0};
float calAveOf3(float *data)
{
  float sum = 0.0;
  for(int i = 0; i < histSize; i++)
    sum += data[i];
  return (sum / histSize);
}
float calLatest5Signal(float *data)
{
  float sum = 0.0;
  for(int i = 0; i < histSize; i++)
    sum += data[i];

  if(sum <= 1)
    return -1.0f;
  else if(sum > 1&&sum <= 4)
    return 0.0f;
  else
    return 1.0f;
}

static bool processHistoryData(float *steerAngle, float *collision, float *signFromation)
{
  // update history data
  latest3_data.steer_ctl[latest3_data.index] = *steerAngle;
  latest3_data.coll_ctl[latest3_data.index] = *collision;
  latest3_data.sign_ctl[latest3_data.index] = *signFromation;
  latest3_data.index = (latest3_data.index + 1) % histSize;
  if (latest3_data.index == 0)
  {
    // calculate average
    *steerAngle = calAveOf3(latest3_data.steer_ctl);
    *collision = calAveOf3(latest3_data.coll_ctl);
    *signFromation = calLatest5Signal(latest3_data.sign_ctl);
    DEBUG_PRINT("steerAngle:%.2f \tcollision:%.2f \tsignFromation: %.2f\n", *steerAngle, *collision, *signFromation);
    return true;
  }
  return false;
}

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

  Yaw = ((1 - beta) * steerAngle + beta * (PI / 3) * steerAngle);
  Vel = (1 - alpha) * Vel + alpha * (1 - collision) * velMax;
  DEBUG_PRINT("after Filter-Yaw:%.2f\tVel:%.2f\n", Yaw, Vel);
  for (int i = 1; i < 100; i++)
  {
    setHoverSetpoint(&setpoint, Vel, 0, ai_height, Yaw);
    vTaskDelay(M2T(10));
  }
}

#define SIGN(a) ((a >= 0) ? 1 : -1)
static float_t targetX;
static float_t targetY;
static float PreErr_x = 0;
static float PreErr_y = 0;
static float IntErr_x = 0;
static float IntErr_y = 0;
static uint32_t PreTime;
static void formation0asCenter(float_t tarX, float_t tarY,float_t ai_height)
{
  float dt = (float)(xTaskGetTickCount() - PreTime) / configTICK_RATE_HZ;
  PreTime = xTaskGetTickCount();
  if (dt > 1) // skip the first run of the EKF
    return;
  // pid control for formation flight 当前是1号无人机
  float err_x = -(tarX - relaVarInCtrl[0][STATE_rlX]);
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

  setHoverSetpoint(&setpoint, pid_vx, pid_vy, ai_height, 0);
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
  static const float_t targetList[7][STATE_DIM_rl] = {{0.0f, 0.0f, 0.0f}, {-0.5f, -0.5f, 0.0f}, {-1.13f, 0.82f, 0.0f}, {0.43f, 1.33f, 0.0f}, {1.4f, 0.0f, 0.0f}, {0.43f, -1.33f, 0.0f}, {1.5f, 1.5f, 0.0f}};
  systemWaitStart();
  reset_estimators(); // 判断无人机数值是否收敛

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
        // onGround = false;
        takeoff_tick = xTaskGetTickCount();
      }

      // control loop
      tickInterval = xTaskGetTickCount() - takeoff_tick;
      // DEBUG_PRINT("tick:%d,rlx:%f,rly:%f,rlraw:%f\n", tickInterval, relaVarInCtrl[1][STATE_rlX], relaVarInCtrl[1][STATE_rlY], relaVarInCtrl[1][STATE_rlYaw]);
      // DEBUG_PRINT("tick:%f\n", relaVarInCtrl[0][STATE_rlYaw]);
      if (tickInterval <= 5000)
      {

        float_t randomVel = 0.3;      // 0-1 m/s
        flyRandomIn1meter(randomVel); // random flight within first 10 seconds
        targetX = relaVarInCtrl[0][STATE_rlX];
        targetY = relaVarInCtrl[0][STATE_rlY];
      }
      else
      {
        if ((tickInterval > 5000) && (tickInterval <= 15000))
        {
          if (MY_UWB_ADDRESS == 0)
          {
            float_t randomVel = 0.3;
            flyRandomIn1meter(randomVel);
          }
          else
          {
            formation0asCenter(targetX, targetY,height);
          }
          // NDI_formation0asCenter(targetX, targetY);
        }
        else if ((tickInterval > 15000) && (tickInterval <= 90000))
        {
          if (MY_UWB_ADDRESS == 0)
          {
            // float_t randomVel = (rand() / (float)RAND_MAX) * 1;     // 0-1 m/s
            // flyRandomIn1meter(randomVel);
            if (get0AiStateInfo(&steerAngle, &collision, &signFromation) && processHistoryData(&steerAngle, &collision, &signFromation))
            {
              flyRandomByAIResult(steerAngle, collision, height);
            }
            else
            {
              flyRandomByAIResult(0, 0.5, height);
            }
          }
          else
          {
            targetX = -cosf(relaVarInCtrl[0][STATE_rlYaw]) * targetList[MY_UWB_ADDRESS][STATE_rlX] + sinf(relaVarInCtrl[0][STATE_rlYaw]) * targetList[MY_UWB_ADDRESS][STATE_rlY];
            targetY = -sinf(relaVarInCtrl[0][STATE_rlYaw]) * targetList[MY_UWB_ADDRESS][STATE_rlX] - cosf(relaVarInCtrl[0][STATE_rlYaw]) * targetList[MY_UWB_ADDRESS][STATE_rlY];
            formation0asCenter(targetX, targetY,height);
          }
        }
        else if (tickInterval > 90000 && tickInterval <= 100000)
        {
          if (MY_UWB_ADDRESS == 0)
          {
            setHoverSetpoint(&setpoint, 0, 0, height, 0);
          }
          else
          {
            formation0asCenter(targetX, targetY,height);
          }
        }
        else
        {
          // 运行90s之后，落地
          land();
        }
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