#include <math.h>

#include "al_mvmts.h"
#include "dl_general.h"
#include "commander.h"

#define MAX(a,b) ((a>b)?a:b)
#define MIN(a,b) ((a<b)?a:b)
#define HEIGHT_ALPHA 0.8f

static const float velMax = 0.5f;
static const uint16_t radius = 300;
static const uint16_t radius_up_down = 100;
static const float up_down_delta = 0.002f;
static float oldHeight = 0.0f;

float factor = velMax/radius;

void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawrate)
{
  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;


  setpoint->mode.yaw = modeAbs;
  setpoint->attitude.yaw = yawrate;

  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;
  setpoint->velocity.x = vx;
  setpoint->velocity.y = vy;

  setpoint->velocity_body = true;
}

void setStabilizer(multiranger_data_t *rngData, drone_data_t *drnData)
{
    uint16_t left_o = radius - MIN(rngData->left, radius);
    uint16_t right_o = radius - MIN(rngData->right, radius);
    drnData->l_comp = (-1) * left_o * factor;
    drnData->r_comp = right_o * factor;
    drnData->velSide = drnData->l_comp + drnData->r_comp;

    uint16_t front_o = radius - MIN(rngData->front, radius);
    uint16_t back_o = radius - MIN(rngData->back, radius);
    drnData->f_comp = (-1) * front_o * factor;
    drnData->b_comp = back_o * factor;
    drnData->velFront = drnData->b_comp + drnData->f_comp;

      // we want to go up when there are obstacles (hands) closer than radius_up_down on both sides
      if(rngData->left < radius_up_down && rngData->right < radius_up_down)
      {
        drnData->des_height += up_down_delta;
      }

      // we want to go down when there are obstacles (hands) closer than radius_up_down in front and back (or there is something on top)
      if((rngData->front < radius_up_down && rngData->back < radius_up_down) || rngData->up < radius)
      {
        drnData->des_height -= up_down_delta;
      }

      uint16_t up_o = radius - MIN(rngData->up, radius);
      drnData->height = drnData->des_height - up_o/1000.0f;

}

void setStabilizerDir(multiranger_data_t *rngData, drone_data_t *drnData)
{
    float angleRad = drnData->dirAngle * (3.14159265f / 180.0f);

    uint16_t left_o = radius - MIN(rngData->left, radius);
    uint16_t right_o = radius - MIN(rngData->right, radius);
    drnData->l_comp = (-1) * left_o * factor;
    drnData->r_comp = right_o * factor;
    drnData->velSide = sinf(angleRad) * drnData->desSpeed;

    uint16_t front_o = radius - MIN(rngData->front, radius);
    uint16_t back_o = radius - MIN(rngData->back, radius);
    drnData->f_comp = (-1) * front_o * factor;
    drnData->b_comp = back_o * factor;
    drnData->velFront = cosf(angleRad) * drnData->desSpeed;;

      // we want to go up when there are obstacles (hands) closer than radius_up_down on both sides
      if(rngData->left < radius_up_down && rngData->right < radius_up_down)
      {
        drnData->des_height += up_down_delta;
      }

      // we want to go down when there are obstacles (hands) closer than radius_up_down in front and back (or there is something on top)
      if((rngData->front < radius_up_down && rngData->back < radius_up_down) || rngData->up < radius)
      {
        drnData->des_height -= up_down_delta;
      }

      uint16_t up_o = radius - MIN(rngData->up, radius);
      drnData->height = drnData->des_height - up_o/1000.0f;
      drnData->height = HEIGHT_ALPHA * oldHeight + (1.0f - HEIGHT_ALPHA) * drnData->height;

}