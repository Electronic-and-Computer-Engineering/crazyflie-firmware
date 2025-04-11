#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "app.h"

#include "commander.h"

#include "FreeRTOS.h"
#include "task.h"

#include "log.h"
#include "param.h"
#include "supervisor.h"

#include "hal_general.h"
#include "dl_general.h"
#include "dl_telemetry.h"

#include "al_mvmts.h"

extern sensor_data_t   sensorDecks;
extern drone_data_t    droneData;
extern State state;

#define DEG_TO_RAD (3.14159265f / 180.0f)
#define RAD_TO_DEG (180.0f / 3.14159265f)

float height_freeze;

void appMain(void)
{
    /* DRONE DATA*/

    droneData.degree = 0.0f;
    droneData.desSpeed = 0.65f; // m/s
    droneData.dirAngle = 45.0f;
    droneData.des_height = 1.2f;

    bool hitLeft = false;
    bool hitRight = false; 
    bool hitFront = false;
    sensorDecks.endNorth = false;
    sensorDecks.endSouth = false;
    sensorDecks.virtualWall = false;

    setpoint_t setpoint;
    vTaskDelay(M2T(1000));  // Startup delay
    DL_init();
    vTaskDelay(M2T(3000));  // Compensating the init-delay of the IMU
    HAL_init();

    while (1) 
    {
        switch(state)
        {
            case ARM:
                if (sensorDecks.armed == false)
                {
                    supervisorRequestArming(true);
                    sensorDecks.armed = true;
                    vTaskDelay(M2T(20));
                    state = IDLE;  
                }
                break;
            case UNARM:
                if (sensorDecks.armed == true)
                {
                    supervisorRequestArming(false);
                    sensorDecks.armed = false;
                    vTaskDelay(M2T(20)); 
                    state = IDLE;  
                }
                break;
            case HOVER:
                if (sensorDecks.armed == true)
                {
                    setStabilizer(&sensorDecks.multiranger, &droneData);
                    setHoverSetpoint(&setpoint, droneData.velFront, droneData.velSide, droneData.height, droneData.degree);            
                    commanderSetSetpoint(&setpoint, 3);                       
                }   
                break;
            case LAND:
                if (sensorDecks.armed == true)
                {
                    while(droneData.height > 0.05f)
                    {
                       droneData.des_height = droneData.des_height - 0.1f;     
                       setStabilizer(&sensorDecks.multiranger, &droneData);
                       setHoverSetpoint(&setpoint, droneData.velFront, droneData.velSide, droneData.height, droneData.degree);       
                       commanderSetSetpoint(&setpoint, 3);
                       vTaskDelay(M2T(250));  
                    }
                    //state = UNARM;
                }
                break;
            case ROTATE180:
                if (sensorDecks.armed == true)
                {   
                    height_freeze = droneData.height;            
                    while(droneData.degree < 180.0f)
                    {
                        setStabilizer(&sensorDecks.multiranger, &droneData);
                        setHoverSetpoint(&setpoint, droneData.velFront, droneData.velSide, height_freeze, droneData.degree);            
                        commanderSetSetpoint(&setpoint, 3);
                        droneData.degree = droneData.degree + 2;
                        if (droneData.degree >= 180.0f)
                        {
                            droneData.degree = 180.0f;
                        }
                        vTaskDelay(M2T(20));                        
                    }  
                    state = HOVER;                     
                }   
                break;
            case ROTATE0:
                height_freeze = droneData.height; 
                if (sensorDecks.armed == true)
                {                                             
                    while(droneData.degree > 0.0f)
                    {
                        setStabilizer(&sensorDecks.multiranger, &droneData);
                        setHoverSetpoint(&setpoint, droneData.velFront, droneData.velSide, height_freeze, droneData.degree);            
                        commanderSetSetpoint(&setpoint, 3);
                        droneData.degree = droneData.degree - 2;
                        if (droneData.degree <= 0.0f)
                        {
                            droneData.degree = 0.0f;
                        }
                        vTaskDelay(M2T(20));                        
                    }  
                    state = HOVER;                       
                }   
                break;
            case ROTATE0DIR:
                height_freeze = droneData.height; 
                if (sensorDecks.armed == true)
                {       
                    while(droneData.degree > 0.0f)
                    {
                        setStabilizer(&sensorDecks.multiranger, &droneData);
                        setHoverSetpoint(&setpoint, droneData.velFront, droneData.velSide, height_freeze, droneData.degree);            
                        commanderSetSetpoint(&setpoint, 3);
                        droneData.degree = droneData.degree - 2;
                        if (droneData.degree <= 0.0f)
                        {
                            droneData.degree = 0.0f;
                        }
                        vTaskDelay(M2T(20));                        
                    }
                    vTaskDelay(M2T(50));    
                    state = DIRECTIONAL;                       
                }   
                break;
                case ROTATE180DIR:
                    height_freeze = droneData.height;
                    if (sensorDecks.armed == true)
                    {               
                        while(droneData.degree < 180.0f)
                        {
                            setStabilizer(&sensorDecks.multiranger, &droneData);
                            setHoverSetpoint(&setpoint, droneData.velFront, droneData.velSide, height_freeze, droneData.degree);            
                            commanderSetSetpoint(&setpoint, 3);
                            droneData.degree = droneData.degree + 2;
                            if (droneData.degree >= 180.0f)
                            {
                                droneData.degree = 180.0f;
                            }
                            vTaskDelay(M2T(20));                        
                        }
                        vTaskDelay(M2T(50));   
                        state = DIRECTIONAL;                     
                    }   
                    break;        
                case DIRECTIONAL:
                    if (sensorDecks.armed == true)
                    {
                        setStabilizerDir(&sensorDecks.multiranger, &droneData);
                        setHoverSetpoint(&setpoint, droneData.velFront, droneData.velSide, droneData.height, droneData.degree);            
                        commanderSetSetpoint(&setpoint, 3);    

                        if((sensorDecks.multiranger.right <= 300.0f || sensorDecks.virtualWall) && (hitRight == false))
                        {
                            droneData.dirAngle *= -1;
                            sensorDecks.virtualWall = false;
                            hitRight = true;
                            hitLeft = false;
                            hitFront = false;
                        }
                        else if((sensorDecks.multiranger.left <= 300.0f || sensorDecks.virtualWall) && (hitLeft == false))
                        {
                            droneData.dirAngle *= -1;
                            sensorDecks.virtualWall = false;
                            hitRight = false;
                            hitLeft = true;
                            hitFront = false;
                        }                 
                        else if((sensorDecks.multiranger.front <= 300.0f || sensorDecks.patchDetected) && (hitFront == false))
                        {
                            droneData.dirAngle *= -1;
                            if (droneData.degree == 180)
                                state = ROTATE0DIR;
                            else if (droneData.degree == 0)
                                state = ROTATE180DIR;

                            hitRight = false;
                            hitLeft = false;
                            hitFront = true;
                        }
                    }
                    break;    
                default:

                break;
        }
        vTaskDelay(M2T(10));  // 10 Hz transmission
    }
}