/**
 * ,---------,       ____                         ______   
 * |  ,-^-,  |      / __ \____  ____  ____  _____/  __  \____  ____  ____
 * | (  O  ) |     / / / / ___\/ __ \/ __ \/ ___/  /_/  / __ \/ __ \/ __ \
 * | / ,--´  |    / /_/ / /   / /_/ / / / / /__/  _____/ /_/ / / / / /_/ /
 *    +------`   /_____/_/    \____/ / / /____/__/     \____/ / / / ____/
 *                                                               / /_
 *                                                              /___/
 * Crazyflie Pong Firmware
 *
 * dronePong.c - Interactive Crazyflie Pong Game with BLE data transmission.
 */

 #include <string.h>
 #include <stdint.h>
 #include <stdbool.h>
 
 #include "app.h"
 #include "app_channel.h"
 #include "commander.h"
 #include "FreeRTOS.h"
 #include "task.h"
 #include "timers.h"
 #include "debug.h"
 #include "led.h"
 #include "log.h"
 #include "param.h"
 #include "supervisor.h"
 
 #define DEBUG_MODULE "DRONEPONG"
 
 #define SENSOR_UPDATE_INTERVAL pdMS_TO_TICKS(10)  // 100Hz state updates
 #define BLE_SEND_INTERVAL pdMS_TO_TICKS(100)      // 10Hz BLE data transmission
 
 typedef enum {
     idle,
     lowUnlock,
     unlocked,
     stopping
 } State;
 
 static State state = idle;
 
 static const uint16_t unlockThLow = 100;
 static const uint16_t unlockThHigh = 300;
 static const uint16_t stoppedTh = 500;
 static const float velMax = 1.0f;
 static const uint16_t radius = 300;
 static const uint16_t radius_up_down = 100;
 static const float up_down_delta = 0.002f;
 static float height_sp = 0.5f;
 
 #define MAX(a,b) ((a>b)?a:b)
 #define MIN(a,b) ((a<b)?a:b)
 
 // Struct to hold sensor data
 typedef struct {
     uint16_t up;
     uint16_t left;
     uint16_t right;
     uint16_t front;
     uint16_t back;
     uint8_t positioningInit;
     uint8_t multirangerInit;
 } SensorData;
 
 uint8_t tester = 0xAA;
 static setpoint_t setpoint;
 static SensorData sensorData;
 
 // Sensor variable IDs
 static logVarId_t idUp, idLeft, idRight, idFront, idBack;
 static paramVarId_t idPositioningDeck, idMultiranger;
 
 // Function Declarations
 void initSensorData();
 void fetchSensorData(SensorData *data);
 void handleArming();
 void stateMachine(setpoint_t *setpoint, SensorData *data);
 void sensorUpdateCallback(TimerHandle_t xTimer);
 void bleSendCallback(TimerHandle_t xTimer);
 static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawrate);
 
 // Timers
 static TimerHandle_t sensorTimer;
 static TimerHandle_t bleTimer;
 
 void appMain() {
     vTaskDelay(M2T(3000));  // Startup delay
 
     initSensorData();       // Initialize sensor variables
     handleArming();         // Handle arming
 
     DEBUG_PRINT("BLE Debug: Crazyflie is running...\n");
 
     // Turn on GREEN LED at startup
     ledSet(LED_GREEN_L, true);
 
     // Create and start timers
     sensorTimer = xTimerCreate("SensorTimer", SENSOR_UPDATE_INTERVAL, pdTRUE, NULL, sensorUpdateCallback);
     bleTimer = xTimerCreate("BLETimer", BLE_SEND_INTERVAL, pdTRUE, NULL, bleSendCallback);
 
     if (sensorTimer) xTimerStart(sensorTimer, 0);
     if (bleTimer) xTimerStart(bleTimer, 0);
 
     while (1) {
         vTaskDelay(M2T(1000));
     }
 }
 
 // ====================================================
 // FUNCTION DEFINITIONS
 // ====================================================
 
 // Initialize sensor log variables
 void initSensorData() {
     idUp = logGetVarId("range", "up");
     idLeft = logGetVarId("range", "left");
     idRight = logGetVarId("range", "right");
     idFront = logGetVarId("range", "front");
     idBack = logGetVarId("range", "back");
 
     idPositioningDeck = paramGetVarId("deck", "bcFlow2");
     idMultiranger = paramGetVarId("deck", "bcMultiranger");
 }
 
 // Fetch current sensor data and store in struct
 void fetchSensorData(SensorData *data) {
     data->up = logGetUint(idUp);
     data->left = logGetUint(idLeft);
     data->right = logGetUint(idRight);
     data->front = logGetUint(idFront);
     data->back = logGetUint(idBack);
 
     data->positioningInit = paramGetUint(idPositioningDeck);
     data->multirangerInit = paramGetUint(idMultiranger);
 }
 
 // Timer callback - State Machine Updates (100Hz)
 void sensorUpdateCallback(TimerHandle_t xTimer) {
     fetchSensorData(&sensorData);
     stateMachine(&setpoint, &sensorData);
 }
 
 // Timer callback - BLE Data Transmission (10Hz)
 void bleSendCallback(TimerHandle_t xTimer) {
     appchannelSendDataPacket((uint8_t*)&tester, 1);
 
     // Toggle RED LED to indicate BLE data transmission
     static bool ledState = false;
     ledSet(LED_RED_L, ledState);
     ledState = !ledState;
 }
 
 // Handle the arming process
 void handleArming() {
     DEBUG_PRINT("Checking if we can arm the drone...\n");
 
     if (supervisorCanArm()) {
         bool armed = supervisorRequestArming(true);
         if (armed) {
             DEBUG_PRINT("Drone is now ARMED!\n");
         } else {
             DEBUG_PRINT("Arming request was NOT granted!\n");
         }
     } else {
         DEBUG_PRINT("Drone is NOT ALLOWED to arm!\n");
     }
 }
 
 // State machine handling the drone's behavior
 void stateMachine(setpoint_t *setpoint, SensorData *data) {
     if (state == unlocked) {
         float factor = velMax / radius;
         float velSide = factor * (MIN(data->right, radius) - MIN(data->left, radius));
         float velFront = factor * (MIN(data->back, radius) - MIN(data->front, radius));
 
         if (data->left < radius_up_down && data->right < radius_up_down) height_sp += up_down_delta;
         if ((data->front < radius_up_down && data->back < radius_up_down) || data->up < radius) height_sp -= up_down_delta;
 
         float height = height_sp - (MIN(data->up, radius) / 1000.0f);
         setHoverSetpoint(setpoint, velFront, velSide, height, 0);
         commanderSetSetpoint(setpoint, 3);
 
         if (height < 0.1f) {
             state = stopping;
             DEBUG_PRINT("Stopping...\n");
         }
     } else {
         if (state == stopping && data->up > stoppedTh) state = idle;
         if (data->up < unlockThLow && state == idle) state = lowUnlock;
         if (data->up > unlockThHigh && state == lowUnlock) state = unlocked;
     }
 }
 
 // Setpoint helper function
 static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawrate) {
     setpoint->mode.z = modeAbs;
     setpoint->position.z = z;
     setpoint->mode.yaw = modeVelocity;
     setpoint->attitudeRate.yaw = yawrate;
     setpoint->mode.x = modeVelocity;
     setpoint->mode.y = modeVelocity;
     setpoint->velocity.x = vx;
     setpoint->velocity.y = vy;
     setpoint->velocity_body = true;
 }
 