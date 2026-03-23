#ifndef TASKS_H
#define TASKS_H

#include "Arduino.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Task handles for management
extern TaskHandle_t taskEEPHandle;
extern TaskHandle_t taskUIUpdateHandle;
extern TaskHandle_t taskGPSHandle;
extern TaskHandle_t taskDSGHandle;
extern TaskHandle_t taskSpeedHandle;
extern TaskHandle_t taskRPMHandle;
extern TaskHandle_t taskBroadcastSpeedHandle;

// Task function prototypes
void taskWriteEEP(void *parameter);
void taskUpdateUI(void *parameter);
void taskParseGPS(void *parameter);
void taskParseDSG(void *parameter);
void taskProcessSpeed(void *parameter);
void taskProcessRPM(void *parameter);
void taskBroadcastSpeed(void *parameter);

// Task management functions
void tasksInit();
void tasksSuspendAll();
void tasksResumeAll();
void tasksCleanup();
void setBroadcastSpeedTaskEnabled(bool enabled);

// Task priorities (configMAX_PRIORITIES - 1 is highest)
#define TASK_PRIORITY_EEPROM 1
#define TASK_PRIORITY_UI 2
#define TASK_PRIORITY_GPS 1
#define TASK_PRIORITY_DSG 1
#define TASK_PRIORITY_SPEED 3
#define TASK_PRIORITY_RPM 3
#define TASK_PRIORITY_BROADCAST 2

// Stack sizes (in 32-bit words, not bytes)
#define STACK_SIZE_EEPROM 2048
#define STACK_SIZE_UI 3072
#define STACK_SIZE_GPS 2048
#define STACK_SIZE_DSG 2048
#define STACK_SIZE_SPEED 3072
#define STACK_SIZE_RPM 2048
#define STACK_SIZE_BROADCAST 2048

// Task delays (in milliseconds)
#define DELAY_EEPROM 5000
#define DELAY_UI 200
#define DELAY_GPS 100
#define DELAY_DSG 50
#define DELAY_SPEED 50
#define DELAY_RPM 50
#define DELAY_BROADCAST_SPEED 20

#endif // TASKS_H
