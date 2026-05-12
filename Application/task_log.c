#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "log_service.h"
#include "message_def.h"
#include "task_log.h"
#include <stdio.h>

#define LOG_TASK_POLL_MS                     1000
#define LOG_TASK_STACK_WATERMARK_PERIOD_MS   10000

static void Log_Task_PrintStackWatermark(TickType_t *last_tick)
{
#if (INCLUDE_uxTaskGetStackHighWaterMark == 1)
   TickType_t now;

   if (last_tick == 0)
   {
       return;
   }

   now = xTaskGetTickCount();
   if ((*last_tick == 0) ||
       ((now - *last_tick) >= pdMS_TO_TICKS(LOG_TASK_STACK_WATERMARK_PERIOD_MS)))
   {
       *last_tick = now;

       if (uart_mutex != NULL)
       {
           xSemaphoreTake(uart_mutex, portMAX_DELAY);
       }

       printf("[INFO] STACK log_task free=%u words\r\n",
              (unsigned int)uxTaskGetStackHighWaterMark(NULL));

       if (uart_mutex != NULL)
       {
           xSemaphoreGive(uart_mutex);
       }
   }
#else
   (void)last_tick;
#endif
}

static const char *Log_LevelToText(LogLevel_t level)
{
   switch (level)
   {
   case LOG_LEVEL_INFO:
       return "INFO";

   case LOG_LEVEL_WARN:
       return "WARN";

   case LOG_LEVEL_ERROR:
       return "ERROR";

   default:
       return "UNK";
   }
}

void log_task(void *pvParameters)
{
   LogMessage_t msg;
   TickType_t last_stack_log_tick = 0;

   (void)pvParameters;

   while (1)
   {
       if (xQueueReceive(log_queue, &msg, pdMS_TO_TICKS(LOG_TASK_POLL_MS)) == pdTRUE)
       {
           if (uart_mutex != NULL)
           {
               xSemaphoreTake(uart_mutex, portMAX_DELAY);
           }

           printf("[%s] %s\r\n", Log_LevelToText(msg.level), msg.text);

           if (uart_mutex != NULL)
           {
               xSemaphoreGive(uart_mutex);
           }
       }

       Log_Task_PrintStackWatermark(&last_stack_log_tick);
   }
}
