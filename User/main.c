#include "stm32f10x.h"
#include "app_main.h"
#include "bsp_power.h"
#include "drv_console.h"
#include <stdio.h>

int main(void)
{
    if (BSP_Power_BootCheckAndHold() == 0)
    {
        while (1)
        {
        }
    }

    Console_Init(115200);
    /*
     * 这里 FreeRTOS 和日志队列还没有启动，只能直接输出早期启动信息。
     * 系统进入任务调度后，业务日志统一改由 log_service 处理。
     */
    printf("[BOOT] PWR_HOLD on\r\n");

    if (BSP_Power_WaitKeyRelease(1500))
    {
        printf("[BOOT] power key released\r\n");
    }
    else
    {
        printf("[BOOT] power key release timeout, continue\r\n");
    }

    freeRTOS_start();
    while (1)
    {

    }
}
