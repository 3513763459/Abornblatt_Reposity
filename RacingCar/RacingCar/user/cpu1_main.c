#include "zf_common_typedef.h"
#include "zf_device_mt9v03x.h"
#include "zf_common_clock.h"
#include "seekfree_assistant.h"
#include "Camera.h"
#include "Car.h"
#pragma section all "cpu1_dsram"

void core1_main(void)
{
    disable_Watchdog();
    interrupt_global_enable(0);
    //Cpu1_Init();
    cpu_wait_event_ready();
    while (TRUE)
    {

    }
}
#pragma section all restore

