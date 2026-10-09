#include "main.h"
#include "tests/component/test_oled.h"
#include "drivers/oled.h"

void Test_OLED(void)
{
    uint32_t counter = 0U;

    OLED_Init();
    OLED_Clear();
    OLED_Display_On();

    OLED_ShowString(0, 0, "MDP ROBOT");

    while (1)
    {
    	OLED_ShowNumber(0, 16, counter, 3, 16);
    	counter++;
    	OLED_Refresh_Gram();
        HAL_Delay(1000);
    }
}
