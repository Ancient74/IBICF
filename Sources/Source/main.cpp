#include "main.h"
#include "cmsis_os.h"

extern "C" void main_cpp()
{

    while (true)
    {
        osDelay(1);
    }
}
