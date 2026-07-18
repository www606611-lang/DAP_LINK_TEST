#include "car_runtime.h"

int main(void)
{
    CarRuntime_Init();
    while (1) {
        CarRuntime_Step();
    }
}
