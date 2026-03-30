#include "multi_lidar_calibrator.h"

extern "C"
{
#include "node_api.h"
}

#include <iostream>

int main()
{
    std::cout << "[" << __APP_NAME__ << "] starting...\n";

    void *dora_context = init_dora_context_from_env();
    if (dora_context == nullptr)
    {
        std::cerr << "[" << __APP_NAME__ << "] failed to init dora context\n";
        return -1;
    }

    MultiLidarCalibratorApp app;
    app.Run(dora_context);

    free_dora_context(dora_context);

    std::cout << "[" << __APP_NAME__ << "] exited cleanly\n";
    return 0;
}
