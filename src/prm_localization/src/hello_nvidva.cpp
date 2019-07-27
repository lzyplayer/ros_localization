#include <iostream>
#include <dw/core/Context.h>
int main(int argc, char **argv)
{
    dwContextHandle_t sdk   = DW_NULL_HANDLE;
    // instantiate Driveworks SDK context
    dwContextParameters sdkParams = {};
    dwVersion sdkVersion;
    dwGetVersion(&sdkVersion);
    dwInitialize(&sdk, sdkVersion, &sdkParams);
    std::cout << "Context of Driveworks SDK successfully initialized." <<std::endl;
    std::cout << "Version: " << sdkVersion.major << "." << sdkVersion.minor << "." << sdkVersion.patch << std::endl;
    int32_t gpuCount;
    dwContext_getGPUCount(&gpuCount, sdk);
    std::cout << "Available GPUs: " << gpuCount << std::endl;
    // release Driveworks SDK context
    dwRelease(&sdk);
    return 0;
}