//cpp
#include <vector>
#include <iostream>
#include <string>
//driveworks
#include <dw/core/Context.h>
#include <dw/icp/icp.h>
//pcl
#include <pcl/io/pcd_io.h>


typedef dwLidarPointXYZI dwPoint;
typedef std::vector<dwPoint> dwPCD;
using namespace std;

std::vector<dwPoint> pcd2dwpc (std::string filepath){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(filepath,*cloud);
    dwPCD cdwpcd(5000);
    for (size_t i = 0 ; i<cloud->size();i++){
        dwPoint dwp = {cloud->points[0].x,cloud->points[0].y,cloud->points[0].z,1};
        cdwpcd.push_back(dwp);
    }
    return cdwpcd;
}

int main(int argc, char **argv)
{
    dwContextHandle_t sdk   = DW_NULL_HANDLE;
    // instantiate Driveworks SDK context
    dwContextParameters sdkParams = {};
    dwVersion sdkVersion;
    dwGetVersion(&sdkVersion);
    dwInitialize(&sdk, sdkVersion, &sdkParams);
    /** status **/
    std::cout << "Context of Driveworks SDK successfully initialized." <<std::endl;
    std::cout << "Version: " << sdkVersion.major << "." << sdkVersion.minor << "." << sdkVersion.patch << std::endl;
    int32_t gpuCount;
    dwContext_getGPUCount(&gpuCount, sdk);
    std::cout << "Available GPUs: " << gpuCount << std::endl;
    /** prepare data **/
    dwPCD dataCloud = pcd2dwpc("/home/vickylzy/workspaceROS/MAP_BAG/yuyao/shunyuFac1.pcd");
    dwPCD ModelCloud = pcd2dwpc("/home/vickylzy/workspaceROS/MAP_BAG/yuyao/shunyuFac2.pcd");
   





    // release Driveworks SDK context
    dwRelease(&sdk);
    return 0;
}