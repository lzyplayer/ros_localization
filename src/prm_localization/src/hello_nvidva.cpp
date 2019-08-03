//cpp
#include <vector>
#include <iostream>
#include <string>
//driveworks
#include <dw/core/Context.h>
#include <dw/icp/icp.h>
//pcl
#include <pcl/io/pcd_io.h>
//eigen
#include <Eigen/Dense>


typedef dwLidarPointXYZI dwPoint;
typedef std::vector<dwPoint> dwPCD;
using namespace std;
using namespace Eigen;

std::vector<dwPoint> pcd2dwpc (std::string filepath){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(filepath,*cloud);
    dwPCD cdwpcd;
    cdwpcd.reserve(5000);
    for (size_t i = 0 ; i<cloud->size();i++){
        dwPoint dwp = {cloud->points[i].x,cloud->points[i].y,cloud->points[i].z,1};
        cdwpcd.push_back(dwp);
    }
    return cdwpcd;
}

Eigen::Matrix4f dwt2eigent(dwTransformation& dwt){
    Eigen::Matrix4f ematrix;
    for(int8_t i = 0; i < 16 ; ++i){
        ematrix(i) = dwt.array[i] ;
    }
    return ematrix;
}

dwTransformation eigent2dwt(Eigen::Matrix4f& ematrix){
    dwTransformation deM = DW_IDENTITY_TRANSFORMATION;
    for(int8_t i = 0; i < 16 ; ++i){
        deM.array[i] = ematrix(i);
    }
    return deM;
}

int main(int argc, char **argv)
{

    dwContextHandle_t context   = DW_NULL_HANDLE;
    dwICPHandle_t  icpHandle    = DW_NULL_HANDLE;

    /**instantiate Driveworks SDK context**/
    dwContextParameters sdkParams = {};
    dwVersion sdkVersion;
    dwGetVersion(&sdkVersion);
    dwInitialize(&context, sdkVersion, &sdkParams);

    /** status **/
    std::cout << "Context of Driveworks SDK successfully initialized." <<std::endl;
    std::cout << "Version: " << sdkVersion.major << "." << sdkVersion.minor << "." << sdkVersion.patch << std::endl;
    int32_t gpuCount;
    dwContext_getGPUCount(&gpuCount, context);
    std::cout << "Available GPUs: " << gpuCount << std::endl;

    /**intial dwICP**/
    dwICPParams params{};
    params.maxPoints=30000;
    params.icpType=dwICPType::DW_ICP_TYPE_LIDAR_POINT_CLOUD;
    dwICP_initialize(&icpHandle, &params, context);
    dwICP_setMaxIterations(30, icpHandle);
    dwICP_setConvergenceTolerance(1e-3, 1e-3, icpHandle);

    /**perform icp**/
    dwPCD dataCloud = pcd2dwpc("/home/vickylzy/workspaceROS/MAP_BAG/yuyao/shunyuFac1.pcd");
    dwPCD ModelCloud = pcd2dwpc("/home/vickylzy/workspaceROS/MAP_BAG/yuyao/shunyuFac2.pcd");
    dwICPIterationParams icpPatams{};
    icpPatams.sourcePts =  dataCloud.data();
    icpPatams.targetPts =  ModelCloud.data();
    icpPatams.nSourcePts = dataCloud.size();
    icpPatams.nTargetPts = ModelCloud.size();
    dwTransformation icpPriorPose = DW_IDENTITY_TRANSFORMATION;
    icpPatams.initialSource2Target = &icpPriorPose;
    dwTransformation resultPose;

    clock_t start = clock();
    dwICP_optimize(&resultPose, &icpPatams, icpHandle);
    float64_t icpTime = 1000*(float64_t(clock()) - start ) / CLOCKS_PER_SEC;

    // Get some stats about the ICP perforlmance
    dwICPResultStats icpResultStats;
    dwICP_getLastResultStats(&icpResultStats, icpHandle);
    cout << "ICP Time: " << icpTime << "ms" << endl
         << "Number of Iterations: " << icpResultStats.actualNumIterations << endl
         << "Number of point correspondences: " << icpResultStats.numCorrespondences << endl
         << "RMS cost: " << icpResultStats.rmsCost << endl
         << "Inlier fraction: " << icpResultStats.inlierFraction << endl
         << "ICP Spin Transform: " <<endl <<dwt2eigent(resultPose)<< endl;

    /**release Driveworks SDK context**/
    dwICP_release(&icpHandle);
    dwRelease(&context);
    return 0;
}