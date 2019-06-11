
#ifndef DISCRIPTOR_HPP
#define DISCRIPTOR_HPP

#include <Eigen/Dense>

#include <mutex>
#include <memory>
#include <iostream>

#include <boost/shared_ptr.hpp>

#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

using namespace Eigen;
using pointT=pcl::PointXYZ;

namespace prm_localization {

    class discriptor {
    public:
        /**
         * extractEig feature from cloud with spec gridstep
         * @param src_cloud
         * @param gridstep
         */
        void extractEig(const pcl::PointCloud<pointT>::Ptr src_cloud, float gridstep){
            //down sample
            boost::shared_ptr<pcl::VoxelGrid<pointT>> voxelgrid (new pcl::VoxelGrid<pointT>());
            voxelgrid->setLeafSize(gridstep,gridstep,gridstep);
            voxelgrid->setInputCloud(src_cloud);
            pcl::PointCloud<pointT>::Ptr down_src_cloud (new pcl::PointCloud<pointT>());
            voxelgrid->filter(*down_src_cloud);
            //ready gridstep
            boost::shared_array< float > gridsteps (new float_t[4]);
            for (int i = 0; i < 4 ; ++i) {
                gridsteps[i] = (i+1)*2*gridstep;
            }




        }

        const boost::shared_ptr<Matrix<double, 9, -1, 0, 9, 9000>> &getDespcript() const {
            return despcrip;
        }

        const boost::shared_ptr<Matrix<double, 3, -1, 0, 3, 9000>> &getPosition() const {
            return position;
        }

        const boost::shared_ptr<Matrix<double, 12, -1, 0, 12, 9000>> &getNorm() const {
            return norm;
        }

        double getPsize() const {
            return psize;
        }

    private:
        boost::shared_ptr<Matrix<double ,9,-1,0,9,9000>> despcrip;
        boost::shared_ptr<Matrix<double ,3,-1,0,3,9000>> position;
        boost::shared_ptr<Matrix<double ,12,-1,0,12,9000>> norm;
        double psize;







    };

}

#endif //DISCRIPTOR_HPP
