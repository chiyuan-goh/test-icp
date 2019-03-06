#include <iostream>
#include <string>
#include <fstream>
#include <pcl/common/transforms.h>
#include <pcl/registration/ndt.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>

using namespace std;
using namespace pcl;
//
typedef pcl::PointXYZ Point;
typedef pcl::PointNormal PointNorm;
typedef PointCloud<Point> PC;
//

void get_pcd_pointcloud(PC::Ptr &pc, string filename, bool rmPoints){
    if (io::loadPCDFile<Point> (filename, *pc) == -1) {
        PCL_ERROR ("Couldn't read file \n");
    }

    vector<int> indices;
    removeNaNFromPointCloud(*pc, *pc, indices);

//    if (rmPoints) {
//        PointIndices::Ptr inliers(new PointIndices());
//        ExtractIndices<Point> extract;
//        for (int i = 0; i < (*pc).size(); i++) {
//            Point pt(pc->points[i].x, pc->points[i].y, pc->points[i].z, pc->points[i].intensity);
//            if (pt.x < -5 || pt.x > 4.5) // e.g. remove all pts below zAvg
//            {
//                inliers->indices.push_back(i);
//            }
//        }
//
//        extract.setInputCloud(pc);
//        extract.setIndices(inliers);
//        extract.setNegative(true);
//        extract.filter(*pc);
//    }
}

typedef Eigen::Matrix<float, 4, 4> Mat;
Mat get_ndt_transform(PC::Ptr &prev, PC::Ptr &cur){
    NormalDistributionsTransform<Point, Point> ndt;

    //test-icp params
    ndt.setTransformationEpsilon(0.01);
    ndt.setStepSize(.01);
    ndt.setResolution(0.25);
    ndt.setMaximumIterations(1000);

    ndt.setInputSource(prev);
    ndt.setInputTarget(cur);

    //initial guess
    Eigen::Translation3f init_translate(0., -0.6, 0.4);
    Eigen::AngleAxisf init_rotate(30 * 3.142/180., Eigen::Vector3f::UnitX());
    Eigen::Matrix4f init_guess = (init_translate * init_rotate).matrix();

    //do it
    PC::Ptr output_cloud(new PC);
    cout << "running test-icp.." << endl;
    ndt.align(*output_cloud);

    cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
         << " score: " << ndt.getFitnessScore () << endl;

    return ndt.getFinalTransformation();
}

Mat get_icp_transform(PC::Ptr &prev, PC::Ptr &cur){
    NormalEstimation<Point, PointNorm> ne, ne2;

    ne.setInputCloud(prev);
    ne2.setInputCloud(cur);

    search::KdTree<Point>::Ptr tree (new search::KdTree<Point> ());
    search::KdTree<Point>::Ptr tree2 (new search::KdTree<Point> ());

    ne.setSearchMethod(tree);
    ne2.setSearchMethod(tree2);
    ne.setRadiusSearch(0.05);
    ne2.setRadiusSearch(0.05);
//    ne.setKSearch ( 15 );
//    ne2.setKSearch ( 15 );

    PointCloud<PointNorm>::Ptr cloud_normals (new PointCloud<PointNorm>);
    PointCloud<PointNorm>::Ptr cloud_normals2 (new PointCloud<PointNorm>);

    cout << "computing normals..." << endl;
    ne.compute (*cloud_normals);
    ne2.compute (*cloud_normals2);

    vector<int> indices1, indices2;
    removeNaNNormalsFromPointCloud(*cloud_normals, *cloud_normals, indices1);
    removeNaNNormalsFromPointCloud(*cloud_normals2, *cloud_normals2, indices2);
    removeNaNFromPointCloud(*cloud_normals, *cloud_normals, indices1);
    removeNaNFromPointCloud(*cloud_normals2, *cloud_normals2, indices2);

    for (int i = 0; i < cloud_normals->size(); i++){
        PointNorm n = cloud_normals->points[i];
        cout << n << endl;
//        if (isinf(n.normal_x) || isinf(n.normal_y) || isinf(n.normal_z) || isinf(n.curvature) || isnan(n.normal_x) || isnan(n.normal_y) || isnan(n.normal_z) || isnan(n.curvature)){
//            cout << "point infinite" << endl;
//        }
    }

    cloud_normals->is_dense = true;
    cloud_normals2->is_dense = true;

    cout << "==========================" << endl;

    for (int i = 0; i < cloud_normals2->size(); i++){
        PointNorm n = cloud_normals2->points[i];
//        cout << n << endl;
//        if (isinf(n.normal_x) || isinf(n.normal_y) || isinf(n.normal_z) || isinf(n.curvature) ||  isnan(n.normal_x) || isnan(n.normal_y) || isnan(n.normal_z) || isnan(n.curvature)){
//            cout << "point infinite" << endl;
//        }
    }

    cout << "Running ICP :) " << endl;


    IterativeClosestPointWithNormals<PointNorm, PointNorm> icp;
    icp.setInputSource(cloud_normals);
    icp.setInputTarget(cloud_normals2);
    icp.setMaximumIterations(5);

    //initial guess
    Eigen::Translation3f init_translate(0., -0.6, 0.4);
    Eigen::AngleAxisf init_rotate(30 * 3.142/180., Eigen::Vector3f::UnitX());
    Eigen::Matrix4f init_guess = (init_translate * init_rotate).matrix();

    PointCloud<PointNorm>::Ptr output(new PointCloud<PointNorm>());
    icp.align(*output);
    Eigen::Matrix4f mat = icp.getFinalTransformation();

    cout << "icp has converged:" << icp.hasConverged ()
         << " score: " << icp.getFitnessScore () << endl;

    return mat;
}

void visualize_pc(PC::Ptr &pc) {
    visualization::PCLVisualizer viewer("final pc");
    visualization::PointCloudColorHandlerCustom<Point> cloud_color_handler(pc, 255, 255, 255);
    viewer.addPointCloud(pc, cloud_color_handler, "acc point cloud");

    viewer.addCoordinateSystem(1., "cloud", 0);
    viewer.setBackgroundColor(0.05, .05, .05, 0);

    while(!viewer.wasStopped()) {
        viewer.spinOnce();
    }
}


int main() {
    cout << "hello world~2" << endl;
    string pcd_left = "/home/cy/Documents/robosense_files/calibration/lpcd/1551841018.883930000.pcd";
    string pcd_centre = "/home/cy/Documents/robosense_files/calibration/cpcd/1551841038.483672000.pcd";

    PC::Ptr left_pointcloud(new PC);
    PC::Ptr centre_pointcloud(new PC);
    PC::Ptr accum_pointcloud(new PC);

    get_pcd_pointcloud(left_pointcloud, pcd_left, false);
    get_pcd_pointcloud(centre_pointcloud, pcd_centre, true);
    cout << "left pc:" << left_pointcloud->size() << endl;
    cout << "centre pc:" << centre_pointcloud->size() << endl;

    for (size_t i = 0; i < left_pointcloud->size(); i++){
        if (isnan(left_pointcloud->points[i].x) || isnan(left_pointcloud->points[i].y) || isnan(left_pointcloud->points[i].z) ){
            cout << "point " << i << " is nan " << left_pointcloud->points[i].x << " " << left_pointcloud->points[i].y <<  " "  << left_pointcloud->points[i].z <<  endl;
        }
    }

    cout << "==================" << endl;

    for (size_t i = 0; i < centre_pointcloud->size(); i++){
            if (isnan(centre_pointcloud->points[i].x) || isnan(centre_pointcloud->points[i].y) || isnan(centre_pointcloud->points[i].z)){
                cout << "point " << i << " is nan " << centre_pointcloud->points[i].x << " " << centre_pointcloud->points[i].y <<  " "  << centre_pointcloud->points[i].z <<  endl;

            }
    }


    Mat transform = get_icp_transform(left_pointcloud, centre_pointcloud);
    Eigen::Matrix4f fake_transform = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f fake2;
    fake2 << 0.9901, 0.140, -0.003, 0.0771,
             -0.139, 0.978, -0.153, -0.8196,
             -0.0176, 0.1522, 0.9881, .4742,
             0 ,    0,  0,  1;

    cout << "transofmration: " << endl << transform << endl;

    PC transform_pc;
    transformPointCloud(*left_pointcloud, transform_pc, transform);

    *accum_pointcloud += *centre_pointcloud;
    *accum_pointcloud += transform_pc;

    //visualize point cloud
    visualize_pc(accum_pointcloud);

    return 0;
}