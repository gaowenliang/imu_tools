#ifndef ROS_UTLS_H
#define ROS_UTLS_H

#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Eigen>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>

namespace ros_utils
{

typedef message_filters::Subscriber< sensor_msgs::Image > ImageSubscriber;

typedef message_filters::sync_policies::ApproximateTime< sensor_msgs::Image, //
                                                         sensor_msgs::Image >
AppSync2Images;

typedef message_filters::sync_policies::ApproximateTime< sensor_msgs::Image, //
                                                         sensor_msgs::Image,
                                                         sensor_msgs::Image >
AppSync3Images;

typedef message_filters::sync_policies::ApproximateTime< sensor_msgs::Image, //
                                                         sensor_msgs::Image,
                                                         sensor_msgs::Image,
                                                         sensor_msgs::Image >
AppSync4Images;

typedef message_filters::sync_policies::ApproximateTime< sensor_msgs::Image, //
                                                         sensor_msgs::Image,
                                                         sensor_msgs::Image,
                                                         sensor_msgs::Image,
                                                         sensor_msgs::Image >
AppSync5Images;

typedef message_filters::sync_policies::ApproximateTime< sensor_msgs::Image, //
                                                         sensor_msgs::Image,
                                                         sensor_msgs::Image,
                                                         sensor_msgs::Image,
                                                         sensor_msgs::Image,
                                                         sensor_msgs::Image >
AppSync6Images;

typedef message_filters::sync_policies::ApproximateTime< sensor_msgs::Image, //
                                                         sensor_msgs::Image,
                                                         sensor_msgs::Image,
                                                         sensor_msgs::Image,
                                                         sensor_msgs::Image,
                                                         sensor_msgs::Image,
                                                         sensor_msgs::Image,
                                                         sensor_msgs::Image >
AppSync8Images;

typedef message_filters::Synchronizer< AppSync2Images > App2ImgSynchronizer;
typedef message_filters::Synchronizer< AppSync3Images > App3ImgSynchronizer;
typedef message_filters::Synchronizer< AppSync4Images > App4ImgSynchronizer;
typedef message_filters::Synchronizer< AppSync5Images > App5ImgSynchronizer;
typedef message_filters::Synchronizer< AppSync6Images > App6ImgSynchronizer;
typedef message_filters::Synchronizer< AppSync8Images > App8ImgSynchronizer;

template< typename T >
T
readParam( ros::NodeHandle& n, std::string name )
{
    T ans;
    if ( n.getParam( name, ans ) )
    {
        ROS_INFO_STREAM( "Loaded " << name << ": " << ans );
    }
    else
    {
        ROS_ERROR_STREAM( "Failed to load " << name );
        n.shutdown( );
    }
    return ans;
}

inline void
sendDepthImage( ros::Publisher& pub, const ros::Time timeStamp, const std::string frame_id, const cv::Mat& depth )
{
    cv_bridge::CvImage out_msg;

    out_msg.header.stamp    = timeStamp;
    out_msg.header.frame_id = frame_id;
    out_msg.encoding        = sensor_msgs::image_encodings::TYPE_32FC1;
    out_msg.image           = depth.clone( );

    pub.publish( out_msg.toImageMsg( ) );
}

inline void
sendCloud( ros::Publisher& pub,
           const ros::Time timestamp,
           const cv::Mat& dense_points_,
           const cv::Mat& un_img_l0,
           Eigen::Matrix3f K1,
           Eigen::Matrix3f R_wc )
{
#define DOWNSAMPLE 0
    const float DEP_INF = 1000.0f;

    sensor_msgs::PointCloud2Ptr points( new sensor_msgs::PointCloud2 );
    points->header.stamp = timestamp;
    ; // key_frame_data.header;
    // TODO: since no publish tf, frame_id need to be modify
    points->header.frame_id = "ref_frame";

    points->height = dense_points_.rows;
    points->width  = dense_points_.cols;
    points->fields.resize( 4 );
    points->fields[0].name     = "x";
    points->fields[0].offset   = 0;
    points->fields[0].count    = 1;
    points->fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    points->fields[1].name     = "y";
    points->fields[1].offset   = 4;
    points->fields[1].count    = 1;
    points->fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    points->fields[2].name     = "z";
    points->fields[2].offset   = 8;
    points->fields[2].count    = 1;
    points->fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    points->fields[3].name     = "rgb";
    points->fields[3].offset   = 12;
    points->fields[3].count    = 1;
    points->fields[3].datatype = sensor_msgs::PointField::FLOAT32;
    // points.is_bigendian = false; ???
    points->point_step = 16;
    points->row_step   = points->point_step * points->width;
    points->data.resize( points->row_step * points->height );
    points->is_dense = false; // there may be invalid points

    float fx = K1( 0, 0 );
    float fy = K1( 1, 1 );
    float cx = K1( 0, 2 );
    float cy = K1( 1, 2 );
    //    std::cout<<" fx "<< fx <<std::endl;
    //    std::cout<<" fy "<< fy <<std::endl;
    //    std::cout<<" cx "<< cx <<std::endl;
    //    std::cout<<" cy "<< cy <<std::endl;

    float bad_point = std::numeric_limits< float >::quiet_NaN( );
    int i           = 0;
    //        std::cout<<" dense_points_.rows "<< dense_points_.rows <<std::endl;
    //        std::cout<<" dense_points_.cols "<< dense_points_.cols <<std::endl;
    //        std::cout<<" un_img_l0.rows "<< un_img_l0.rows <<std::endl;
    //        std::cout<<" un_img_l0.cols "<< un_img_l0.cols <<std::endl;
    for ( int32_t u = 0; u < dense_points_.rows; ++u )
    {
        for ( int32_t v = 0; v < dense_points_.cols; ++v, ++i )
        {
            float dep = dense_points_.at< float >( u, v );
            // TODO: not correct here, since use desire focal length
            // float x = dep * (v - K1.at<double>(0, 2) /*/ (1 << DOWNSAMPLE)*/) /
            // (K1.at<double>(0, 0) /*/ (1
            // << DOWNSAMPLE)*/);
            // float y = dep * (u - K1.at<double>(1, 2) /*/ (1 << DOWNSAMPLE)*/) /
            // (K1.at<double>(1, 1) /*/ (1
            // << DOWNSAMPLE)*/);

            Eigen::Vector3f Point_c( dep * ( v - cx ) / fx, dep * ( u - cy ) / fy, dep );
            Eigen::Vector3f Point_w = R_wc * Point_c;

            if ( dep > 5 )
                continue;
            if ( dep < 0.95 )
                continue;
            if ( Point_w( 2 ) > 0.5 )
                continue;

            if ( dep < 0 )
                continue;
            if ( dep < DEP_INF )
            {
                uint8_t g   = un_img_l0.at< uint8_t >( u, v );
                int32_t rgb = ( g << 16 ) | ( g << 8 ) | g;
                memcpy( &points->data[i * points->point_step + 0], &Point_w( 0 ), sizeof( float ) );
                memcpy( &points->data[i * points->point_step + 4], &Point_w( 1 ), sizeof( float ) );
                memcpy( &points->data[i * points->point_step + 8], &Point_w( 2 ), sizeof( float ) );
                memcpy( &points->data[i * points->point_step + 12], &rgb, sizeof( int32_t ) );
            }
            else
            {
                memcpy( &points->data[i * points->point_step + 0], &bad_point, sizeof( float ) );
                memcpy( &points->data[i * points->point_step + 4], &bad_point, sizeof( float ) );
                memcpy( &points->data[i * points->point_step + 8], &bad_point, sizeof( float ) );
                memcpy( &points->data[i * points->point_step + 12], &bad_point, sizeof( float ) );
            }
        }
    }
    pub.publish( points );
}

inline void
sendCloud( ros::Publisher& pub,
           const ros::Time timestamp,
           const cv::Mat& dense_points_,
           const cv::Mat& un_img_l0,
           Eigen::Matrix3f K1,
           Eigen::Matrix3f R_wc,
           Eigen::Vector3f T_w )
{
#define DOWNSAMPLE 0
    const float DEP_INF = 1000.0f;

    sensor_msgs::PointCloud2Ptr points( new sensor_msgs::PointCloud2 );
    points->header.stamp = timestamp;
    ; // key_frame_data.header;
    // TODO: since no publish tf, frame_id need to be modify
    points->header.frame_id = "ref_frame";

    points->height = dense_points_.rows;
    points->width  = dense_points_.cols;
    points->fields.resize( 4 );
    points->fields[0].name     = "x";
    points->fields[0].offset   = 0;
    points->fields[0].count    = 1;
    points->fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    points->fields[1].name     = "y";
    points->fields[1].offset   = 4;
    points->fields[1].count    = 1;
    points->fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    points->fields[2].name     = "z";
    points->fields[2].offset   = 8;
    points->fields[2].count    = 1;
    points->fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    points->fields[3].name     = "rgb";
    points->fields[3].offset   = 12;
    points->fields[3].count    = 1;
    points->fields[3].datatype = sensor_msgs::PointField::FLOAT32;
    points->is_bigendian       = false;
    points->point_step         = sizeof( float ) * 4;
    points->row_step           = points->point_step * points->width;
    points->data.resize( points->row_step * points->height );
    points->is_dense = true;

    float fx = K1( 0, 0 );
    float fy = K1( 1, 1 );
    float cx = K1( 0, 2 );
    float cy = K1( 1, 2 );
    //    std::cout<<" fx "<< fx <<std::endl;
    //    std::cout<<" fy "<< fy <<std::endl;
    //    std::cout<<" cx "<< cx <<std::endl;
    //    std::cout<<" cy "<< cy <<std::endl;

    float bad_point = std::numeric_limits< float >::quiet_NaN( );
    int i           = 0;
    //        std::cout<<" dense_points_.rows "<< dense_points_.rows <<std::endl;
    //        std::cout<<" dense_points_.cols "<< dense_points_.cols <<std::endl;
    //        std::cout<<" un_img_l0.rows "<< un_img_l0.rows <<std::endl;
    //        std::cout<<" un_img_l0.cols "<< un_img_l0.cols <<std::endl;
    for ( int32_t u = 0; u < dense_points_.rows; ++u )
    {
        for ( int32_t v = 0; v < dense_points_.cols; ++v, ++i )
        {
            float dep = dense_points_.at< float >( u, v );
            // TODO: not correct here, since use desire focal length
            // float x = dep * (v - K1.at<double>(0, 2) /*/ (1 << DOWNSAMPLE)*/) /
            // (K1.at<double>(0, 0) /*/ (1
            // << DOWNSAMPLE)*/);
            // float y = dep * (u - K1.at<double>(1, 2) /*/ (1 << DOWNSAMPLE)*/) /
            // (K1.at<double>(1, 1) /*/ (1
            // << DOWNSAMPLE)*/);

            Eigen::Vector3f Point_c( dep * ( v - cx ) / fx, dep * ( u - cy ) / fy, dep );
            Eigen::Vector3f Point_w = R_wc * Point_c + T_w;

            // if ( dep > 5 )
            //     continue;
            // if ( dep < 0.95 )
            //     continue;
            // if ( Point_w( 2 ) > 0.5 )
            //     continue;

            if ( dep < 0 )
                continue;
            if ( dep < DEP_INF )
            {
                uint8_t g   = un_img_l0.at< uint8_t >( u, v );
                int32_t rgb = ( g << 16 ) | ( g << 8 ) | g;
                memcpy( &points->data[i * points->point_step + 0], &Point_w( 0 ), sizeof( float ) );
                memcpy( &points->data[i * points->point_step + 4], &Point_w( 1 ), sizeof( float ) );
                memcpy( &points->data[i * points->point_step + 8], &Point_w( 2 ), sizeof( float ) );
                memcpy( &points->data[i * points->point_step + 12], &rgb, sizeof( int32_t ) );
            }
            else
            {
                memcpy( &points->data[i * points->point_step + 0], &bad_point, sizeof( float ) );
                memcpy( &points->data[i * points->point_step + 4], &bad_point, sizeof( float ) );
                memcpy( &points->data[i * points->point_step + 8], &bad_point, sizeof( float ) );
                memcpy( &points->data[i * points->point_step + 12], &bad_point, sizeof( float ) );
            }
        }
    }
    pub.publish( points );
}

inline void
pointCloudPub( ros::Publisher& pub, cv::Mat depth, cv::Mat image )
{
    int w, h;
    w = ( int )( depth.cols );
    h = ( int )( depth.rows );

    sensor_msgs::PointCloud2 imagePoint;
    imagePoint.header.stamp    = ros::Time::now( );
    imagePoint.header.frame_id = "world";
    imagePoint.height          = h;
    imagePoint.width           = w;
    imagePoint.fields.resize( 4 );
    imagePoint.fields[0].name     = "x";
    imagePoint.fields[0].offset   = 0;
    imagePoint.fields[0].count    = 1;
    imagePoint.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    imagePoint.fields[1].name     = "y";
    imagePoint.fields[1].offset   = 4;
    imagePoint.fields[1].count    = 1;
    imagePoint.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    imagePoint.fields[2].name     = "z";
    imagePoint.fields[2].offset   = 8;
    imagePoint.fields[2].count    = 1;
    imagePoint.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    imagePoint.fields[3].name     = "rgb";
    imagePoint.fields[3].offset   = 12;
    imagePoint.fields[3].count    = 1;
    imagePoint.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
    imagePoint.is_bigendian       = false;
    imagePoint.point_step         = sizeof( float ) * 4;
    imagePoint.row_step           = imagePoint.point_step * imagePoint.width;
    imagePoint.data.resize( imagePoint.row_step * imagePoint.height );
    imagePoint.is_dense = true;

    int i = 0;
    for ( int row_index = 0; row_index < depth.rows; ++row_index )
    {
        for ( int col_index = 0; col_index < depth.cols; ++col_index, ++i )
        {
            float z = depth.at< float >( row_index, col_index );
            float x = ( float )col_index * z;
            float y = ( float )row_index * z;

            uint g;
            int32_t rgb;

            g   = ( uchar )image.at< uchar >( row_index, col_index );
            rgb = ( g << 16 ) | ( g << 8 ) | g;

            memcpy( &imagePoint.data[i * imagePoint.point_step + 0], &x, sizeof( float ) );
            memcpy( &imagePoint.data[i * imagePoint.point_step + 4], &y, sizeof( float ) );
            memcpy( &imagePoint.data[i * imagePoint.point_step + 8], &z, sizeof( float ) );
            memcpy( &imagePoint.data[i * imagePoint.point_step + 12], &rgb, sizeof( int32_t ) );
        }
    }

    pub.publish( imagePoint );
}
}
#endif // ROS_UTLS_H
