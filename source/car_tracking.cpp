#include "common.h"

// OpenCV 
#include <opencv2/core/core.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/video/tracking.hpp>

// ROS
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/geometry.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

std::vector<cv::KalmanFilter> kf_vector{};
std::vector<int> car_id{};
std::vector<int> obj_id{};
std::vector<int> kf_id{};
ros::Publisher pub_cc;
ros::Publisher marker_pub;
bool first_frame=true;
bool second_frame=true;

// Function to calculate euclidean distance between two points
float euclidean_distance(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2)
{
  return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
}

// Function to find index of the smallest element in a matrix
std::pair<int,int> find_index_of_min(std::vector<std::vector<float>> input_dist_mat)
{
  std::pair<int,int>min_index;
  float min_element{std::numeric_limits<float>::max()};

  for (int i=0; i<input_dist_mat.size();i++) //rows
    {
      for(int j=0;j<input_dist_mat.at(0).size();j++) //columns
        {
          if(input_dist_mat[i][j]<min_element)
            {
              min_element=input_dist_mat[i][j];
              min_index=std::make_pair(i,j);
            }
        }
    }
  return min_index;
}

// Function to predict next state, correct and update the Kalman Filter
void predict_and_correct(const std::vector<geometry_msgs::Point> &input_cc, const int &input_nokf)
{
  std::vector<geometry_msgs::Point> kf_predictions{};

  // For second frame, set initial state as the predicted state
  if(second_frame)
  {
    for(int i=0; i<input_nokf; i++)
    {
      geometry_msgs::Point pt;
      pt.x=kf_vector[i].statePre.at<float>(0);
      pt.y=kf_vector[i].statePre.at<float>(1);
      pt.z=0.0f;
      kf_predictions.push_back(pt);
    }
    second_frame=false;
  }

  // After second frame, predict the next states using the Kalman Filter
  else
  {
    std::vector<cv::Mat> pred{};
    for(auto it=kf_vector.begin(); it!=kf_vector.end(); it++)
    {
      pred.push_back((*it).predict());
    }

    for(auto it=pred.begin(); it!=pred.end(); it++)
    {
      geometry_msgs::Point pt;
      pt.x=(*it).at<float>(0);
      pt.y=(*it).at<float>(1);
      pt.z=0.0f;
      kf_predictions.push_back(pt);
    }
  }

  // for debugging only
  for(int i=0; i<input_nokf; i++)
  {
    std::cout << "kf pred: " << kf_predictions[i].x <<" "
              << kf_predictions[i].y<<" "<< kf_predictions[i].z << std::endl;
  }

  // Create markers for visualisation
  visualization_msgs::MarkerArray cluster_markers{};
  for(int i=0; i<input_nokf; i++)
  {
    visualization_msgs::Marker m;
    m.id=i;
    m.type=visualization_msgs::Marker::SPHERE;
    m.header.frame_id="/laser1"; // point cloud is concatenated with frame /laser1 as a reference point
    m.scale.x=0.1;
    m.scale.y=0.1;
    m.scale.z=0.1;
    m.action=visualization_msgs::Marker::ADD;
    m.color.a=1.0; //opacity
    m.color.r=i%2?1:0; // to generate random colors
    m.color.g=i%3?1:0;
    m.color.b=i%4?1:0;

    geometry_msgs::Point current_cluster(kf_predictions[i]); // create markers on the predicted states
    m.pose.position.x=current_cluster.x;
    m.pose.position.y=current_cluster.y;
    m.pose.position.z=current_cluster.z;
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0;

    cluster_markers.markers.push_back(m);
  }
  marker_pub.publish(cluster_markers);

  // Publish predicted coordinates in an array; increasing order (car1, car2, ...)
  std_msgs::Float32MultiArray picar_cc;
  for(int i=0; i<input_nokf; i++)
  {
    picar_cc.data.push_back(kf_predictions[car_id[i]].x);
    picar_cc.data.push_back(kf_predictions[car_id[i]].y);
    picar_cc.data.push_back(kf_predictions[car_id[i]].z);
  }
  pub_cc.publish(picar_cc);

  // kf_id contains information on which element of kf_prediction belong to a particular input cluster centroid
  kf_id.clear();
  kf_id.resize(input_nokf);

  // creating distance matrix as described in the thesis
  std::vector<std::vector<float>> dist_mat;

  for(int i=0; i<input_nokf; i++)
  {
    std::vector<float> dist_vec{};
    for(int n=0; n<input_nokf; n++)
    {
      dist_vec.push_back(euclidean_distance(kf_predictions[i], input_cc[n]));
    }
    dist_mat.push_back(dist_vec);
  }

  // finding corresponding corresponding cluster center for each predicted state for correction phase
  for(int i=0; i<input_nokf; i++)
  {
    std::pair<int,int> min_index(find_index_of_min(dist_mat));
    kf_id[min_index.first]=min_index.second;
    dist_mat[min_index.first]=std::vector<float>(input_nokf,100000.0); // set to 100000 to prevent repeated element

    for(int row=0;row<input_nokf;row++)
    {
      dist_mat[row][min_index.second]=100000.0;
    }
  }

  // create measurement matrix
  std::vector<cv::Mat> measure_matrices{};
  for(int i=0; i<input_nokf; i++)
  {
    float m_x=input_cc[kf_id[i]].x;
    float m_y=input_cc[kf_id[i]].y;
    cv::Mat measure_mat=cv::Mat(2,1,CV_32F,cv::Scalar(0));
    measure_mat.at<float>(0)=m_x;
    measure_mat.at<float>(1)=m_y;
    measure_matrices.push_back(measure_mat);
  }

  // correct predicted state using measurement matrix
  std::vector<cv::Mat> correct{};
  for(int i=0; i<input_nokf; i++)
  {
    cv::Mat corrected = kf_vector[i].correct(measure_matrices[i]);
    correct.push_back(corrected);
  }
}

// Function to identify cars
void object_id(const std::vector<geometry_msgs::Point> &input_cc, const int &input_noc)
{
  // Clearing and resizing vectors to prevent segfault
  obj_id.clear();
  obj_id.resize(input_noc);
  car_id.clear();
  car_id.resize(input_noc);

  // Initial position of cars
  geometry_msgs::Point picar_1; picar_1.x = -0.6f; picar_1.y = 1.74f; picar_1.z= 0.0f;
  geometry_msgs::Point picar_2; picar_2.x = 0.6f; picar_2.y = 1.74f; picar_2.z= 0.0f;
  geometry_msgs::Point picar_3; picar_3.x = 0.0f; picar_3.y = 0.64f; picar_3.z= 0.0f;
  geometry_msgs::Point picar_4; picar_4.x = -0.6f; picar_4.y = -0.46f; picar_4.z= 0.0f;
  geometry_msgs::Point picar_5; picar_5.x = 0.6f; picar_5.y = -0.46f; picar_5.z= 0.0f;
  geometry_msgs::Point picar_6; picar_6.x = -0.56f; picar_6.y = 1.17f; picar_6.z= 0.0f;
  geometry_msgs::Point picar_7; picar_7.x = 0.56f; picar_7.y = 1.17f; picar_7.z= 0.0f;
  geometry_msgs::Point picar_8; picar_8.x = -0.56f; picar_8.y = 0.11f; picar_8.z= 0.0f;
  geometry_msgs::Point picar_9; picar_9.x = 0.56f; picar_9.y = 0.11f; picar_9.z= 0.0f;

  std::vector<geometry_msgs::Point> obj_init{picar_1, picar_2, picar_3, picar_4, picar_5, picar_6, picar_7, picar_8, picar_9};
  std::vector<std::vector<float>> dist_mat;

  // obj_id contains information on which car a certain cluster center belongs to
  for(int i=0; i<input_noc; i++)
  {
    std::vector<float> dist_vec{};
    for(int n=0; n<9; n++)
    {
      dist_vec.push_back(euclidean_distance(input_cc[i], obj_init[n]));
    }
    dist_mat.push_back(dist_vec);
  }

  // similar distance method as referenced in thesis
  for(int i=0; i<dist_mat.size(); i++)
  {
    std::pair<int,int> min_index(find_index_of_min(dist_mat));
    obj_id[min_index.first]=min_index.second;
    dist_mat[min_index.first]=std::vector<float>(9,10000.0);

    for(int row=0;row<dist_mat.size();row++)
    {
      dist_mat[row][min_index.second]=10000.0;
    }
  }

  // car_id contains the index of obj_id in order of picar from 1 onwards -- for publishing purposes (so cluster centers can be published in order of the cars)
  for(int i=0; i<input_noc; i++)
  {
    int ser{i};
    std::vector<int>::iterator itr = std::find(obj_id.begin(), obj_id.end(), ser);
    if (itr != obj_id.end())
    {
      car_id[i] = itr-obj_id.begin();
    }
    else
    {
      std::cout << "Element not found" << std::endl;
    }
  }
}


// Callback function for every pointcloud data received
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr &input)
{
  // Changing data type from ROS message to PCL and setting kd-tree for input cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::fromROSMsg (*input, *input_cloud);
  tree->setInputCloud (input_cloud);

  // Vector of point indices -- points are clustered using vector of point indices
  std::vector<pcl::PointIndices> cluster_indices;

  // EuclideanClusterExtraction settings
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.1);
  ec.setMinClusterSize (4);
  ec.setMaxClusterSize (125);
  ec.setSearchMethod (tree);
  ec.setInputCloud (input_cloud);
  ec.extract (cluster_indices);

  // Vector of cluster centers/centroids
  std::vector<pcl::PointXYZ> cluster_centroids;

  // Extracting cluster centers from cluster indices
  for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZ>);
    float x=0.0;
    float y=0.0;
    int number_of_points=0;
    for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
    {
      x+=input_cloud->points[*pit].x;
      y+=input_cloud->points[*pit].y;
      number_of_points++;
    }
    pcl::PointXYZ centroid;
    centroid.x=x/number_of_points;
    centroid.y=y/number_of_points;
    centroid.z=0.0;
    cluster_centroids.push_back(centroid);
  }

  if(first_frame)
  {
    int number_of_clusters{static_cast<int>(cluster_indices.size())};

    std::cout<< "number of cluster: " << number_of_clusters << std::endl;

    std::vector<geometry_msgs::Point> cluster_centers{};
    for(int i=0; i<cluster_centroids.size(); i++)
    {
      geometry_msgs::Point pt;
      pt.x=cluster_centroids[i].x;
      pt.y=cluster_centroids[i].y;
      pt.z=0.0f;
      cluster_centers.push_back(pt);
    }

    // Kalman Filter settings
    int state_d{4};
    int measure_d{2};
    int ctrl_d{0};
    float Ax=1.0f;
    float Ay=1.0f;
    float Av_x=0.01f;
    float Av_y=0.01f;
    float Q_value=0.01;
    float R_value=0.08;
    float H_value=1.0;

    // create an array of Kalman Filters
    for(int i=0; i<number_of_clusters; i++)
    {
      cv::KalmanFilter kf(state_d,measure_d,ctrl_d,CV_32F);
      kf_vector.push_back(kf);
    }

    // provide initial state and other matrices
    for(int i=0; i<number_of_clusters; i++)
    {
      kf_vector[i].transitionMatrix = (cv::Mat_<float>(4,4) << Ax,0,1,0, 0,Ay,0,1, 0,0,Av_x,0, 0,0,0,Av_y);
      cv::setIdentity(kf_vector[i].measurementMatrix, cv::Scalar(H_value));
      cv::setIdentity(kf_vector[i].processNoiseCov, cv::Scalar(Q_value));
      cv::setIdentity(kf_vector[i].measurementNoiseCov, cv::Scalar(R_value));
      kf_vector[i].statePre.at<float>(0)=cluster_centers[i].x;
      kf_vector[i].statePre.at<float>(1)=cluster_centers[i].y;
      kf_vector[i].statePre.at<float>(2)=0;
      kf_vector[i].statePre.at<float>(3)=0;
    }

    for(int i=0; i<number_of_clusters; i++)
    {
      std::cout << "kf state: " << kf_vector[i].statePre.at<float>(0) <<" "
                << kf_vector[i].statePre.at<float>(1) << std::endl;
    }

    object_id(cluster_centers, number_of_clusters);

    // for debugging
    std::cout << "obj_id: " << std::endl;
    for(auto it=obj_id.begin(); it!=obj_id.end(); it++)
    {
      std::cout << *it << std::endl;
    }

    // for debugging
    std::cout << "car_id: " << std::endl; // cross check with obj_id
    for(auto it=car_id.begin(); it!=car_id.end(); it++)
    {
      std::cout << *it << std::endl;
    }

    first_frame=false;
  }

  else
  {
    int number_of_kf{static_cast<int>(kf_vector.size())};
    std::cout << "Number of kf: " << number_of_kf << std::endl;

    if(cluster_indices.size()==number_of_kf)
    {
      std::cout << "Updating" <<std::endl;

      std::vector<geometry_msgs::Point> cluster_centers{};
      for(int i=0; i<cluster_centroids.size(); i++)
      {
        geometry_msgs::Point pt;
        pt.x=cluster_centroids[i].x;
        pt.y=cluster_centroids[i].y;
        pt.z=0.0f;
        cluster_centers.push_back(pt);
      }
      predict_and_correct(cluster_centers, number_of_kf);
    }

    // discard frames that have different number of detected clusters
    else {std::cout<< "Not updating" <<std::endl;}
  }
}


int main(int argc, char** argv)
{
  ros::init (argc,argv,"tracker");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe ("filtered_cloud", 1, cloud_cb);

  marker_pub= n.advertise<visualization_msgs::MarkerArray> ("markers",1);

  pub_cc = n.advertise<std_msgs::Float32MultiArray> ("picar_positions",1);

  ros::spin();
}
