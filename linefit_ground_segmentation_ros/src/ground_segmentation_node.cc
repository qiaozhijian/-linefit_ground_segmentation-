#include <ros/ros.h>
#include <pcl/io/ply_io.h>
#include <pcl_ros/point_cloud.h>
#include "utility.h"
#include "ground_segmentation/ground_segmentation.h"

class SegmentationNode {
    ros::Publisher ground_pub_;
    ros::Publisher obstacle_pub_;
    GroundSegmentationParams params_;

public:
    SegmentationNode(ros::NodeHandle& nh,
                     const std::string& ground_topic,
                     const std::string& obstacle_topic,
                     const GroundSegmentationParams& params,
                     const bool& latch = false) : params_(params) {
        ground_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>(ground_topic, 1, latch);
        obstacle_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>(obstacle_topic, 1, latch);
    }

    void scanCallback(const string seq) {
        int nTimes = getTimeStamp(seq);
        stringstream ss;
        ss << setfill('0') << setw(2) << seq;
        string strPathToSequence = "/media/qzj/Document/grow/research/slamDataSet/kitti/data_odometry_velodyne/dataset/sequences/" + ss.str() + "/velodyne/";
//         string strPathToSave = "/media/qzj/Document/grow/research/slamDataSet/kitti/data_odometry_velodyne/dataset/downSample/bin/" + ss.str() + "/velodyne/";
        string strPathToSave = "/media/qzj/Document/grow/research/slamDataSet/kitti/data_odometry_velodyne/dataset/downSample/bin/" + ss.str() + "/velodyne/";
        cout<<"strPathToSequence: "<<strPathToSequence.c_str()<<endl;
        cout<<"strPathToSave: "<<strPathToSave.c_str()<<endl;
        cout<<"bin to save: "<<nTimes<<endl;
        cout<<"x y z intensity"<<endl;

        createDirectory(strPathToSave);
        int pcl_num=0;
        for(pcl_num=0; pcl_num<nTimes; pcl_num++)
        {
            stringstream ss;
			pcl::PointCloud<PointType>::Ptr cloud_xyzi (new pcl::PointCloud<PointType> ());
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ> ());
            ss << setfill('0') << setw(6) << pcl_num;
            string binfile = strPathToSequence + ss.str() + ".bin";
            string binfileToSave = strPathToSave + ss.str() + ".bin";
			if(pcl_num%10==0)
				cout<<"load "<<pcl_num<<endl;
            loadBin(binfile, cloud_xyzi);
			pcl::copyPointCloud(*cloud_xyzi, *cloud);
			GroundSegmentation segmenter(params_);
			std::vector<int> labels;
			segmenter.segment(*cloud, &labels);
			pcl::PointCloud<pcl::PointXYZ> ground_cloud, obstacle_cloud;
			ground_cloud.header = cloud->header;
			ground_cloud.header.frame_id = "world";
			obstacle_cloud.header = cloud->header;
			obstacle_cloud.header.frame_id = "world";
			for (size_t i = 0; i < cloud->size(); ++i) {
				if (labels[i] == 1) ground_cloud.push_back((*cloud)[i]);
				else obstacle_cloud.push_back((*cloud)[i]);
			}
			pcl::PointCloud<PointType>::Ptr ground_cloud_xyzi(new pcl::PointCloud<PointType> ()), obstacle_cloud_xyzi(new pcl::PointCloud<PointType> ());
			pcl::copyPointCloud(ground_cloud, *ground_cloud_xyzi);
			pcl::copyPointCloud(obstacle_cloud, *obstacle_cloud_xyzi);
			writeKittiPclBinData(obstacle_cloud_xyzi,binfileToSave);
			ground_pub_.publish(ground_cloud);
			obstacle_pub_.publish(obstacle_cloud);
        }
		cout<<"finish "<<seq<<" "<<pcl_num<<endl;
    }
	void saveCloud(string path) {
	
//         
        
	
    }
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "ground_segmentation");
    google::InitGoogleLogging(argv[0]);

    ros::NodeHandle nh("~");
	
	if(argc<2)
    {
        printf("ERROR: Please follow the example: rosrun pkg node seq \n");
        return -2;
    }
	
    cout<<" preprocess Started."<<endl;
    std::string seq = argv[1];

    // Do parameter stuff.
    GroundSegmentationParams params;
    nh.param("visualize", params.visualize, params.visualize);
    nh.param("n_bins", params.n_bins, params.n_bins);
    nh.param("n_segments", params.n_segments, params.n_segments);
    nh.param("max_dist_to_line", params.max_dist_to_line, params.max_dist_to_line);
    nh.param("max_slope", params.max_slope, params.max_slope);
    nh.param("long_threshold", params.long_threshold, params.long_threshold);
    nh.param("max_long_height", params.max_long_height, params.max_long_height);
    nh.param("max_start_height", params.max_start_height, params.max_start_height);
    nh.param("sensor_height", params.sensor_height, params.sensor_height);
    nh.param("line_search_angle", params.line_search_angle, params.line_search_angle);
    nh.param("n_threads", params.n_threads, params.n_threads);
    // Params that need to be squared.
    double r_min, r_max, max_fit_error;
    if (nh.getParam("r_min", r_min)) {
        params.r_min_square = r_min*r_min;
    }
    if (nh.getParam("r_max", r_max)) {
        params.r_max_square = r_max*r_max;
    }
    if (nh.getParam("max_fit_error", max_fit_error)) {
        params.max_error_square = max_fit_error * max_fit_error;
    }

    std::string ground_topic, obstacle_topic, input_topic;
    bool latch;
    nh.param<std::string>("input_topic", input_topic, "input_cloud");
    nh.param<std::string>("ground_output_topic", ground_topic, "ground_cloud");
    nh.param<std::string>("obstacle_output_topic", obstacle_topic, "obstacle_cloud");
    nh.param("latch", latch, false);

    // Start node.
    SegmentationNode node(nh, ground_topic, obstacle_topic, params, latch);
	for (int i=0;i<11;i++)
	{
		std::string seq = std::to_string(i);
		node.scanCallback(seq);
	}
}
