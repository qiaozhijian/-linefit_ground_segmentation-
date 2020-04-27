#ifdef WIN32
#include <io.h>
#include <direct.h> 
#else
#include <unistd.h>
#include <sys/stat.h>
#endif
#include <stdint.h>
#include <string>
#define MAX_PATH_LEN 256

#ifdef WIN32
#define ACCESS(fileName,accessMode) _access(fileName,accessMode)
#define MKDIR(path) _mkdir(path)
#else
#define ACCESS(fileName,accessMode) access(fileName,accessMode)
#define MKDIR(path) mkdir(path,S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH)
#endif
#include <typeinfo>
#include "utility.h"

// 从左到右依次判断文件夹是否存在,不存在就创建
// example: /home/root/mkdir/1/2/3/4/
// 注意:最后一个如果是文件夹的话,需要加上 '\' 或者 '/'
int32_t createDirectory(const std::string &directoryPath)
{
    uint32_t dirPathLen = directoryPath.length();
    if (dirPathLen > MAX_PATH_LEN)
    {
        return 1;
    }
    char tmpDirPath[MAX_PATH_LEN] = { 0 };
    for (uint32_t i = 0; i < dirPathLen; ++i)
    {
        tmpDirPath[i] = directoryPath[i];
        if (tmpDirPath[i] == '\\' || tmpDirPath[i] == '/')
        {
            if (ACCESS(tmpDirPath, 0) != 0)
            {
                int32_t ret = MKDIR(tmpDirPath);
                if (ret != 0)
                {
                    return ret;
                }
            }
        }
    }
    return 0;
}


/**
 * 字符串替换函数
 * #function name   : replace_str()
 * #param str       : 操作之前的字符串
 * #param before    : 将要被替换的字符串
 * #param after     : 替换目标字符串
 * #return          : void
 */
void replace_str(std::string& str, const std::string& before, const std::string& after)
{
	for (std::string::size_type pos(0); pos != std::string::npos; pos += after.length())
	{
		pos = str.find(before, pos);
		if (pos != std::string::npos)
			str.replace(pos, before.length(), after);
		else
			break;
	}
}

void writeKittiPclBinData(pcl::PointCloud<PointType>::Ptr input_pointcloud,string dir)
{
	//ROS_INFO("save %d",input_pointcloud->size());
	
	//Create & write .bin file
	ofstream bin_file(dir.c_str(),ios::out|ios::binary);
	if(!bin_file.good()) cout<<"Couldn't open "<<dir.c_str()<<endl;  
	
	for (size_t i = 0; i < input_pointcloud->points.size (); ++i)
	{
		bin_file.write((char*)&input_pointcloud->points[i].x,3*sizeof(float)); 
		bin_file.write((char*)&input_pointcloud->points[i].intensity,sizeof(float));
	}
  	
	bin_file.close();
	
	//std::ofstream binFile(dir);
	//for (size_t i = 0; i < input_pointcloud->points.size (); ++i)
	//	binFile << input_pointcloud->points[i].x << input_pointcloud->points[i].y << input_pointcloud->points[i].z;
	//binFile.close();
}



int getTimeStamp(string seq)
{
	int nTimes = 0;
	stringstream ss;
	//这里seq也可以整数
	ss << setfill('0') << setw(2) << seq;
	string strPathTimeFile = "/media/qzj/Document/grow/research/slamDataSet/kitti/data_odometry_calib/dataset/sequences/" + ss.str() + "/times.txt";
	ifstream fTimes;
	fTimes.open(strPathTimeFile.c_str());
	while(!fTimes.eof())
	{
		string s;
		getline(fTimes,s);
		if(!s.empty())
			nTimes++;
	}
	return nTimes;
}

void loadBin(string binfile, pcl::PointCloud<PointType>::Ptr laserCloudIn)
{	
	ifstream input(binfile.c_str(), ios::in | ios::binary);
	if(!input.is_open() ) {
		cerr << "Could not read file: " << binfile << endl;
	}
	
	const size_t kMaxNumberOfPoints = 1e6;  // From the Readme of raw files.
	laserCloudIn->clear();
	laserCloudIn->reserve(kMaxNumberOfPoints);
	
	int i;
	for (i=0; input.is_open() && !input.eof(); i++) {
		PointType point;
		input.read((char *) &point.x, 3*sizeof(float));
		input.read((char *) &point.intensity, sizeof(float));
		laserCloudIn->push_back(point);
	}
	input.close();
}
