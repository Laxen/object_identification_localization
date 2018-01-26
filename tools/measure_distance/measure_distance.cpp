#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/impl/point_types.hpp>
#include <stdio.h>
#include <math.h>

typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> ColorHandler_N;

int counter = 0;

double euclidean_distance(std::vector<pcl::PointXYZ> point_vec) {
	pcl::PointXYZ p1 = point_vec.back();
	point_vec.pop_back();
	pcl::PointXYZ p2 = point_vec.back();
	point_vec.pop_back();
	return sqrt(pow((p1.x-p2.x),2) + pow((p1.y-p2.y),2) + pow((p1.z-p2.z),2));
}

// Callback function for registerPointPickingCallback
void pp_callback(const pcl::visualization::PointPickingEvent& event, void* temp)
{
	 std::vector<pcl::PointXYZ>* point_vec = (std::vector<pcl::PointXYZ>*) temp;
	 if(event.getPointIndex()!=-1)
	 {
	 	float x,y,z;
	 	event.getPoint(x,y,z);
		pcl::PointXYZ point;
		point.x = x;
		point.y = y;
		point.z = z;
		point_vec->push_back(point);
		if (counter == 1) {
			counter = 0;
			std::cout << "Euclidean distance: " << euclidean_distance(*point_vec) << std::endl;
		} else {
			counter++;
		}
	}
}

int main (int argc, char **argv)
{
	std::vector<pcl::PointXYZ> point_vec;
	pcl::visualization::PCLVisualizer visu("Visualizer");
	visu.registerPointPickingCallback(pp_callback, (void*)&point_vec);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

	for (int i = 1; i < argc; i++) {
		// Load cloud
		if(pcl::io::loadPCDFile<pcl::PointXYZRGBA> (argv[i], *cloud) < 0)
		{
			pcl::console::print_error ("Error loading object file!\n");
			return 0;
		}

		// Visualize the new cloud
		visu.removePointCloud();
    visu.addPointCloud(cloud, ColorHandler_N(cloud, 0.0, 255.0, 0.0), "cloud");
		std::cout<<"Pick two points to measure the euclidean distance between them. Press q to continue"<<std::endl;
		visu.spin();
		counter = 0;
	}

	return 0;
}
