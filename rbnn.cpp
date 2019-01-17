#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <ctime>
#include <string>
#include <map>
#include <Windows.h>

using namespace std;

#pragma region [configuration]
string directory = "C:\\Users\\km\\Desktop\\MAG\\FloatingObjectFilter\\data";
string file_name = "459_100.pcd";
string result_prefix = "result";
vector<double> radius_values {};
#pragma endregion

#pragma region [auxiliary]
int ReadPointCloudFile(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const string& filename) {
	srand(time(NULL));


	if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd \n");
		return (-1);
	}
	std::cout << "Loaded "
		<< cloud->width * cloud->height
		<< " data points from test_pcd.pcd with the following fields: "
		<< std::endl;
	return 0;
}

void mergeClusters(vector<int>& cluster_indices, int idx1, int idx2) {
	for (int i = 0; i < cluster_indices.size(); i++) {
		if (cluster_indices[i] == idx1) {
			cluster_indices[i] = idx2;
		}
	}
}

// return the cluster that each point belongs to
vector<int> rbnn(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double radius, int min_members) {

	vector<int> cluster_indices = vector<int>(cloud->points.size(), -1);
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);
	int current_cluster = 0;

	for (int i = 0; i < cloud->points.size(); i++) {

		if (i % 100000 == 0) {
			cout << i << endl;
		}

		// 1. skip this point if it's already within a cluster
		if (cluster_indices[i] != -1)
			continue;

		// 2. find nearest neigbours for current point
		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;


		if (kdtree.radiusSearch(cloud->points[i], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
		{
			// 3. merge or assign new clusters
			for (size_t j = 0; j < pointIdxRadiusSearch.size(); ++j) {
				int oc = cluster_indices[i]; // original point's cluster
				int nc = cluster_indices[pointIdxRadiusSearch[j]]; // neighbor point's cluster
				if (oc != -1 && nc != -1) {
					if (oc != nc)
						mergeClusters(cluster_indices, oc, nc);
				}
				else {
					if (nc != -1) {
						cluster_indices[i] = nc;
					}
					else {
						if (oc != -1) {
							cluster_indices[pointIdxRadiusSearch[j]] = oc;
						}
					}
				}
			}
		}

		// 4. if there is still no cluster, create a new one and assign all neighbors to it
		if (cluster_indices[i] == -1) {
			current_cluster++;
			cluster_indices[i] = current_cluster;
			for (int j = 0; j < pointIdxRadiusSearch.size(); j++) {
				cluster_indices[pointIdxRadiusSearch[j]] = current_cluster;
			}
		}
	}

	// 5. delete the clusters that are too small
	// not implemented

	// 6. return cluster_indices
	return cluster_indices;
}

int most_frequent_value(vector<int> values) {
	map<int, int> histcounts;
	for (int i = 0; i < values.size(); i++) {
		if (histcounts.find(values[i]) == histcounts.end()) {
			histcounts[values[i]] = 1;
		}
		else {
			histcounts[values[i]] += 1;
		}
	}

	int max = 0, maxi;
	vector<pair<int, int>> tr(histcounts.begin(), histcounts.end());
	for (auto& val : tr) {
		cout << val.first << " : " << val.second << endl;
		if (val.second > max) {
			max = val.second;
			maxi = val.first;
		}
	}

	return maxi;
}
#pragma endregion

int main (int argc, char** argv)
{
	if (argc >= 3) {
		directory = argv[1];
		file_name = argv[2];
		result_prefix = argv[3];

		for (int i = 4; i < argc; i++) {
			radius_values.push_back(atof(argv[i]));
		}
	}

	cout << "begin reading PC file" << endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	ReadPointCloudFile(cloud, directory + "\\" + file_name);

	for (int i = 1000; i < 2000; i++) {
		cout << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << endl;
	}

	// foreach rbnn radius value
	stringstream result_storage;
	for (double radius_value : radius_values) {
		cout << "processing radius value " << radius_value << endl;

		// perform the rbnn
		vector<int> cluster_indices = rbnn(cloud, radius_value, 1);
		cout << "RBNN FINISHED" << endl;

		cout << endl;

		// find the maximal cluster (representative of the floor segment)
		int max_clust = most_frequent_value(cluster_indices);
		cout << "MAXIMUM CLUSTER: " << max_clust << endl;

		// all other points are classified as floating objects
		int cntr_yes = 0, cntr_no = 0;
		result_storage << radius_value << " ";
		for (int i = 0; i < cluster_indices.size(); i++) {
			if (cluster_indices[i] == max_clust) {
				result_storage << -1 << " ";
				cntr_no++;
			}
			else {
				result_storage << cluster_indices[i] << " ";
				cntr_yes++;
			}
		}
		result_storage << endl;
		cout << "POSITIVES: " << cntr_yes << " NEGATIVES: " << cntr_no << endl;
	}
	

	// write the results
	ofstream outfile;
	outfile.open(directory + "\\" + result_prefix + file_name, ios::out);
	outfile << result_storage.str();
	outfile.close();
	cout << "results written to disc. Press any key to exit..." << endl;

	return 0;
}