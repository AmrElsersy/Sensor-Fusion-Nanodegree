#include <chrono>
#include <string>
#include "kdtree.h"

class Clustring 
{
private:
	float distTol;;
	KdTree *kdTree;
	std::vector<std::vector<float>> points;
	std::vector<bool> visited;
public:
	void setDistanceTol(float d)
	{
		this->distTol = d;
	}
	void setTree(KdTree *tree)
	{
		this->kdTree = tree;
	}
	void setPoints(std::vector<std::vector<float>> _points)
	{
		this->points = _points;

		visited.resize(points.size());
		for (int i = 0; i < visited.size(); i++)
			visited[i] = false;
	}
	
	void clusterNeighbours(std::vector<int> &cluster, int pointID)
	{
		// skip visited points (it belongs to a cluster)
		if (visited[pointID])
			return;

		// make point as visited
		visited[pointID] = true;
		// add point to cluster
		cluster.push_back(pointID);

		// get all neighbours of the point and cluster each neighbour 
		auto neighbours = kdTree->search(points[pointID], distTol);
		for (auto neighbour : neighbours)
		{
			if (!visited[neighbour])
				clusterNeighbours(cluster, neighbour);
		}
	}

	std::vector<std::vector<int>> euclideanCluster()
	{
		// TODO: Fill out this function to return list of indices for each cluster
		std::vector<std::vector<int>> clusters;

		for (int i = 0; i < points.size(); i++)
		{
			if (visited[i])
				continue;

			std::vector<int> cluster;
			clusterNeighbours(cluster, i);

			clusters.push_back(cluster);
		}
	
		return clusters;

	}
};
