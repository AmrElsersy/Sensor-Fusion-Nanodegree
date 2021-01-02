// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include "quiz/cluster/clustring.h"
#include <pcl/filters/passthrough.h>

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    typename pcl::PointCloud<PointT>::Ptr rangeCloud(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr voxelCloud(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr filteredCloud(new pcl::PointCloud<PointT>);

    // Range Filter
    typename pcl::CropBox<PointT> rangeFilter(true);
    // range (box) dimentions
    rangeFilter.setMin(minPoint);
    rangeFilter.setMax(maxPoint);
    rangeFilter.setInputCloud(cloud);
    rangeFilter.filter(*rangeCloud);


    // Voxel Resolution Filter
    typename pcl::VoxelGrid<PointT> voxelGrid;
    voxelGrid.setLeafSize(filterRes, filterRes, filterRes);
    voxelGrid.setInputCloud(rangeCloud);
    voxelGrid.filter(*voxelCloud);


    // remove roof points of my Car
    typename pcl::CropBox<PointT> myCarFilter(true);
    myCarFilter.setMin( Eigen::Vector4f (-1.5, -1.7, -1, 1) );
    myCarFilter.setMax( Eigen::Vector4f (2.6, 1.7, -0.4, 1) );
    myCarFilter.setInputCloud(voxelCloud);
    // indices of points of my Car (filtered Area within that box)
    std::vector<int> indices;
    myCarFilter.filter(indices);
    // convert std::vector to PointIndices to use it in ExtractIndices
    pcl::PointIndices::Ptr inlieres (new pcl::PointIndices);
    inlieres->indices = indices;
    // Subtract PointCloud - myCarPointCloud = filtered point cloud without my car points
    pcl::ExtractIndices<PointT> extract;
    extract.setNegative(true);
    extract.setInputCloud(voxelCloud);
    extract.setIndices(inlieres);
    extract.filter(*filteredCloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return filteredCloud;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT> *planePointCloud = new pcl::PointCloud<PointT>;
    typename pcl::PointCloud<PointT> *obstaclesPointCloud = new pcl::PointCloud<PointT>;
    
    // extract can substract the input cloud from the cloud that contains the inliers (indices)
    typename pcl::ExtractIndices<PointT>::Ptr extract (new pcl::ExtractIndices<PointT>) ;

    extract->setInputCloud(cloud);
    extract->setIndices(inliers);

    // false = InputCloud - not inliers = inliers cloud
    extract->setNegative(false);
    extract->filter(*planePointCloud);
    // OR
    // for (auto index : inliers->indices)
    //     planePointCloud->points.push_back(cloud->points[index]);


    // true = InputCloud - inliers
    extract->setNegative(true);
    extract->filter(*obstaclesPointCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(planePointCloud, obstaclesPointCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in this function to find inliers for the cloud.

    // pcl::ModelCoefficients::Ptr coff ( new pcl::ModelCoefficients);
    // pcl::PointIndices::Ptr inliers ( new pcl::PointIndices);

    // typename pcl::SACSegmentation<PointT> *segmentation = new pcl::SACSegmentation<PointT>;

    // segmentation->setOptimizeCoefficients(true);
    // segmentation->setModelType(pcl::SACMODEL_PLANE);
    // segmentation->setMethodType(pcl::SAC_RANSAC);
    // segmentation->setMaxIterations(maxIterations);
    // segmentation->setDistanceThreshold(distanceThreshold);

    // segmentation->setInputCloud(cloud);
    // segmentation->segment(*inliers, *coff);

    // // our segmentation method
    pcl::PointIndices::Ptr inliers ( new pcl::PointIndices);
    auto inliers_set = Ransac3D(cloud, maxIterations, distanceThreshold);
    // copy the set of indices to a vector
    inliers->indices.assign(inliers_set.begin(), inliers_set.end());

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr kdTree (new pcl::search::KdTree<PointT>);
    kdTree->setInputCloud(cloud);

    // PointIndices for each cluster
    std::vector<pcl::PointIndices> clustersIndices;

    // Clustering Handler
    typename pcl::EuclideanClusterExtraction<PointT> euclideanClustring;

    // distnace of nearest neighbors ... to consider the point a neighbor
    euclideanClustring.setClusterTolerance(clusterTolerance); 
    // min number of points of any cluster (as very small clusters will be considered as noise)
    euclideanClustring.setMinClusterSize(minSize);
    // max number of points of any cluster (very large clusters will be splited into many clusters)
    euclideanClustring.setMaxClusterSize(maxSize);
    // with KD Tree, many points is skiped during the search (O(log(n))
    euclideanClustring.setSearchMethod(kdTree);
    // point cloud to be clustered
    euclideanClustring.setInputCloud(cloud);
    // Clustring
    euclideanClustring.extract(clustersIndices);
    std::cout << "clusters= " << clustersIndices.size() << std::endl;
    
    for (int i = 0; i < clustersIndices.size(); i++)
    {
        // cluster PointCloud
        typename pcl::PointCloud<PointT>::Ptr clusterCloud (new pcl::PointCloud<PointT>);
        // Indices Extractor to make Cluster PointCloud from the big Cloud
        typename pcl::ExtractIndices<PointT> extractClusterCloud;
        // Indices for each cluster
        pcl::PointIndices::Ptr clusterIndices (new pcl::PointIndices);
        // we need to pass a pointer to the comming functions so we hold the indices into a Ptr (we cannot use its address (don't know why))
        clusterIndices->indices = clustersIndices[i].indices;
        
        // extract the sub cloud (cloud of indices) via the extractor
        extractClusterCloud.setInputCloud(cloud);
        extractClusterCloud.setIndices(clusterIndices);
        // false = get the cloud of the setted indices
        extractClusterCloud.setNegative(false);
        extractClusterCloud.filter(*clusterCloud);

        // optional
        clusterCloud->width = clusterCloud->size();
        clusterCloud->height = 1;
        clusterCloud->is_dense = true;

        clusters.push_back(clusterCloud);
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SersyClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance)
{
    // PointCloud into the Kd tree for clustering process
    KdTree *tree = new KdTree();
    std::vector<std::vector<float> > points;
    for (int i = 0; i < cloud->points.size(); i++)
    {
        std::vector<float> point;
        point.push_back(cloud->points[i].x);
        point.push_back(cloud->points[i].y);
        tree->insert(point, i);
        points.push_back(point);
    }

    // clustering 
    Clustring clustering;
    clustering.setDistanceTol(clusterTolerance);
    clustering.setPoints(points);
    clustering.setTree(tree);
    auto clustersPoints = clustering.euclideanCluster();

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;

    for (auto cluster : clustersPoints)
    {
        // make the point cloud from the cluster indices
        typename pcl::PointCloud<PointT>::Ptr clusterCloud (new pcl::PointCloud<PointT>);
        for (auto pointIndex : cluster)
            clusterCloud->points.push_back(cloud->points[pointIndex]);

        clusters.push_back(clusterCloud);
    }
    
    return clusters;
}

template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

template<typename PointT>
BoxQ ProcessPointClouds<PointT>::BoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    // Find bounding box for one of the clusters
    BoxQ box;
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cluster, pcaCentroid);
    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*cluster, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());

    typename pcl::PointCloud<PointT>::Ptr cloudPointsProjected (new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);
    // Get the minimum and maximum points of the transformed cloud.
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());
    // Final transform

    //Quaternions are a way to do rotations
    box.bboxQuaternion = eigenVectorsPCA;

    // auto temp = eigenVectorsPCA;
    // Eigen::Vector3f vecTemp;
    // vecTemp.x() = 0;
    // vecTemp.y() = 0;
    // vecTemp.z() = 1;
    
    // // temp.block<3,1>(0,2) = vecTemp.head<3>();
    // // box.bboxQuaternion =  temp;
    // box.bboxQuaternion.matrix().block<3,1>(0,2) = vecTemp.head<3>();
    // std::cout << box.bboxQuaternion.matrix() << std::endl;
    // std::cout << "=================" << std::endl;

    // https://www.youtube.com/watch?v=d4EgbgTm0Bg
    // https://www.youtube.com/watch?v=zjMuIxRvygQ
    // https://eater.net/quaternions
    box.bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

    box.cube_length = maxPoint.x-minPoint.x;
    box.cube_width = maxPoint.y-minPoint.y;
    box.cube_height = maxPoint.z-minPoint.z;
    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::Ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)

{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// For max iterations 
	for (int i = 0; i < maxIterations; i++)
	{
		// Randomly sample subset and fit Plane
		auto size = cloud->points.size();
		auto point1 = cloud->points[rand() % size];
		auto point2 = cloud->points[rand() % size];
		auto point3 = cloud->points[rand() % size];

		// Plane Equation Ax+By+Cz+D=0 ....... v1×v2=<(y2−y1)(z3−z1)−(z2−z1)(y3−y1),
											// (z2-z1)(x3-x1)-(x2-x1)(z3-z1)
											// (x2-x1)(y3-y1)-(y2-y1)(x3-x1)>
		// A = i component, B = j component, C = k component of cross mult of v1 & v2 which are vectors from point1 to point2 & point3
		auto x1 = point1.x; 
		auto x2 = point2.x; 
		auto x3 = point3.x; 
		
		auto y1 = point1.y; 
		auto y2 = point2.y; 
		auto y3 = point3.y; 
		
		auto z1 = point1.z; 
		auto z2 = point2.z; 
		auto z3 = point3.z; 
		
		
		auto A = (y2 - y1)*(z3 - z1)-(z2 - z1)* (y3 - y1);
		auto B = (z2 - z1)*(x3 - x1)-(x2 - x1)* (z3 - z1);
		auto C = (x2 - x1)*(y3 - y1)-(y2 - y1)* (x3 - x1);
		auto D = -( A * x1 + B * y1 + C * z1 );

		std::unordered_set<int> inliers;

		// Measure distance between every point and fitted line
		for (int j = 0; j < cloud->points.size(); j++)
		{
			auto point = cloud->points[j];
			// orthogonal distance d = |Ax+By+C|/sqrt(A^2+B^2)d=∣Ax+By+C∣/sqrt(A^2 +B^2 )
			auto d = fabs( A * point.x + B * point.y + C * point.z + D ) / sqrt(A*A + B*B + C*C);

			// If distance is smaller than threshold count it as inlier
			if (d <= distanceTol)
				inliers.insert(j);
		}

		// Return indicies of inliers from fitted line with most inliers
		if (inliers.size() > inliersResult.size())
			inliersResult = inliers;
	}
	return inliersResult;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::TransformPointCloud(typename pcl::PointCloud<PointT>::Ptr cloud, int theta)
{
  /* Reminder: how transformation matrices work :

           |-------> This column is the translation
    | 1 0 0 x |  \
    | 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
    | 0 0 1 z |  /
    | 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)

    x , y , z used for Translation in each direction

    Note: We apply 2D Rotation in the ground plane (No Yaw nor pitch) so we don't move the k (Z basis)
          and leave the 3rd Column as it is + leave the z coordinates of the first 2 basis un changed
  */

  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
 
  // rotation
  transform (0,0) = std::cos (theta);
  transform (0,1) = - std::sin(theta);
  transform (1,0) = std::sin (theta);
  transform (1,1) = std::cos (theta);

    // MAKE IT Configrable NOT Hard coded
  // translation
  transform (3,0) = 20; // translation 1 in x direction
  transform (3,2) = 1; // translation 2 in z direction

  typename pcl::PointCloud<PointT>::Ptr cloudTransformed (new pcl::PointCloud<PointT>);
  pcl::transformPointCloud(*cloud, *cloudTransformed, transform);

  return cloudTransformed;
}
