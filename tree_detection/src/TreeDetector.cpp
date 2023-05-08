#include "tree_detection/TreeDetector.hpp"

#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

namespace tree_detection {

void TreeDetector::setTreeDetectionParameters(const TreeDetectionParameters &p) {
	treeDetectionParam_ = p;
}

const TreeDetectionParameters& TreeDetector::getParameters() const {
	return treeDetectionParam_;
}

void TreeDetector::setInputPointCloudPtr(PointCloud::Ptr inputCloud) {
	inputCloud_.reset();
	inputCloud_ = inputCloud;
}

void TreeDetector::setInputPointCloud(const PointCloud &inputCloud) {
	inputCloud_.reset(new PointCloud());
	*inputCloud_ = inputCloud;
}
void TreeDetector::detectTrees() {
	const auto startTime = std::chrono::steady_clock::now();
	detectedTreeClusters_.clear();

	if (inputCloud_->empty()) {
		std::cout << "[TreeDetector] inputCloud is empty. Returning. \n";
		return;
	}

	PointIndices setsOfClusterIndices = extractClusterIndices(inputCloud_);

	auto validClusters = extractValidPointClusters(setsOfClusterIndices, inputCloud_);

	if (validClusters.empty()) {
		std::cout << "[TreeDetector] No valid clusters found\n";
		return;
	}

	auto clusterInfo = extractClusterInformation(validClusters);
	detectedTreeClusters_ = clusterInfo;
	namespace ch = std::chrono;
	const auto endTime = std::chrono::steady_clock::now();
	double numSec = ch::duration_cast<ch::microseconds>(endTime - startTime).count() / 1e6;
	if (treeDetectionParam_.isPrintTiming_) {
		std::cout << "Tree detection took: " << numSec << " seconds \n";
	}
	std::cout << "[TreeDetector]: " << detectedTreeClusters_.size() << " trees were detected \n";

}
const TreeDetector::ClusterVector& TreeDetector::getDetectedTreeClusters() const {
	return detectedTreeClusters_;
}

PointIndices TreeDetector::extractClusterIndices(const PointCloud::ConstPtr cloud) const {

	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdTree(new pcl::search::KdTree<pcl::PointXYZ>);
	kdTree->setInputCloud(cloud);

	PointIndices clusterIndices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclideanClusterExtraction;
	euclideanClusterExtraction.setClusterTolerance(treeDetectionParam_.clusterTolerance_);
	euclideanClusterExtraction.setMinClusterSize(treeDetectionParam_.minNumPointsTree_);
	euclideanClusterExtraction.setMaxClusterSize(treeDetectionParam_.maxNumPointsTree_);
	euclideanClusterExtraction.setSearchMethod(kdTree);
	euclideanClusterExtraction.setInputCloud(cloud);
	euclideanClusterExtraction.extract(clusterIndices);
	std::cout << "[TreeDetector]: " << clusterIndices.size() << " clusters from cloud extracted." << "\n";

	return clusterIndices;
}

PointCloudPtrVector TreeDetector::extractValidPointClusters(const PointIndices &setOfClusterIndices,
		PointCloud::ConstPtr cloud) const {

	PointCloudPtrVector validClusters;
	for (std::vector<pcl::PointIndices>::const_iterator it = setOfClusterIndices.begin();
			it != setOfClusterIndices.end(); ++it) {

		PointCloud::Ptr cloudCluster = extractClusterFromIndices(it->indices, cloud);

		const bool isDiscardCluster = !isGravityAlignmentStraightEnough(cloudCluster) || isClusterTooLow(cloudCluster)
				|| isClusterRadiusTooBig(cloudCluster);

		if (isDiscardCluster) {
			continue;
		}

		validClusters.push_back(PointCloud::Ptr(new PointCloud()));
		validClusters.back() = cloudCluster;
	}

	return validClusters;
}

PointCloud::Ptr TreeDetector::extractClusterFromIndices(const std::vector<int> &indices,
        PointCloud::ConstPtr cloud) const {
    PointCloud::Ptr cloudCluster(new PointCloud);
    cloudCluster->points.reserve(indices.size());
    for (int index : indices) {
        cloudCluster->points.emplace_back(cloud->points[index]);
    }
    cloudCluster->reserve(indices.size());
    cloudCluster->width = cloudCluster->points.size();
    cloudCluster->height = 1;
    cloudCluster->is_dense = true;
    return cloudCluster;
}

bool TreeDetector::isClusterTooLow(PointCloud::ConstPtr cloudCluster) const {
	auto clusterDimension = getClusterDimensions(cloudCluster);

	if (clusterDimension.dimZ_ < treeDetectionParam_.minHeightTree_) {
		if (treeDetectionParam_.isPrintDiscardedTreeInstances_) {
			std::cout << "Discard cluster candidate with height " << clusterDimension.dimZ_ << "m. \n";
		}
		return true;
	}
	return false;
}

bool TreeDetector::isClusterRadiusTooBig(PointCloud::ConstPtr cloudCluster) const {

	auto clusterDimension = getClusterDimensions(cloudCluster);

	if (clusterDimension.dimX_ > treeDetectionParam_.maxDiameterTree_
			|| clusterDimension.dimY_ > treeDetectionParam_.maxDiameterTree_) {
		if (treeDetectionParam_.isPrintDiscardedTreeInstances_) {
			std::cout << "Discard cluster with width in x axis " << clusterDimension.dimX_ << "m. \n";
			std::cout << "Width in y axis " << clusterDimension.dimY_ << "m. \n";
		}
		return true;
	}

	return false;
}

ClusterDimensions TreeDetector::getClusterDimensions(PointCloud::ConstPtr clusterCloud) const {
	pcl::PointXYZ minPt, maxPt;
	pcl::getMinMax3D(*clusterCloud, minPt, maxPt);

	ClusterDimensions clusterDimensions;
	clusterDimensions.dimX_ = maxPt.x - minPt.x;
	clusterDimensions.dimY_ = maxPt.y - minPt.y;
	clusterDimensions.dimZ_ = maxPt.z - minPt.z;

	return clusterDimensions;
}

bool TreeDetector::isGravityAlignmentStraightEnough(PointCloud::ConstPtr cloudCluster) const {
	double gravityAlignement = getClusterGravityAlignment(cloudCluster);
	if (gravityAlignement > treeDetectionParam_.minEigenVectorAlignment_) {
		return true;
	} else {
		if (treeDetectionParam_.isPrintDiscardedTreeInstances_) {
			std::cout << "Discarding because the the gravity alignment " << gravityAlignement << " is less than: "
					<< treeDetectionParam_.minEigenVectorAlignment_ << "\n";
		}
		return false;
	}

}

double TreeDetector::getClusterGravityAlignment(PointCloud::ConstPtr cloudCluster) const {

	pcl::PCA<pcl::PointXYZ> cloudPCA = new pcl::PCA<pcl::PointXYZ>;
	cloudPCA.setInputCloud(cloudCluster);

	Eigen::Matrix3f eigenVectors = cloudPCA.getEigenVectors();
	Eigen::Vector3f gravityAligned = Eigen::Vector3f::UnitZ();
	Eigen::Vector3f principleAxis = eigenVectors.col(0);
	double principleAxisAlignment = std::abs(principleAxis.dot(gravityAligned));
	std::pair<Eigen::Vector3d, double> returnValue;

	return principleAxisAlignment;

}

Eigen::Vector3d TreeDetector::getClusterMean(PointCloud::ConstPtr cloudCluster) const {
    const int N = cloudCluster->size();
    Eigen::Array3d mean = Eigen::Array3d::Zero();
    for (int i = 0; i < N; ++i) {
        mean += cloudCluster->points[i].getArray3fMap().cast<double>();
    }

    return mean / N;
}

TreeDetector::ClusterVector TreeDetector::extractClusterInformation(const PointCloudPtrVector &validClusters) const {
    auto clusterCloudToCluster = [this](const PointCloud::Ptr cloud) {
        Cluster cluster;
        cluster.position_ = getClusterMean(cloud);
        cluster.clusterDimensions_ = getClusterDimensions(cloud);
        return cluster;
    };

    TreeDetector::ClusterVector clusters;
    clusters.reserve(validClusters.size());
    std::transform(validClusters.begin(), validClusters.end(), std::back_inserter(clusters), clusterCloudToCluster);

    return clusters;
}


} 
