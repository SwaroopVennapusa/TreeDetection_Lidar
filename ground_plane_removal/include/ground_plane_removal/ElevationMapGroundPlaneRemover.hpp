

#pragma once
#include "ground_plane_removal/Parameters.hpp"
#include "ground_plane_removal/typedefs.hpp"
#include "ground_plane_removal/GroundPlaneRemover.hpp"
#include "grid_map_pcl/GridMapPclLoader.hpp"


namespace ground_removal{


class ElevationMapGroundPlaneRemover : public GroundPlaneRemover {
public:
	ElevationMapGroundPlaneRemover() = default;
	explicit ElevationMapGroundPlaneRemover(ros::NodeHandle *nh);
	virtual ~ElevationMapGroundPlaneRemover() = default;

	 void removeGroundPlane() override;
     void setParameters(const GroundPlaneRemoverParam &p);
     const ElevationMapGroundPlaneRemoverParam &getParameters() const;
     const grid_map::GridMap &getElevationMap() const;

private:

     void snapToMapLimits(const grid_map::GridMap &map, double *x, double *y) const;

	 ElevationMapGroundPlaneRemoverParam param_;
	 grid_map::GridMapPclLoader pclToGridMap_;
	 grid_map::GridMap elevationMap_;
	 ros::NodeHandle *nh_ = nullptr;

};


}  // namespace ground_removal
