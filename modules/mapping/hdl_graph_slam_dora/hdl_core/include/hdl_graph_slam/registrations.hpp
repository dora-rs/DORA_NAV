// SPDX-License-Identifier: BSD-2-Clause

#ifndef HDL_GRAPH_SLAM_REGISTRATIONS_HPP
#define HDL_GRAPH_SLAM_REGISTRATIONS_HPP

#include <yaml-cpp/yaml.h>
#include <pcl/registration/registration.h>

namespace hdl_graph_slam {

/**
 * @brief select a scan matching algorithm according to config
 * @param config YAML configuration node
 * @return selected scan matching
 */
pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr select_registration_method(const YAML::Node& config);

}  // namespace hdl_graph_slam

#endif  //
