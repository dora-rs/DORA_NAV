#include "multi_lidar_calibrator.h"

extern "C"
{
#include "node_api.h"
}

#include <cstring>
#include <cmath>
#include <sstream>

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
MultiLidarCalibratorApp::MultiLidarCalibratorApp()
    : dora_context_(nullptr),
      parent_timestamp_us_(0),
      child_timestamp_us_(0),
      current_guess_(Eigen::Matrix4f::Identity()),
      voxel_size_(0.1),
      ndt_epsilon_(0.01),
      ndt_step_size_(0.1),
      ndt_resolution_(1.0),
      ndt_iterations_(400),
      child_topic_num_(0)
{
}

// ---------------------------------------------------------------------------
// ParsePointCloud
// ---------------------------------------------------------------------------
bool MultiLidarCalibratorApp::ParsePointCloud(const uint8_t *data, size_t len,
                                               uint64_t &timestamp_us,
                                               pcl::PointCloud<PointT>::Ptr &cloud)
{
    // Minimum: 16-byte header
    if (len < 16)
    {
        std::cerr << "[" << __APP_NAME__ << "] ParsePointCloud: buffer too small (" << len << " bytes)\n";
        return false;
    }

    // Parse header
    // uint32_t seq;
    // std::memcpy(&seq, data + 0, sizeof(uint32_t));
    std::memcpy(&timestamp_us, data + 8, sizeof(uint64_t));

    const size_t point_bytes = len - 16;
    if (point_bytes % 16 != 0)
    {
        std::cerr << "[" << __APP_NAME__ << "] ParsePointCloud: unexpected payload size\n";
        return false;
    }
    const size_t points_num = point_bytes / 16;

    cloud->clear();
    cloud->reserve(points_num);

    for (size_t i = 0; i < points_num; ++i)
    {
        float x, y, z, intensity;
        const size_t base = 16 + i * 16;
        std::memcpy(&x,         data + base + 0,  sizeof(float));
        std::memcpy(&y,         data + base + 4,  sizeof(float));
        std::memcpy(&z,         data + base + 8,  sizeof(float));
        std::memcpy(&intensity, data + base + 12, sizeof(float));

        // Skip invalid (NaN / Inf) points
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
            continue;

        PointT pt;
        pt.x = x;
        pt.y = y;
        pt.z = z;
        cloud->push_back(pt);
    }

    return true;
}

// ---------------------------------------------------------------------------
// DownsampleCloud
// ---------------------------------------------------------------------------
void MultiLidarCalibratorApp::DownsampleCloud(pcl::PointCloud<PointT>::ConstPtr in_cloud_ptr,
                                               pcl::PointCloud<PointT>::Ptr       out_cloud_ptr,
                                               double in_leaf_size)
{
    pcl::VoxelGrid<PointT> voxelized;
    voxelized.setInputCloud(in_cloud_ptr);
    voxelized.setLeafSize(static_cast<float>(in_leaf_size),
                          static_cast<float>(in_leaf_size),
                          static_cast<float>(in_leaf_size));
    voxelized.filter(*out_cloud_ptr);
}

// ---------------------------------------------------------------------------
// PublishCloud – serialize and send via dora
// Output id: "pointcloud_calibrated"
// Format: same binary layout as the lidar driver nodes
// ---------------------------------------------------------------------------
void MultiLidarCalibratorApp::PublishCloud(pcl::PointCloud<PointT>::ConstPtr cloud_ptr,
                                            uint64_t timestamp_us)
{
    const size_t points_num  = cloud_ptr->size();
    const size_t payload_len = 16 + points_num * 16;

    std::vector<uint8_t> payload(payload_len, 0);

    // Header
    static uint32_t seq = 0;
    std::memcpy(payload.data() + 0, &seq,          sizeof(uint32_t));
    std::memcpy(payload.data() + 8, &timestamp_us, sizeof(uint64_t));
    ++seq;

    // Points
    for (size_t i = 0; i < points_num; ++i)
    {
        const PointT &pt = (*cloud_ptr)[i];
        float x = pt.x, y = pt.y, z = pt.z, intensity = 0.0f;
        const size_t base = 16 + i * 16;
        std::memcpy(payload.data() + base + 0,  &x,         sizeof(float));
        std::memcpy(payload.data() + base + 4,  &y,         sizeof(float));
        std::memcpy(payload.data() + base + 8,  &z,         sizeof(float));
        std::memcpy(payload.data() + base + 12, &intensity, sizeof(float));
    }

    std::string out_id = "pointcloud_calibrated";
    int result = dora_send_output(dora_context_,
                                  &out_id[0], out_id.size(),
                                  reinterpret_cast<char *>(payload.data()), payload_len);
    if (result != 0)
    {
        std::cerr << "[" << __APP_NAME__ << "] PublishCloud: failed to send output\n";
    }
}

// ---------------------------------------------------------------------------
// PerformNdtOptimize
// ---------------------------------------------------------------------------
void MultiLidarCalibratorApp::PerformNdtOptimize()
{
    if (!in_parent_cloud_ || !in_child_cloud_ || in_parent_cloud_->empty() || in_child_cloud_->empty())
    {
        return;
    }

    // Print centroids to help diagnose coordinate/offset issues
    {
        Eigen::Vector4f centroid_parent, centroid_child;
        pcl::compute3DCentroid(*in_parent_cloud_, centroid_parent);
        pcl::compute3DCentroid(*in_child_cloud_,  centroid_child);
        std::cout << "[" << __APP_NAME__ << "] parent centroid: ("
                  << centroid_parent[0] << ", " << centroid_parent[1] << ", " << centroid_parent[2]
                  << ")  points=" << in_parent_cloud_->size() << "\n";
        std::cout << "[" << __APP_NAME__ << "] child  centroid: ("
                  << centroid_child[0]  << ", " << centroid_child[1]  << ", " << centroid_child[2]
                  << ")  points=" << in_child_cloud_->size() << "\n";
        std::cout << "[" << __APP_NAME__ << "] centroid diff (child-parent): ("
                  << centroid_child[0] - centroid_parent[0] << ", "
                  << centroid_child[1] - centroid_parent[1] << ", "
                  << centroid_child[2] - centroid_parent[2] << ")\n";
    }

    // NDT setup
    pcl::NormalDistributionsTransform<PointT, PointT> ndt;
    ndt.setTransformationEpsilon(ndt_epsilon_);
    ndt.setStepSize(ndt_step_size_);
    ndt.setResolution(static_cast<float>(ndt_resolution_));
    ndt.setMaximumIterations(ndt_iterations_);

    ndt.setInputSource(in_child_filtered_cloud_);
    ndt.setInputTarget(in_parent_cloud_);

    // Build initial guess from config (only on first call)
    if (current_guess_ == Eigen::Matrix4f::Identity())
    {
        const auto &init_params = transfer_map_[points_child_topic_str_];
        // init_params: [x, y, z, yaw, pitch, roll]
        Eigen::Translation3f    init_translation(static_cast<float>(init_params[0]),
                                                 static_cast<float>(init_params[1]),
                                                 static_cast<float>(init_params[2]));
        Eigen::AngleAxisf init_rotation_x(static_cast<float>(init_params[5]), Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf init_rotation_y(static_cast<float>(init_params[4]), Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf init_rotation_z(static_cast<float>(init_params[3]), Eigen::Vector3f::UnitZ());

        current_guess_ = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();
    }

    pcl::PointCloud<PointT>::Ptr output_cloud(new pcl::PointCloud<PointT>);
    ndt.align(*output_cloud, current_guess_);

    std::cout << "[" << __APP_NAME__ << "] NDT converged: " << ndt.hasConverged()
              << "  score: "       << ndt.getFitnessScore()
              << "  prob: "        << ndt.getTransformationProbability() << "\n";
    std::cout << "[" << __APP_NAME__ << "] transformation from "
              << child_frame_ << " to " << parent_frame_ << "\n";

    // Apply final transform to the full (unfiltered) child cloud
    pcl::transformPointCloud(*in_child_cloud_, *output_cloud, ndt.getFinalTransformation());

    current_guess_ = ndt.getFinalTransformation();

    Eigen::Matrix3f rotation_matrix    = current_guess_.block<3, 3>(0, 0);
    Eigen::Vector3f translation_vector = current_guess_.block<3, 1>(0, 3);

    std::cout << "[" << __APP_NAME__ << "] Transformation (translation | euler ZYX):\n"
              << "  translation: " << translation_vector.transpose() << "\n"
              << "  euler (ZYX): " << rotation_matrix.eulerAngles(2, 1, 0).transpose() << "\n";

    std::cout << "[" << __APP_NAME__ << "] Transformation matrix:\n" << current_guess_ << "\n\n";

    // Publish calibrated cloud
    uint64_t ts = (parent_timestamp_us_ > child_timestamp_us_) ? parent_timestamp_us_ : child_timestamp_us_;
    PublishCloud(output_cloud, ts);
}

// ---------------------------------------------------------------------------
// Initialize – load config file, set parameters
// ---------------------------------------------------------------------------
void MultiLidarCalibratorApp::Initialize(const std::string &init_file_path)
{
    // ---------- NDT / filter defaults ----------
    voxel_size_    = 0.1;
    ndt_epsilon_   = 0.01;
    ndt_step_size_ = 0.1;
    ndt_resolution_= 1.0;
    ndt_iterations_= 400;

    // ---------- dora topic names ----------
    // Parent = AIRY (rslidar), Child = MID360 (livox)
    points_parent_topic_str_ = "pointcloud_rs";
    points_child_topic_str_  = "pointcloud_livox";

    // Frame ids (used for log messages only, not actual tf)
    parent_frame_ = "rslidar";
    child_frame_  = "livox";

    // ---------- Load initial pose from config ----------
    std::ifstream ifs(init_file_path);
    if (!ifs.is_open())
    {
        std::cerr << "[" << __APP_NAME__ << "] Warning: cannot open init file: "
                  << init_file_path << ", using identity initial guess\n";
        return;
    }

    ifs >> child_topic_num_;
    for (int j = 0; j < child_topic_num_; ++j)
    {
        std::string child_name;
        ifs >> child_name;
        std::vector<double> tmp_transfer;
        for (int k = 0; k < 6; ++k)
        {
            double val;
            ifs >> val;
            tmp_transfer.push_back(val);
        }
        transfer_map_[child_name] = tmp_transfer;
    }

    std::cout << "[" << __APP_NAME__ << "] Loaded " << child_topic_num_
              << " child topic(s) from " << init_file_path << "\n";
}

// ---------------------------------------------------------------------------
// Run – dora event loop
// ---------------------------------------------------------------------------
void MultiLidarCalibratorApp::Run(void *dora_context)
{
    dora_context_ = dora_context;

    // Default config path; can also be passed as env variable if needed
    const std::string init_file_path = "multi_lidar_calibration/cfg/child_topic_list";
    Initialize(init_file_path);

    // Allocate cloud storage
    in_parent_cloud_.reset(new pcl::PointCloud<PointT>());
    in_child_cloud_.reset(new pcl::PointCloud<PointT>());
    in_child_filtered_cloud_.reset(new pcl::PointCloud<PointT>());

    std::cout << "[" << __APP_NAME__ << "] Ready. Waiting for data...\n";

    // Maximum allowed time difference between parent and child frames (µs)
    const uint64_t sync_threshold_us = 100000; // 100 ms

    while (true)
    {
        void *event = dora_next_event(dora_context_);
        if (event == nullptr)
        {
            std::cerr << "[" << __APP_NAME__ << "] ERROR: unexpected end of event\n";
            break;
        }

        enum DoraEventType ty = read_dora_event_type(event);

        if (ty == DoraEventType_Input)
        {
            // Identify which input arrived
            char  *id_ptr = nullptr;
            size_t id_len = 0;
            read_dora_input_id(event, &id_ptr, &id_len);
            std::string input_id(id_ptr, id_len);

            char  *data_ptr = nullptr;
            size_t data_len = 0;
            read_dora_input_data(event, &data_ptr, &data_len);

            const uint8_t *data = reinterpret_cast<const uint8_t *>(data_ptr);

            if (input_id == points_parent_topic_str_)
            {
                // Parent cloud (AIRY)
                uint64_t ts = 0;
                pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
                if (ParsePointCloud(data, data_len, ts, cloud))
                {
                    in_parent_cloud_    = cloud;
                    parent_timestamp_us_= ts;
                    std::cout << "[" << __APP_NAME__ << "] parent  ts=" << ts
                              << " us  (" << static_cast<double>(ts) * 1e-6 << " s)\n";
                }
            }
            else if (input_id == points_child_topic_str_)
            {
                // Child cloud (MID360)
                uint64_t ts = 0;
                pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
                if (ParsePointCloud(data, data_len, ts, cloud))
                {
                    in_child_cloud_    = cloud;
                    child_timestamp_us_= ts;
                    std::cout << "[" << __APP_NAME__ << "] child   ts=" << ts
                              << " us  (" << static_cast<double>(ts) * 1e-6 << " s)\n";

                    // Downsample child cloud for NDT
                    in_child_filtered_cloud_.reset(new pcl::PointCloud<PointT>());
                    DownsampleCloud(in_child_cloud_, in_child_filtered_cloud_, voxel_size_);
                }
            }

            // Trigger NDT when both clouds are available and roughly time-synchronised
            if (in_parent_cloud_ && !in_parent_cloud_->empty() &&
                in_child_cloud_  && !in_child_cloud_->empty())
            {
                uint64_t dt = (parent_timestamp_us_ > child_timestamp_us_)
                              ? (parent_timestamp_us_ - child_timestamp_us_)
                              : (child_timestamp_us_  - parent_timestamp_us_);

                std::cout << "[" << __APP_NAME__ << "] dt=" << dt
                          << " us  (threshold=" << sync_threshold_us << " us)"
                          << (dt <= sync_threshold_us ? "  -> NDT triggered" : "  -> skipped (out of sync)") << "\n";

                if (dt <= sync_threshold_us)
                {
                    PerformNdtOptimize();
                }
            }
        }
        else if (ty == DoraEventType_Stop)
        {
            std::cout << "[" << __APP_NAME__ << "] received stop event\n";
            free_dora_event(event);
            break;
        }
        else if (ty == DoraEventType_InputClosed)
        {
            std::cout << "[" << __APP_NAME__ << "] input closed\n";
        }
        else
        {
            std::cout << "[" << __APP_NAME__ << "] received unexpected event: " << ty << "\n";
        }

        free_dora_event(event);
    }

    std::cout << "[" << __APP_NAME__ << "] END\n";
}
