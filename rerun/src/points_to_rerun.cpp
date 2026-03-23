extern "C"
{
#include "node_api.h"
#include "operator_api.h"
#include "operator_types.h"
}
#include <string>
#include <assert.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <rerun.hpp>
#include <thread>
#include <pthread.h>
#include <cmath>
#include <deque>
#include <time.h>
#include <sys/time.h>
#include <iomanip>
#include "SlamPose.h"

using namespace std;

Pose2D_h pose;

// Trajectory history for robot trail
static std::deque<rerun::Position3D> pose_history;
static const size_t MAX_TRAIL_POINTS = 500;



bool read_map(const string globalmap_pcd, rerun::RecordingStream& rec)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::io::loadPCDFile(globalmap_pcd, *map_cloud);

    std::vector<rerun::Position3D> map_points;
    std::vector<rerun::Color> map_colors;  

    for (const auto& pt : map_cloud->points) {
        map_points.emplace_back(pt.x, pt.y, pt.z);
        map_colors.emplace_back(255, 0, 0);
    }

    rec.log("map_points_static", rerun::Points3D(map_points).with_colors(map_colors).with_radii({0.02f}));
    std::cout << "Loaded " << map_cloud->points.size()<< " map to Rerun." << std::endl;

    return true;
}

// pcl::PointCloud<pcl::PointXYZI>::Ptr bytes2cloud(const char *bytes, int32_t size)
// {
//     if (size <= 0)
//     {
//         std::cerr << "Error: Point cloud size <= 0!" << std::endl;
//         return NULL;
//     }

//     pcl::PointCloud<pcl::PointXYZI>::Ptr row_cloud(new pcl::PointCloud<pcl::PointXYZI>());
//     row_cloud->header.seq = *(std::uint32_t *)bytes;
//     row_cloud->header.stamp = *(std::uint64_t *)(bytes + 8);
//     row_cloud->header.frame_id = "rslidar";
//     row_cloud->width = size;
//     row_cloud->height = 1;
//     row_cloud->is_dense = true;
//     for (size_t i = 0; i < size; i++) {
//         pcl::PointXYZI tem_point;
//         tem_point.x = *(float *)(bytes + 16 + 16 * i);
//         tem_point.y = *(float *)(bytes + 16 + 4 + 16 * i);
//         tem_point.z = *(float *)(bytes + 16 + 8 + 16 * i);
//         tem_point.intensity = *(float *)(bytes + 16 + 12 + 16 * i);
//         row_cloud->points.push_back(tem_point);
//     }

//     return row_cloud;
// }

// pcl::PointCloud<pcl::PointXYZI>::Ptr rslidar2baselink(const pcl::PointCloud<pcl::PointXYZI>::Ptr points)
// {
//     Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
//     //TODO 90°
//     transform(0, 1) = -1; 
//     transform(1, 0) = 1; 
//     transform(0, 0) = 0; 
//     transform(1, 1) = 0; 

//     pcl::PointCloud<pcl::PointXYZI>::Ptr trans_cloud(new pcl::PointCloud<pcl::PointXYZI>());
//     pcl::transformPointCloud(*points, *trans_cloud, transform);

//     return trans_cloud;
// }


// bool points2rerun(const pcl::PointCloud<pcl::PointXYZI>::Ptr trans_clouds, rerun::RecordingStream& rec)
// {
//     // struct timeval tv;
//     // gettimeofday(&tv, NULL);//获取时间
//     pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
//     std::vector<rerun::Position3D> rerun_points;
//     std::vector<rerun::Color> colors;

//     for (const auto& pt : trans_clouds->points) {
//         rerun_points.emplace_back(pt.x, pt.y, pt.z);
//         colors.emplace_back(0, 255, 0);
//     }

//     rec.log("live_points", rerun::Points3D(rerun_points).with_colors(colors).with_radii({0.05f}));
//     // std::cout << "Logging " << rerun_points.size() << " points to Rerun." << std::endl;
//     // struct timeval tv_1;
//     // gettimeofday(&tv_1, NULL);//获取时间
//     // auto start = tv.tv_sec + tv.tv_usec * 1e-6;
//     // auto end = tv_1.tv_sec + tv_1.tv_usec * 1e-6;
//     // auto all_time = end - start;
//     // std::cout << "Time: " << all_time << std::endl;
//     // double stamp = (double)cloud->header.stamp*1.0*1e-6;

//     return true;
// }

bool clouds2rerun(const char *bytes, int32_t size, rerun::RecordingStream& rec)
{
    if (size <= 0)
    {
        std::cerr << "Error: Point cloud size <= 0!" << std::endl;
        return false;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr row_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    row_cloud->header.seq = *(std::uint32_t *)bytes;
    row_cloud->header.stamp = *(std::uint64_t *)(bytes + 8);
    row_cloud->header.frame_id = "rslidar";
    row_cloud->width = size;
    row_cloud->height = 1;
    row_cloud->is_dense = true;
    std::vector<rerun::Position3D> rerun_points;
    std::vector<rerun::Color> colors;
    for (size_t i = 0; i < size; i++) {
        pcl::PointXYZI tem_point;
        tem_point.x = *(float *)(bytes + 16 + 16 * i);
        tem_point.y = *(float *)(bytes + 16 + 4 + 16 * i);
        tem_point.z = *(float *)(bytes + 16 + 8 + 16 * i);
        tem_point.intensity = *(float *)(bytes + 16 + 12 + 16 * i);
        rerun_points.emplace_back(tem_point.x, tem_point.y, tem_point.z);
    }
    rec.log("live_points", rerun::Points3D(rerun_points).with_colors(0x00FF00FF).with_radii({0.02f}));

    rec.log(
        "live_points",
        rerun::Transform3D::from_translation_rotation(
            {pose.x, pose.y, 0.0f},
            rerun::RotationAxisAngle{
                {0.0f, 0.0f, 1.0f},
                rerun::Angle::degrees(pose.theta),
            }
        )
    );
    // std::cout << "Logging " << rerun_points.size() << " points to Rerun." << std::endl;

    return true;
}


void path2rerun(char *msg, size_t data_len, rerun::RecordingStream& rec)
{

    int num_points = data_len / sizeof(float); 
    float* float_array = reinterpret_cast<float*>(msg); 

    int num_xy_points = num_points / 2; 

    std::vector<float> x_v(float_array, float_array + num_xy_points);
    std::vector<float> y_v(float_array + num_xy_points, float_array + num_points); 

    vector<rerun::Position3D> points;
    points.reserve(num_xy_points);
    for (int i = 0; i < num_xy_points; ++i)
    {
        points.emplace_back(x_v[i], y_v[i], 0.0);
    }
    std::vector<rerun::LineStrip3D> line_strips = {rerun::LineStrip3D(points)};
    rec.log("path_points", rerun::LineStrips3D(line_strips).with_colors(0x0000FFFF).with_radii({0.08f}));
    rec.log(
        "path_points",
        rerun::Transform3D::from_translation_rotation(
            {pose.x, pose.y, 0.0f},
            rerun::RotationAxisAngle{
                {0.0f, 0.0f, 1.0f},
                rerun::Angle::degrees(pose.theta - 90.0f),//车辆坐标系的正前方是y坐标，为了统一到全局，需要逆时针旋转90°
            }
        )
    );

    return;
}


void pose2rerun(char *msg, rerun::RecordingStream& rec)
{
    Pose2D_h *cur_pose = reinterpret_cast<Pose2D_h *>(msg);
    pose.x = cur_pose->x;
    pose.y = cur_pose->y;
    pose.theta = cur_pose->theta;

    float theta = pose.theta;
    float angle = theta * (M_PI / 180.0f);

    // 1) Direction arrow - bright cyan, longer
    float length = 2.5f;
    std::vector<rerun::Position3D> origins = {{pose.x, pose.y, 0.2f}};
    std::vector<rerun::Vector3D> vectors = {{length * cosf(angle), length * sinf(angle), 0.0f}};
    rec.log(
        "robot/arrow",
        rerun::Arrows3D::from_vectors(vectors).with_origins(origins).with_colors(0x00FFFFFF)
    );

    // 2) Robot body - yellow box at robot position, rotated
    rec.log(
        "robot/body",
        rerun::Boxes3D::from_centers_and_half_sizes(
            {{pose.x, pose.y, 0.1f}},
            {{0.25f, 0.15f, 0.1f}}
        )
        .with_rotation_axis_angles({rerun::RotationAxisAngle({0.0f, 0.0f, 1.0f}, rerun::Angle::radians(angle))})
        .with_colors({rerun::Rgba32(255, 255, 0)})
        .with_fill_mode(rerun::FillMode::Solid)
        .with_labels({"Robot"})
    );

    // 3) Trajectory trail - orange line showing where robot has been
    pose_history.push_back({pose.x, pose.y, 0.05f});
    if (pose_history.size() > MAX_TRAIL_POINTS) {
        pose_history.pop_front();
    }
    if (pose_history.size() >= 2) {
        std::vector<rerun::Position3D> trail(pose_history.begin(), pose_history.end());
        std::vector<rerun::LineStrip3D> trail_strips = {rerun::LineStrip3D(trail)};
        rec.log(
            "robot/trail",
            rerun::LineStrips3D(rerun::Collection<rerun::components::LineStrip3D>(std::move(trail_strips)))
                .with_colors(0xFF6600FF)
                .with_radii({0.06f})
        );
    }

    return;
}

vector<rerun::Position3D> read_path(const string& waypoints_txt) 
{
    ifstream file(waypoints_txt);
    vector<rerun::Position3D> points;

    if (!file.is_open()) {
        cerr << "Error: Cannot open file " << waypoints_txt << endl;
        return points;
    }

    string line;
    while (getline(file, line)) 
    {
        stringstream ss(line);
        float x, y;
        if (ss >> x >> y) 
        {   
            // cout << "x: " << x << " y: " << y << endl;
            points.emplace_back(rerun::Position3D(x, y, 0));
        }
    }
    cout << "PATH Loaded " << points.size() << endl;
    file.close();
    return points;
}

bool waypoints2rerun(const string& waypoints_txt, rerun::RecordingStream& rec)
{
    auto path_points = read_path(waypoints_txt);
    if (path_points.empty()) {
        cerr << "can not open txt file." << endl;
        return false;
    }
    std::vector<rerun::LineStrip3D> line_strips = {rerun::LineStrip3D(path_points)};
    rec.log("global_path_points", rerun::LineStrips3D(line_strips).with_colors(0x00FFFFFF).with_radii({0.08f}));

    return true;
}


int run(void *dora_context, rerun::RecordingStream& rec)
{
    while (true)
    {
        void * event = dora_next_event(dora_context);
        if (event == NULL)
        {
            printf("[c node] ERROR: unexpected end of event\n");
            return -1;
        }

        enum DoraEventType ty = read_dora_event_type(event);

        if (ty == DoraEventType_Input)
        {
            char *data;
            size_t data_len;
            char *data_id;
            size_t data_id_len;
            read_dora_input_data(event, &data, &data_len);
            read_dora_input_id(event, &data_id, &data_id_len);
            if (strncmp("pointcloud", data_id, 10) == 0)
            {
                //----------------------------------------------------------------------------------------------------
                    // struct timeval tv;
                    // gettimeofday(&tv, NULL);//获取时间
                int32_t point_len = (data_len - 16) / 16;          
                // auto clouds = bytes2cloud(data, point_len);
                // if (clouds == NULL)
                // {
                //     std::cerr << "Error: Failed to rec point cloud!" << std::endl;
                // }

                // auto trans_clouds = rslidar2baselink(clouds);

                // auto res = points2rerun(trans_clouds, rec);
                // if (!res)
                // {
                //     std::cerr << "Error: Failed to send point cloud to Rerun!" << std::endl;
                // }
                    // struct timeval tv_1;
                    // gettimeofday(&tv_1, NULL);//获取时间
                    // auto start = tv.tv_sec + tv.tv_usec * 1e-6;
                    // auto end = tv_1.tv_sec + tv_1.tv_usec * 1e-6;
                    // auto all_time = end - start;
                    // std::cout << "Time: " << all_time << std::endl;
                //----------------------------------------------------------------------------------------------------
                auto clouds = clouds2rerun(data, point_len, rec);
                if (!clouds)
                {
                    std::cerr << "Error: Failed to rec point cloud!" << std::endl;
                }
            }
            else if (strncmp("raw_path", data_id, 8) == 0)
            {
                path2rerun(data, data_len, rec);  
            }
            else if (strncmp("cur_pose", data_id, 8) == 0)
            {
                pose2rerun(data, rec);
            }
        }    
        else if (ty == DoraEventType_Stop)
        {
            printf("[c node] received stop event\n");
        }                
        else
        {
            printf("[c node] received unexpected event: %d\n", ty);
        }

        free_dora_event(event);    
    }

    return 0;
}

int main()
{
    std::cout << "rerun_test" << std::endl;
    void* dora_context = init_dora_context_from_env();

    rerun::RecordingStream rec("lidarpoints_viewer");
    // Connect to a Rerun viewer running on the host (docker-compose uses host networking).
    // Start the viewer on the host first: `rerun`
    rec.connect_grpc("rerun+http://127.0.0.1:9876/proxy").exit_on_failure();

    std::string globalmap_pcd ="./data/map.pcd";
    auto map_load = read_map(globalmap_pcd, rec);
    if (!map_load)
    {
        std::cerr << "Error: Failed to open map!" << std::endl;
    }
    string waypoints_txt = "./Waypoints.txt";
    auto way_loader = waypoints2rerun(waypoints_txt, rec);
    if(way_loader)
    {
        cout << "[PATH]SUCCESS to Rerun !! " << endl;
    }
    else   
    {
        cout << "[PATH]Failed to load point cloud " << endl;
    }

    auto ret = run(dora_context, rec);
    free_dora_context(dora_context);
    std::cout << "END rerun_test" << std::endl;
    return ret;
}