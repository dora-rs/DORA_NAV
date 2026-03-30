extern "C"
{
#include "node_api.h"
}

#include <Eigen/Dense>
#include <cstring>
#include <cstdint>
#include <vector>
#include <iostream>
#include <algorithm>

static const Eigen::Matrix4f T_CHILD_TO_PARENT =
    (Eigen::Matrix4f() <<
         0.999404f, -0.005929f,  0.033997f, -0.451992f,
         0.005726f,  0.999965f,  0.006054f, -0.007711f,
        -0.034031f, -0.005856f,  0.999404f,  0.076423f,
         0.0f,       0.0f,       0.0f,       1.0f)
    .finished();

struct Point { float x, y, z, intensity; };

struct CloudBuf {
    bool valid = false;
    uint64_t timestamp_us = 0;
    uint32_t seq = 0;
    std::vector<Point> points;
};

static bool parse(const char* data, size_t len, CloudBuf& out)
{
    if (len < 16) return false;
    std::memcpy(&out.seq,          data + 0, sizeof(uint32_t));
    std::memcpy(&out.timestamp_us, data + 8, sizeof(uint64_t));
    size_t n = (len - 16) / 16;
    out.points.resize(n);
    for (size_t i = 0; i < n; ++i) {
        float* dst = reinterpret_cast<float*>(&out.points[i]);
        std::memcpy(dst + 0, data + 16 + i*16 + 0,  sizeof(float));
        std::memcpy(dst + 1, data + 16 + i*16 + 4,  sizeof(float));
        std::memcpy(dst + 2, data + 16 + i*16 + 8,  sizeof(float));
        std::memcpy(dst + 3, data + 16 + i*16 + 12, sizeof(float));
    }
    out.valid = true;
    return true;
}

static Point transform(const Point& p)
{
    Eigen::Vector4f v(p.x, p.y, p.z, 1.0f);
    Eigen::Vector4f r = T_CHILD_TO_PARENT * v;
    return {r[0], r[1], r[2], p.intensity};
}

static void send_merged(void* ctx, const CloudBuf& rs, const CloudBuf& livox, uint32_t& seq)
{
    uint32_t total = static_cast<uint32_t>(rs.points.size() + livox.points.size());
    size_t sz = 16 + total * 16;
    std::vector<uint8_t> buf(sz);

    uint32_t* seq_ptr = (uint32_t*)buf.data();
    *seq_ptr = seq++;
    uint64_t ts = std::max(rs.timestamp_us, livox.timestamp_us);
    uint64_t* ts_ptr = (uint64_t*)(buf.data() + 8);
    *ts_ptr = ts;

    size_t idx = 0;
    for (const auto& p : rs.points) {
        float* dst = (float*)(buf.data() + 16 + idx * 16);
        *(dst + 0) = p.x;
        *(dst + 1) = p.y;
        *(dst + 2) = p.z;
        *(dst + 3) = p.intensity;
        ++idx;
    }
    for (const auto& p : livox.points) {
        Point q = transform(p);
        float* dst = (float*)(buf.data() + 16 + idx * 16);
        *(dst + 0) = q.x;
        *(dst + 1) = q.y;
        *(dst + 2) = q.z;
        *(dst + 3) = q.intensity;
        ++idx;
    }

    std::string out_id = "pointcloud";
    int result = dora_send_output(ctx, &out_id[0], out_id.length(),
                                  reinterpret_cast<char*>(buf.data()), sz);
    if (result != 0)
        std::cerr << "[merger] failed to send output" << std::endl;
}

int main()
{
    std::cout << "[merger] lidar merger node start" << std::endl;

    void* ctx = init_dora_context_from_env();
    if (ctx == NULL) {
        fprintf(stderr, "[merger] failed to init dora context\n");
        return -1;
    }

    CloudBuf rs_buf, livox_buf;
    uint32_t seq = 0;

    while (true) {
        void* event = dora_next_event(ctx);
        if (event == NULL) {
            printf("[merger] ERROR: unexpected end of event\n");
            break;
        }

        enum DoraEventType ty = read_dora_event_type(event);

        if (ty == DoraEventType_Stop) {
            printf("[merger] received stop event\n");
            free_dora_event(event);
            break;
        }
        else if (ty == DoraEventType_Input) {
            char* id_ptr;
            size_t id_len;
            read_dora_input_id(event, &id_ptr, &id_len);
            std::string id(id_ptr, id_len);

            char* data;
            size_t data_len;
            read_dora_input_data(event, &data, &data_len);

            if (id == "pointcloud_rs") {
                parse(data, data_len, rs_buf);
            } else if (id == "pointcloud_livox") {
                parse(data, data_len, livox_buf);
            } else {
                printf("[merger] received unexpected input: %s\n", id.c_str());
            }

            if (rs_buf.valid && livox_buf.valid) {
                send_merged(ctx, rs_buf, livox_buf, seq);
                rs_buf.valid = false;
                livox_buf.valid = false;
            }
        }
        else {
            printf("[merger] received unexpected event: %d\n", ty);
        }

        free_dora_event(event);
    }

    free_dora_context(ctx);
    printf("[merger] finished\n");
    return 0;
}
