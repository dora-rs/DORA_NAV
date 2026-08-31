#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <vector>
#include <string>
#include <cstdint>
#include <cstring>

namespace py = pybind11;

// ── PointCloud2 fast parser ──────────────────────────────────────────────────
// Extracts x, y, z fields from a raw PointCloud2 byte buffer
py::dict parse_pointcloud2(
    py::bytes raw_data,
    uint32_t width,
    uint32_t height,
    uint32_t point_step,
    uint32_t x_offset,
    uint32_t y_offset,
    uint32_t z_offset
) {
    std::string buf(raw_data);
    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(buf.data());
    size_t n_points = width * height;

    auto xs = py::array_t<float>(n_points);
    auto ys = py::array_t<float>(n_points);
    auto zs = py::array_t<float>(n_points);

    float* xp = xs.mutable_data();
    float* yp = ys.mutable_data();
    float* zp = zs.mutable_data();

    for (size_t i = 0; i < n_points; ++i) {
        const uint8_t* point = ptr + i * point_step;
        std::memcpy(&xp[i], point + x_offset, sizeof(float));
        std::memcpy(&yp[i], point + y_offset, sizeof(float));
        std::memcpy(&zp[i], point + z_offset, sizeof(float));
    }

    py::dict result;
    result["x"] = xs;
    result["y"] = ys;
    result["z"] = zs;
    result["n_points"] = n_points;
    return result;
}

// ── Image fast decoder ───────────────────────────────────────────────────────
// Returns a numpy array shaped (height, width, channels)
py::array_t<uint8_t> decode_image(
    py::bytes raw_data,
    uint32_t height,
    uint32_t width,
    uint32_t channels
) {
    std::string buf(raw_data);
    size_t expected = height * width * channels;

    auto out = py::array_t<uint8_t>({height, width, channels});
    std::memcpy(out.mutable_data(), buf.data(), std::min(expected, buf.size()));
    return out;
}

PYBIND11_MODULE(ourrosbag_cpp, m) {
    m.doc() = "ourrosbag C++ fast parser (pybind11)";

    m.def("parse_pointcloud2", &parse_pointcloud2,
        py::arg("raw_data"),
        py::arg("width"),
        py::arg("height"),
        py::arg("point_step"),
        py::arg("x_offset"),
        py::arg("y_offset"),
        py::arg("z_offset"),
        "Parse raw PointCloud2 bytes into x/y/z numpy arrays"
    );

    m.def("decode_image", &decode_image,
        py::arg("raw_data"),
        py::arg("height"),
        py::arg("width"),
        py::arg("channels") = 3,
        "Decode raw image bytes into a (H, W, C) numpy array"
    );
}