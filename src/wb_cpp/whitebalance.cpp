#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <chrono>
#include <vector>
#include <opencv2/opencv.hpp>

namespace py = pybind11;
using namespace cv;
using namespace std;
using namespace std::chrono;

// 快速白平衡（灰度世界法 + HSV 滤波保留黄色）
cv::Mat white_balance_3_fast(const cv::Mat& img) {
    CV_Assert(img.type() == CV_8UC3);
    cv::Mat result = img.clone();

    // 转换到 HSV 空间（仅用于滤波）
    Mat hsv;
    cvtColor(result, hsv, COLOR_BGR2HSV);

    // 定义黄色范围（H:20~30，S/V 可调）
    Scalar lower_yellow = Scalar(20, 100, 100);
    Scalar upper_yellow = Scalar(30, 255, 255);
    Mat yellow_mask;
    inRange(hsv, lower_yellow, upper_yellow, yellow_mask);

    // 创建非黄色区域的掩膜
    Mat non_yellow_mask;
    bitwise_not(yellow_mask, non_yellow_mask);

    // 计算非黄色区域的平均值
    Scalar avg = mean(result, non_yellow_mask);
    double k = (avg[0] + avg[1] + avg[2]) / 3.0;

    // 增益计算
    double kb = k / avg[0];
    double kg = k / avg[1];
    double kr = k / avg[2];

    // 手动调增益（可选）
    kb *= 1.2;  // 加强蓝色通道（去黄）
    kr *= 0.9;  // 减弱红色通道（去黄）

    // 分离通道（向量化）
    vector<Mat> channels(3);
    split(result, channels);

    // 应用增益（向量化）
    channels[0].convertTo(channels[0], CV_32F, kb);
    channels[1].convertTo(channels[1], CV_32F, kg);
    channels[2].convertTo(channels[2], CV_32F, kr);

    // 向量化限制范围 [0, 255]
    min(channels[0], Scalar(255), channels[0]); max(channels[0], Scalar(0), channels[0]);
    min(channels[1], Scalar(255), channels[1]); max(channels[1], Scalar(0), channels[1]);
    min(channels[2], Scalar(255), channels[2]); max(channels[2], Scalar(0), channels[2]);

    // 转换回 8U
    channels[0].convertTo(channels[0], CV_8U);
    channels[1].convertTo(channels[1], CV_8U);
    channels[2].convertTo(channels[2], CV_8U);

    merge(channels, result);
    return result;
}

// numpy数组转OpenCV Mat
cv::Mat numpy_to_mat(py::array_t<uchar>& array) {
    py::buffer_info buf = array.request();
    if (buf.ndim != 3 || buf.shape[2] != 3) {
        throw std::runtime_error("输入必须是HWC格式的RGB图像 (高度, 宽度, 3)");
    }
    cv::Mat mat(buf.shape[0], buf.shape[1], CV_8UC3, (uchar*)buf.ptr);
    cv::Mat bgr;
    cvtColor(mat, bgr, COLOR_RGB2BGR);
    // return mat.clone();
    return bgr;
}

// // OpenCV Mat转numpy数组
// py::array_t<uchar> mat_to_numpy(const cv::Mat& mat) {
//     auto shape = vector<size_t>{
//         static_cast<size_t>(mat.rows),
//         static_cast<size_t>(mat.cols),
//         static_cast<size_t>(3) // 总是返回3通道
//     };
//     // py::array_t<uchar> result({mat.rows, mat.cols, mat.channels()});
//     py::array_t<uchar> result(shape);
//     py::buffer_info buf = result.request();
//     cv::Mat rgb;
//     cvtColor(mat, rgb, COLOR_BGR2RGB);
//     memcpy(buf.ptr, rgb.data, rgb.total() * rgb.elemSize());
//     // uchar* ptr = (uchar*)buf.ptr;
    
//     // // 复制数据
//     // for (int i = 0; i < mat.rows; i++) {
//     //     for (int j = 0; j < mat.cols; j++) {
//     //         for (int c = 0; c < mat.channels(); c++) {
//     //             ptr[(i * mat.cols + j) * mat.channels() + c] = mat.at<cv::Vec3b>(i, j)[c];
//     //         }
//     //     }
//     // }
    
//     return result;
// }

// Python绑定
PYBIND11_MODULE(whitebalance, m) {
    m.doc() = "OpenCV快速白平衡模块";
    
    m.def("process", [](py::array_t<uchar> image) -> py::array_t<uchar> {
        // 转换输入图像
        cv::Mat img = numpy_to_mat(image);
        
        // 确保图像是CV_8UC3类型
        cv::Mat img_8uc3;
        if (img.type() != CV_8UC3) {
            if (img.channels() == 1) {
                cvtColor(img, img_8uc3, COLOR_GRAY2BGR);
            } else {
                img.convertTo(img_8uc3, CV_8UC3);
            }
        } else {
            img_8uc3 = img;
        }
        
        // 白平衡处理
        auto start = high_resolution_clock::now();
        cv::Mat result = white_balance_3_fast(img_8uc3);
        auto end = high_resolution_clock::now();
        duration<double, milli> elapsed = end - start;
        std::cout << "处理耗时：" << elapsed.count() << "ms" << std::endl;
        
        // 转换回numpy数组
        // return mat_to_numpy(result);
        return 1;
    }, "对输入图像进行快速白平衡处理", py::arg("image"));
}