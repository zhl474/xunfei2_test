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

Mat white_balance(const Mat& img) {
    Mat yuv;
    cvtColor(img, yuv, COLOR_BGR2YCrCb);
    vector<Mat> yuv_channels;
    split(yuv, yuv_channels);
    Mat y = yuv_channels[0], u = yuv_channels[1], v = yuv_channels[2];
    
    // 使用mean计算平均值
    Scalar mean_u = mean(u);
    Scalar mean_v = mean(v);
    double avl_u = mean_u[0];
    double avl_v = mean_v[0];
    
    // 计算绝对偏差
    Mat du = abs(u - avl_u);
    Mat dv = abs(v - avl_v);
    double avl_du = mean(du)[0];
    double avl_dv = mean(dv)[0];
    
    // 使用矩阵运算创建掩码
    double sign_u = avl_u < 0 ? -1.0 : 1.0;
    double sign_v = avl_v < 0 ? -1.0 : 1.0;
    double target_u = avl_u + avl_du * sign_u;
    double target_v = avl_v + avl_dv * sign_v;
    
    Mat cond_u = abs(u - target_u) < (0.5 * avl_du);
    Mat cond_v = abs(v - target_v) < (0.5 * avl_dv);
    Mat mask = cond_u | cond_v;
    
    Mat num_y = Mat::zeros(y.size(), CV_8UC1);
    y.copyTo(num_y, mask);
    
    // 直方图计算
    const int histSize = 256;
    float range[] = {0, 256};
    const float* histRange = {range};
    Mat hist;
    calcHist(&num_y, 1, 0, Mat(), hist, 1, &histSize, &histRange);
    
    int ysum = countNonZero(num_y);
    int Y = 255;
    int cumSum = 0;
    int key = 0;
    
    while (Y >= 0) {
        cumSum += static_cast<int>(hist.at<float>(Y));
        if (cumSum > 0.1 * ysum) {
            key = Y;
            break;
        }
        Y--;
    }
    
    // 创建选择掩码
    Mat select_mask = (num_y > key);
    
    // 计算平均值
    Scalar avg = mean(img, select_mask);
    double avg_b = avg[0], avg_g = avg[1], avg_r = avg[2];
    
    // 矩阵运算调整像素值
    vector<Mat> channels;
    split(img, channels);
    
    channels[0].convertTo(channels[0], CV_32F);
    channels[1].convertTo(channels[1], CV_32F);
    channels[2].convertTo(channels[2], CV_32F);
    
    channels[0] *= 255.0 / avg_b;
    channels[1] *= 255.0 / avg_g;
    channels[2] *= 255.0 / avg_r;
    
    Mat result;
    merge(channels, result);
    result.convertTo(result, CV_8UC3);
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
py::array_t<uchar> mat_to_numpy(const cv::Mat& mat) {
    auto shape = vector<size_t>{
        static_cast<size_t>(mat.rows),
        static_cast<size_t>(mat.cols),
        static_cast<size_t>(3) // 总是返回3通道
    };
    // py::array_t<uchar> result({mat.rows, mat.cols, mat.channels()});
    py::array_t<uchar> result(shape);
    py::buffer_info buf = result.request();
    cv::Mat rgb;
    cvtColor(mat, rgb, COLOR_BGR2RGB);
    memcpy(buf.ptr, rgb.data, rgb.total() * rgb.elemSize());
    // uchar* ptr = (uchar*)buf.ptr;
    
    // // 复制数据
    // for (int i = 0; i < mat.rows; i++) {
    //     for (int j = 0; j < mat.cols; j++) {
    //         for (int c = 0; c < mat.channels(); c++) {
    //             ptr[(i * mat.cols + j) * mat.channels() + c] = mat.at<cv::Vec3b>(i, j)[c];
    //         }
    //     }
    // }
    
    return result;
}

// Python绑定
PYBIND11_MODULE(whitebalance, m) {
    m.doc() = "OpenCV快速白平衡模块";
    
    m.def("process", [](py::array_t<uchar> image) -> py::array_t<uchar> {
        // 转换输入图像
        cv::Mat img = numpy_to_mat(image);
        
        // 确保图像是CV_8UC3类型
        // cv::Mat img_8uc3;
        // if (img.type() != CV_8UC3) {
        //     if (img.channels() == 1) {
        //         cvtColor(img, img_8uc3, COLOR_GRAY2BGR);
        //     } else {
        //         img.convertTo(img_8uc3, CV_8UC3);
        //     }
        // } else {
        //     img_8uc3 = img;
        // }
        
        // 白平衡处理
        // auto start = high_resolution_clock::now();
        cv::Mat result = white_balance(img);
        // auto end = high_resolution_clock::now();
        // duration<double, milli> elapsed = end - start;
        // std::cout << "处理耗时：" << elapsed.count() << "ms" << std::endl;
        
        // 转换回numpy数组
        return mat_to_numpy(result);
        // return 1;
    }, "对输入图像进行快速白平衡处理", py::arg("image"));
}