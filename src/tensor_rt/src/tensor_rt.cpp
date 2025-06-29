#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include "tensor_rt/Messages.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "NvInfer.h"
#include <cuda_fp16.h>

#include <vector>
#include <string>
#include <chrono>

// 记录 TensorRT 运行时的日志信息
class Logger : public nvinfer1::ILogger
{
	void log(Severity severity, const char* msg)  noexcept
	{
		// suppress info-level messages
		if (severity != Severity::kINFO)
			std::cout << msg << std::endl;
	}
} gLogger;

class Tensor_rt
{
private:
  int num_classes_ = 9;
  int img_size_ = 640;
  int output_h_ = 8400;
  int output_w_ = num_classes_+4;
  float conf_thres = 0.75;
  float iou_thres = 0.45;
  std::vector<std::string> classnum2name_ = {"chilli","tomato","banana'","apple'","watermelon'","cola","cake","milk","potato"};
  std::vector<const char*> input_names_ = {"images"};
  std::vector<const char*> output_names_ = {"output0"};
  cv::Mat cv_image;

  nvinfer1::IExecutionContext* context_ = nullptr;
  nvinfer1::ICudaEngine* engine_ = nullptr;
  nvinfer1::IRuntime* runtime_ = nullptr;
  void* buffers_[2] = { NULL, NULL };
  cudaStream_t stream_;
  std::vector<__half> prob_;
  const char* enginepath_ = "/home/zythyra/gazebo_test_ws/src/tensor_rt/model/best.engine";

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::ServiceServer server_;

  void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
      // 将 ROS 图像消息转换为 OpenCV 图像
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv_image = cv_ptr->image;
      resize_img(cv_image,cv_image,640);
      cv::imshow("ROS Camera Viewer", cv_image);
      int key = cv::waitKey(20);
      if(key == 176) {  // 按下'0'键时保存图片
            // 生成时间戳作为文件名
            std::time_t now = std::time(nullptr);
            std::string timestamp = std::to_string(now);
            std::string filename = "/home/zythyra/gazebo_image_data/saved_image_" + timestamp + ".jpg";
            
            // 调试输出2：检查图像数据是否有效
            if(cv_image.empty()) {
                ROS_ERROR("Cannot save image: image data is empty!");
                return;
            }
            ROS_INFO("Attempting to save image to: %s", filename.c_str());
            
            // 保存图片
            bool result = cv::imwrite(filename, cv_image);
      }
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }
  
  void max_class(const __half* output, std::vector<cv::Rect>& box,std::vector<float>& confidence,std::vector<int>& clsname)//图像后处理的一部分，筛选整理边界框
  {
    for(int i=0;i<8400;i++)
    {
        float max = __half2float(*(output+4*8400+i));//对于某一个检测框来说，比较class个置信度的a大小，最大的小于阈值就删除，否则保留最大
        int clsnum = 0;
        for(int j=1;j<num_classes_;j++)
        {
            float conf = __half2float(*(output+(j+4)*8400+i));
            if(conf>max)
            {
                max = conf;
                clsnum = j;
            }
        }
        if(max>conf_thres)
        {   
            int x = static_cast<int>(__half2float(*(output+i))),y = static_cast<int>(__half2float(*(output+i+8400)));
            int w = static_cast<int>(__half2float(*(output+i+2*8400))),h = static_cast<int>(__half2float(*(output+i+3*8400)));
            cv::Rect rect(x-w/2,y-h/2,w,h);//xy在左上角
            box.push_back(rect);
            clsname.push_back(clsnum);
            confidence.push_back(max);
        }
    }
    return;
  }

  void process_output(__half* output_tensor,std::vector<int32_t> &result) 
  {   
    std::vector<cv::Rect> boxes;
    std::vector<int> clsnames;
    std::vector<float> confidences;
    max_class(output_tensor,boxes,confidences,clsnames);
    ROS_INFO("clsname:%d",clsnames.size());
    // 非极大值抑制
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, conf_thres, iou_thres, indices);
    
    for(int idx : indices) {
      const cv::Rect& box = boxes[idx];
      int cls_id = clsnames[idx];
      float conf = confidences[idx]; 
      // 将坐标和尺寸转换为 int32
      result.push_back(static_cast<int32_t>(box.x));
      result.push_back(static_cast<int32_t>(box.y));
      result.push_back(static_cast<int32_t>(box.width));
      result.push_back(static_cast<int32_t>(box.height));
      // 类别ID直接转换
      result.push_back(static_cast<int32_t>(cls_id));
      result.push_back(static_cast<int32_t>(conf * 100));

      //--------------------------------可视化，后续可删除---------------------//
      cv::rectangle(cv_image, box, cv::Scalar(0,255,0), 2);
      std::string label = classnum2name_[cls_id].c_str();
      int baseline;
      cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
      int top = std::max(box.y, labelSize.height);
      cv::rectangle(cv_image, cv::Point(box.x, top - labelSize.height),
            cv::Point(box.x + labelSize.width, top + baseline), 
            cv::Scalar(0, 255, 0), cv::FILLED);
      cv::rectangle(cv_image, 
      cv::Point(box.x, box.y - labelSize.height - 5),
      cv::Point(box.x + labelSize.width, box.y),
      cv::Scalar(0, 255, 0), 
      cv::FILLED);
      cv::putText(cv_image, label, cv::Point(box.x, top), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0), 1); 
      //--------------------------------可视化，后续可删除---------------------//
    }
    cv::imshow("Result", cv_image);
    cv::waitKey(1);
    return;
  }

  bool detect_img(tensor_rt::Messages::Request& req,tensor_rt::Messages::Response& resp){
    ROS_INFO("服务器接收到请求数据");
    cv::Mat blob32 = cv::dnn::blobFromImage(cv_image,1.0/255,cv::Size(640, 640),cv::Scalar(),true,false,CV_32F);
    cv::Mat blob;
    blob32.convertTo(blob, CV_16F);
    cudaMemcpyAsync(buffers_[0], blob.ptr<__half>(), img_size_*img_size_*3*sizeof(__half), cudaMemcpyHostToDevice, stream_);
    context_->enqueueV2(buffers_, stream_, nullptr);
    cudaMemcpyAsync(prob_.data(), buffers_[1], output_h_*output_w_*sizeof(__half), cudaMemcpyDeviceToHost, stream_);
    cudaStreamSynchronize(stream_);
    std::vector<int32_t>result;
    process_output(prob_.data(),result);
    resp.result = result;

    return true;
  }


public:
  Tensor_rt() 
    : it_(nh_),
    runtime_(nvinfer1::createInferRuntime(gLogger)) { // 初始化 runtime
    ROS_INFO("初始化tensorRT和原始图像订阅");
    image_sub_ = it_.subscribe("/cam", 1, &Tensor_rt::imageCallback, this);
    cv::namedWindow("ROS Camera Viewer");

    std::ifstream file(enginepath_, std::ios::binary);
    char* trtModelStream = NULL;
    int size = 0;
    if (file.good()) {
      // 将读指针移动到文件末尾
      file.seekg(0, file.end);
      // 获取文件大小
      size = file.tellg();
      // 将读指针移动到文件开始
      file.seekg(0, file.beg);
      trtModelStream = new char[size];
      assert(trtModelStream);
      // 从关联的输入流中读取了指定数量的字符
      file.read(trtModelStream, size);
      file.close();
    }
    // 初始化推理运行时对象
    runtime_ = nvinfer1::createInferRuntime(gLogger);
    assert(runtime_ != nullptr);

    // 加载预先构建好的引擎
    engine_ = runtime_->deserializeCudaEngine(trtModelStream, size);

    // 创建执行上下文
    context_ = engine_->createExecutionContext();

    // 释放数组类型的内存
    delete[] trtModelStream;

    int input_index = engine_->getBindingIndex(input_names_[0]);
    int output_index = engine_->getBindingIndex(output_names_[0]);

    // 创建GPU显存输入/输出缓冲区(有几个就初始化几个)
    cudaMalloc(&buffers_[input_index], 1*3*img_size_*img_size_*sizeof(__half));
    cudaMalloc(&buffers_[output_index], 1*3*img_size_*img_size_*sizeof(__half));

    // 创建临时缓存输出
    prob_.resize(output_h_*output_w_);
    // 创建cuda流
    cudaStreamCreate(&stream_);

    //预热
    cv::Mat blank_image = cv::Mat::zeros(img_size_, img_size_, CV_8UC3); 
    cv_image = blank_image;
    cv::Mat blob32 = cv::dnn::blobFromImage(blank_image,1.0/255,cv::Size(640, 640),cv::Scalar(),true,false,CV_32F);
    cv::Mat blob;
    blob32.convertTo(blob, CV_16F);
    cudaMemcpyAsync(buffers_[0], blob.ptr<__half>(), img_size_*img_size_*3*sizeof(__half), cudaMemcpyHostToDevice, stream_);
    context_->enqueueV2(buffers_, stream_, nullptr);
    cudaStreamSynchronize(stream_);
    cudaMemcpyAsync(prob_.data(), buffers_[1], output_h_*output_w_*sizeof(__half), cudaMemcpyDeviceToHost, stream_);

    server_ = nh_.advertiseService("tensorRT_detect",&Tensor_rt::detect_img,this);
    ROS_INFO("模型加载、预热完成");
  }

  ~Tensor_rt()
  {
    cv::destroyWindow("ROS Camera Viewer");
    // 1. 释放 CUDA 流
    if (stream_) {
        cudaStreamDestroy(stream_);
    }

    // 2. 释放 TensorRT 对象
    if (context_) {
        context_->destroy();
    }
    if (engine_) {
        engine_->destroy();
    }
    if (runtime_) {
        runtime_->destroy();
    }

    // 3. 释放 GPU 内存
    if (buffers_) {
        const int num_bindings = engine_ ? engine_->getNbBindings() : 0;
        for (int i = 0; i < num_bindings; ++i) {
            if (buffers_[i]) {
                cudaFree(buffers_[i]);
            }
        }
    }    
  }

  Tensor_rt(const Tensor_rt&) = delete;
  Tensor_rt& operator=(const Tensor_rt&) = delete;

  bool resize_img(cv::Mat &img_i, cv::Mat &img_o, int target_size)
  {
      // 输入校验
      if(img_i.empty()) {
          std::cerr << "Error: Input image is empty" << std::endl;
          return false;
      }
      if(target_size <= 0) {
          std::cerr << "Error: Invalid target size" << std::endl;
          return false;
      }

      // 获取原始尺寸
      const cv::Size orig_size = img_i.size();
      const int orig_w = orig_size.width;
      const int orig_h = orig_size.height;

      // 计算缩放比例
      const double scale = std::min(
          static_cast<double>(target_size) / orig_w,
          static_cast<double>(target_size) / orig_h
      );

      // 计算新尺寸
      const int new_w = static_cast<int>(orig_w * scale);
      const int new_h = static_cast<int>(orig_h * scale);

      // 选择插值算法（优化选择逻辑）
      const int interpolation = (scale < 1.0) ? cv::INTER_AREA : cv::INTER_CUBIC;

      // 缩放图像
      cv::Mat resized;
      cv::resize(img_i, resized, cv::Size(new_w, new_h), 0, 0, interpolation);

      // 创建白色画布（自动处理颜色通道）
      cv::Mat canvas(target_size, target_size, img_i.type(), cv::Scalar::all(255));

      // 计算粘贴位置（精确居中）
      const cv::Rect roi(
          (target_size - new_w) / 2,  // x 偏移
          (target_size - new_h) / 2,  // y 偏移
          new_w,                      // 宽度
          new_h                       // 高度
      );

      // 将缩放后的图像复制到画布中心
      resized.copyTo(canvas(roi));

      // 输出结果
      img_o = canvas;

      return true;
  }
};

int main(int argc, char** argv) {
  setlocale(LC_ALL,"");
  ros::init(argc, argv, "tensor_rt");
  ROS_INFO("测试");
  Tensor_rt tensor_rt;
  ros::spin();
  return 0;
}