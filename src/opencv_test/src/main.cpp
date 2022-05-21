#include "opencv2/imgcodecs.hpp"
#include "opencv2/stitching.hpp"
#include <iostream>


// 图片拼接代码，参考如下链接
// https://docs.opencv.org/4.5.1/d1/d46/group__stitching.html
// https://docs.opencv.org/master/d8/d19/tutorial_stitcher.html


using namespace std;
using namespace cv;


Stitcher::Mode mode;
vector<Mat> imgs;
string result_name;



int parameter(void)
{
    result_name = "/home/ucar/ucar_ws/src/opencv_test/src/pic/result.jpg";	// 输出结果存放
    mode = Stitcher::PANORAMA;		// 或者mode = Stitcher::SCANS;前者是普通模式，后者是专业模式
    Mat img = imread("/home/ucar/ucar_ws/src/opencv_test/src/pic/1.jpg");	//读取要拼接的图片
    imgs.push_back(img);		//将读到的图片依次放到imgs里，push_back为vector标准类模板操作函数，指在尾端加入数据
    img = imread("/home/ucar/ucar_ws/src/opencv_test/src/pic/2.jpg");	//读取要拼接的图片
    imgs.push_back(img);		//将读到的图片依次放到imgs里，push_back为标准类模板操作函数，指在尾端加入数据
    return EXIT_SUCCESS;
}





int main(int argc, char* argv[])
{
    int retval = parameter();
    if (retval){
        cout << "load parameter failed" << endl;
        return EXIT_FAILURE;
    }else{
        cout << "load parameter succeed" << endl;
    }

    //![stitching]
    Mat pano;
    Ptr<Stitcher> stitcher = Stitcher::create(mode);  		// 创建了一个新的stitcher实例，Ptr<>是一个类模板，这里表示继承Stitcher
    Stitcher::Status status = stitcher->stitch(imgs, pano); 	// 将imgs合并到pano。并返回一个状态。

    if (status != Stitcher::OK)
    {
        cout << "Can't stitch images, error code = " << int(status) << endl;
        return EXIT_FAILURE;
    }
    //![stitching]

    imwrite(result_name, pano);                			// 保存图片   		
    cout << "stitching completed successfully\n" << result_name << " saved!";
    return EXIT_SUCCESS;
}




