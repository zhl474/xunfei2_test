from setuptools import setup
from pybind11.setup_helpers import Pybind11Extension

opencv_includes = ["/usr/include/opencv4"]
opencv_libdirs = ["/usr/lib/x86_64-linux-gnu"]
opencv_libs = ["opencv_core", "opencv_imgproc", "opencv_imgcodecs", "opencv_highgui"]

setup(
    name="whitebalance",
    ext_modules=[
        Pybind11Extension(
            "whitebalance",
            ["whitebalance.cpp"],
            include_dirs=opencv_includes,
            libraries=opencv_libs,
            library_dirs=opencv_libdirs,
            extra_compile_args=["-O3", "-Wall"],
        ),
    ],
)