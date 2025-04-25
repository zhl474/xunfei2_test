#!/usr/bin/env python3
import ctypes
from ctypes import (
    POINTER, Structure, byref, cast, c_void_p, c_int, c_uint, c_char_p
)
from PIL import Image

try:
    # Linux/macOS
    zbar = ctypes.CDLL("libzbar.so.0")
except OSError as e:
    try:
        # Windows
        zbar = ctypes.CDLL("libzbar-64.dll")
    except OSError as e2:
        print(f"Failed to load ZBar library: {e} (Linux) and {e2} (Windows)")
        import sys
        sys.exit(1)

# 定义 ZBar 结构体和常量
class zbar_image_s(Structure):
    pass

class zbar_symbol_s(Structure):
    pass

class zbar_symbol_set_s(Structure):
    pass

ZBarSymbol = ctypes.c_int
ZBar_QRCODE = 64  # ZBar 中 QR Code 的标识符

# 函数原型声明
zbar.zbar_image_create.restype = POINTER(zbar_image_s)
zbar.zbar_image_destroy.argtypes = [POINTER(zbar_image_s)]
zbar.zbar_image_set_size.argtypes = [POINTER(zbar_image_s), c_uint, c_uint]
zbar.zbar_image_set_data.argtypes = [POINTER(zbar_image_s), c_void_p, c_uint, c_void_p]
zbar.zbar_scan_image.argtypes = [POINTER(zbar_image_s), c_int]
zbar.zbar_image_first_symbol.restype = POINTER(zbar_symbol_s)
zbar.zbar_symbol_get_data.restype = c_char_p
zbar.zbar_symbol_get_type.restype = ZBarSymbol
zbar.zbar_symbol_get_loc_size.restype = c_uint

# 初始化 ZBar 扫描器
zbar.zbar_processor_create.restype = c_void_p
zbar.zbar_processor_init.argtypes = [c_void_p, c_int, c_int]
zbar.zbar_processor_set_visible.argtypes = [c_void_p]
zbar.zbar_processor_destroy.argtypes = [c_void_p]

def decode_barcode(image_path):
    try:
        # 初始化扫描器
        proc = zbar.zbar_processor_create()
        if not proc:
            print("Failed to create zbar processor.")
            return []
        zbar.zbar_processor_init(proc, 0, 1)
        zbar.zbar_processor_set_visible(proc)

        # 打开图像并转换为灰度图
        img = Image.open(image_path).convert("L")
        width, height = img.size
        raw_data = img.tobytes()
        print(f"Image width: {width}, height: {height}, data length: {len(raw_data)}")

        # 创建 ZBar 图像对象
        zimg = zbar.zbar_image_create()
        if not zimg:
            print("Failed to create zbar image object.")
            zbar.zbar_processor_destroy(proc)
            return []

        # 设置图像大小
        zbar.zbar_image_set_size(zimg, c_uint(width), c_uint(height))

        # 设置图像数据
        data_ptr = cast(raw_data, c_void_p)
        data_len = c_uint(len(raw_data))
        zbar.zbar_image_set_data(zimg, data_ptr, data_len, None)

        # 扫描图像
        result = zbar.zbar_scan_image(zimg, ZBar_QRCODE)
        print(f"zbar_scan_image result: {result}")
        if result < 0:
            zbar.zbar_image_destroy(zimg)
            zbar.zbar_processor_destroy(proc)
            return []

        # 提取解码结果
        symbols = []
        symbol = zbar.zbar_image_first_symbol(zimg)
        while symbol:
            data = zbar.zbar_symbol_get_data(symbol)
            sym_type = zbar.zbar_symbol_get_type(symbol)
            symbols.append((data.decode("utf-8"), sym_type))
            symbol = zbar.zbar_symbol_next(symbol)

        # 释放资源
        zbar.zbar_image_destroy(zimg)
        zbar.zbar_processor_destroy(proc)
        return symbols
    except Exception as e:
        print(f"An error occurred in decode_barcode: {e}")
        return []

# 示例使用
try:
    results = decode_barcode("1.jpg")
    for data, sym_type in results:
        print(f"Type: {sym_type}, Data: {data}")
except Exception as e:
    print(f"An error occurred: {e}")

