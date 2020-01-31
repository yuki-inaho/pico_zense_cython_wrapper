# distutils: language = c++
# distutils: sources = pico_zense_manager.cpp

from libc.stdint cimport int32_t
from libcpp.vector cimport vector
from libcpp.string cimport string
from libcpp cimport bool
import toml

import numpy as np
cimport numpy as np
import opencv_mat
from opencv_mat cimport *
import pdb


cdef extern from "opencv2/opencv.hpp":
  cdef int  CV_WINDOW_AUTOSIZE
  cdef int CV_8UC3
  cdef int CV_8UC1
  cdef int CV_32FC1
  cdef int CV_8U
  cdef int CV_32F

cdef extern from "opencv2/opencv.hpp" namespace "cv":
  cdef cppclass Mat:
    Mat() except +
    void create(int, int, int)
    void* data
    int rows
    int cols
    int channels()
    int depth()
    size_t elemSize()

# For Buffer usage
cdef extern from "Python.h":
    ctypedef struct PyObject
    object PyMemoryView_FromBuffer(Py_buffer *view)
    int PyBuffer_FillInfo(Py_buffer *view, PyObject *obj, void *buf, Py_ssize_t len, int readonly, int infoflags)
    enum:
        PyBUF_FULL_RO


cdef extern from "pico_zense_manager.hpp" namespace "zense":
    cdef cppclass PicoZenseManager:
        PicoZenseManager() except +
        string getSerialNumber()
        vector[double] getCameraParameter()
        bool updateDevice()
        Mat getRGBImage()
        Mat getDepthImage()


cdef class PyPicoZenseManager:
    cdef PicoZenseManager *thisptr  
    cdef object rgbImg_npy
    cdef object depthImg_npy

    def __cinit__(self):
        self.thisptr = new PicoZenseManager()

    def __dealloc__(self):
        del self.thisptr

    def updateDevice(self):
        cdef bool status
        cdef Mat rgbImg
        cdef Mat depthImg
        cdef object rgbImg_npy
        cdef object depthImg_npy
        status = self.thisptr.updateDevice()

        if status:
            rgbImg = self.thisptr.getRGBImage()
            depthImg = self.thisptr.getDepthImage()
            self.rgbImg_npy = Mat2np(rgbImg)
            self.depthImg_npy = Mat2np(depthImg)
        return status

    def getCameraParameter(self):
        return self.thisptr.getCameraParameter()

    def getSerialNumber(self):
        return self.thisptr.getSerialNumber()

    def getRGBImage(self):
        return self.rgbImg_npy

    def getDepthImage(self):
        return self.depthImg_npy

