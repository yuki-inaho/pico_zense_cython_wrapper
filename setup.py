from distutils.core import setup, Extension
from Cython.Build import cythonize
from Cython.Distutils import build_ext
import numpy
import sys
import os
import glob

lib_folder = os.path.join(sys.prefix, 'lib')

cvlibs = list()
for file in glob.glob(os.path.join(lib_folder, 'libopencv_*')):
    cvlibs.append(file.split('.')[0])
cvlibs = list(set(cvlibs))
cvlibs = ['-L{}'.format(lib_folder)] + \
        ['opencv_{}'.format(lib.split(os.path.sep)[-1].split('libopencv_')[-1]) for lib in cvlibs]

setup(
    name = "zense_pywrapper",
    ext_modules = cythonize(
                 [
                    Extension("zense_pywrapper",
                        sources=["zense_pywrapper.pyx", "pico_zense_manager.cpp"],
                        extra_compile_args=["-std=gnu++11", "-O3", 
                                                "-I/home/inaho-00/Libraries/PicoZenseSDK/Include/",
                                                ],
                        extra_link_args=["-L/home/inaho-00/Libraries/PicoZenseSDK/Lib/x64", "-lpicozense_api", 
                                            "-L/usr/local/lib", "-lopencv_shape" ,"-lopencv_highgui", "-lopencv_imgcodecs", "-lopencv_imgproc", "-lopencv_core"],
                        language="c++",
                    ),

                    Extension("opencv_mat",
                        sources=["opencv_mat.pyx"],
                        include_dirs=[numpy.get_include(),
                                        os.path.join(sys.prefix, 'include', 'opencv2'),
                                        ],
                        extra_link_args=["-L/home/inaho-00/Libraries/PicoZenseSDK/Lib/x64", "-lpicozense_api", 
                                            "-L/usr/local/lib", "-lopencv_shape" ,"-lopencv_highgui", "-lopencv_imgcodecs", "-lopencv_imgproc", "-lopencv_core"],
                        library_dirs=[lib_folder],
                        libraries=cvlibs,
                        language="c++"                    
                    )                    
                 ]
    ),
    cmdclass = {'build_ext': build_ext},
)