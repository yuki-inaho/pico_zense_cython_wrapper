from distutils.core import setup, Extension
from Cython.Build import cythonize
from Cython.Distutils import build_ext
import numpy
import sys
import os
import glob
import pdb

ZENSE_LIB_DIR = "/home/{}/Libraries/PicoZenseSDK/Lib/x64".format(os.environ.get('USER'))
ZENSE_INCLUDE_DIR = "/home/{}/Libraries/PicoZenseSDK/Include".format(os.environ.get('USER'))

cvlib_folder = os.path.join(sys.prefix,'local', 'lib')
lib_dirs = [cvlib_folder, ZENSE_LIB_DIR]

cvlibs = list()
for file in glob.glob(os.path.join(cvlib_folder, 'libopencv_*')):
    cvlibs.append(file.split('.')[0])
cvlibs = list(set(cvlibs))
cvlibs = ['opencv_{}'.format(lib.split(os.path.sep)[-1].split('libopencv_')[-1]) for lib in cvlibs]

setup(
    name = "zense_pywrapper",
    ext_modules = cythonize(
                 [
                    Extension("zense_pywrapper",
                        sources=["zense_pywrapper.pyx", "pico_zense_manager.cpp"],
                        extra_compile_args=["-std=gnu++11", "-O3"],
                        include_dirs=[ZENSE_INCLUDE_DIR],
                        library_dirs=lib_dirs,
                        libraries= cvlibs + ["picozense_api"],
                        language="c++",
                    ),

                    Extension("opencv_mat",
                        sources=["opencv_mat.pyx"],
                        include_dirs=[numpy.get_include(),
                                        os.path.join(sys.prefix, 'include', 'opencv2'),
                                        ],
                        library_dirs=lib_dirs,
                        libraries=cvlibs,
                        language="c++"                    
                    )                    
                 ]
    ),
    cmdclass = {'build_ext': build_ext},
)