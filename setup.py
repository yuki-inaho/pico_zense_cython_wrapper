from distutils.core import setup, Extension
from Cython.Build import cythonize
from Cython.Distutils import build_ext
import numpy
import sys
import os
import glob
import pkgconfig

zense_cflags = pkgconfig.cflags('libpicozense')
zense_libs = pkgconfig.libs('libpicozense')

zense_install_dir = os.environ["PICOZENSE_INSTALL_DIR"]
cvlib_folder = os.path.join(
    zense_install_dir,
    'Thirdparty', 'opencv-3.4.1', 'lib'
)

cvlib_include_folder = os.path.join(
    zense_install_dir,
    'Thirdparty', 'opencv-3.4.1', 'include'
)

lib_dirs = [cvlib_folder]

cvlibs = list()
for file in glob.glob(os.path.join(cvlib_folder, 'libopencv_*.so')):
    cvlibs.append(os.path.basename(file).split('.')[0])
cvlibs = list(set(cvlibs))
cvlibs = ['opencv_{}'.format(
    lib.split(os.path.sep)[-1].split('libopencv_')[-1]) for lib in cvlibs]

setup(
    name="zense_pywrapper",
    ext_modules=cythonize(
        [
            Extension("zense_pywrapper",
                      sources=["zense_pywrapper.pyx",
                               "pico_zense_manager.cpp"],
                      extra_compile_args=["-std=gnu++11",
                                          "-O3", zense_cflags, zense_libs],
                      include_dirs=[numpy.get_include(), cvlib_include_folder],
                      library_dirs=lib_dirs,
                      libraries=cvlibs + ["vzense_api", "ImgPreProcess"],
                      language="c++",
                      ),

            Extension("opencv_mat",
                      sources=["opencv_mat.pyx"],
                      include_dirs=[numpy.get_include(),
                                    cvlib_include_folder,
                                    ],
                      library_dirs=lib_dirs,
                      libraries=cvlibs,
                      language="c++"
                      )
        ]
    ),
    cmdclass={'build_ext': build_ext},
)
