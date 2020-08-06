from setuptools import setup, Extension
from Cython.Build import cythonize
from Cython.Distutils import build_ext

import numpy
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

'''
opencv_cflags = pkgconfig.cflags('opencv').split()
cvlibs_string = pkgconfig.libs('opencv')
cvinclude = [str('{}'.format(elem.split('-I')[-1])) for elem in opencv_cflags]

lib_dirs = []
cvlibs = list()
cvlibs_pkgcfg_list = cvlibs_string.split()
for elem in cvlibs_pkgcfg_list:
    # like u'-L/usr/local/lib'
    if elem.startswith("-L"):
        lib_dirs.append(str('{}'.format(elem.split('-L')[-1])))
    # like u'-lopencv_stitching'
    elif elem.startswith("-l"):
        _cvlib = 'opencv_{}'.format(elem.split('-lopencv_')[-1])
        cvlibs.append(_cvlib)
    else:
        pass
'''

setup(
    name="zense_pywrapper",
    ext_modules=cythonize(
        [
            Extension("zense_pywrapper",
                      sources=[
                          "zense_pywrapper.pyx",
                          "src/pico_zense_wrapper_impl.cpp",
                          "src/common.cpp",
                          "src/pico_zense_manager.cpp",
                          "src/parameter_manager.cpp",
                      ],
                      extra_compile_args=[
                          "-std=gnu++14",
                          "-O3",
                          zense_cflags,
                          zense_libs
                      ],
                      include_dirs=[
                          numpy.get_include(),
                          'include',
                          cvlib_include_folder
                      ],
                      library_dirs=lib_dirs,
                      libraries=cvlibs + ["vzense_api", "ImgPreProcess", "stdc++fs"],
                      language="c++",
                      )
        ]
    ),
    cmdclass={'build_ext': build_ext},
)
