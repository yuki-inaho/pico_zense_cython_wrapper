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


opencv_cflags = pkgconfig.cflags('opencv')
cvlibs_string = pkgconfig.libs('opencv')


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

setup(
    name="zense_pywrapper",
    ext_modules=cythonize(
        [
            Extension("zense_pywrapper",
                      sources=["zense_pywrapper.pyx",
                               "pico_zense_manager.cpp"],
                      extra_compile_args=["-std=gnu++11",
                                          "-O3", zense_cflags, zense_libs],
                      include_dirs=[numpy.get_include()],
                      library_dirs=lib_dirs,
                      libraries=cvlibs + ["picozense_api"],
                      language="c++",
                      ),

            Extension("opencv_mat",
                      sources=["opencv_mat.pyx"],
                      include_dirs=[numpy.get_include(),
                                    os.path.join(
                          sys.prefix, 'include', 'opencv2'),
                      ],
                      library_dirs=lib_dirs,
                      libraries=cvlibs,
                      language="c++"
                      )
        ]
    ),
    cmdclass={'build_ext': build_ext},
)
