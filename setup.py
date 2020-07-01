from setuptools import setup, Extension
from Cython.Build import cythonize
from Cython.Distutils import build_ext
import numpy
import os
import glob
import pkgconfig

zense_cflags = pkgconfig.cflags('libpicozense')
zense_libs = pkgconfig.libs('libpicozense')

<<<<<<< HEAD
zense_install_dir = os.environ["PICOZENSE_INSTALL_DIR"]
cvlib_folder = os.path.join(
    zense_install_dir,
    'Thirdparty', 'opencv-3.4.1', 'lib'
)

cvlib_include_folder = os.path.join(
    zense_install_dir,
    'Thirdparty', 'opencv-3.4.1', 'include'
)

=======
# TODO:Rewrite without hardcoding
cvlib_folder = os.path.join('/usr', 'local', 'lib')
>>>>>>> develop
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
                      sources=[
                          "scripts/zense_pywrapper.pyx", "src/pico_zense_wrapper_impl.cpp",
                          "src/common.cpp", "src/parameter_manager.cpp",
<<<<<<< HEAD
                          "src/pico_zense_manager.cpp"
=======
                          "src/pico_zense_manager.cpp", "src/pico_zense_undistorter.cpp"
>>>>>>> develop
                      ],
                      extra_compile_args=[
                          "-std=gnu++11",
                          "-O3",
                          zense_cflags,
                          zense_libs,
                          "-w",
                      ],
                      include_dirs=[
                          numpy.get_include(),
<<<<<<< HEAD
                          cvlib_include_folder,
                          'include',
                      ],
                      library_dirs=lib_dirs,
                      libraries=cvlibs + ["vzense_api", "ImgPreProcess"],
                      language="c++",
                      )
=======
                          os.path.join(sys.prefix, 'include', 'opencv2'),
                          'include'
                      ],
                      library_dirs=lib_dirs,
                      libraries=cvlibs + ["picozense_api"],
                      language="c++",
                      ),
>>>>>>> develop
        ]
    ),
    cmdclass={'build_ext': build_ext},
)
