from setuptools import Extension, find_packages, setup
from setuptools.command.build_ext import build_ext
import warnings


package_root = 'gym'

warnings.filterwarnings(
    'ignore',
    message='In the future `np.bytes` will be defined.*',
    category=FutureWarning)


class BuildExt(build_ext):
    def finalize_options(self):
        super().finalize_options()
        import numpy
        for extension in self.extensions:
            extension.include_dirs.append(numpy.get_include())


setup(name='f110_gym',
      version='0.2.1',
      author='Hongrui Zheng',
      author_email='billyzheng.bz@gmail.com',
      url='https://f1tenth.org',
      packages=find_packages(where=str(package_root)),
      package_dir={'': str(package_root)},
      package_data={
          'f110_gym.envs': ['maps/*.pgm', 'maps/*.png', 'maps/*.yaml'],
      },
      ext_modules=[
          Extension(
              'f110_gym._cpp_backend',
              ['gym/f110_gym/cpp_backend.cpp'],
              language='c++',
              extra_compile_args=['-std=c++17', '-O3'],
          ),
      ],
      cmdclass={'build_ext': BuildExt},
      install_requires=['gymnasium>=0.29',
                        'numpy>=1.22,<3.0.0',
                        'Pillow>=9.0.1',
                        'numba>=0.60.0',
                        'pyyaml>=5.3.1',
                        'pyglet<1.5',
                        'pyopengl']
      )
