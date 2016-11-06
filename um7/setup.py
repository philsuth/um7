from setuptools import setup

setup(name='um7',
      version='0.12',
      description='Classes to interface with CH Robotics / Redshift Labs UM7 IMU',
      url='https://github.com/buxit/um7',
      author='Till Busch, Daniel Kurek',
      author_email='till@bux.at',
      license='MIT',
      packages=['um7'],
      install_requires=['pyserial'],
      zip_safe=False)
