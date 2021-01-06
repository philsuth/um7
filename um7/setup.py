from setuptools import setup

setup(name='um7',
      version='0.16+rct0',
      description='Classes to interface with CH Robotics / Redshift Labs UM7 IMU',
      url='https://github.com/philsuth/um7',
      author='Till Busch, Daniel Kurek, Phil Sutherland',
      author_email='phils@rct-global.com',
      license='MIT',
      packages=['um7'],
      install_requires=['pyserial'],
      zip_safe=False)
