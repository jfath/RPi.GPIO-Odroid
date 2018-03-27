"""
Copyright (c) 2012-2016 Ben Croston

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

from distutils.core import setup, Extension

classifiers = ['Development Status :: 4 - Beta',
               'Operating System :: POSIX :: Linux',
               'License :: OSI Approved :: LGPL v3.0 License',
               'Intended Audience :: Developers',
               'Programming Language :: Python :: 2.7',
               'Programming Language :: Python :: 3',
               'Topic :: Software Development',
               'Topic :: Home Automation',
               'Topic :: System :: Hardware']

setup(name             = 'RPi.GPIO',
      version          = '0.6.3.post1',
      author           = 'Ben Croston',
      author_email     = 'ben@croston.org',
      description      = 'A module to control Raspberry Pi and ODROID GPIO channels',
      long_description = open('README.txt').read() + open('CHANGELOG.txt').read(),
      license          = 'LGPL v3.0',
      keywords         = 'Raspberry Pi GPIO',
      url              = 'https://github.com/jfath/RPi.GPIO-Odroid',
      classifiers      = classifiers,
      packages         = ['RPi','RPi.GPIO'],
      ext_modules      = [Extension('RPi._GPIO', ['source/py_gpio.c', 'source/c_gpio.c', 'source/cpuinfo.c', 'source/event_gpio.c', 'source/soft_pwm.c', 'source/py_pwm.c', 'source/common.c', 'source/constants.c', 'source/odroid.c'])])
