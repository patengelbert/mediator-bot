import io
from setuptools import setup, find_packages
import mediator_bot


def read(*filenames, **kwargs):
    encoding = kwargs.get('encoding', 'utf-8')
    sep = kwargs.get('sep', '\n')
    buf = []
    for filename in filenames:
        with io.open(filename, encoding=encoding) as f:
            buf.append(f.read())
    return sep.join(buf)


long_description = read('README.md')

setup(
    name='Mediator-Bot',
    version=mediator_bot.__version__,
    url='http://github.com/patengelbert/mediator-bot/',
    license='Apache Software License',
    author='Patrick Engelbert',
    tests_require=['pytest',
                   'pytest-cov',
                   'tox'],
    author_email='patrick.engelbert13@imperial.ac.uk',
    description='Mediator Robot for meeting rooms',
    long_description=long_description,
    packages=find_packages(),
    include_package_data=False,
    platforms='any',
    test_suite='mediator_bot.test.test_mediator_bot',
    classifiers=[
        'Development Status :: 2 - Pre-Alpha',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: MIT License',
        'Natural Language :: English',
        'Operating System :: Unix',
        'Programming Language :: Python :: 2.7',
        'Operating System :: OS Independent',
    ],
)
