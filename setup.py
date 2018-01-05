from setuptools import setup

setup(name='axidrawcontrol',
    version='0.0.1',
    description='interface to Evil Mad Scientist\'s AxiDraw machine',
    url='https://github.com/snoyer/axidraw-control',
    author='snoyer',
    author_email='reach me on github',
    packages=['axidrawcontrol'],
    zip_safe=False,
    install_requires=[
        'pyserial', 'numpy'
    ],
)
