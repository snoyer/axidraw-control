from setuptools import setup

setup(name='axidrawcontrol',
    version='0.0.3',
    description='interface to Evil Mad Scientist\'s AxiDraw machine',
    url='https://github.com/snoyer/axidraw-control',
    author='snoyer',
    author_email='reach me on github',
    packages=['axidrawcontrol', 'axidrawcontrol.util'],
    zip_safe=False,
    install_requires=[
        'pyserial', 'pyserial-asyncio',
        'svgelements',
        'numpy', 'rdp',
        'pint',
    ],
)
