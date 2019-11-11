import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="osgar",
    version="0.2.0",
    install_requires=[
          'pyserial',
          'msgpack>=0.5.0',
      ],
    extras_require={
        'tools': ['opencv-python>=3,<4', 'Pygame'],
    },
    author="Robotika.cz",
    author_email="osgar@robotika.cz",
    description="Open Source Garden/Generic Autonomous Robot / Python library",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/robotika/osgar",
    python_requires=">=3.6",
    packages=['osgar', 'osgar.drivers', 'osgar.lib', 'osgar.tools', 'subt'],
    package_data={
        '': ['config/*.json'],
    },
    entry_points={
        'console_scripts': [
            'lidarview = osgar.tools.lidarview:main [tools]',
            'log2video = osgar.tools.log2video:main [tools]',
            'log2pcap = osgar.tools.log2pcap:main [tools]',
            'logger = osgar.logger:main',
            'record = osgar.record:main',
            'subt = subt.main:main',
        ],
    },
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
)

# vim: expandtab sw=4 ts=4
