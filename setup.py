import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="osgar",
    version="0.0.1",
    install_requires=[
          'pyserial',
          'msgpack>=0.5.0',
      ],
    author="Robotika.cz",
    author_email="osgar@robotika.cz",
    description="Open Source Garden/Generic Autonomous Robot / Python library",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/robotika/osgar",
#    packages=setuptools.find_packages(),
    packages=['osgar', 'osgar.drivers', 'osgar.lib'],  # TODO use line above after cleanup
    package_data={
        '': ['config/*.json'],
    },
    classifiers=(
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ),
)

# vim: expandtab sw=4 ts=4
