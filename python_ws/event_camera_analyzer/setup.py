from setuptools import find_packages, setup

setup(
    name="event_camera_analyzer",
    version="0.1.0",
    description="ROSBag analyzer and root cause diagnosis tool for event camera flickering",
    author="JetPilot Team",
    packages=find_packages(),
    python_requires=">=3.8",
    install_requires=[
        "rosbags>=0.9.0",
        "numpy>=1.20.0",
        "pandas>=1.3.0",
        "plotly>=5.0.0",
        "opencv-python>=4.5.0",
    ],
    entry_points={
        "console_scripts": [
            "openeb-bag-analyzer=event_camera_analyzer.cli:main",
        ],
    },
    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: OS Independent",
    ],
)
