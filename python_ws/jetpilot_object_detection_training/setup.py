from setuptools import find_packages, setup


setup(
    name="jetpilot_object_detection_training",
    version="0.1.0",
    description="YOLOv8 object-detection training and deployment tools for JetPilot.",
    author="JetPilot Team",
    package_dir={"": "src"},
    packages=find_packages("src"),
    include_package_data=True,
    package_data={"object_detection_learning": ["conf/*.yaml"]},
    python_requires=">=3.10",
    install_requires=[
        "onnx",
        "onnxslim",
        "PyYAML>=6.0",
        "ultralytics>=8.0,<9",
    ],
    entry_points={
        "console_scripts": [
            "jetpilot-yolo-validate-dataset=object_detection_learning.cli.validate_dataset:main",
            "jetpilot-yolo-train=object_detection_learning.cli.train:main",
            "jetpilot-yolo-export-onnx=object_detection_learning.cli.export_onnx:main",
        ],
    },
)
