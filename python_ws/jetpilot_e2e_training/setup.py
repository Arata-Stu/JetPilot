from setuptools import find_packages, setup


setup(
    name="jetpilot_e2e_training",
    version="0.1.0",
    description="Image-based end-to-end driving training tools for JetPilot.",
    author="JetPilot Team",
    package_dir={"": "src"},
    packages=find_packages("src"),
    include_package_data=True,
    package_data={
        "e2e_learning": ["conf/**/*.yaml", "conf/*.yaml", "conf/*.json"],
    },
    python_requires=">=3.10",
    install_requires=[
        "torch",
        "torchvision",
        "numpy",
        "opencv-python",
        "tqdm",
        "rosbags",
        "omegaconf",
        "hydra-core",
        "PyYAML",
        "tensorboard",
        "onnx",
        "onnxscript",
        "onnxruntime",
        "matplotlib",
    ],
    entry_points={
        "console_scripts": [
            "jetpilot-e2e-preprocess=e2e_learning.cli.preprocess_bag:main",
            "jetpilot-e2e-train=e2e_learning.cli.train:main",
            "jetpilot-e2e-export-onnx=e2e_learning.cli.export_onnx:main",
            "jetpilot-e2e-validate-onnx=e2e_learning.cli.validate_onnx:main",
            "jetpilot-e2e-compare-runs=e2e_learning.cli.compare_runs:main",
        ],
    },
)
