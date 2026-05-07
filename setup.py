from setuptools import setup, find_packages

setup(
    name="franka_rt",
    version="0.1.0",
    description="Python real-time control bindings for Franka FR3 via libfranka",
    packages=find_packages(),
    package_data={"franka_rt": ["*.so", "*.pyd"]},
    python_requires=">=3.8",
    install_requires=["numpy", "pybind11"],
)
