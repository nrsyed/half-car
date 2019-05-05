from setuptools import setup

setup(
    name="halfcar",
    version="2.0.0",
    description="Half-car dynamic model simulation",
    url="https://github.com/nrsyed/half-car",
    author="Najam R Syed",
    author_email="najam.r.syed@gmail.com",
    license="GPL",
    packages=["halfcar"],
    install_requires=[
        "numpy",
        "scipy",
        "matplotlib"
    ]
)
