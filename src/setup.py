# setup.py
import setuptools
import os

with open("requirements.txt", "r") as fh:
    requirements = fh.readlines()

setup_py_dir = os.path.dirname(os.path.realpath(__file__))

need_files = []
datadir = "triped_sim"

hh = setup_py_dir + "/" + datadir

for root, dirs, files in os.walk(hh):
    for fn in files:
        ext = os.path.splitext(fn)[1][1:]
        if ext and ext in 'png gif jpg urdf sdf obj mtl dae off stl STL xml '.split():
            fn = root + "/" + fn
            need_files.append(fn[1+len(hh):])

setuptools.setup(
    name='triped_gym',
    version='1.0',
    author='Jan Baumgärtner',
    license='MIT',
    description='A gym environment for the TriPed robot',
    install_requires=[req for req in requirements if req[:2] != "# "],
    packages=setuptools.find_packages(),
    package_data={'triped_sim': need_files}
)
