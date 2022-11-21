# reference : https://qiita.com/nabenabe0928/items/6d24360f03fda975d95a]

import setuptools

requirements = [
    "numba",
    'pymap2d>=0.1.15',
    'pytest',
    'fire',
]
# with open("requirements.txt", "r") as f:
#     for line in f:
#         requirements.append(line.strip())

requirements.append(
    "navigation_simulator_2d "
    "@ git+ssh://git@"
    "github.com/kohonda/navigation_simulator_2d"
    "#egg=navigation_simulator_2d"
)

setuptools.setup(
    name="navigation_planner_2d",
    version="0.0.0",
    author="Kohei Honda",
    author_email="honda.kohei.b0@s.mail.nagoya-u.ac.jp",
    packages=setuptools.find_packages(),
    python_requires='>=3.8.10',
    platforms=['Linux'],
    install_requires=requirements,
    include_package_data=True
)