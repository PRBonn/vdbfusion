# VDBFusion: Flexible and Efficient TSDF Integration

[![C++](https://img.shields.io/badge/c++-%2300599C.svg?style=flat-square&logo=c%2B%2B&logoColor=white)](./src/vdbfusion/vdbfusion)
[![Python](https://img.shields.io/badge/python-3670A0?style=flat-square&logo=python&logoColor=ffdd54)](src/vdbfusion/pybind)
[![Linux](https://svgshare.com/i/Zhy.svg?style=flat-square)](https://svgshare.com/i/Zhy.svg)
[![PyPI version shields.io](https://img.shields.io/pypi/v/vdbfusion.svg?style=flat-square)](https://pypi.python.org/pypi/vdbfusion/)
[![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg?style=flat-square)](https://github.com/PRBonn/vdbfusion/pulls)
[![Paper](https://img.shields.io/badge/paper-get-<COLOR>.svg?style=flat-square)](https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/vizzo2022sensors.pdf)
[![MIT license](https://img.shields.io/badge/License-MIT-blue.svg?style=flat-square)](https://lbesson.mit-license.org/)
[![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg?style=flat-square)](https://colab.research.google.com/github/PRBonn/vdbfusion/blob/main/examples/notebooks/kitti_odometry.ipynb)

![example](docs/vdbfusion.gif)

This is a small utility library that implements the VDBFusion algorithm, similar to TSDF-based
reconstruction pipelines but using a different data-structure (VDB).

## Installation

Take a seat and relax, you only need to:

```shell
pip install vdbfusion
```

If you plan to use our C++ API then you should build this project from source. More details in the [Installation instructions](INSTALL.md).

The ROS-1 C++ wrapper for this library is available at https://github.com/PRBonn/vdbfusion_ros

## Usage

The code shown below is not intended to be copy pasted but rather be a spiritual guide for developers. If you really want to give this library a try you should consider checking the standalone [Python](examples/python), [Notebooks](examples/notebooks), and [C++](examples/cpp) examples.

### Data loading

NOTE: This step is not **mandatory**. Our API only expects `points` and `poses` but this is the easiest way to deal with 3D data.

<table>
<tr>
<td> <b> Python  </td> <td> <b> C++ </td>
</tr>
<tr>
<td>

```python
class Dataset:
    def __init__(self, *args, **kwargs):
        # Initialize your dataset here ..

    def __len__(self) -> int:
        return len(self.n_scans)

    def __getitem__(self, idx: int):
        # Returns a PointCloud(np.array(N, 3))
        # and sensor origin(Eigen::Vector3d)
        # in the global coordinate frame.
        return points, origin
```

</td>
<td>

```c++
class Dataset {
  // Initialize your dataset here ..
  Dataset(...);

  // Return length of the dataset
  std::size_t size() const { return n_scans_; }

  // Returns a Cloud(std::vector<Eigen::Vector3d>)
  // and the sensor origin(Eigen::Vector3d) in the
  // global coordinate frame.
  std::tuple<Cloud, Point> operator[](int idx) const;
};
```

</td>
</tr>
</table>

### TSDF Fusion pipeline

<table>
<tr>
<td> <b> Python  </td> <td> <b> C++ </td>
</tr>
<tr>
<td>

```python
import vdbfusion

vdb_volume = vdbfusion.VDBVolume(voxel_size,
                                 sdf_trunc,
                                 space_carving
dataset = Dataset(...)

for scan, origin in dataset:
    vdb_volume.integrate(scan, origin)
```

</td>
<td>

```cpp
#include "vdbfusion/VDBVolume.h"

vdb_fusion::VDBVolume vdb_volume(voxel_size,
                                 sdf_trunc,
                                 space_carving);
const auto dataset = Dataset(...);

for (const auto& [scan, origin] : iterable(dataset)) {
  vdb_volume.Integrate(scan, origin);
}
```

</td>
</tr>
</table>

### Visualization

For visualization you can use any 3D library you like. For this example we are going to be using `Open3D`. If you are using the Python API make sure to `pip install open3d` before trying this snippet.

<table>
<tr>
<td> <b> Python  </td> <td> <b> C++ </td>
</tr>
<tr>
<td>

```python
import open3d as o3d

# Extract triangle mesh (numpy arrays)
vert, tri = vdb_volume.extract_triangle_mesh()

# Visualize the results
mesh = o3d.geometry.TriangleMesh(
    o3d.utility.Vector3dVector(vert),
    o3d.utility.Vector3iVector(tri),
)

mesh.compute_vertex_normals()
o3d.visualization.draw_geometries([mesh])
```

</td>
<td>

```cpp
#include <open3d/Open3D.h>

// Extract triangle mesh (Eigen).
auto [verts, tris] = vdb_volume.ExtractTriangleMesh();

// Visualize the results
auto mesh = o3d::geometry::TriangleMesh(
    verts,
    tris,
)

mesh.ComputeVertexNormals()
o3d::visualization::DrawGeometries({&mesh})
```

</td>
</tr>
</table>

## LICENSE

The [LICENSE](./LICENSE.txt) can be found at the root of this repository. It only applies to the code of `VDBFusion` but not to its [3rdparty dependencies](3rdparty/). Please make sure to check the licenses in there before using any form of this code.

## Credits

I would like to thank the [Open3D](https://github.com/isl-org/Open3D) and [OpenVDB](https://github.com/AcademySoftwareFoundation/openvdb) authors and contributors for making their implementations open source which inspired, helped and guided the implementation of the VDBFusion library.

## Citation

If you use this library for any academic work, please cite the original [paper](https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/vizzo2022sensors.pdf).

```bibtex
@article{vizzo2022sensors,
  author         = {Vizzo, Ignacio and Guadagnino, Tiziano and Behley, Jens and Stachniss, Cyrill},
  title          = {VDBFusion: Flexible and Efficient TSDF Integration of Range Sensor Data},
  journal        = {Sensors},
  volume         = {22},
  year           = {2022},
  number         = {3},
  article-number = {1296},
  url            = {https://www.mdpi.com/1424-8220/22/3/1296},
  issn           = {1424-8220},
  doi            = {10.3390/s22031296}
}
```
