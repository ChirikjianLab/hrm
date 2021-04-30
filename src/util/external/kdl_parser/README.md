# urdf-to-kdl
This is a pure C++ version of URDF parser with KDL output which is based on [ROS stack](http://wiki.ros.org/kdl_parser) and code by *Wim Meeussen*. The code is modified according to [urdf-to-kdl](https://github.com/Pouya-moh/urdf-to-kdl) by *Pouya Mohammadi*.

### How to use
```cmake
add_subdirectory(urdf-to-kdl) # or however you call it
include_directories(${KDL_Parser_INCLUDE_DIRS})
target_link_libraries(KDL_Parser)
```

### To test
Run cmake by passing `-DBUILD_TEST=ON`. It will create an executable which accepts a urdf as command line argument.
