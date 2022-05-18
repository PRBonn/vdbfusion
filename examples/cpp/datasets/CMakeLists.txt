add_library(datasets OBJECT)
target_sources(datasets PRIVATE KITTIOdometry.cpp)
target_compile_features(datasets PRIVATE cxx_std_17)
target_link_libraries(datasets PUBLIC Eigen3::Eigen)
