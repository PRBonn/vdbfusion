include(FetchContent)
FetchContent_Declare(
		YAML_CPP
		GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
		GIT_SHALLOW	ON
		GIT_TAG yaml-cpp-0.6.3
	)

set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "disable yaml tests")
FetchContent_MakeAvailable(YAML_CPP)