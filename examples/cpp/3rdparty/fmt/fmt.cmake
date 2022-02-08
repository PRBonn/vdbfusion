include(FetchContent)
FetchContent_Declare(
  external_fmt
  PREFIX fmt
  URL https://github.com/fmtlib/fmt/archive/refs/tags/8.0.1.tar.gz
  URL_HASH SHA256=b06ca3130158c625848f3fb7418f235155a4d389b2abc3a6245fb01cb0eb1e01)

FetchContent_MakeAvailable(external_fmt)
