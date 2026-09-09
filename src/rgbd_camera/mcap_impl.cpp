// The mcap C++ library is header-only with a single-translation-unit implementation switch. This
// file is that translation unit, kept out of the main glob so the implementation is compiled once
// and with the library's own warnings off.
#define MCAP_IMPLEMENTATION
#include <mcap/reader.hpp>
