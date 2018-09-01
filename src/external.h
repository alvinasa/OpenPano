#pragma once

#ifdef BUILD_OPEN_PANO
#   define OPEN_PANO_API __attribute__ ((visibility ("default")))
#else
#   define OPEN_PANO_API
#endif