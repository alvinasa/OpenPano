#include <external.h>
#include <imageref.hh>
#include <CImg.h>
#include <lib/mat.h>

OPEN_PANO_API Mat32f Stitch(std::vector<pano::ImageRef>&& imgs);