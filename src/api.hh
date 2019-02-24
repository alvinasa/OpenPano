#include <CImg.h>
#include <external.h>
#include <imageref.hh>
#include <lib/mat.h>

OPEN_PANO_API Mat32f Stitch(std::vector<pano::ImageRef> &&imgs);
OPEN_PANO_API Mat32f Stitch(std::vector<Matuc> &&imgs, bool debug = false);