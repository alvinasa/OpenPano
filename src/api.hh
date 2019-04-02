#include <CImg.h>
#include <external.h>
#include <imageref.hh>
#include <lib/mat.h>
#include <feature/feature.hh>

OPEN_PANO_API Mat32f Stitch(std::vector<pano::ImageRef> &&imgs);
OPEN_PANO_API Mat32f Stitch(std::vector<Matuc> &&imgs, bool debug = false);
OPEN_PANO_API bool Stitch(Mat32f& stitched,
                          std::vector<pano::ImageRef> &&imgs,
                          std::vector<std::vector<pano::Descriptor>>& descriptor,
                          bool debug = false);
OPEN_PANO_API void init_config();