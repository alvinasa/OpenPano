//File: imageref.hh
//Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#pragma once
#include <string>
#include <memory>
#include "lib/mat.h"
#include "lib/imgproc.hh"
#include "match_info.hh"

namespace pano {
		// A transparent reference to a image in file
		struct ImageRef {
			ImageRef(const ImageRef& rhs) {
				fname = rhs.fname;
				_width = rhs._width;
				_height = rhs._height;
				img = new Mat32f(rhs.img->clone());
			}

			ImageRef(ImageRef&& rhs) {
				fname = move(rhs.fname);
				img = rhs.img;
				_width = rhs._width;
				_height = rhs._height;
				rhs.img = nullptr;
			}

			ImageRef& operator=(ImageRef&& rhs) {
				fname = move(rhs.fname);
				img = rhs.img;
				_width = rhs._width;
				_height = rhs._height;
				rhs.img = nullptr;
				return *this;
			}

			std::string fname;
			Mat32f* img = nullptr;
			int _width, _height;

			void load() {
				if (img) return;
				img = new Mat32f{read_img(fname.c_str())};
				_width = img->width();
				_height = img->height();
			}

			void release() { if (img) delete img; img = nullptr; }

			int width() const { return _width; }
			int height() const { return _height; }
			Shape2D shape() const { return {_width, _height}; }

			ImageRef(const std::string& fname): fname(fname) {}

			~ImageRef() { if (img) delete img; }
		};

}
