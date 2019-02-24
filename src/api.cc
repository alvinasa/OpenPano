// File: main.cc
// Date: Wed Jun 17 20:29:58 2015 +0800
// Author: Yuxin Wu <ppwwyyxxc@gmail.com>

#define _USE_MATH_DEFINES
#include <cmath>

// // Define plugin and include the CImg Library.
// #define cimg_plugin "jpeg_buffer.h"
#define cimg_display 0
#define cimg_use_jpeg
#include <CImg.h>

#include "feature/extrema.hh"
#include "feature/matcher.hh"
#include "feature/orientation.hh"
#include "lib/mat.h"
#include "lib/config.hh"
#include "lib/geometry.hh"
#include "lib/imgproc.hh"
#include "lib/planedrawer.hh"
#include "lib/polygon.hh"
#include "lib/timer.hh"
#include "stitch/cylstitcher.hh"
#include "stitch/match_info.hh"
#include "stitch/stitcher.hh"
#include "stitch/transform_estimate.hh"
#include "stitch/warp.hh"
#include <ctime>
#include <cassert>
#include <api.hh>
#include <external.h>

using namespace std;
using namespace pano;
using namespace config;
using namespace cimg_library;

bool TEMPDEBUG = false;

const int LABEL_LEN = 7;

void test_extrema(const char* fname, int mode) {
	auto mat = read_img(fname);

	ScaleSpace ss(mat, NUM_OCTAVE, NUM_SCALE);
	DOGSpace dog(ss);
	ExtremaDetector ex(dog);

	PlaneDrawer pld(mat);
	if (mode == 0) {
		auto extrema = ex.get_raw_extrema();
		PP(extrema.size());
		for (auto &i : extrema)
			pld.cross(i, LABEL_LEN / 2);
	} else if (mode == 1) {
		auto extrema = ex.get_extrema();
		cout << extrema.size() << endl;
		for (auto &i : extrema) {
			Coor c{(int)(i.real_coor.x * mat.width()), (int)(i.real_coor.y * mat.height())};
			pld.cross(c, LABEL_LEN / 2);
		}
	}
	write_rgb("extrema.jpg", mat);
}

void test_orientation(const char* fname) {
	auto mat = read_img(fname);
	ScaleSpace ss(mat, NUM_OCTAVE, NUM_SCALE);
	DOGSpace dog(ss);
	ExtremaDetector ex(dog);
	auto extrema = ex.get_extrema();
	OrientationAssign ort(dog, ss, extrema);
	auto oriented_keypoint = ort.work();

	PlaneDrawer pld(mat);
	pld.set_rand_color();

	cout << "FeaturePoint size: " << oriented_keypoint.size() << endl;
	for (auto &i : oriented_keypoint)
		pld.arrow(Coor(i.real_coor.x * mat.width(), i.real_coor.y * mat.height()), i.dir, LABEL_LEN);
	write_rgb("orientation.jpg", mat);
}

// draw feature and their match
void test_match(const char* f1, const char* f2) {
	list<Mat32f> imagelist;
	Mat32f pic1 = read_img(f1);
	Mat32f pic2 = read_img(f2);
	imagelist.push_back(pic1);
	imagelist.push_back(pic2);

	unique_ptr<FeatureDetector> detector;
	detector.reset(new SIFTDetector);
	vector<Descriptor> feat1 = detector->detect_feature(pic1),
										 feat2 = detector->detect_feature(pic2);
	print_debug("Feature: %lu, %lu\n", feat1.size(), feat2.size());

	Mat32f concatenated = hconcat(imagelist);
	PlaneDrawer pld(concatenated);

	FeatureMatcher match(feat1, feat2);
	auto ret = match.match();
	print_debug("Match size: %d\n", ret.size());
	for (auto &x : ret.data) {
		pld.set_rand_color();
		Vec2D coor1 = feat1[x.first].coor,
					coor2 = feat2[x.second].coor;
		Coor icoor1 = Coor(coor1.x + pic1.width()/2, coor1.y + pic1.height()/2);
		Coor icoor2 = Coor(coor2.x + pic2.width()/2 + pic1.width(), coor2.y + pic2.height()/2);
		pld.circle(icoor1, LABEL_LEN);
		pld.circle(icoor2, LABEL_LEN);
		pld.line(icoor1, icoor2);
	}
	write_rgb("match.jpg", concatenated);
}

// draw inliers of the estimated homography
void test_inlier(const char* f1, const char* f2) {
	list<Mat32f> imagelist;
	Mat32f pic1 = read_img(f1);
	Mat32f pic2 = read_img(f2);
	imagelist.push_back(pic1);
	imagelist.push_back(pic2);

	unique_ptr<FeatureDetector> detector;
	detector.reset(new SIFTDetector);
	vector<Descriptor> feat1 = detector->detect_feature(pic1),
										 feat2 = detector->detect_feature(pic2);
	vector<Vec2D> kp1; for (auto& d : feat1) kp1.emplace_back(d.coor);
	vector<Vec2D> kp2; for (auto& d : feat2) kp2.emplace_back(d.coor);
	print_debug("Feature: %lu, %lu\n", feat1.size(), feat2.size());

	Mat32f concatenated = hconcat(imagelist);
	PlaneDrawer pld(concatenated);
	FeatureMatcher match(feat1, feat2);
	auto ret = match.match();
	print_debug("Match size: %d\n", ret.size());

	TransformEstimation est(ret, kp1, kp2,
			{pic1.width(), pic1.height()}, {pic2.width(), pic2.height()});
	MatchInfo info;
	est.get_transform(&info);
	print_debug("Inlier size: %lu, conf=%lf\n", info.match.size(), info.confidence);
	if (info.match.size() == 0)
		return;

	for (auto &x : info.match) {
		pld.set_rand_color();
		Vec2D coor1 = x.first,
					coor2 = x.second;
		Coor icoor1 = Coor(coor1.x + pic1.width()/2, coor1.y + pic1.height()/2);
		Coor icoor2 = Coor(coor2.x + pic2.width()/2, coor2.y + pic2.height()/2);
		pld.circle(icoor1, LABEL_LEN);
		pld.circle(icoor2 + Coor(pic1.width(), 0), LABEL_LEN);
		pld.line(icoor1, icoor2 + Coor(pic1.width(), 0));
	}
	pld.set_color(Color(0,0,0));
	Vec2D offset1(pic1.width()/2, pic1.height()/2);
	Vec2D offset2(pic2.width()/2 + pic1.width(), pic2.height()/2);

	// draw convex hull of inliers
	/*
	 *vector<Vec2D> pts1, pts2;
	 *for (auto& x : info.match) {
	 *  pts1.emplace_back(x.first + offset1);
	 *  pts2.emplace_back(x.second + offset2, 0));
	 *}
	 *auto hull = convex_hull(pts1);
	 *pld.polygon(hull);
	 *hull = convex_hull(pts2);
	 *pld.polygon(hull);
	 */

	// draw warped four edges
	Shape2D shape2{pic2.width(), pic2.height()}, shape1{pic1.width(), pic1.height()};

	// draw overlapping region
	Matrix homo(3,3);
	REP(i, 9) homo.ptr()[i] = info.homo[i];
	Homography inv = info.homo.inverse();
	auto p = overlap_region(shape1, shape2, homo, inv);
	PA(p);
	for (auto& v: p) v += offset1;
	pld.polygon(p);

	Matrix invM(3, 3);
	REP(i, 9) invM.ptr()[i] = inv[i];
	p = overlap_region(shape2, shape1, invM, info.homo);
	PA(p);
	for (auto& v: p) v += offset2;
	pld.polygon(p);

	write_rgb("inlier.jpg", concatenated);
}

void test_warp(int argc, char* argv[]) {
	CylinderWarper warp(1);
	REPL(i, 2, argc) {
		Mat32f mat = read_img(argv[i]);
		warp.warp(mat);
		write_rgb(("warp" + to_string(i) + ".jpg").c_str(), mat);
	}
}


void work(int argc, char* argv[]) {
/*
 *  vector<Mat32f> imgs(argc - 1);
 *  {
 *    GuardedTimer tm("Read images");
 *#pragma omp parallel for schedule(dynamic)
 *    REPL(i, 1, argc)
 *      imgs[i-1] = read_img(argv[i]);
 *  }
 */
	vector<string> imgs;
	REPL(i, 1, argc) imgs.emplace_back(argv[i]);
	Mat32f res;
	if (CYLINDER) {
		CylinderStitcher p(move(imgs));
		res = p.build();
	} else {
		Stitcher p(move(imgs));
		res = p.build();
	}

	if (CROP) {
		int oldw = res.width(), oldh = res.height();
		res = crop(res);
		print_debug("Crop from %dx%d to %dx%d\n", oldw, oldh, res.width(), res.height());
	}
	{
		GuardedTimer tm("Writing image");
		write_rgb("out.jpg", res);
	}
}

void work2(int argc, char* argv[]) {
/*
 *  vector<Mat32f> imgs(argc - 1);
 *  {
 *    GuardedTimer tm("Read images");
 *#pragma omp parallel for schedule(dynamic)
 *    REPL(i, 1, argc)
 *      imgs[i-1] = read_img(argv[i]);
 *  }
 */
	vector<string> paths;
	REPL(i, 1, argc) paths.emplace_back(argv[i]);

	vector<ImageRef> imgs;
	for(const auto& path : paths) {
		ImageRef img(path);
		img.load();
		imgs.emplace_back(move(img));
	}

	Mat32f res;
	if (CYLINDER) {
		CylinderStitcher p(move(imgs));
		res = p.build();
	} else {
		Stitcher p(move(imgs));
		res = p.build();
	}

	if (CROP) {
		int oldw = res.width(), oldh = res.height();
		res = crop(res);
		print_debug("Crop from %dx%d to %dx%d\n", oldw, oldh, res.width(), res.height());
	}
	{
		GuardedTimer tm("Writing image");
		write_rgb("out.jpg", res);
	}
}

void init_config() {
	static bool inited = false;
	if(inited) return;

	inited = true;

	CYLINDER = 0;
	ESTIMATE_CAMERA = 1;
	TRANS = 0;
	ORDERED_INPUT = 0;
	CROP = 1;
	MAX_OUTPUT_SIZE = 8000;
	LAZY_READ = 1;
	FOCAL_LENGTH = 37;
	SIFT_WORKING_SIZE = 800;
	NUM_OCTAVE = 3;
	NUM_SCALE = 7;
	SCALE_FACTOR = 1.4142135623;
	GAUSS_SIGMA = 1.4142135623;
	GAUSS_WINDOW_FACTOR = 4;
	CONTRAST_THRES = 3e-2;
	JUDGE_EXTREMA_DIFF_THRES = 2e-3;
	EDGE_RATIO = 10;
	PRE_COLOR_THRES = 5e-2;
	CALC_OFFSET_DEPTH = 4;
	OFFSET_THRES = 0.5;
	ORI_RADIUS = 4.5;
	ORI_HIST_SMOOTH_COUNT = 2;
	DESC_HIST_SCALE_FACTOR = 3;
	DESC_INT_FACTOR = 512;
	MATCH_REJECT_NEXT_RATIO = 0.8;
	RANSAC_ITERATIONS = 1500;
	RANSAC_INLIER_THRES = 3.5;
	INLIER_IN_MATCH_RATIO = 0.1;
	INLIER_IN_POINTS_RATIO = 0.04;
	STRAIGHTEN = 1;
	SLOPE_PLAIN = 8e-3;
	LM_LAMBDA = 5;
	MULTIPASS_BA = 1;
	MULTIBAND = 0;
}

OPEN_PANO_API Mat32f Stitch(std::vector<pano::ImageRef>&& imgs) {
	init_config();

	Mat32f res;
	if (CYLINDER) {
		CylinderStitcher p(move(imgs));
		res = p.build();
	} else {
		Stitcher p(move(imgs));
		res = p.build();
	}

	if (CROP) {
		int oldw = res.width(), oldh = res.height();
		res = crop(res);
		// print_debug("Crop from %dx%d to %dx%d\n", oldw, oldh, res.width(), res.height());
	}
	{
		// GuardedTimer tm("Writing image");
		write_rgb("out.jpg", res);
	}

	return res;
}

OPEN_PANO_API Mat32f Stitch(std::vector<Matuc>&& imgs, bool debug) {
	init_config();

	Mat32f res;
	if (CYLINDER) {
		CylinderStitcher p(move(imgs));
		res = p.build();
	} else {
		Stitcher p(move(imgs), debug);
		res = p.build();
	}

	if (CROP) {
		int oldw = res.width(), oldh = res.height();
		res = crop(res);
		// print_debug("Crop from %dx%d to %dx%d\n", oldw, oldh, res.width(), res.height());
	}
	// {
	// 	// GuardedTimer tm("Writing image");
	// 	write_rgb("out.jpg", res);
	// }

	return res;
}

void planet(const char* fname) {
	Mat32f test = read_img(fname);
	int w = test.width(), h = test.height();
	const int OUTSIZE = 1000, center = OUTSIZE / 2;
	Mat32f ret(OUTSIZE, OUTSIZE, 3);
	fill(ret, Color::NO);

	REP(i, OUTSIZE) REP(j, OUTSIZE) {
		real_t dist = hypot(center - i, center - j);
		if (dist >= center || dist == 0) continue;
		dist = dist / center;
		//dist = sqr(dist);	// TODO you can change this to see different effect
		dist = h - dist * h;

		real_t theta;
		if (j == center) {
			if (i < center)
				theta = M_PI / 2;
			else
				theta = 3 * M_PI / 2;
		} else {
			theta = atan((real_t)(center - i) / (center - j));
			if (theta < 0) theta += M_PI;
			if ((theta == 0) && (j > center)) theta += M_PI;
			if (center < i) theta += M_PI;
		}
		m_assert(0 <= theta);
		m_assert(2 * M_PI + EPS >= theta);

		theta = theta / (M_PI * 2) * w;

		update_min(dist, (real_t)h - 1);
		Color c = interpolate(test, dist, theta);
		float* p = ret.ptr(i, j);
		c.write_to(p);
	}
	write_rgb("planet.jpg", ret);
}