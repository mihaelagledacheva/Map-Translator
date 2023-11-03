#include <cstdio>
#include <vector>
#include <opencv2/opencv.hpp>
#include <boost/polygon/voronoi.hpp>
using boost::polygon::voronoi_builder;
using boost::polygon::voronoi_diagram;


struct Point {
  int a;
  int b;
  Point(int x, int y) : a(x), b(y) {}
};

struct Segment {
  Point p0;
  Point p1;
  Segment(int x1, int y1, int x2, int y2) : p0(x1, y1), p1(x2, y2) {}
};

namespace boost {
    namespace polygon {
        template <>
        struct geometry_concept<Point> {
            typedef point_concept type;
        };

        template <>
        struct point_traits<Point> {
            typedef int coordinate_type;
            static inline coordinate_type get(
            const Point& point, orientation_2d orient) {
                return (orient == HORIZONTAL) ? point.a : point.b;
            }
        };
    }
}


int main() {
    std::string image_path = "North_America_map.jpg";
    cv::Mat img = cv::imread(image_path, cv::IMREAD_GRAYSCALE);

    if (img.empty()) {
        std::cerr << "Error: Could not read the image." << std::endl;
        return 1;
    }

    std::vector<Point> points;

    cv::RNG rng;
    for (int i = 0; i < 200; ++i) {
        points.emplace_back(rng.uniform(0, img.cols), rng.uniform(0, img.rows));
    }

    voronoi_diagram<double> vd;
    construct_voronoi(points.begin(), points.end(), &vd);

    for (const auto& edge : vd.edges()) {
        if (edge.is_finite()) {
            line(img, cv::Point(edge.vertex0()->x(), edge.vertex0()->y()),
                 cv::Point(edge.vertex1()->x(), edge.vertex1()->y()), cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
        }
    }

    imwrite("voronoi_cells.jpg", img);

    return 0;
}
