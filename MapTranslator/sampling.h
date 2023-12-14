#ifndef SAMPLING_H
#define SAMPLING_H

#include <vector>
#include <opencv2/opencv.hpp>
#include <boost/geometry.hpp>
#include <boost/polygon/voronoi.hpp>

typedef boost::polygon::voronoi_vertex<double> VoronoiVertex;
typedef boost::polygon::voronoi_edge<double> VoronoiEdge;
typedef boost::polygon::voronoi_cell<double> VoronoiCell;
typedef boost::polygon::voronoi_diagram<double> VoronoiDiagram;
typedef boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double> > Polygon;

// Definition of Point and Segment types
struct Point {
    int x;
    int y;
    Point(int a, int b) : x(a), y(b) {}
};

struct Segment {
    Point p0;
    Point p1;
    Segment(int a1, int b1, int a2, int b2) : p0(a1, b1), p1(a2, b2) {}
};

// Specialize the defined types within the boost::polygon namespace 
namespace boost {
    namespace polygon {
        template <>
        struct geometry_concept<Point> {
            typedef point_concept type;                                                     // associate Point with the geometric concept point_concept
        };

        template <>
        struct point_traits<Point> {
            typedef int coordinate_type;                                                    // indicate that coordinates are of type int
            static inline coordinate_type get(const Point& point, orientation_2d orient) {  // retrieve coordinates based on the orientation_2d :
                return (orient == HORIZONTAL) ? point.x : point.y;                          // if orient == HORIZONTAL then return x , else (VERTICAL) return y
            }
        };

        template <>
        struct geometry_concept<Segment> {                                                  // associate Segment with the geometric concept segment_concept
            typedef segment_concept type;
        };

        template <>
        struct segment_traits<Segment> {
            typedef int coordinate_type;                                               
            typedef Point point_type;                                                      // indicate that endpoints of a Segment are of type Point
            static inline point_type get(const Segment& segment, direction_1d dir) {       // retrieve one of the endpoints of the segment based on the direction_1d :
                return dir.to_int() ? segment.p1 : segment.p0;                             // if dir == 1 then return endpoint p0 else (-1) return p1
            };
        };
    }
}

class sampling {
public:
    void custom_sample(const cv::Mat *input_image, const std::vector<cv::Point> *contour, std::vector<Point> *points, std::vector<Segment> *segments);
    void random_sample(const cv::Mat *input_image, const std::vector<cv::Point> *contour, std::vector<Point> *points, std::vector<Segment> *segments);
    void uniform_sample(const cv::Mat *input_image, const std::vector<cv::Point> *contour, std::vector<Point> *points, std::vector<Segment> *segments);
    void grid_sample(const cv::Mat *input_image, const std::vector<cv::Point> *contour, std::vector<Point> *points, std::vector<Segment> *segments);
    void get_vertices(const VoronoiCell *cell, std::vector<Point> *vertices);

private:
    void get_neighbors(const VoronoiCell *cell, const std::vector<Point> *points, std::vector<Point> *neighbors);
    std::string polygonString(const std::vector<Point> vertices);
    double find_intersection_area(const std::vector<Point> vertices1, const std::vector<Point> vertices2);
    double find_full_area(const std::vector<Point> vertices);
    uchar calculate_intensity(const VoronoiCell *newCell, const std::vector<Point> *newPoints, const std::vector<Point> *points, const VoronoiDiagram *vd, const cv::Mat *img);
    Point generate_point(const std::vector<Segment> *segments);
    void add_point(std::vector<Point> *points, const std::vector<Segment> *segments, const VoronoiDiagram *vd, const cv::Mat *img);
};

#endif // SAMPLING_H
