#include <deque>
#include <iostream>
#include <sstream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <boost/foreach.hpp>
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

// Specialization of the defined types within the boost::polygon namespace 
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

// Extraction of the vertices of a Voronoi cell
std::vector<Point> get_vertices(VoronoiCell* cell) {                                     // arguments :  a Voronoi cell (ptr)
    std::vector<Point> vertices;                                                         // initialize an empty vector of Points
    const VoronoiEdge* edge = cell->incident_edge();                                     // choose one edge as a reference
    do {                                                                                 // iterate over the edges ... : 
        edge = edge->next();                                                             // go to the next edge
        const VoronoiVertex* vertex = edge->vertex0();                                   // take the first vertex of the current edge
        vertices.push_back(Point(vertex->x(), vertex->y()));                             // add the vertex to the vector
    } while (edge != cell->incident_edge());                                             // ... until we reach the reference edge 
    return vertices;                                                                     // return the vector 
}

// Extraction of the neighbour points of a Voronoi cell
std::vector<Point> get_neighbors(VoronoiCell* cell, std::vector<Point>* points) {       // arguments: a Voronoi cell and a set of points (ptr)
    std::vector<Point> neighbors;                                                       // initialize an empty vector of points
    const VoronoiEdge* edge = cell->incident_edge();                                    // iterate over the edges as in 'get_vertices'
    do {
        edge = edge->next();
        if (edge->twin()->cell()->contains_point()) {                                   // check if the twin cell of the current edge contains a point
            size_t neighborIndex = edge->twin()->cell()->source_index();                // if so, retrieve the source index of the twin cell
            neighbors.push_back((*points)[neighborIndex]);                              // use it to access the corresponding point of the set and add it to the neighbours
        }
    } while (edge != cell->incident_edge());                                            // end of iteration
    return neighbors;                                                                   // return the vector of neighbours
}

// Creation of a string representing a Polygon with its vertices
std::string polygonString(std::vector<Point> vertices) {                                // arguments : a set of vertices of a polygon
    std::ostringstream string;                                                          // initialize a string (can be constructed using the insertion operator)
    string << "POLYGON((";                                                              // begin with the header POLYGON
    for (const auto& vertex : vertices) {                                               // iterate through each vertex 
        string << vertex.x << " " << vertex.y << ",";                                   // add its coordinates to the string
    }
    string << vertices.front().x << " " << vertices.front().y;                          // add in the string the coordinates of the first vertex (not taken into account in the loop)
    string << "))";                                                                     // close the string
    return string.str();                                                                // returns a string of the form "POLYGON ((0 0, 1 0, 1 1, 0 1))"
}

// Computation of the area of the intersection of two sets of vertices representing polygons
double find_intersection_area(std::vector<Point> vertices1, std::vector<Point> vertices2) { //arguments: two sets of vertices
    std::string string1 = polygonString(vertices1);                                         //create the string polygon (in well-known text) for each set of vertices
    std::string string2 = polygonString(vertices2);
    Polygon polygon1, polygon2;                                                             // initialize two polygons
    boost::geometry::read_wkt(string1, polygon1);                                           // convert the string polygons into geometric polygons
    boost::geometry::read_wkt(string2, polygon2);
    std::deque<Polygon> resultPolygon;                                                      // initialize a deque of polygons...
    boost::geometry::intersection(polygon1, polygon2, resultPolygon);                       // ...to the intersection 
    BOOST_FOREACH(Polygon const& p, resultPolygon) {
        return boost::geometry::area(p);                                                    // return the area of the intersection
    }
    return 0;                                                                               // fallback: if no intersection is found, return 0
}

// Computation of the area of a polygon formed by a set of vertices
double find_full_area(std::vector<Point> vertices) {                                         // arguments : a set of vertices
    double area = 0.0;                                                                       // initialize area to 0.0 
    size_t n = vertices.size();
    for (size_t i = 0; i < n; i++) {                                                         // iterate over the vertices
        if (i != n-1) {
            area += (vertices[i].x * vertices[i+1].y - vertices[i+1].x * vertices[i].y);     // calculate area for all the vertices except the last one
        } else {
            area += (vertices[i].x * vertices[0].y - vertices[0].x * vertices[i].y);         // calculate area for the last vertex
        }
    }
    return std::abs(area)/2;                                                                 // use the shoelace formula to return the final area
}

uchar calculate_intensity(VoronoiCell* newCell, std::vector<Point>* newPoints, std::vector<Point>* points,VoronoiDiagram* vd, cv::Mat* img) {
    uchar intensity = 0;
    std::vector<Point> vertices = get_vertices(newCell);
    double area = find_full_area(vertices);
    std::vector<Point> neighbors = get_neighbors(newCell, newPoints);
    for (const auto& neighbor : neighbors) {
        uchar neighborIntensity = img->at<uchar>(neighbor.y, neighbor.x);
        VoronoiCell neighborCell = (*newCell);
        for (const auto& cell : vd->cells()) {
            if((*points)[cell.source_index()].x == neighbor.x && (*points)[cell.source_index()].y == neighbor.y) {
                VoronoiCell neighborCell = cell;
            }
        }
        std::vector<Point> neighborVertices = get_vertices(&neighborCell);
        double neighborIntersection = find_intersection_area(vertices, neighborVertices);
        intensity += (neighborIntensity * neighborIntersection / area);
    }
    return intensity;
}

void add_point(std::vector<Point>* points, VoronoiDiagram* vd, cv::Mat* img) {
    Point newPoint(1+rand()%(img->cols-2), 1+rand()%(img->rows-2));
    std::vector<Point> newPoints(*points); 
    newPoints.push_back(newPoint);
    std::vector<Segment> segments;
    segments.push_back(Segment(0, 0, img->cols, 0));
    segments.push_back(Segment(img->cols, 0, img->cols, img->rows));
    segments.push_back(Segment(img->cols, img->rows, 0, img->rows));
    segments.push_back(Segment(0, img->rows, 0, 0));
    VoronoiDiagram new_vd;
    boost::polygon::construct_voronoi(newPoints.begin(), newPoints.end(), segments.begin(), segments.end(), &new_vd);
    VoronoiCell newCell = new_vd.cells().back();
    for (const auto& cell : new_vd.cells()) {
        if(newPoints[cell.source_index()].x == newPoint.x && newPoints[cell.source_index()].y == newPoint.y) {
            newCell = cell;
        }
    }
    uchar realIntensity = img->at<uchar>(newPoint.y, newPoint.x);
    uchar expectedIntensity = calculate_intensity(&newCell, &newPoints, points, vd, img);
    if (std::abs(realIntensity - expectedIntensity) > 5) {
        points->push_back(newPoint);
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
    std::vector<Segment> segments;
    segments.push_back(Segment(0, 0, img.cols, 0));
    segments.push_back(Segment(img.cols, 0, img.cols, img.rows));
    segments.push_back(Segment(img.cols, img.rows, 0, img.rows));
    segments.push_back(Segment(0, img.rows, 0, 0));
    VoronoiDiagram vd;
    construct_voronoi(points.begin(), points.end(), segments.begin(), segments.end(), &vd);
    for (int i = 0; i < 5; ++i) {
        for (int i = 0; i < 500; ++i) {
            add_point(&points, &vd, &img);
            vd.clear();
            construct_voronoi(points.begin(), points.end(), segments.begin(), segments.end(), &vd);
        }
        cv::Mat copy = img.clone();
        for (const auto& edge : vd.edges()) {
            if (edge.is_finite()) {
                line(copy, cv::Point(edge.vertex0()->x(), edge.vertex0()->y()),
                    cv::Point(edge.vertex1()->x(), edge.vertex1()->y()), cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
            }
        }
        std::string name = "voronoi_cells_" + std::to_string(i+1) + ".jpg";
        imwrite(name, copy);
    }
    return 0;
}
