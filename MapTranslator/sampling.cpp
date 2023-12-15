#include "sampling.h"
#include <iostream>
#include <fstream>
#include <random>

const int INTENSITY_DIFF = 25;
int SAMPLE_SIZE = 1000;

// Extract the vertices of a Voronoi cell
void sampling::get_vertices(                                                // arguments :
    const VoronoiCell *cell,                                                // a Voronoi cell (ptr)
    std::vector<Point> *vertices)                                           // a vector of Points for the result (ptr)
{
    if (cell && vertices && !cell->is_degenerate()) {
        vertices->clear();
        const VoronoiEdge *edge = cell->incident_edge();                    // choose one edge as a reference
        do {                                                                // iterate over the edges ... :
            edge = edge->next();                                            // go to the next edge
            if (edge->is_finite()) {
                const VoronoiVertex *vertex = edge->vertex0();              // take the first vertex of the current edge
                vertices->emplace_back(Point(vertex->x(), vertex->y()));    // add the vertex to the output vector
            }
        } while (edge != cell->incident_edge());                            // ... until we reach the reference edge 
    }
}

// Extract the neighbour points of a Voronoi cell
void sampling::get_neighbors(                                               // arguments :
    const VoronoiCell *cell,                                                // a Voronoi cell (ptr)
    const std::vector<Point> *points,                                       // a set of points (ptr)
    std::vector<Point> *neighbors)                                          // an vector of Points for the result (ptr)
{
    if (cell && points && neighbors && !cell->is_degenerate()) {
        const VoronoiEdge *edge = cell->incident_edge();                    // iterate over the edges as in 'get_vertices'
        neighbors->clear();
        do {
            edge = edge->next();
            if (edge->twin()->cell()->contains_point()) {                   // check if the twin cell of the current edge contains a point
                size_t neighborIndex = 
                    edge->twin()->cell()->source_index();                   // if so, retrieve the source index of the twin cell
                if (neighborIndex < points->size()) {
                    neighbors->emplace_back((*points)[neighborIndex]);      // use it to access the corresponding point of the set and add it to the neighbours
                }
            }
        } while (edge != cell->incident_edge());                            // end of iteration
    }
}

// Create a string representing a Polygon with its vertices
std::string sampling::polygonString(                                        // arguments : a set of vertices of a polygon
    const std::vector<Point> vertices)
{
    if (!vertices.empty()) {
        std::ostringstream string;                                          // initialize a string (can be constructed using the insertion operator)
        string << "POLYGON((";                                              // begin with the header POLYGON
        for (const auto &vertex : vertices) {                               // iterate through each vertex
            string << vertex.x << " " << vertex.y << ",";                   // add its coordinates to the string
        }
        string << vertices.front().x << " " << vertices.front().y;          // add in the string the coordinates of the first vertex (not taken into account in the loop)
        string << "))";                                                     // close the string
        return string.str();                                                // return a string of the form "POLYGON ((0 0, 1 0, 1 1, 0 1))"
    }
    return "POLYGON()";
}

// Compute the area of the intersection of two sets of vertices representing polygons
double sampling::find_intersection_area(                                    // arguments : two sets of vertices
    const std::vector<Point> vertices1,
    const std::vector<Point> vertices2)
{
    if (!vertices1.empty() && !vertices2.empty()) {
        std::string string1 = polygonString(vertices1);                     // create the string polygon (in well-known text) for each set of vertices
        std::string string2 = polygonString(vertices2);
        Polygon polygon1, polygon2;                                         // initialize two polygons
        try {
            boost::geometry::read_wkt(string1, polygon1);                   // convert the string polygons into geometric polygons
            boost::geometry::read_wkt(string2, polygon2);
        } catch (const boost::geometry::read_wkt_exception &e) {
            return 0.0;
        }
        std::deque<Polygon> resultPolygon;                                  // initialize a deque of polygons...
        boost::geometry::intersection(
            polygon1, polygon2, 
            resultPolygon);                                                 // ...to the intersection
        if (resultPolygon.empty()) {
            return 0.0;
        }
        for (const auto &p : resultPolygon) {
            return boost::geometry::area(p);                                // return the area of the intersection
        }
    }
    return 0;                                                               // fallback: if no intersection is found, return 0
}

// Compute the area of a polygon formed by a set of vertices
double sampling::find_full_area(                                            // arguments : a set of vertices
    const std::vector<Point> vertices)
{
    if (vertices.empty()) {
        return 0.0;
    } else {
        double area = 0.0;                                                  // initialize area to 0.0
        size_t n = vertices.size();
        for (size_t i = 0; i < n; i++) {                                    // iterate over the vertices
            size_t j = (i + 1) % n;
            area += (
                vertices[i].x * vertices[j].y - 
                vertices[j].x * vertices[i].y);                             // calculate area for all the vertices
        }
        return std::abs(area) / 2;                                          // use the shoelace formula to return the final area
    }
}

// Calculate the intensity of a Voronoi cell expected based on the intensity of its neighbours
uchar sampling::calculate_intensity(
    const VoronoiCell *newCell, 
    const std::vector<Point> *newPoints, 
    const std::vector<Point> *points, 
    const VoronoiDiagram *vd, 
    const cv::Mat *img)
{
    uchar intensity = 0;                                                    // initialize intensity to 0
    std::vector<Point> vertices;
    get_vertices(newCell, &vertices);                                       // retrieve vertices of the Voronoi cell using get_vertices
    if (vertices.empty()) {
        return 0;
    }
    double area = find_full_area(vertices);                                 // retrieve the area of the V cell using find_full_area on the obtained vertices
    std::vector<Point> neighbors;
    get_neighbors(newCell, points, &neighbors);                             // retrieve the neighbours of the cell among the candidates newPoints
    if (neighbors.empty()) {
        return 0;
    }
    for (const auto &neighbor : neighbors) {                                // iterate over each neighbours:
        uchar neighborIntensity = 0;
        if (neighbor.x >= 0 && neighbor.x < img->cols && 
            neighbor.y >= 0 && neighbor.y < img->rows) {
            neighborIntensity = img->at<uchar>(neighbor.y, neighbor.x);     // retrieve the intensity value from the image matrix at the corresponding pixel coordinates
        }
        VoronoiCell neighborCell = (*newCell);                              // find the Voronoi cell corresponding to the current neighbor :
        for (const auto &cell : vd->cells()) {                              // iterate through all cells in the Voronoi diagram
            if ((*points)[cell.source_index()].x == neighbor.x && 
                (*points)[cell.source_index()].y == neighbor.y) {           // if a cell matches the coordinates of the neighbor point:
                neighborCell = cell;                                        // set the neighborCell variable to that cell.
                break;
            }
        }
        std::vector<Point> neighborVertices;
        get_vertices(&neighborCell, &neighborVertices);                     // retrieve the vertices of the neighboring cell
        if (neighborVertices.empty()) {
            continue;
        }
        double neighborIntersection = 
            find_intersection_area(vertices, neighborVertices);             // calculate the intersection area between the current cell and its neighbor using find_intersection_area
        if (area != 0) {
            intensity += (neighborIntensity * neighborIntersection / area); // update the overall intensity based on the neighbor's intensity, the intersection area, and the current cell's area
        }
        
    }
    return intensity;                                                       // return the accumulated intensity of the Voronoi cell
}

// Generate a random point within given boundaries
Point sampling::generate_point(
    const std::vector<Segment> *segments) 
{
    if (segments->size() == 4) {
        cv::Point bottomLeft((*segments)[0].p1.x, (*segments)[0].p1.y);
        cv::Point topRight((*segments)[2].p1.x, (*segments)[2].p1.y);
        cv::Rect rect(bottomLeft, topRight);
        int randomX = rect.x + rand() % rect.width;
        int randomY = rect.y + rand() % rect.height;
        return Point(randomX, randomY);
    }
    return Point(0, 0);
}

// Add a random point to a set of points, update the Voronoi diagram, and check if the intensity of the new point matches the expected intensity based on its neighbors
void sampling::add_point(                                                   // arguments : 
    std::vector<Point> *points,                                             // the current point sample
    const std::vector<Segment> *segments,                                   // the set of segments representing a contour
    const VoronoiDiagram *vd,                                               // the Voronoi diagram
    const cv::Mat *img)                                                     // an image (an openCV matrix)
{
    Point newPoint = generate_point(segments);                              // generate a random point
    if (img->at<uchar>(newPoint.y, newPoint.x) > 20) {
        std::vector<Point> newPoints(*points);                              // create a copy of the existing points
        newPoints.emplace_back(newPoint);                                   // add the newly generated point to the copy
        VoronoiDiagram new_vd;                                              // initialize a new Voronoi diagram
        boost::polygon::construct_voronoi(
            newPoints.begin(), newPoints.end(), 
            segments->begin(), segments->end(), &new_vd);                   // construct the new diagram using the new set of points and the bounding segments
        VoronoiCell newCell = new_vd.cells().back();                        // initialize a new Voronoi cell in order to :
        for (const auto &cell : new_vd.cells()) {                           // search over the Voronoi cells of the new diagram
            if (newPoints[cell.source_index()].x == newPoint.x && 
                newPoints[cell.source_index()].y == newPoint.y) {           // look for the cell corresponding to the new point
                newCell = cell;                                             // and retrieve it
                break;
            }
        }
        uchar realIntensity = img->at<uchar>(newPoint.y, newPoint.x);       // retrieve the real intensity value of the newly added point from the image
        uchar expectedIntensity = 
            calculate_intensity(&newCell, &newPoints, points, vd, img);     // calculate the expected intensity using calculate_intensity  __ why not use the new_vd instead of vd??
        if (std::abs(realIntensity - expectedIntensity) > INTENSITY_DIFF) { // compare the absolute difference between the real and expected intensity values
            points->emplace_back(newPoint);                                 // if the difference is significant, add the new point to the original set of points
        }
    }
}

void sampling::custom_sample(
    const cv::Mat *input_image, 
    const std::vector<cv::Point> *contour,
    std::vector<Point> *points,
    std::vector<Segment> *segments)
{
    points->clear();
    segments->clear();

    cv::Rect rect = cv::boundingRect((*contour));
    segments->emplace_back(rect.x, rect.y, rect.x + rect.width, rect.y);
    segments->emplace_back(rect.x + rect.width, rect.y, rect.x + rect.width, rect.y + rect.height);
    segments->emplace_back(rect.x + rect.width, rect.y + rect.height, rect.x, rect.y + rect.height);
    segments->emplace_back(rect.x, rect.y + rect.height, rect.x, rect.y);
    VoronoiDiagram vd;
    boost::polygon::construct_voronoi(
        points->begin(), points->end(), 
        segments->begin(), segments->end(), &vd);                           // construct the initial Voronoi diagram using the empty set of points and the bounding box segments
    int iterations = rect.width * rect.height / 1000;
    for (int i = 0; i < iterations; i++) {                                  // iterate n times:
        add_point(points, segments, &vd, input_image);                      // add a point to the current set of points on the current vd of the current image
        vd.clear();                                                         // clear the current vd
        construct_voronoi(
            points->begin(), points->end(), 
            segments->begin(), segments->end(), &vd);                       // reconstruct it with the updated set of points
    }
}

void sampling::random_sample(
    const cv::Mat *input_image, 
    const std::vector<cv::Point> *contour, 
    std::vector<Point> *points,
    std::vector<Segment> *segments)
{
    points->clear();
    segments->clear();

    cv::Rect rect = cv::boundingRect((*contour));
    segments->emplace_back(rect.x, rect.y, rect.x + rect.width, rect.y);
    segments->emplace_back(rect.x + rect.width, rect.y, rect.x + rect.width, rect.y + rect.height);
    segments->emplace_back(rect.x + rect.width, rect.y + rect.height, rect.x, rect.y + rect.height);
    segments->emplace_back(rect.x, rect.y + rect.height, rect.x, rect.y);

    std::random_device rd;
    std::mt19937 gen(rd());
    
    for (int i = 0; i < SAMPLE_SIZE; ++i) {
        Point p(rect.x + gen() % rect.width, rect.y + gen() % rect.height);
        points->emplace_back(p);
    }
}

void sampling::uniform_sample(
    const cv::Mat *input_image, 
    const std::vector<cv::Point> *contour, 
    std::vector<Point> *points,
    std::vector<Segment> *segments)
{
    points->clear();
    segments->clear();

    cv::Rect rect = cv::boundingRect((*contour));
    segments->emplace_back(rect.x, rect.y, rect.x + rect.width, rect.y);
    segments->emplace_back(rect.x + rect.width, rect.y, rect.x + rect.width, rect.y + rect.height);
    segments->emplace_back(rect.x + rect.width, rect.y + rect.height, rect.x, rect.y + rect.height);
    segments->emplace_back(rect.x, rect.y + rect.height, rect.x, rect.y);

    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dist_x(rect.x, rect.x + rect.width);
    std::uniform_int_distribution<int> dist_y(rect.y, rect.y + rect.height);

    for (int i = 0; i < SAMPLE_SIZE; ++i) {
        Point p(dist_x(gen), p.y = dist_y(gen));
        points->emplace_back(p);
    }
}

void sampling::grid_sample(
    const cv::Mat *input_image, 
    const std::vector<cv::Point> *contour, 
    std::vector<Point> *points,
    std::vector<Segment> *segments)
{
    points->clear();
    segments->clear();

    cv::Rect rect = cv::boundingRect((*contour));
    segments->emplace_back(rect.x, rect.y, rect.x + rect.width, rect.y);
    segments->emplace_back(rect.x + rect.width, rect.y, rect.x + rect.width, rect.y + rect.height);
    segments->emplace_back(rect.x + rect.width, rect.y + rect.height, rect.x, rect.y + rect.height);
    segments->emplace_back(rect.x, rect.y + rect.height, rect.x, rect.y);

    int grid_spacing_x = std::max(1, rect.width / static_cast<int>(std::sqrt(SAMPLE_SIZE)));
    int grid_spacing_y = std::max(1, rect.height / static_cast<int>(std::sqrt(SAMPLE_SIZE)));

    for (int y = rect.y + 1; y < rect.y + rect.height; y += grid_spacing_y) {
        for (int x = rect.x + 1; x < rect.x + rect.width; x += grid_spacing_x) {
            points->emplace_back(Point(x, y));
        }
    }
}
