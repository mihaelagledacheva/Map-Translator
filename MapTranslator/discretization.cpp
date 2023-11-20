#include "discretization.h"
#include <iostream>

//Convert a contour to a list of segments that constitute it
void discretization::contourToSegments(                                 // arguments :
    std::vector<cv::Point> *contour,                                    // a vector of Points representing the contour (ptr)
    std::vector<Segment> *segments)                                     // a vector of Segments for the result (ptr)
{
    size_t n = contour->size();
    for (size_t i = 0; i < n - 1; i++) {
        segments->emplace_back(Segment((*contour)[i].x, (*contour)[i].y, (*contour)[i + 1].x, (*contour)[i + 1].y));
    }
    segments->emplace_back(Segment((*contour)[n - 1].x, (*contour)[n - 1].y, (*contour)[0].x, (*contour)[0].y));
}

// Extract the vertices of a Voronoi cell
void discretization::get_vertices(                                      // arguments :
    VoronoiCell *cell,                                                  // a Voronoi cell (ptr)
    std::vector<Point> *vertices)                                       // a vector of Points for the result (ptr)
{
    const VoronoiEdge *edge = cell->incident_edge();                    // choose one edge as a reference
    do {                                                                // iterate over the edges ... :
        edge = edge->next();                                            // go to the next edge
        const VoronoiVertex *vertex = edge->vertex0();                  // take the first vertex of the current edge
        vertices->emplace_back(Point(vertex->x(), vertex->y()));        // add the vertex to the output vector
    } while (edge != cell->incident_edge());                            // ... until we reach the reference edge                                                                     // return the vector
}

// Extract the neighbour points of a Voronoi cell
void discretization::get_neighbors(                                     // arguments :
    VoronoiCell *cell,                                                  // a Voronoi cell (ptr)
    std::vector<Point> *points,                                         // a set of points (ptr)
    std::vector<Point> *neighbors)                                      // an vector of Points for the result (ptr)
{                                                                
    const VoronoiEdge *edge = cell->incident_edge();                    // iterate over the edges as in 'get_vertices'
    do {
        edge = edge->next();
        if (edge->twin()->cell()->contains_point()) {                   // check if the twin cell of the current edge contains a point
            size_t neighborIndex = 
                edge->twin()->cell()->source_index();                   // if so, retrieve the source index of the twin cell
            neighbors->emplace_back((*points)[neighborIndex]);          // use it to access the corresponding point of the set and add it to the neighbours
        }
    } while (edge != cell->incident_edge());                            // end of iteration
}

// Create a string representing a Polygon with its vertices
std::string discretization::polygonString(                              // arguments : a set of vertices of a polygon
    std::vector<Point> vertices)
{                              
    std::ostringstream string;                                          // initialize a string (can be constructed using the insertion operator)
    string << "POLYGON((";                                              // begin with the header POLYGON
    for (const auto &vertex : vertices) {                               // iterate through each vertex
        string << vertex.x << " " << vertex.y << ",";                   // add its coordinates to the string
    }
    string << vertices.front().x << " " << vertices.front().y;          // add in the string the coordinates of the first vertex (not taken into account in the loop)
    string << "))";                                                     // close the string
    return string.str();                                                // return a string of the form "POLYGON ((0 0, 1 0, 1 1, 0 1))"
}

// Compute the area of the intersection of two sets of vertices representing polygons
double discretization::find_intersection_area(                          // arguments : two sets of vertices
    std::vector<Point> vertices1,
    std::vector<Point> vertices2)
{                                                   
    std::string string1 = polygonString(vertices1);                     // create the string polygon (in well-known text) for each set of vertices
    std::string string2 = polygonString(vertices2);
    Polygon polygon1, polygon2;                                         // initialize two polygons
    boost::geometry::read_wkt(string1, polygon1);                       // convert the string polygons into geometric polygons
    boost::geometry::read_wkt(string2, polygon2);
    std::deque<Polygon> resultPolygon;                                  // initialize a deque of polygons...
    boost::geometry::intersection(
        polygon1, polygon2, 
        resultPolygon);                                                 // ...to the intersection
    BOOST_FOREACH (Polygon const &p, resultPolygon) {
        return boost::geometry::area(p);                                // return the area of the intersection
    }
    return 0;                                                           // fallback: if no intersection is found, return 0
}

// Compute the area of a polygon formed by a set of vertices
double discretization::find_full_area(                                  // arguments : a set of vertices
    std::vector<Point> vertices)
{                      
    double area = 0.0;                                                  // initialize area to 0.0
    size_t n = vertices.size();
    for (size_t i = 0; i < n; i++) {                                    // iterate over the vertices
        if (i != n - 1) {
            area += (
                vertices[i].x * vertices[i + 1].y - 
                vertices[i + 1].x * vertices[i].y);                     // calculate area for all the vertices except the last one
        } else {
            area += (
                vertices[i].x * vertices[0].y - 
                vertices[0].x * vertices[i].y);                         // calculate area for the last vertex
        }
    }
    return std::abs(area) / 2;                                          // use the shoelace formula to return the final area
}

// Calculate the intensity of a Voronoi cell expected based on the intensity of its neighbours
uchar discretization::calculate_intensity(
    VoronoiCell *newCell, 
    std::vector<Point> *newPoints, 
    std::vector<Point> *points, 
    VoronoiDiagram *vd, 
    cv::Mat *img)
{
    uchar intensity = 0;                                                // initialize intensity to 0
    std::vector<Point> vertices;
    get_vertices(newCell, &vertices);                                   // retrieve vertices of the Voronoi cell using get_vertices
    double area = find_full_area(vertices);                             // retrieve the area of the V cell using find_full_area on the obtained vertices
    std::vector<Point> neighbors;
    get_neighbors(newCell, newPoints, &neighbors);                      // retrieve the neighbours of the cell among the candidates newPoints
    for (const auto &neighbor : neighbors) {                            // iterate over each neighbours:
        uchar neighborIntensity = 
            img->at<uchar>(neighbor.y, neighbor.x);                     // retrieve the intensity value from the image matrix at the corresponding pixel coordinates
        VoronoiCell neighborCell = (*newCell);                          // find the Voronoi cell corresponding to the current neighbor :
        for (const auto &cell : vd->cells()) {                          // iterate through all cells in the Voronoi diagram
            if ((*points)[cell.source_index()].x == neighbor.x && 
                (*points)[cell.source_index()].y == neighbor.y) {       // if a cell matches the coordinates of the neighbor point:
                VoronoiCell neighborCell = cell;                        // set the neighborCell variable to that cell.
                break;
            }
        }
        std::vector<Point> neighborVertices;
        get_vertices(&neighborCell, &neighborVertices);                 // retrieve the vertices of the neighboring cell
        double neighborIntersection = 
            find_intersection_area(vertices, neighborVertices);         // calculate the intersection area between the current cell and its neighbor using find_intersection_area
        intensity += (neighborIntensity * neighborIntersection / area); // update the overall intensity based on the neighbor's intensity, the intersection area, and the current cell's area
    }
    return intensity;                                                   // return the accumulated intensity of the Voronoi cell
}

// Generate a random point within a contour
Point discretization::generate_point(
    std::vector<Segment> *segments) 
{
    std::vector<cv::Point> contour;
    for (const auto& segment : (*segments)) {
        contour.emplace_back(cv::Point(segment.p0.x, segment.p0.y));
    }
    cv::RotatedRect boundingBox = cv::minAreaRect(contour);
    while (true) {
        float randomX = boundingBox.center.x + (rand() % static_cast<int>(boundingBox.size.width)) - boundingBox.size.width / 2;
        float randomY = boundingBox.center.y + (rand() % static_cast<int>(boundingBox.size.height)) - boundingBox.size.height / 2;
        if (cv::pointPolygonTest(contour, cv::Point2f(randomX, randomY), false) >= 0) {
            return Point(static_cast<int>(randomX), static_cast<int>(randomY));
        }
    }
}

// Add a random point to a set of points, update the Voronoi diagram, and check if the intensity of the new point matches the expected intensity based on its neighbors
void discretization::add_point(                                         // arguments : 
    std::vector<Point> *points,                                         // the current point sample
    std::vector<Segment> *segments,                                     // the set of segments representing a contour
    VoronoiDiagram *vd,                                                 // the Voronoi diagram
    cv::Mat *img)                                                       // an image (an openCV matrix)
{                                   
    Point newPoint = generate_point(segments);                          // generate a random point
    std::vector<Point> newPoints(*points);                              // create a copy of the existing points
    newPoints.push_back(newPoint);                                      // add the newly generated point to the copy
    
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
    if (std::abs(realIntensity - expectedIntensity) > 5) {              // compare the absolute difference between the real and expected intensity values
        points->push_back(newPoint);                                    // if the difference is significant, add the new point to the original set of points
    }
}

void discretization::discretize(
    cv::Mat *input_image, 
    cv::Mat *output_image, 
    std::vector<cv::Point> *contour)
{
    std::vector<Point> points;                                          // initialize an empty set of points (for point sampling)
    std::vector<Segment> segments;                                      // initialize segments to bound the image
    contourToSegments(contour, &segments);
    VoronoiDiagram vd;

    boost::polygon::construct_voronoi(
        points.begin(), points.end(), 
        segments.begin(), segments.end(), &vd);                         // construct the initial Voronoi diagram using the empty set of points and the bounding box segments
    for (int i = 0; i < 500; i++) {                                     // iterate 500 times:
        add_point(&points, &segments, &vd, input_image);                // add a point to the current set of points on the current vd of the current image
        vd.clear();                                                     // clear the current vd
        construct_voronoi(
            points.begin(), points.end(), 
            segments.begin(), segments.end(), &vd);                     // reconstruct it with the updated set of points
    }
    for (const auto &edge : vd.edges()) {                               // by iterating over the finite edges of the vd
        if (edge.is_finite()) {
            line((*output_image), 
                cv::Point(edge.vertex0()->x(), edge.vertex0()->y()),    // and drawing the corresponding line on the copy
                cv::Point(edge.vertex1()->x(), edge.vertex1()->y()), 
                cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
            }
        }
}
