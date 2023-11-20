# Map Translator Project
**Roadmap:** <br>
https://docs.google.com/document/d/12jGmhHy2zh67J_WpTOkt1yA4MaFLO7MbK4U43IU0HuQ/edit

**Current state of the project:** <br>
<img src="https://github.com/mihaelagledacheva/Map-Translator/assets/113371744/7858fa67-7fcb-4b94-b54f-988aa72b5dbb" style="display:inline-block; width:30%; margin-right:10px;">
<img src="https://github.com/mihaelagledacheva/Map-Translator/assets/113371744/8dbc94d4-f9cd-43a3-aef1-bf052252b68e" style="display:inline-block; width:30%; margin-right:10px;">
<img src="https://github.com/mihaelagledacheva/Map-Translator/assets/113371744/296011ea-0680-4306-a16d-27b00694ca74" style="display:inline-block; width:30%;">

Implementation of the initial version of the algorithm for adding points: generate a random point within the map, compute expected intensity based on its neighbors, compare to the actual intensity and if the difference is more than x=5 add the new point to the Voronoi diagram; repeat y=5*500 times.

Currently : spatial interpolation where we are estimating the intensity at a particular point based on the intensities of nearby points.
Whereas in the natural neighbours interpolation: the weights assigned to neighboring points are determined by their proximity to the point where we want to interpolate, and the interpolation is a weighted average. To replace our interpolation with NN, we would need to modify the calculate_intensity function 
(and where it is called) either by : 
- using an extern NN implementation
- or modifying calculate_intensity by including weights of neighboring points and using those weights to interpolate the intensity at the desired point

**Next steps:**
* test contours
* include the natural neighbour method 
* test the correctness of the current algorithm and improve its efficiency
* formally describe the methodology for the report
* find optimal parameters (maximum difference in intensity x, number of points added y)

***

**Previous state of the project:** <br>
A single main.cpp file (located in MapTranslator_VSCode) which opens a map, selects n=200 points at random, constructs their voronoi cells and saves the resulting image.

<img src="https://github.com/mihaelagledacheva/Map-Translator/assets/113371744/f82bf146-e886-4018-8a86-edfdc8b63b55" style="display:inline-block; width:40%; margin-right:10px;">
<img src="https://github.com/mihaelagledacheva/Map-Translator/assets/113371744/8368ab89-91b5-42ec-b55f-e63d0b5848db" style="display:inline-block; width:40%;">

| Libraries used  | Documentation                                                               |
| --------------- | --------------------------------------------------------------------------- |
| OpenCV          | https://docs.opencv.org/4.x/                                                |
| Boost           | https://www.boost.org/doc/libs/1_80_0/libs/polygon/doc/voronoi_diagram.htm  |

**Previous goals:**
* test the current set-up on another device
* create separate header/source files for the different parts of the project
* explore the built-in methods offered by boost
* begin implementing the algorithm for adding points
* add annotations to the existing code
* include contours

***

**Previous state of the project:** <br>
A simple interface corresponding to the outlined methodology; no functionalities implemented.

**Previous goals:**
* choose and install an appropriate library for working with images
* declare global variables like pixel map, set of points, etc.
* create the pipeline skeleton (by defining empty functions and establishing dependencies between them)
