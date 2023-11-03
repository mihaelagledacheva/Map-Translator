# Map Translator Project
**Roadmap:** <br>
https://docs.google.com/document/d/12jGmhHy2zh67J_WpTOkt1yA4MaFLO7MbK4U43IU0HuQ/edit

**Current state of the project:** <br>
A single main.cpp file (located in MapTranslator_VSCode) which opens a map, selects n=200 points at random, constructs their voronoi cells and saves the resulting image.

<img src="https://github.com/mihaelagledacheva/Map-Translator/assets/113371744/f82bf146-e886-4018-8a86-edfdc8b63b55" style="display:inline-block; width:40%; margin-right:10px;">
<img src="https://github.com/mihaelagledacheva/Map-Translator/assets/113371744/8368ab89-91b5-42ec-b55f-e63d0b5848db" style="display:inline-block; width:40%;">

| Libraries used  | Documentation                                                               |
| --------------- | --------------------------------------------------------------------------- |
| OpenCV          | https://docs.opencv.org/4.x/                                                |
| Boost           | https://www.boost.org/doc/libs/1_80_0/libs/polygon/doc/voronoi_diagram.htm  |

**Next steps:**
* test the current set-up on another device
* create separate header/source files for the different parts of the project
* explore the built-in methods offered by boost
* begin implementing the algorithm for adding points 

***

**Previous state of the project:** <br>
A simple interface corresponding to the outlined methodology; no functionalities implemented.

**Previous goals:**
* choose and install an appropriate library for working with images
* declare global variables like pixel map, set of points, etc.
* create the pipeline skeleton (by defining empty functions and establishing dependencies between them)
