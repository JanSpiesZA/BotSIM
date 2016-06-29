# BotSIM

BotSIM is a simple robot simulator used to show basic robotic concepts to my students at the Tshwane University of Technology for an intro in robotics class.
It uses the following concepts to arrive at the goal loaction:
- The map is divided into nodes using the Quad-Tree method.
- The A* path finding algorithm is now used to calculate the best path to the goal.
- When getting too close to an obstacle it uses the simulated sensors to avoid the obstacle

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for your own personal use.

### Prerequisities

The files are written in Processing.
- Processing 3.1 or later (www.processing org)

### Installing

No installation is necessary.
- Processing is run from the directory in which it is unzipped
- Open BotSIM.pde and run using Processing

### Usage

- LEFT mouse click: Move the goal circle to a new position
- The 'o' key: Place/remove obstacles at the current mouse position

## Import your own base map

You can use your own base-map to start your simulation with. Any *.jpg or *.png file with an 800x800 resolution should work.
- Use any graphic editor and create a black on white image and save it into the same directory as BotSIM.
- Find the line of code 'img = loadImage("YOURMAP.jpg");'
- Replace 'YOURMAP.jpg' with the name of your file.

Your base map should now load and the initial Quad Tree divisions is done.

## Acknowledgments

* Thanks PurpleBooth for the Readme.md template: https://gist.github.com/PurpleBooth/109311bb0361f32d87a2

