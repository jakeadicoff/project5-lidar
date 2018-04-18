AProject 5: LIDAR
----------------

This code applies hill shading and identifies ground points for LIDAR
data.

To run this project, make the project using make. The project
processes plaintext LIDAR data, which can be generated using the
las2txt tool from the LAStools package.

The parameters of the program are:
$ ./lidarview <file>.txt <density> <building slope threshold>

The density parameter governs the grid size, such that each grid will
have x lidar points per grid cell on average, where x is the density
parameter.

The building slope threshold parameter is used by the ground finding
algorithm to dertemine what the slope needs to be to be considered a
building. Smaller thresholds will classify gentler slopes as building,
while larger thresholds will require steeper slopes to count as a
building. This value can be changed during runtime with + and -.

Controls
--------
's': Swaps between HILL SHADE view and GROUND POINTS view.
'+': Increases the building slope threshold. Stricter requirements for
being classified as a building.
'-': Decreases the building slope threshold. Easier requirements for
being classified as a building.
