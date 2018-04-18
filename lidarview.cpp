/* lidarview file.txt

   Reads a lidar point cloud in txt form and renders the points in
   3D. Has options to filter by first and last return, and number of
   returns; has options to filter by classification codes (ground,
   building, vegetation and other).

   The lidar file is obtained from a .las or .laz file with
   LAStools:las2txt;

   last2txt -o file.las -o file.txt -parse xyznrc

   NOTE: -parse xyznrc in this order

   keypress:

   l/r/u/d/f/bx/X,y/Y,z/Z: translate and rotate
   w: toggle wire/filled polygons
   v,g,h,o: toggle veg, ground, buildings,other on/off
   c: cycle through colormaps (one color, based on code, based on your code)
   t: cycle through filter  options: first-return, lsat return, many-returns, all-returns

   OpenGL 1.x
   Laura Toma
*/

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <iostream>
#include <queue>

//this allows this code to compile both on apple and linux platforms
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#include <vector>
#include <math.h>

using namespace std;

typedef struct _lidarPoint {
  float x,y,z;
  int nb_of_returns; //how many returns this pulse has
  int return_number; //the number of this return
  int code;  //classification code read from file

  int mycode; //classification code assigned by us
} lidarPoint;

class Point{
public:
  float x, y, z;
  Point(float i, float j, float k): x(i), y(j), z(k) {}
};

//the array of lidar points; note: this needs to be global because it
//needs to be rendered
vector<lidarPoint>  points;

//bounding box of the points, updated when the terrain is loaded from file
float minx, maxx, miny, maxy, minz, maxz;
const int NODATA = -9999;
const int WINDOWSIZE = 500;
const int BIGINT = 0x0fffffff;

//for hill shade
vector<vector<float> > elevation;
float min_elevation; //This is used instead of the minz value since
		     //minz is affected by weird LIDAR noise.
Point sun_incidence(0.577, 0.577, -0.577); //sun vector

//for ground find
vector<vector<float> > last_grid;
vector<vector<int> > is_ground;
vector<vector<int> > find_ground();
float building_slope_threshold = 0.5;


int point_density = 5; //average points per grid cell

//whenever the user rotates and translates the scene, we update these
//global translation and rotation
GLfloat pos[3] = {0,0,0};
GLfloat theta[3] = {0,0,0};

// draw polygons line or filled. This will be used when rendering the
// surface.
GLint fillmode = 0;


/* ************************************************************ */
/* FILTERING POINTS BY THEIR RETURN SITUATION */
/* A LiDAR point has a return number and a number of returns (for its
   pulse).  A pulse may get several returns over vegetation, lets say
   3, and this will result in three points:

   return number/number of returns
   1/3 x y z
   2/3 x y z
   3/3 x y z
   note the x,y,z may be different for different return numbers

   For bare earth, you will only have one return:
   1/1


   If ALL_RETURN, all points  are included
   IF FIRST_RETURN, only the first returns are included, ie points with return number=1

   If LAST_RETURN, only the last returns are included, i.e points with
   1/1 and points with 2/2 or 3/3 or 4/4
*/
#define ALL_RETURN 0
#define FIRST_RETURN 1
#define LAST_RETURN 2
#define MORE_THAN_ONE_RETURN 3
#define NB_WHICH_RETURN_OPTIONS 4
// WHICH_RETURN  cycles through all options via keypress 'm'
int WHICH_RETURN = ALL_RETURN;



/* ************************************************************ */
/* FILTERING POINTS BY THEIR classification code */
/*
lidar classification codes  [from ...somewhere on internet]

0 never classified
1 unassigned
2 ground
3 low vegetation
4 medium vegetation
5 high vegetation
6 building
7 low point (noise)
8 model key-point (mass point)
9 water
10 railroad
11 road
12 overlap
13 wire-guard (shield)
14 wire-conductor (phase)
15 transmission tower
17 bridge
18 hight point (noise)
19-255 reserved for asprs definition
 */

//These are used by render() to decide  what points to render.
//By default draw everything; each one of these classes can be toggled
//on/off in keypress()
int GROUND=1;
int VEG=1;
int BUILDING=1;
int OTHER=1;
int HILL_SHADE = 1;


/* **************************************** */
/* chosing a color map:
g
   If COLORMAP == ONE_COLOR:  draw all filtered points in one color

   If COLORMAP == CODE_COLOR: draw all filtered points with a color
   based on their classification code p.code which was read from the
   las file

   If COLORMAP == MYCODE_COLOR:  draw all filtered points with a color
   based on their classification code p.mycode which we computed

   COLORMAP starts by default as ONE_COLOR and cycles through all
   options via keypress 'c'.
*/
#define ONE_COLOR 0
#define CODE_COLOR 1
#define MYCODE_COLOR 2
#define NB_COLORMAP_CHOICES 3
int COLORMAP = ONE_COLOR;



//predefine some colors for convenience
GLfloat red[3] = {1.0, 0.0, 0.0};
GLfloat green[3] = {0.0, 1.0, 0.0};
GLfloat blue[3] = {0.0, 0.0, 1.0};
GLfloat black[3] = {0.0, 0.0, 0.0};
GLfloat white[3] = {1.0, 1.0, 1.0};
GLfloat gray[3] = {0.5, 0.5, 0.5};
GLfloat yellow[3] = {1.0, 1.0, 0.0};
GLfloat magenta[3] = {1.0, 0.0, 1.0};
GLfloat cyan[3] = {0.0, 1.0, 1.0};

/* from https://www.opengl.org/discussion_boards/showthread.php/132502-Color-tables  */
GLfloat brown[3] = { 0.647059, 0.164706, 0.164706};
GLfloat DarkBrown[3] = { 0.36, 0.25, 0.20};
GLfloat DarkTan[3] = { 0.59, 0.41, 0.31};
GLfloat Maroon[3]= { 0.556863, 0.137255, 0.419608};
GLfloat DarkWood[3] = { 0.52, 0.37, 0.26};

GLfloat  Copper[3] = { 0.72,  0.45,  0.20};

GLfloat green1[3] = {.5, 1, 0.5};
GLfloat green2[3] = {0.0, .8, 0.0};
GLfloat green3[3] = {0.0, .5, 0.0};
GLfloat ForestGreen[3] = { 0.137255, 0.556863, 0.137255};
GLfloat MediumForestGreen[3] = { 0.419608 , 0.556863 , 0.137255};
GLfloat LimeGreen[3] ={ 0.196078,  0.8 , 0.196078};

GLfloat Orange[3] = { 1, .5, 0};
GLfloat Silver[3] = { 0.90, 0.91, 0.98};
GLfloat Wheat[3] = { 0.847059 , 0.847059, 0.74902};


/* forward declarations of functions */
void display(void);
void keypress(unsigned char key, int x, int y);

void draw_hill_shade();
void draw_ground();
void draw_xy_rect(GLfloat z, GLfloat* col);
void draw_xz_rect(GLfloat y, GLfloat* col);
void draw_yz_rect(GLfloat x, GLfloat* col);
void cube(GLfloat side);
void filledcube(GLfloat side);
void draw_axes();

//reads the points from file in global array points
void readPointsFromFile(char* fname);

//FUNCTIONS CREATED BY ETHAN AND JAKE
//puts points into elevation grid
void gridify(){
  int num_cells = points.size()/point_density;

  //bounding box size
  float h = maxy - miny;
  float w  = maxx - minx;

  //average grid square length
  float delta = sqrt(h*w/float(num_cells));

  int rows = ceil(h/delta);
  int cols = ceil(w/delta);

  //temporary height grid vectors, holds multiple height values per grid cell
  vector< vector< vector<float> > >first_returns(rows,
						   vector< vector<float> >(cols, vector<float>()));
  vector< vector< vector<float> > >last_returns(rows,
						   vector< vector<float> >(cols, vector<float>()));

  //put FIRST RETURN and LAST RETURN lidar points into their grids
  for(unsigned int i = 0; i < points.size(); i++) {
    lidarPoint p = points[i];

    int r = floor((p.y - miny)/delta);
    int c = floor((p.x - minx)/delta);

    if(p.return_number == 1){
      first_returns[r][c].push_back(p.z);
    }

    if(p.return_number == p.return_number){
      last_returns[r][c].push_back(p.z);
    }
  }

  vector< vector<float> > avg_height(rows, vector<float>(cols, NODATA));
  vector< vector<float> > avg_depth(rows, vector<float>(cols, NODATA));

  //initialize to a large z
  min_elevation = maxz;

  //average out all points in each grid cell, for first and last
  //return grids
  for(unsigned int i = 0; i < first_returns.size(); i++) {
    for(unsigned int j = 0; j < first_returns.size(); j++) {
      float height_sum = 0;
      float depth_sum = 0;
      unsigned int k = 0;
      unsigned int l = 0;

      //sum all points in the both grids
      for(k = 0; k < first_returns[i][j].size(); k++) {
	height_sum += first_returns[i][j][k];
      }
      for(l = 0; l < last_returns[i][j].size(); l++) {
	depth_sum += last_returns[i][j][l];
      }

      //use the counter of the above for loop to tell if there were
      //any points in the current grid cell. If there are, set
      //elevation equal to average of the points in this grid cell.
      if(k > 0) {
	avg_height[i][j] = height_sum/k;
      }

      if(l > 0) {
	avg_depth[i][j] = depth_sum/l;
      }


      //find the lowest average ground point. This is used instead of
      //the min_z value since min_z is affected by weird LIDAR noise.
      if(avg_height[i][j] != NODATA &&
	 avg_height[i][j] < min_elevation) {
	min_elevation = avg_height[i][j];
      }
    }
  }

  elevation = avg_height;
  last_grid = avg_depth;
  //find the ground
  is_ground = find_ground();
}



/* NOTE: file.txt must be obtained from file.las with las2txt with
   -parse xyznrc in this order

   last2txt -o file.las -o file.txt -parse xyznrc
 */
//reads the points from file in global array points
void readPointsFromFile(char* fname) {

  FILE* file = fopen(fname, "r");
  if (!file) {
    printf("cannot open file %s\n",  fname);
    exit(1);
  }

  lidarPoint p;
  while (1) {
    //-parse xyzcr
    if (fscanf(file, "%f %f %f %d %d %d",
	       &p.x, &p.y, &p.z, &p.nb_of_returns, &p.return_number, &p.code) <6)  break;

    //else: if we are here,  we got another point
    //printf("reading %f, %f, %f\n", p.x, p.y, p.z);


    //insert the point in points[] array
    p.mycode=0; //everything unclassified
    points.push_back(p);

    //update bounding box
    if (points.size() == 1) {
      minx=maxx = p.x;
      miny=maxy = p.y;
      minz=maxz = p.z;
    } else {
      if (minx > p.x) minx=p.x;
      if (maxx < p.x) maxx = p.x;
      if (miny > p.y) miny=p.y;
      if (maxy < p.y) maxy = p.y;
      if (minz > p.z) minz=p.z;
      if (maxz < p.z) maxz = p.z;
    }
  } //while

  fclose(file);

  //print info
  printf("total %d points in  [%f, %f], [%f,%f], [%f,%f]\n",
	 (int)points.size(), minx, maxx, miny,maxy, minz, maxz);
  gridify();
}



int main(int argc, char** argv) {
  //read number of points from user
  if (argc!=4) {
    printf("usage: %s <file>.txt <density> <building slope threshold>\n", argv[0]);
    exit(1);
  }
  //this allocates and initializes the array that holds the points
  point_density = atoi(argv[2]);
  building_slope_threshold = atof(argv[3]);

  readPointsFromFile(argv[1]);

  /* OPEN GL STUFF */
  /* open a window and initialize GLUT stuff */
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB | GLUT_DEPTH);
  glutInitWindowSize(WINDOWSIZE, WINDOWSIZE);
  glutInitWindowPosition(100,100);
  glutCreateWindow(argv[0]);

  /* register callback functions */
  glutDisplayFunc(display);
  glutKeyboardFunc(keypress);

  /* OpenGL init */
  /* set background color black*/
  glClearColor(0, 0, 0, 0);
  glEnable(GL_DEPTH_TEST);

  // //CAMERA ANGLE CODE
  // //CAMERA ANGLE CODE
  // //CAMERA ANGLE CODE
  // // setup the camera (i.e. the projection transformation)
  // glMatrixMode(GL_PROJECTION);
  // glLoadIdentity();
  // gluPerspective(60, 1 /* aspect */, 1, 10.0); /* the frustrum is from z=-1 to z=-10 */
  // /* camera is at (0,0,0) looking along negative z axis */

  // //initialize the translation to bring the points in the view frustrum which is [-1, -10]
  // pos[2] = -2;

  // //initialize rotation to look at it from above
  // theta[0] = -45;


  /* start the event handler */
  glutMainLoop();

  return 0;
}




/* this function is called whenever the window needs to be rendered */
void display(void) {

  //clear the screen
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  //clear all modeling transformations
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  /* The default GL window is x=[-1,1], y= [-1,1] with the origin in
     the center.  The view frustrum was set up from z=-1 to z=-10. The
     camera is at (0,0,0) looking along negative z axis.
  */

 /* First we translate and rotate our local reference system with the
    user transformation. pos[] represents the cumulative translation
    entered by the user, and theta[] the cumulative rotation entered
    by the user */
  glTranslatef(pos[0], pos[1], pos[2]);
  glRotatef(theta[0], 1,0,0); //rotate theta[0] around x-axis, etc
  glRotatef(theta[1], 0,1,0);
  glRotatef(theta[2], 0,0,1);

  /* We translated the local reference system where we want it to be; now we draw the
     object in the local reference system.  */

  if (HILL_SHADE) {
      draw_ground();
    }
    else {
      draw_hill_shade();
    }

  //don't need to draw a cube but I found it nice for perspective
  //  cube(1); //draw a cube of size 1

  glFlush();
}



/* this function is called whenever  key is pressed */
void keypress(unsigned char key, int x, int y) {
  switch(key) {
  case 's':
    HILL_SHADE = !HILL_SHADE;
    glutPostRedisplay();
    break;

  case '+':
    building_slope_threshold += 0.05;
    cout << "Building slope threshold is now: " <<
      building_slope_threshold << endl;

    is_ground = find_ground();
    glutPostRedisplay();
    break;

  case '-':
    building_slope_threshold -= 0.05;
    cout << "Building slope threshold is now: " <<
      building_slope_threshold << endl;

    is_ground = find_ground();
    glutPostRedisplay();
    break;

  case '2':
    //3d orthogonal projection, view from straight above
    glMatrixMode(GL_PROJECTION);
    //the view frustrum is z=[0, -20]
    glOrtho(-1, 1, -1, 1, 0,-20); //left, right, top, bottom, near, far

    //initial view is from (0,0,-5) ie above the terrain looking straight down
    pos[0]=pos[1]=0; pos[2] = -7;
    //initial view: no rotation
    theta[0]=theta[1] = theta[2]= 0;
    glutPostRedisplay();
    break;

  case '3':
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60, 1 /* aspect */, 1, 10.0); /* the frustrum is from z=-1 to z=-10 */
    /* camera is at (0,0,0) looking along negative z axis */
    //initialize the translation; view frustrum is z= [-1, -10] and
    //initial position (0,0,-2)
    pos[0]=pos[1]=0; pos[2] = -2;
    //initialize rotation to look  from above
    theta[1] = theta[2] = 0;  theta[0] = -45;
    glutPostRedisplay();
    break;

  case 'c':
    //change the colormap
    COLORMAP= (COLORMAP+1) % NB_COLORMAP_CHOICES;
    switch (COLORMAP) {
    case ONE_COLOR:
      printf("colormap: one color\n");
      break;
    case CODE_COLOR:
      printf("colormap: by code\n");
      break;
    case MYCODE_COLOR:
      printf("colormap: by mycode\n");
      break;
    default:
      printf("colormap: unknown. oops, something went wrong.\n");
      exit(1);
    }
    glutPostRedisplay();
    break;

  case 't':
    //render points with one/more than one return
    WHICH_RETURN = (WHICH_RETURN + 1) % NB_WHICH_RETURN_OPTIONS;
    switch (WHICH_RETURN) {
    case ALL_RETURN:
      printf("draw all returns\n");
      break;
    case FIRST_RETURN:
      printf("draw only first return (i.e.points with return_number=1)\n");
      break;
    case LAST_RETURN:
      printf("draw only last return (i.e. points with return_number = number_of_returns)\n");
      break;
    case MORE_THAN_ONE_RETURN:
      printf("draw only points that has >1 returns\n");
      break;
    default:
      break;
    }
    glutPostRedisplay();
    break;

  case 'g':
    //toggle off rendering ground points   (code=2)
    GROUND = !GROUND;
    glutPostRedisplay();
    break;

  case 'v':
    //toggle off rendering vegetation points  (code=3,4,5)
    VEG = !VEG;
    glutPostRedisplay();
    break;

  case 'h':
    //toggle off rendering building points  (code=6)
    BUILDING = !BUILDING;
    glutPostRedisplay();
    break;

  case 'o':
    //toggle off rendering "other" ie points that are not ground, vegetation or building
    OTHER=!OTHER;
    glutPostRedisplay();
    break;

    //ROTATIONS
  case 'x':
    theta[0] += 5.0;
    glutPostRedisplay();
    break;
  case 'y':
    theta[1] += 5.0;
    glutPostRedisplay();
    break;
  case 'z':
    theta[2] += 5.0;
    glutPostRedisplay();
    break;
  case 'X':
    theta[0] -= 5.0;
    glutPostRedisplay();
    break;
  case 'Y':
    theta[1] -= 5.0;
    glutPostRedisplay();
    break;
  case 'Z':
    theta[2] -= 5.0;
    glutPostRedisplay();
    break;

    //TRANSLATIONS
    //backward (zoom out)
  case 'b':
    pos[2] -= 0.1;
    glutPostRedisplay();
    break;
    //forward (zoom in)
  case 'f':
    pos[2] += 0.1;
    //glTranslatef(0,0, 0.5);
    glutPostRedisplay();
    break;
    //down
  case 'd':
     pos[1] -= 0.1;
    //glTranslatef(0,0.5,0);
    glutPostRedisplay();
    break;
    //up
  case 'u':
    pos[1] += 0.1;
    //glTranslatef(0,-0.5,0);
    glutPostRedisplay();
    break;
    //left
  case 'l':
    pos[0] -= 0.1;
    glutPostRedisplay();
    break;
    //right
  case 'r':
    pos[0] += 0.1;
    glutPostRedisplay();
    break;

    //fillmode
  case 'w':
    fillmode = !fillmode;
     glutPostRedisplay();
    break;

  case 'q':
    exit(0);
    break;
  }
}//keypress









/* x is a value in [minx, maxx]; it is mapped to [-1,1] */
GLfloat xtoscreen(GLfloat x, int num_cols) {
  //return (-1 + 2*x/WINDOWSIZE);
  return (-1 + 2*(x)/float(num_cols));
}


/* y is a value in [miny, maxy]; it is mapped to [-1,1] */
GLfloat ytoscreen(GLfloat y, int num_rows) {
  return (-1 + 2*(y)/float(num_rows));
}

/* z is a value in [minz, maxz]; it is mapped so that [0, maxz] map to [0,1] */
GLfloat ztoscreen(GLfloat z) {
    return (-1 + 2*(z-minz)/(maxz-minz))/1.5;
}


//set color based on p.code
void setColorByCode(lidarPoint p) {
  switch (p.code) {
  case 0: //never classified
    glColor3fv(yellow);
    break;
  case 1: //unnasigned
    glColor3fv(Orange);
    break;
  case 2: //ground
    //    glColor3fv(DarkWood);
    glColor3fv(DarkBrown);
    break;
  case 3: //low vegetation
    glColor3fv(LimeGreen);
    break;
  case 4: //medium vegetation
    glColor3fv(MediumForestGreen);
    break;
  case 5: //high vegetation
    glColor3fv(ForestGreen);
    break;
  case 6: //building
    //glColor3fv(Tan);
    glColor3fv(Copper);
    break;
  case 7: //noise
    glColor3fv(magenta);
    break;
  case 8: //reserved
    glColor3fv(white);
    break;
  case 9: //water
    glColor3fv(blue);
    break;
  case 10: //rail
    glColor3fv(gray);
    break;
  case 11: //road surface
    glColor3fv(gray);
    break;
  case 12:  //reserved
    glColor3fv(white);
    break;
  case 13:
  case 14: //wire
    glColor3fv(gray);
    break;
  case 15: //transmission tower
    glColor3fv(Wheat);
    break;
  case 16: //wire
  case 17: //bridge deck
    glColor3fv(blue);
    break;
  case 18: //high noise
    glColor3fv(magenta);
    break;
  default:
    printf("panic: encountered unknown code >18");
  }
} //setColorByCode



//put your own colormap here  based on p.mycode
void setColorByMycode(lidarPoint p) {

  glColor3fv(blue);
}

//draw everything with one color
void  setColorOneColor(lidarPoint p) {

   glColor3fv(yellow); //yellow should be a constant...
  return;
}



//point p has passed all the filters and must be rendered. Set its
//color.
void setColor(lidarPoint p) {

  if (COLORMAP == ONE_COLOR) {
    //draw all points with same color
    setColorOneColor(p);

  } else if (COLORMAP == MYCODE_COLOR) {
    setColorByCode(p);

  } else if (COLORMAP == CODE_COLOR) {
    setColorByMycode(p);

  } else {
    printf("unkown colormap options.oops.\n");
    exit(1);
  }
} //setColor()

//calculates how bright a triangle should be, based on how much it
//faces the sun. Uses the dot product between the incident sun vector
//and the normal vector of the triangle.
void hill_shade(Point p1, Point p2, Point p3, GLfloat* shade){
  //calculate normal vector from triangle
  Point U(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
  Point V(p3.x - p1.x, p3.y - p1.y, p3.z - p1.z);
  Point N(U.y*V.z - U.z*V.y, U.z*V.x-U.x*V.z, U.x*V.y - U.y*V.x);

  //normalize N:
  float n_len = sqrt(N.x*N.x + N.y*N.y + N.z*N.z);
  N.x = N.x/n_len;
  N.y = N.y/n_len;
  N.z = N.z/n_len;

  //calculate dot product between sun vector and normal vector
  float dot_product =
    N.x*sun_incidence.x + N.y*sun_incidence.y + N.z*sun_incidence.z;

  shade[0] = dot_product;
  shade[1] = dot_product;
  shade[2] = dot_product;
}

/* ****************************** */
/* Draw the array of points stored in global variable elevation, and
   hill shade it.

   NOTE: The points are in the range x=[minx, maxx], y=[miny,
   maxy], z=[minz, maxz] and they must be mapped into
   x=[-1,1], y=[-1, 1], z=[-1,1]
  */
void draw_hill_shade(){
  int num_rows = elevation.size();
  int num_cols = elevation[0].size();

  //draw two triangles for each grid cell. shade with hill_shade
  //dot product calculation.
  glBegin(GL_TRIANGLES);
  for (unsigned int i=0; i < elevation.size()-1; i++) {
    for (unsigned int j=0; j < elevation[i].size()-1; j++) {
      //get the four heights of the cell
      float h = elevation[i][j];
      float h_i = elevation[i+1][j];
      float h_j = elevation[i][j+1];
      float h_2 = elevation[i+1][j+1];

      //triangle 1
      Point p1(float(i+1), float(j), h_i);
      Point p2(float(i), float(j), h);
      Point p3(float(i), float(j+1), h_j);

      GLfloat shade[3];
      hill_shade(p1, p2, p3, shade);

      //if NODATA, make triangle a different color
      if(h == NODATA || h_i == NODATA || h_j == NODATA){
	h = min_elevation;
	h_i = min_elevation;
	h_j = min_elevation;
	shade[0] = 1.0;
	shade[1] = 0.0;
	shade[2] = 0.6;
      }

      //draw triangle
      glColor3fv(shade);
      glVertex3f(xtoscreen(i, num_cols),
      		 ytoscreen(j, num_rows),
      		 ztoscreen(h));

      glVertex3f(xtoscreen(i+1, num_cols),
      		 ytoscreen(j, num_rows),
      		 ztoscreen(h_i));

      glVertex3f(xtoscreen(i, num_cols),
      		 ytoscreen(j+1, num_rows),
      		 ztoscreen(h_j));


      //triangle 2
      Point pa(float(i+1), float(j+1), h_2);
      Point pb(float(i+1), float(j), h_i);
      Point pc(float(i), float(j+1), h_j);

      hill_shade(pa, pb, pc, shade);

      //if NODATA, make triangle a different color
      if(h_2 == NODATA || h_i == NODATA || h_j == NODATA){
	h_i = min_elevation;
	h_j = min_elevation;
	h_2 = min_elevation;
	shade[0] = 1.0;
	shade[1] = 0.0;
	shade[2] = 0.6;
      }

      //draw second triangle
      glColor3fv(shade);
      glVertex3f(xtoscreen(i+1, num_cols),
		 ytoscreen(j, num_rows),
		 ztoscreen(h_i));

      glVertex3f(xtoscreen(i, num_cols),
		 ytoscreen(j+1, num_rows),
		 ztoscreen(h_j));

      glVertex3f(xtoscreen(i+1, num_cols),
		 ytoscreen(j+1, num_rows),
		 ztoscreen(h_2));
    }
  }
  glEnd();
}//draw_hill_shade

//note to self: can make this short to save memory
//Finds possible ground points using BFS.
//
//The BFS starts at the lowest unsearched point, and considers that
//point "ground". The BFS continues in all directions, until it
//encounters a large slope, labeling this point building. The BFS
//continues, labeling all children of this building point building as
//well, until it encounters a large negative slope. Upon encountering
//this large negative slope, the BFS terminates. The BFS also
//terminates upon encountering already classified points.
//
//This procedure is then repeated, starting from the next lowest
//unsearched point, until all points are classified.
vector<vector<int> > find_ground() {
  int num_rows = last_grid.size();
  int num_cols = last_grid[0].size();

  //initialize everything to -1, for unvisited
  vector<vector<int> > is_ground(num_rows, vector<int>(num_cols, -1));
  queue<Point> q;

  //keep track of number of unclassified points
  int unclassified_count = num_rows*num_cols;

  //take out nodata points
  for (unsigned int i=0; i < last_grid.size(); i++)
    for (unsigned int j=0; j < last_grid[i].size(); j++)
      if (last_grid[i][j] == NODATA)
	unclassified_count--;

  //loop until all points classified
  do {
    //find lowest UNCLASSIFIED ground point
    float min_height = BIGINT;
    int min_i = 0;
    int min_j = 0;
    for (unsigned int i=0; i < last_grid.size(); i++) {
      for (unsigned int j=0; j < last_grid[i].size(); j++) {
	if (last_grid[i][j] != NODATA &&
	    is_ground[i][j] == -1 &&
	    last_grid[i][j] < min_height) {
	  min_height = last_grid[i][j];
	  min_i = i;
	  min_j = j;
	}
      }
    }

    //push lowest point into queue
    q.push(Point(min_i, min_j, min_height));
    is_ground[min_i][min_j] = 1;
    unclassified_count--;

    //do BFS
    while(q.size()) {
      Point current = q.front();
      q.pop();
      int curr_i = current.x;
      int curr_j = current.y;
      float curr_h = current.z;

      bool curr_type = is_ground[curr_i][curr_j];

      for (int di = -1; di <= 1; ++di) {
	for (int dj = -1; dj <= 1; ++dj) {
	  //look in compass directions
	  if(di != dj && (di == 0 || dj == 0)) {
	    //check if in bounds
	    if(curr_i + di >= 0 && curr_j + dj >= 0 &&
	       curr_i + di < last_grid.size() &&
	       curr_j + dj < last_grid[0].size()) {

	      int new_i = curr_i + di;
	      int new_j = curr_j + dj;

	      //if point is already visited, or new point is NODATA,
	      //terminate this branch
	      if(is_ground[new_i][new_j] != -1 ||
		 last_grid[new_i][new_j] == NODATA) {
		continue;
	      }

	      float new_h = last_grid[new_i][new_j];
	      float slope = new_h - curr_h;

	      //if gentle slope, new point is same type as current point
	      if (slope <= building_slope_threshold && slope >= 0) {
		q.push(Point(new_i, new_j, new_h));
		is_ground[new_i][new_j] = curr_type;
		unclassified_count--;
	      }
	      //if steep upwards slope, new point is building
	      else if (slope > building_slope_threshold) {
		q.push(Point(new_i, new_j, new_h));
		is_ground[new_i][new_j] = 0;
		unclassified_count--;
	      }
	      //if negative slope terminate this branch
	      else if (slope < 0){
		continue;
	      }
	    }// if in bounds check
	  }// if compass direction
	}// dj
      }//di
    }// while q not empty
  } while(unclassified_count > 0);
  return is_ground;
}

/* ****************************** */
/* Draw the array of points stored in global variable last_grid,
   and shade where the ground is.

   NOTE: The points are in the range x=[minx, maxx], y=[miny,
   maxy], z=[minz, maxz] and they must be mapped into
   x=[-1,1], y=[-1, 1], z=[-1,1]
  */
void draw_ground(){
  int num_rows = last_grid.size();
  int num_cols = last_grid[0].size();

  //draw two triangles for each grid cell. shade with hill_shade
  //dot product calculation.
  glBegin(GL_POINTS);
  for (unsigned int i=0; i < last_grid.size(); i++) {
    for (unsigned int j=0; j < last_grid[i].size(); j++) {
      float h = last_grid[i][j];

      //if NODATA, make triangle a different color
      if(h == NODATA){
	h = min_elevation;
	glColor3fv(magenta);
      }
      else if(is_ground[i][j] == 1){
      	glColor3fv(brown);
      }
      else if(is_ground[i][j] == 0){
	glColor3fv(white);
      }
      else if(is_ground[i][j] == -1){
	glColor3fv(green);
      }

      glVertex3f(xtoscreen(i, num_cols),
		 ytoscreen(j, num_rows),
		 ztoscreen(h));
    }
  }
  glEnd();
}//draw_ground

//draw a square x=[-side,side] x y=[-side,side] at depth z
void draw_xy_rect(GLfloat z, GLfloat side, GLfloat* col) {

  glColor3fv(col);
  glBegin(GL_POLYGON);
  glVertex3f(-side,-side, z);
  glVertex3f(-side,side, z);
  glVertex3f(side,side, z);
  glVertex3f(side,-side, z);
  glEnd();
}


//draw a square y=[-side,side] x z=[-side,side] at given x
void draw_yz_rect(GLfloat x, GLfloat side, GLfloat* col) {

  glColor3fv(col);
  glBegin(GL_POLYGON);
  glVertex3f(x,-side, side);
  glVertex3f(x,side, side);
  glVertex3f(x,side, -side);
  glVertex3f(x,-side, -side);
  glEnd();
}


//draw a square x=[-side,side] x z=[-side,side] at given y
void draw_xz_rect(GLfloat y, GLfloat side, GLfloat* col) {

  glColor3fv(col);
  glBegin(GL_POLYGON);
  glVertex3f(-side,y, side);
  glVertex3f(-side,y, -side);
  glVertex3f(side,y, -side);
  glVertex3f(side,y, side);
  glEnd();
}

//draw a cube
void cube(GLfloat side) {
  GLfloat f = side, b = -side;

  if (fillmode) {
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  } else {
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  }


  /* back face  BLUE*/
  draw_xy_rect(b,side, blue);
 /* front face  RED*/
  draw_xy_rect(f,side, red);
  /* side faces  GREEN*/
  draw_yz_rect(b, side, green);
  draw_yz_rect(f, side, green);
  //up, down faces missing to be able to see inside

  /* middle z=0 face CYAN*/
  draw_xy_rect(0, side, cyan);
  /* middle x=0 face WHITE*/
  draw_yz_rect(0,side, gray);
  /* middle y=0 face  pink*/
  draw_xz_rect(0, side, magenta);
}



//draw a filled cube  [-side,side]^3
void filledcube(GLfloat side) {

  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

  /* back, front faces */
  draw_xy_rect(-side,side, yellow);
  draw_xy_rect(side,side, yellow);

  /* left, right faces*/
  draw_yz_rect(-side, side, yellow);
  draw_yz_rect(side, side, yellow);

  /* up, down  faces  */
  draw_xz_rect(side,side, yellow);
  draw_xz_rect(-side,side, yellow);
}
