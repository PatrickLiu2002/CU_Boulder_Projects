/*
 *  Final Project
 *
 *  Demonstrates lighting, textures, and other object interactions. For full details, see README.
 *
 *  Key bindings:
 *  wasd:      Move the currently controlled object on the X/Z plane
 *  q/e:       Rotate the currently controlled object left/right. 
 *  r/f:       Raise/Lower the currently controlled object. Only works if controlling the Drone.
 *  p:         Change perspective.
 *  m/M:       Change controlled object.
 *  g/G:       Toggle Spotlight/Flashlight. 
 *  h/H:       Display/hide Help Menu
 *  t/T:       Toggle Drone UI
 *  l/L        Toggle Debug Mode
 *  [/]:       Change displayed object in debug mode. 
 *  Mouse:     Change camera angle
 *  ESC        Exit
 */


#include "CSCIx229.h" //We're allowed to use the library this time

//World values based on example code
int axes=0;       //  Display axes (only for debugging, set to off)
int mode = 1;       //  Projection mode (Set to projection)
int move = 1;       //  Move light (used for debug, default on)
double dim = 3;     //  Size of world

//Camera controls
int th=0;         //  Azimuth of view angle
int ph=0;         //  Elevation of view angle
int fov=55;       //  Field of view (for perspective)
double asp=1;     //  Aspect ratio

// Light values. Based on example code. Used for fine-tuning. Default fixed
int one       =   1;  // Unit value
int distance  =   5;  // Light distance
int inc       =  5;  // Ball increment (default 10)
int smooth    =   1;  // Smooth/Flat shading
int local     =   0;  // Local Viewer Model
int emission  =   0;  // Emission intensity (%)
int ambient   =  10;  // Ambient intensity (%) //formerly 10
int diffuse   =  70;  // Diffuse intensity (%)
int specular  =   1;  // Specular intensity (%)
int shininess =   0;  // Shininess (power of two)
float shiny   =   1;  // Shininess (value)
int zh        =  90;  // Light azimuth
float ylight  =   0;  // Elevation of light

typedef struct {float x,y,z;} vtx;
typedef struct {int A,B,C;} tri;
#define n 500
vtx is[n];

#define NUM_LIGHTS 4 // The amount of streetlights we have. These are all 30 degree arc lights. 

// Define light positions for lights
GLfloat light_positions[NUM_LIGHTS][4] = {
    { 0.493f, 0.6f, -0.75f, 1.0f },
    { 0.493f, 0.6f, 0.75f, 1.0f },
    {-0.493f, 0.6f, -0.75f, 1.0f},
    {-0.493f, 0.6f, 0.75f, 1.0f },
};

//Define light colors
GLfloat light_colors[NUM_LIGHTS][4] = {
    { 1.0f, 1.0f, 0.0f, 1.0f }, // White
    { 1.0f, 0.0f, 0.0f, 1.0f }, // Red
    { 0.0f, 1.0f, 0.0f, 1.0f }, // Green
    { 1.0f, 1.0f, 1.0f, 1.0f }, // White
};

int objectChosen = 0; //Object chosen to display.
int night = 0; //Night Mode
int debug = 0; //debug mode

//For moving the camera
double Ex = 0;
double Ey = 0.2;
double Ez = 0;
double dt = 0.05; //How far you move in one step. Affects all objects!

//For textures
unsigned int texture[25];  //  Texture names

//Check if we're controlling the drone or person. Ignored in debug.
int controllingDrone = 1;
int controllingMan = 0;
int firstPerson = 0;

//Drone coordinates:
double droneX = 0;
double droneY = 1; //Coordinates are approx. the center of the drone's body
double droneZ = 0;
int droneTh = 90;
double camTh = 0;
double camPh = 0;
int spotlight = 1;

//Man coordinates:
double manX = 0;
double manY = 0; //For simplicity, man will be considered to be on the ground always. Y is irrelevant as he cannot fly. 
double manZ = 0;
int manTh = 90;
double headTh = 0;
int flashlight = 1;

//Camera movement for mouse
int X = 0, Y = 0;
int moveCam = 0;

//For UI
int displayHelp = 0;
int drawUI = 1;

//For traffic lights
int last_update_time = 0;
int trafficState = 1;

//For drawing
double droneScale = 0.1;
double manScale = 0.1;

// The terrain map is stored as a 2d array of ints and rendered using a loop

//Block types: 0 = Water, 1 = Grass, 3 = Road (X), 4 = Road(Z), 5 = Intersection, 6 = Sand. "Water" is rendered as half-water, half-sand.
int worldMap[15][15] = { 
   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
   {0, 0, 0, 6, 6, 6, 6, 6, 0, 0, 0, 0, 0, 0, 0},
   {0, 0, 0, 6, 6, 1, 1, 6, 0, 0, 0, 0, 0, 0, 0}, 
   {0, 0, 6, 6, 3, 1, 1, 6, 6, 6, 6, 0, 0, 0, 0},
   {0, 0, 6, 1, 3, 1, 1, 1, 1, 1, 6, 6, 0, 0, 0},
   {0, 0, 6, 1, 3, 1, 1, 1, 1, 1, 1, 6, 0, 0, 0},
   {0, 0, 6, 1, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4}, //Z = 0 here
   {0, 0, 6, 1, 3, 1, 1, 1, 1, 1, 6, 6, 0, 0, 0},
   {0, 0, 6, 1, 3, 1, 1, 1, 1, 1, 6, 0, 0, 0, 0},
   {0, 0, 6, 6, 6, 1, 1, 1, 1, 6, 6, 0, 0, 0, 0},
   {0, 0, 0, 0, 6, 6, 6, 6, 6, 6, 0, 0, 0, 0, 0},
   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};
int worldRows = 15;
int worldCols = 15;

/*
 For collision detection we use bounding cylinders 
 As the ground handles collision differently it does not need to be a struct. 
*/

typedef struct {
   char type[20]; //String
   //For drawing:
   double rotation; //Rotation. 
   double scale; //Drawn size

   //Position
   double x;
   double y;
   double z;

   //Collision detection (these use the "real" size of the object, before it is scaled)
   double radius; //For collision (XZ plane)
   double height; //Height in coordinate system, for collision in Y-axis
} Object;


//All the things in the scene.

int opaqueThingsSize = 21;

Object opaqueThings[21] = {
   {"House", -90, 0.3, 1.5, 0, 1.5, 2.5, 2.5},
   {"House", 90, 0.3, -1.5, 0, 1.5, 2.5, 2.5},
   {"House", -90, 0.3, 1.5, 0, 0, 2.5, 2.5},
   {"House", 90, 0.3, -1.5, 0, 0, 2.5, 2.5},
   {"House", -90, 0.3, 1.5, 0, -1.5, 2.5, 2.5},
   {"House", 90, 0.3, -1.5, 0, -1.5, 2.5, 2.5},
   {"Tree", 0, 0.2, 1, 0, -4, 2, 8},
   {"Tree", 0, 0.2, 2, 0, -4, 2, 8},
   {"Tree", 0, 0.2, 0, 0, -4, 2, 8},
   {"Tree", 0, 0.2, -1, 0, -4, 2, 8},
   {"Tree", 0, 0.2, -2, 0, -4, 2, 8},

   {"Streetlight", -90, 0.3, 0.7, 0, -0.75, 0.5, 2.05},
   {"Streetlight", 90, 0.3, -0.7, 0, -0.75, 0.5, 2.05},

   {"Streetlight", -90, 0.3, 0.7, 0, 0.75, 0.5,2.05},
   {"Streetlight", 90, 0.3, -0.7, 0, 0.75, 0.5,2.05},

   {"Trafficlight", -90, 0.2, 0.7, 0, -2.3, 0.5, 3},

   {"Trafficlight", 0, 0.2, 0.7, 0, -3.7, 0.5, 3},
   {"Trafficlight", 90, 0.2, -0.7, 0, -3.7, 0.5, 3},


   {"Trafficlight", 0, 0.2, 0.7, 0, 3.3, 0.5, 3},
   {"Trafficlight", 180, 0.2, -0.7, 0, 3.3, 0.5, 3},

};

//Transparent things for debug
int transparentThingsSize = 10;

Object transparentThings[10] = {
   

   {"Streetlight2", -90, 0.3, 0.7, 0, -0.75, 0.5, 2.05},
   {"Streetlight2", 90, 0.3, -0.7, 0, -0.75, 0.5, 2.05},

   {"Streetlight2", -90, 0.3, 0.7, 0, 0.75, 0.5,2.05},
   {"Streetlight2", 90, 0.3, -0.7, 0, 0.75, 0.5,2.05},

   {"Trafficlight2", -90, 0.2, 0.7, 0, -2.3, 0.5, 3},

   {"Trafficlight2", 0, 0.2, 0.7, 0, -3.7, 0.5, 3},
   {"Trafficlight2", 90, 0.2, -0.7, 0, -3.7, 0.5, 3},


   {"Trafficlight2", 0, 0.2, 0.7, 0, 3.3, 0.5, 3},
   {"Trafficlight2", 180, 0.2, -0.7, 0, 3.3, 0.5, 3},
};

// The skybox. 
static void SkyBox()
{
   //Stop writing to depth buffer
   glDepthMask(0); 

   //  Set specular color to white (based on examples)
   float white[] = {1,1,1,1};
   float black[] = {0,0,0,1};
   glMaterialf(GL_FRONT_AND_BACK,GL_SHININESS,shiny);
   glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,white);
   glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION,black);

   //  Save transformation
   glPushMatrix();
   //  Offset
   glTranslated(Ex,Ey,Ez);
   glScaled(dim*2,dim*2,dim*2);

   //  Enable textures
   glDisable(GL_LIGHTING);
   glEnable(GL_TEXTURE_2D);
   glTexEnvi(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_MODULATE);
   glColor3f(1,1,1); //Use true texture color

   //Day/night mode
   if(night){
      glBindTexture(GL_TEXTURE_2D,texture[17]);
   }
   else {
      glBindTexture(GL_TEXTURE_2D,texture[16]);
   }

   //Set normal vector
   glNormal3f( 0,-1, 0);

   //Build object. Few vertices are used as skybox does not have lighting.
   //  Left
   glBegin(GL_QUADS);
   glNormal3f(1, 0, 0);
   glTexCoord2f(0, 0.34); glVertex3f(-0.5,-0.5,-0.5);
   glTexCoord2f(0.25, 0.34); glVertex3f(-0.5,-0.5,+0.5);
   glTexCoord2f(0.25, 0.66); glVertex3f(-0.5,0.5,+0.5);
   glTexCoord2f(0, 0.66); glVertex3f(-0.5,0.5,-0.5);
   glEnd();

   //  Front
   glNormal3f(0, 0, -1);
   glBegin(GL_QUADS);
   glTexCoord2f(0.5, 0.34); glVertex3f(+0.5,-0.5,+0.5);
   glTexCoord2f(0.25, 0.34); glVertex3f(-0.5,-0.5,+0.5);
   glTexCoord2f(0.25, 0.66); glVertex3f(-0.5,0.5,+0.5);
   glTexCoord2f(0.5, 0.66); glVertex3f(+0.5,0.5,+0.5);
   glEnd();


   //  Bottom
   glBegin(GL_QUADS);
   glNormal3f(0,1, 0);
   glTexCoord2f(0.25, 0); glVertex3f(-0.5,-0.5, -0.5);
   glTexCoord2f(0.25, 0.34); glVertex3f(-0.5,-0.5, 0.5);
   glTexCoord2f(0.5, 0.34); glVertex3f(0.5,-0.5, 0.5);
   glTexCoord2f(0.5, 0); glVertex3f(0.5,-0.5, -0.5);
   glEnd();


   //Top
   glBegin(GL_QUADS);
   glTexCoord2f(0.5, 0.66); glVertex3f(0.5,0.5, 0.5);
   glTexCoord2f(0.5, 1); glVertex3f(0.5,0.5, -0.5);
   glTexCoord2f(0.25, 1); glVertex3f(-0.5,0.5, -0.5);
   glTexCoord2f(0.25, 0.66); glVertex3f(-0.5,0.5, 0.5);
   glEnd();

   //  Right
   glNormal3f(-1, 0, 0);
   glBegin(GL_QUADS);
   glTexCoord2f(0.5, 0.34); glVertex3f(+0.5,-0.5,+0.5);
   glTexCoord2f(0.75, 0.34); glVertex3f(+0.5,-0.5,-0.5);
   glTexCoord2f(0.75, 0.66); glVertex3f(+0.5,+0.5,-0.5);
   glTexCoord2f(0.5, 0.66); glVertex3f(+0.5,+0.5,+0.5);
   glEnd();

   //  Back
   glNormal3f(0, 0, 1);
   glBegin(GL_QUADS);
   glTexCoord2f(1, 0.34); glVertex3f(-0.5,-0.5,-0.5);
   glTexCoord2f(0.75, 0.34); glVertex3f(+0.5,-0.5,-0.5);
   glTexCoord2f(0.75, 0.66); glVertex3f(+0.5,0.5,-0.5);
   glTexCoord2f(1, 0.66); glVertex3f(-0.5,0.5,-0.5);
   glEnd();

   //  Undo transformations
   glPopMatrix();
   glDisable(GL_TEXTURE_2D);
   glDepthMask(1);
   glEnable(GL_LIGHTING);
}

/*
 *  Draw vertex, based on example code, modified to support more colors and textures
 */
static void Vertex(double th,double ph)
{
   //Color switch

   double x = Sin(th)*Cos(ph);
   double y = Cos(th)*Cos(ph);
   double z =         Sin(ph);
   //  For a sphere at the origin, the position
   //  and normal vectors are the same
   glNormal3d(x,y,z);
   glTexCoord2d(th/360.0,ph/180.0 + 0.5); //Set texture coordinates. 
   glVertex3d(x,y,z);
}

// This is the Sun/Moon
// If it's below the "horizon", we don't render it. 
static void SunMoon(double x,double y,double z,double r, int mode)
{
   if(y < 0){
      return;
   }
   //  Save transformation
   glPushMatrix();
   //  Offset, scale and rotate
   glTranslated(x,y,z);
   glTranslated(Ex,Ey,Ez);
   glScaled(r,r,r);
   //  White ball with yellow specular
   float yellow[]   = {1.0,1.0,0.0,1.0};
   float Emission[] = {0.0,0.0,0.01*emission,1.0};
   glColor3f(1,1,1);
   glMaterialf(GL_FRONT,GL_SHININESS,shiny);
   glMaterialfv(GL_FRONT,GL_SPECULAR,yellow);
   glMaterialfv(GL_FRONT,GL_EMISSION,Emission);


   glEnable(GL_TEXTURE_2D);
   if(mode == 1){
      glBindTexture(GL_TEXTURE_2D, texture[15]);
   }
   else{
      glBindTexture(GL_TEXTURE_2D, texture[14]);
   }

   //  Bands of latitude
   for (int ph=-90;ph<90;ph+=inc)
   {
      glBegin(GL_QUAD_STRIP);
      for (int th=0;th<=360;th+=2*inc)
      {
         Vertex(th,ph);
         Vertex(th,ph+inc);
      }
      glEnd();
   }
   //  Undo transofrmations
   glPopMatrix();
}

/*
 *  Draw a sphere at (x,y,z), radius (r)
 *     Improved in-class example code's sphere to handle multiple colors and lighting
 *     Improved to handle textures for leaves and fruit.
 */
static void sphere(double x,double y,double z,double r, int mode)
{
   //  Save transformation
   glPushMatrix();
   //  Offset and scale
   glTranslated(x,y,z);
   glScaled(r,r,r);

   //  Enable textures
   glEnable(GL_TEXTURE_2D);
   glTexEnvi(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE, GL_MODULATE);


   //  Set specular color to white
   float white[] = {1,1,1,1};
   float black[] = {0,0,0,1};
   glMaterialf(GL_FRONT_AND_BACK,GL_SHININESS,shiny);
   glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,white);
   glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION,black);
   if(mode == 1){
      glColor3f(1, 1, 1);
      glBindTexture(GL_TEXTURE_2D,texture[9]);
   }
   else if (mode == 2){
      glColor3f(1, 1, 1);
      glBindTexture(GL_TEXTURE_2D,texture[5]);

   }
   else{
      glColor3f(0.3, 0.8, 0.3);
      glBindTexture(GL_TEXTURE_2D,texture[9]);
   }
   if(mode == 1){
      glColor3f(1, 1, 1);
   }
   else if (mode == 2){
      glColor3f(0.8, 0, 0);
   }
   else{
      glColor3f(0, 0.5, 0);
   }
   //  Latitude bands
   for (int ph=-90;ph<90;ph+=inc)
   {
      glBegin(GL_QUAD_STRIP);
      for (int th=0;th<=360;th+=inc)
      {
         Vertex(th,ph);
         Vertex(th,ph+inc);
      }
      glEnd();
   }
   glDisable(GL_TEXTURE_2D);

   //  Undo transformations
   glPopMatrix();
}

//A "split quad", with more vertices so that the ground is lit smoother.
static void drawSplitQuad(float y, float texX, float texY) {
    // Grid dimensions: 10x10 smaller quads
    int gridSize = 20;
    float textureStepX = texX / gridSize;
    float textureStepY = texY / gridSize;
    float vertexStep = 1.0f / gridSize;
    glBegin(GL_QUADS);

    for (int i = 0; i < gridSize; i++) {
        for (int j = 0; j < gridSize; j++) {
            // Calculate the texture coordinates for the smaller quad
            float texX0 = j * textureStepX;
            float texY0 = i * textureStepY;
            float texX1 = (j + 1) * textureStepX;
            float texY1 = (i + 1) * textureStepY;

            // Calculate the vertex coordinates for the smaller quad
            float x0 = -0.5f + j * vertexStep;
            float z0 = 0.5f - i * vertexStep;
            float x1 = -0.5f + (j + 1) * vertexStep;
            float z1 = 0.5f - (i + 1) * vertexStep;

            // Draw the smaller quad using the calculated texture coordinates and vertices

            // First vertex (top-left)
            glTexCoord2f(texX0, texY0);
            glVertex3f(x0, y, z0);

            // Second vertex (top-right)
            glTexCoord2f(texX1, texY0);
            glVertex3f(x1, y, z0);

            // Third vertex (bottom-right)
            glTexCoord2f(texX1, texY1);
            glVertex3f(x1, y, z1);

            // Fourth vertex (bottom-left)
            glTexCoord2f(texX0, texY1);
            glVertex3f(x0, y, z1);

        }
    }
   glEnd();
}

/*
 *  Draw solid apple tree at (x,y,z) scaled by dx, dy, dz.
 */

static void SolidTree(double x,double y,double z,
                 double dx,double dy,double dz,
                 int winter)
{

   //  Set specular color to white
   float white[] = {1,1,1,1};
   float black[] = {0,0,0,1};
   glMaterialf(GL_FRONT_AND_BACK,GL_SHININESS,shiny);
   glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,white);
   glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION,black);

   //  Save transformation
   glPushMatrix();

   //  Offset
   glTranslated(x,y,z);
   glScaled(dx,dy,dz);


   glEnable(GL_TEXTURE_2D);
   glTexEnvi(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE, GL_MODULATE);
   
   glColor3f(1, 1, 1); //Set color to white to use the pure texture
   glBindTexture(GL_TEXTURE_2D,texture[8]); //Set texture

   
   //  Rectangular tree trunk. 
   glBegin(GL_QUADS);
   glNormal3f(0, -1, 0);
   glTexCoord2f(0, 0); glVertex3f(-0.25,0,0.25);
   glTexCoord2f(1, 0); glVertex3f(0.25,0,0.25);
   glTexCoord2f(0, 1); glVertex3f(0.25,0,-0.25);
   glTexCoord2f(1, 1); glVertex3f(-0.25,0,-0.25);
   glEnd();

   //Sides
   glBegin(GL_QUADS);
   glNormal3f(0, 0, 1);
   glTexCoord2f(1, 0); glVertex3f(0.25,0,0.25);
   glTexCoord2f(1, 20); glVertex3f(0.25,5,0.25);
   glTexCoord2f(0, 20); glVertex3f(-0.25,5,0.25);
   glTexCoord2f(0, 0); glVertex3f(-0.25,0,0.25);
   glEnd();

   glBegin(GL_QUADS);
   glNormal3f(1, 0, 0);
   glTexCoord2f(1, 0); glVertex3f(0.25,0,0.25);
   glTexCoord2f(1, 20); glVertex3f(0.25,5,0.25);
   glTexCoord2f(0, 20); glVertex3f(0.25,5,-0.25);
   glTexCoord2f(0, 0); glVertex3f(0.25,0,-0.25);
   glEnd();

   glBegin(GL_QUADS);
   glNormal3f(-1, 0, 0);
   glTexCoord2f(1, 0); glVertex3f(-0.25,0,0.25);
   glTexCoord2f(1, 20); glVertex3f(-0.25,5,0.25);
   glTexCoord2f(0, 20); glVertex3f(-0.25,5,-0.25);
   glTexCoord2f(0, 0); glVertex3f(-0.25,0,-0.25);
   glEnd();

   glBegin(GL_QUADS);
   glNormal3f(0, 0, -1);
   glTexCoord2f(1, 0); glVertex3f(0.25,0,-0.25);
   glTexCoord2f(1, 20); glVertex3f(0.25,5,-0.25);
   glTexCoord2f(0, 20); glVertex3f(-0.25,5,-0.25);
   glTexCoord2f(0, 0); glVertex3f(-0.25,0,-0.25);
   glEnd();

   //Top
   glBegin(GL_QUADS);
   glNormal3f(0, 1, 0);
   glTexCoord2f(0, 1); glVertex3f(-0.25,5,0.25);
   glTexCoord2f(1, 1); glVertex3f(0.25,5,0.25);
   glTexCoord2f(1, 0); glVertex3f(0.25,5,-0.25);
   glTexCoord2f(0, 0); glVertex3f(-0.25,5,-0.25);
   glEnd();

   glDisable(GL_TEXTURE_2D); //Clean up to let spheres handle own texturing

   //Foliage. Spheres handle their own normals
   glPushMatrix();
   glScaled(0.5,1,0.5);
   sphere(0, 5, 0, 3, winter);
   glPopMatrix();

   //Generate fruit if it's not winter.
   if(winter != 1){
      sphere(0.3, 2, 0.3, 0.1, 2);
      sphere(-0.5, 2.1, 0.5, 0.2, 2);
      sphere(-0.75, 3, -0.75, 0.25, 2);
      sphere(0.75, 3, -0.75, 0.25, 2);
      sphere(-0.75, 3, 0.72, 0.26, 2);
      sphere(0.73, 3.1, 0.75, 0.25, 2);
   }

   glPopMatrix();
}

/*
 *  Draw solid house
 *    at (x,y,z)
 *    scaled by dx, dy, dz,
 *    rotated by th,
 *    with planter planters (max of 2 planters)
 *    Modified to handle lighting
 */

static void SolidHouse(double x,double y,double z,
                 double dx,double dy,double dz,
                 double th, 
                 double houser, double houseg, double houseb,
                 int planter, int winter)
{
   //  Set specular color to white
   float white[] = {1,1,1,1};
   float black[] = {0,0,0,1};
   glMaterialf(GL_FRONT_AND_BACK,GL_SHININESS,shiny);
   glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,white);
   glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION,black);

   //  Save transformation
   glPushMatrix();
   //  Offset
   glTranslated(x,y,z);
   glRotated(th,0,1,0);
   glScaled(dx,dy,dz);
   glTranslated(0,1,0);

   //  Enable textures
   glEnable(GL_TEXTURE_2D);
   glTexEnvi(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_MODULATE);

   //  Front of house
   glColor3f(houser, houseg, houseb);
   glBindTexture(GL_TEXTURE_2D,texture[2]);
   glNormal3f(0, 0, 1);

   glBegin(GL_QUADS);
   glTexCoord2f(16.0, 0.0); glVertex3f(2,-1, 2);
   glTexCoord2f(16.0, 8.0); glVertex3f(2,1, 2);
   glTexCoord2f(0.0, 8.0); glVertex3f(-2,1, 2);
   glTexCoord2f(0.0, 0.0); glVertex3f(-2,-1, 2);

   glEnd();

   //  Back
   glColor3f(houser, houseg, houseb);
   glNormal3f(0, 0, -1);
   glBegin(GL_QUADS);
   glTexCoord2f(16.0, 0.0); glVertex3f(+2,-1,-2);
   glTexCoord2f(16.0, 8.0); glVertex3f(+2,+1,-2);
   glTexCoord2f(0.0, 8.0); glVertex3f(-2,+1,-2);
   glTexCoord2f(0.0, 0.0); glVertex3f(-2,-1,-2);

   glEnd();

   //  Right
   glNormal3f(1, 0, 0);
   glBegin(GL_QUADS);
   glTexCoord2f(16.0, 0.0); glVertex3f(+2,-1,+2);
   glTexCoord2f(16.0, 8.0); glVertex3f(+2,+1,+2);
   glTexCoord2f(0.0, 8.0); glVertex3f(+2,+1,-2);
   glTexCoord2f(0.0, 0.0); glVertex3f(+2,-1,-2);
   glEnd();
   //  Left
   glNormal3f(-1, 0, 0);
   glBegin(GL_QUADS);
   glTexCoord2f(16.0, 0.0); glVertex3f(-2,-1,+2);
   glTexCoord2f(16.0, 8.0); glVertex3f(-2,+1,+2);
   glTexCoord2f(0.0, 8.0); glVertex3f(-2,+1,-2);
   glTexCoord2f(0.0, 0.0); glVertex3f(-2,-1,-2);

   glEnd();
   //Roof triangle
   glNormal3f(0, 0, 1);
   glBegin(GL_QUADS);
   glTexCoord2f(0.0, 0.0); glVertex3f(-2,+1,+2);
   glTexCoord2f(16.0, 0.0); glVertex3f(+2,+1,+2);
   glTexCoord2f(8.0, 4.0); glVertex3f(0,+2,+2);
   glTexCoord2f(8.0, 4.0); glVertex3f(0,+2,+2);
   glEnd();
   //Roof triangle 2
   glNormal3f(0, 0, -1);
   glBegin(GL_QUADS);
   glTexCoord2f(16.0, 0.0); glVertex3f(+2,+1,-2);
   glTexCoord2f(0.0, 0.0); glVertex3f(-2,+1,-2);
   glTexCoord2f(8.0, 4.0); glVertex3f(0,+2,-2);
   glTexCoord2f(8.0, 4.0); glVertex3f(0,+2,-2);
   glEnd();
   //  Bottom
   
   glNormal3f(0, -1, 0);
   glBegin(GL_QUADS);
   glTexCoord2f(0.0, 0.0); glVertex3f(-2,-1,-2);
   glTexCoord2f(16.0, 0.0); glVertex3f(+2,-1,-2);
   glTexCoord2f(16.0, 16.0); glVertex3f(+2,-1,+2);
   glTexCoord2f(0.0, 16.0); glVertex3f(-2,-1,+2);
   glEnd();

   // Offset doors and windows
   // We push the doors/windows towards camera instead of the wall away
   // To prevent strange lighting interactions
   glEnable(GL_POLYGON_OFFSET_FILL);

   if(night){
      glDisable(GL_LIGHTING);
   }

   //Window 1
   glPolygonOffset(-1,-1);

   glColor3f(1, 1, 1); //Ensure textures show up right
   glBindTexture(GL_TEXTURE_2D,texture[3]);
   glNormal3f(0, 0, 1);
   
   glBegin(GL_QUADS);
   glTexCoord2f(0.0, 0.0); glVertex3f(-1.5, -0.5, 2);
   glTexCoord2f(0.0, 4.0); glVertex3f(-1.5,0.5, 2);
   glTexCoord2f(1.0, 4.0); glVertex3f(-1,0.5, 2);
   glTexCoord2f(1.0, 0.0); glVertex3f(-1,-0.5, 2);
   glEnd();

   //Window 2
   glNormal3f(0, 0, 1);
   glBegin(GL_QUADS);
   glTexCoord2f(1.0, 0.0); glVertex3f(1.5, -0.5, 2);
   glTexCoord2f(1.0, 4.0); glVertex3f(1.5,0.5, 2);
   glTexCoord2f(0.0, 4.0); glVertex3f(1,0.5, 2);
   glTexCoord2f(0.0, 0.0); glVertex3f(1,-0.5, 2);
   glEnd();

   glEnable(GL_LIGHTING);

   //Door (front)
   glBindTexture(GL_TEXTURE_2D,texture[0]);
   glNormal3f(0, 0, 1);
   glBegin(GL_QUADS);
   glTexCoord2f(0.0, 0.0); glVertex3f(-0.5,-1,+2);
   glTexCoord2f(0.0, 1.0); glVertex3f(-0.5,+0,+2);
   glTexCoord2f(1.0, 1.0); glVertex3f(+0.5,+0,+2);
   glTexCoord2f(1.0, 0.0); glVertex3f(+0.5,-1,+2);
   glEnd();
   glDisable(GL_POLYGON_OFFSET_FILL);


   // Roof panels. 
   // Set color to white as we do not want modulation
   // to change our roof color
   glColor3f(1, 1, 1);
   if(winter == 1){
      glBindTexture(GL_TEXTURE_2D,texture[7]);
   }
   else{
      glBindTexture(GL_TEXTURE_2D,texture[1]);
   }
   glNormal3f(1, 2, 0);
   glBegin(GL_QUADS);
   glTexCoord2f(0.0, 0.0); glVertex3f(+2,+1,-2);
   glTexCoord2f(0.0, 6.0); glVertex3f(0,+2,-2);
   glTexCoord2f(16.0, 6.0); glVertex3f(0,+2,+2);
   glTexCoord2f(16.0, 0.0); glVertex3f(+2,+1,+2);
   glEnd();


   //Roof Panel 2
   glNormal3f(-1, 2, 0);
   glBegin(GL_QUADS);
   glTexCoord2f(0.0, 0.0); glVertex3f(-2,+1,-2);
   glTexCoord2f(0.0, 6.0); glVertex3f(0,+2,-2);
   glTexCoord2f(16.0, 6.0); glVertex3f(0,+2,+2);
   glTexCoord2f(16.0, 0.0); glVertex3f(-2,+1,+2);
   glEnd();


   //Smokestack will use the roof's brick texture
   glColor3f(1, 1, 1);
   glBindTexture(GL_TEXTURE_2D,texture[1]);

   //Smokestack side 1
   glNormal3f(-1, 0, 0);
   glBegin(GL_QUADS);
   glTexCoord2f(0.0, 0.0); glVertex3f(+0.25,+1,+0.25);
   glTexCoord2f(1.0, 0.0); glVertex3f(+0.25,+1,+0.5);
   glTexCoord2f(1.0, 2.0); glVertex3f(+0.25,+2.5,+0.5);
   glTexCoord2f(0.0, 2.0); glVertex3f(+0.25,+2.5,+0.25);
   glEnd();

   //Smokestack side 2
   glNormal3f(1, 0, 0);
   glBegin(GL_QUADS);
   glTexCoord2f(0.0, 0.0); glVertex3f(+0.5,+1,+0.25);
   glTexCoord2f(1.0, 0.0); glVertex3f(+0.5,+1,+0.5);
   glTexCoord2f(1.0, 2.0); glVertex3f(+0.5,+2.5,+0.5);
   glTexCoord2f(0.0, 2.0); glVertex3f(+0.5,+2.5,+0.25);
   glEnd();

   //Smokestack side 3
   glNormal3f(0, 0, -1);
   glBegin(GL_QUADS);
   glTexCoord2f(1.0, 0.0); glVertex3f(+0.5,+1,+0.25);
   glTexCoord2f(0.0, 0.0); glVertex3f(+0.25,+1,+0.25);
   glTexCoord2f(0.0, 2.0); glVertex3f(+0.25,+2.5,+0.25);
   glTexCoord2f(1.0, 2.0); glVertex3f(+0.5,+2.5,+0.25);
   glEnd();

   //Smokestack side 4
   glNormal3f(0, 0, 1);
   glBegin(GL_QUADS);
   glTexCoord2f(0.0, 0.0); glVertex3f(+0.25,+1,+0.5);
   glTexCoord2f(1.0, 0.0); glVertex3f(+0.5,+1,+0.5);
   glTexCoord2f(1.0, 2.0); glVertex3f(+0.5,+2.5,+0.5);
   glTexCoord2f(0.0, 2.0); glVertex3f(+0.25,+2.5,+0.5);
   glEnd();

   //Construct planters if needed. As we want the true texture color we will not switch colors.

   if(planter == 1 || planter == 2){
      glColor3f(1, 1, 1);
      glBindTexture(GL_TEXTURE_2D,texture[0]); //Bind texture (reusing the door texture)

      //front of first planter 
      glNormal3f(0, 0, 1);
      glBegin(GL_QUADS);
      glTexCoord2f(0.0, 0.0); glVertex3f(-2,-1,2.5);
      glTexCoord2f(0.0, 6.0); glVertex3f(-0.5,-1,2.5);
      glTexCoord2f(1.0, 6.0); glVertex3f(-0.5,-0.75,2.5);
      glTexCoord2f(1.0, 0.0); glVertex3f(-2,-0.75,2.5);
      glEnd();

      //sides of first planter
      glNormal3f(-1, 0, 0);
      glBegin(GL_QUADS);
      glTexCoord2f(0.0, 0.0); glVertex3f(-2,-1,2);
      glTexCoord2f(0.0, 2.0); glVertex3f(-2,-1,2.5);
      glTexCoord2f(1.0, 2.0); glVertex3f(-2,-0.75,2.5);
      glTexCoord2f(1.0, 0.0); glVertex3f(-2,-0.75,2);
      glEnd();

      glNormal3f(1, 0, 0);
      glBegin(GL_QUADS);
      glTexCoord2f(0.0, 0.0); glVertex3f(-0.5,-1,2);
      glTexCoord2f(0.0, 2.0); glVertex3f(-0.5,-1,2.5);
      glTexCoord2f(1.0, 2.0); glVertex3f(-0.5,-0.75,2.5);
      glTexCoord2f(1.0, 0.0); glVertex3f(-0.5,-0.75,2);
      glEnd();

      //bottom of first planter
      glNormal3f(0, -1, 0);
      glBegin(GL_QUADS);
      glTexCoord2f(2.0, 0.0); glVertex3f(-2,-1,2.5);
      glTexCoord2f(2.0, 6.0); glVertex3f(-0.5,-1,2.5);
      glTexCoord2f(0.0, 6.0); glVertex3f(-0.5,-1,2);
      glTexCoord2f(0.0, 0.0); glVertex3f(-2,-1,2);
      glEnd();

      //top of first planter
      if(winter == 1){
         glBindTexture(GL_TEXTURE_2D,texture[7]);

      }
      else{
         glBindTexture(GL_TEXTURE_2D,texture[6]);
      }

      glNormal3f(0, 1, 0);
      glBegin(GL_QUADS);
      glTexCoord2f(0.0, 1.0); glVertex3f(-2,-0.75,2.5);
      glTexCoord2f(3.0, 1.0); glVertex3f(-0.5,-0.75,2.5);
      glTexCoord2f(3.0, 0.0); glVertex3f(-0.5,-0.75,2);
      glTexCoord2f(0.0, 0.0); glVertex3f(-2,-0.75,2);
      glEnd();

   }
   if(planter == 2){
      glBindTexture(GL_TEXTURE_2D,texture[0]); //Bind texture (reusing the door texture)

      //front of 2nd planter
      glNormal3f(0, 0, 1);
      glBegin(GL_QUADS);
      glTexCoord2f(0.0, 0.0); glVertex3f(2,-1,2.5);
      glTexCoord2f(0.0, 6.0); glVertex3f(0.5,-1,2.5);
      glTexCoord2f(1.0, 6.0); glVertex3f(0.5,-0.75,2.5);
      glTexCoord2f(1.0, 0.0); glVertex3f(2,-0.75,2.5);
      glEnd();

      //sides of 2nd planter
      glNormal3f(1, 0, 0);
      glBegin(GL_QUADS);
      glTexCoord2f(0.0, 0.0); glVertex3f(2,-1,2);
      glTexCoord2f(0.0, 2.0); glVertex3f(2,-1,2.5);
      glTexCoord2f(1.0, 2.0); glVertex3f(2,-0.75,2.5);
      glTexCoord2f(1.0, 0.0); glVertex3f(2,-0.75,2);
      glEnd();

      glNormal3f(-1, 0, 0);
      glBegin(GL_QUADS);
      glTexCoord2f(0.0, 0.0); glVertex3f(0.5,-1,2);
      glTexCoord2f(0.0, 2.0); glVertex3f(0.5,-1,2.5);
      glTexCoord2f(1.0, 2.0); glVertex3f(0.5,-0.75,2.5);
      glTexCoord2f(1.0, 0.0); glVertex3f(0.5,-0.75,2);
      glEnd();

      //bottom of 2nd planter
      glNormal3f(0, -1, 0);
      glBegin(GL_QUADS);
      glTexCoord2f(2.0, 6.0); glVertex3f(2,-1,2.5);
      glTexCoord2f(2.0, 0.0); glVertex3f(0.5,-1,2.5);
      glTexCoord2f(0.0, 0.0); glVertex3f(0.5,-1,2);
      glTexCoord2f(0.0, 6.0); glVertex3f(2,-1,2);
      glEnd();

      //top of 2nd planter

      if(winter == 1){
         glBindTexture(GL_TEXTURE_2D,texture[7]);

      }
      else{
         glBindTexture(GL_TEXTURE_2D,texture[6]);
      }
      glNormal3f(0, 1, 0);
      glBegin(GL_QUADS);
      glTexCoord2f(0.0, 1.0); glVertex3f(2,-0.75,2.5);
      glTexCoord2f(3.0, 1.0); glVertex3f(0.5,-0.75,2.5);
      glTexCoord2f(3.0, 0.0); glVertex3f(0.5,-0.75,2);
      glTexCoord2f(0.0, 0.0); glVertex3f(2,-0.75,2);
      glEnd();

   }

   // Smokestack top. 
   // Lighting and textures DO NOT APPLY as this is a hole
   // So we do not want light effects or textures here.
   glColor3f(0, 0, 0);
   glNormal3f(0, 1, 0);
   glDisable(GL_LIGHTING);
   glBegin(GL_QUADS);
   glVertex3f(+0.25,+2.5,+0.25);
   glVertex3f(0.25,+2.5,+0.5);
   glVertex3f(+0.5,+2.5,+0.5);
   glVertex3f(0.5,+2.5,+0.25);
   glEnd();
   glEnable(GL_LIGHTING);


   //Draw bushes (spheres)
   if(planter == 1 || planter == 2){
      sphere(-1.625, -0.75, 2.25, 0.25, winter);
      sphere(-1.25,-0.75, 2.25, 0.25, winter);
      sphere(-0.875,-0.75, 2.25, 0.25, winter);
   }
   if(planter == 2){
      sphere(1.625, -0.75, 2.25, 0.25, winter);
      sphere(1.25,-0.75, 2.25, 0.25, winter);
      sphere(0.875,-0.75, 2.25, 0.25, winter);
   }
   //  Undo transformations
   glPopMatrix();
   glDisable(GL_TEXTURE_2D);
}

// 1x1 block of Intersection. 
static void IntersectionBlock(double x,double y,double z,
                  double dx, double dy, double dz)
{

   //  Set specular color to white (based on examples)
   float white[] = {1,1,1,1};
   float black[] = {0,0,0,1};
   glMaterialf(GL_FRONT_AND_BACK,GL_SHININESS,shiny);
   glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,white);
   glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION,black);

   //  Save transformation
   glPushMatrix();
   //  Offset
   glTranslated(x,y,z);
   glScaled(dx,dy,dz);

   //  Enable textures
   glEnable(GL_TEXTURE_2D);
   glTexEnvi(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_MODULATE);
   glColor3f(1,1,1); //Use true texture color

   glBindTexture(GL_TEXTURE_2D,texture[18]);
   //Set normal vector
   glNormal3f( 0,+1, 0);
   //Build object

   //Top
   drawSplitQuad(1, 1, 1);

   glBindTexture(GL_TEXTURE_2D,texture[4]);


   //  Bottom
   glNormal3f( 0,-1, 0);
   drawSplitQuad(0, 1, 1);


   //  Right
   glNormal3f(+1, 0, 0);
   glBegin(GL_QUADS);
   glTexCoord2f(0, 0); glVertex3f(+0.5,0,+0.5);
   glTexCoord2f(32, 0); glVertex3f(+0.5,0,-0.5);
   glTexCoord2f(0, 32); glVertex3f(+0.5,+1,-0.5);
   glTexCoord2f(32, 32); glVertex3f(+0.5,+1,+0.5);
   glEnd();

   //  Left
   glBegin(GL_QUADS);
   glNormal3f(-1, 0, 0);
   glTexCoord2f(0, 0); glVertex3f(-0.5,0,-0.5);
   glTexCoord2f(0, 32); glVertex3f(-0.5,0,+0.5);
   glTexCoord2f(32, 32); glVertex3f(-0.5,1,+0.5);
   glTexCoord2f(32, 0); glVertex3f(-0.5,1,-0.5);
   glEnd();

   //  Front

   glNormal3f(0, 0, +1);
   glBegin(GL_QUADS);
   glTexCoord2f(0, 0); glVertex3f(+0.5,0,+0.5);
   glTexCoord2f(32, 0); glVertex3f(-0.5,0,+0.5);
   glTexCoord2f(32, 32); glVertex3f(-0.5,1,+0.5);
   glTexCoord2f(0, 32); glVertex3f(+0.5,1,+0.5);
   glEnd();

   //  Back
   glNormal3f(0, 0, -1);
   glBegin(GL_QUADS);
   glTexCoord2f(0, 0); glVertex3f(-0.5,0,-0.5);
   glTexCoord2f(32, 0); glVertex3f(+0.5,0,-0.5);
   glTexCoord2f(32, 32); glVertex3f(+0.5,1,-0.5);
   glTexCoord2f(0, 32); glVertex3f(-0.5,1,-0.5);
   glEnd();

   //  Undo transformations
   glPopMatrix();
   glDisable(GL_TEXTURE_2D);
}

// 1x1 block of Road
static void RoadBlock(double x,double y,double z,
                  double dx, double dy, double dz,
                 double th)
{


   //  Set specular color to white (based on examples)
   float white[] = {1,1,1,1};
   float black[] = {0,0,0,1};
   glMaterialf(GL_FRONT_AND_BACK,GL_SHININESS,shiny);
   glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,white);
   glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION,black);

   //  Save transformation
   glPushMatrix();
   //  Offset
   glTranslated(x,y,z);
   glRotated(th,0,1,0);
   glScaled(dx,dy,dz);   


   //  Enable textures
   glEnable(GL_TEXTURE_2D);
   glTexEnvi(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_MODULATE);
   glColor3f(1,1,1); //Use true texture color

   glBindTexture(GL_TEXTURE_2D,texture[10]);
   //Set normal vector
   glNormal3f( 0,+1, 0);
   //Build object

   //Top
   drawSplitQuad(1, 1, 1);


   glBindTexture(GL_TEXTURE_2D,texture[4]);


   //  Bottom
   glNormal3f( 0,-1, 0);
   drawSplitQuad(0, 32, 32);


   //  Right
   glNormal3f(+1, 0, 0);
   glBegin(GL_QUADS);
   glTexCoord2f(0, 0); glVertex3f(+0.5,0,+0.5);
   glTexCoord2f(32, 0); glVertex3f(+0.5,0,-0.5);
   glTexCoord2f(0, 32); glVertex3f(+0.5,+1,-0.5);
   glTexCoord2f(32, 32); glVertex3f(+0.5,+1,+0.5);
   glEnd();

   //  Left
   glBegin(GL_QUADS);
   glNormal3f(-1, 0, 0);
   glTexCoord2f(0, 0); glVertex3f(-0.5,0,-0.5);
   glTexCoord2f(0, 32); glVertex3f(-0.5,0,+0.5);
   glTexCoord2f(32, 32); glVertex3f(-0.5,1,+0.5);
   glTexCoord2f(32, 0); glVertex3f(-0.5,1,-0.5);
   glEnd();

   //  Front

   glNormal3f(0, 0, +1);
   glBegin(GL_QUADS);
   glTexCoord2f(0, 0); glVertex3f(+0.5,0,+0.5);
   glTexCoord2f(32, 0); glVertex3f(-0.5,0,+0.5);
   glTexCoord2f(32, 32); glVertex3f(-0.5,1,+0.5);
   glTexCoord2f(0, 32); glVertex3f(+0.5,1,+0.5);
   glEnd();

   //  Back
   glNormal3f(0, 0, -1);
   glBegin(GL_QUADS);
   glTexCoord2f(0, 0); glVertex3f(-0.5,0,-0.5);
   glTexCoord2f(32, 0); glVertex3f(+0.5,0,-0.5);
   glTexCoord2f(32, 32); glVertex3f(+0.5,1,-0.5);
   glTexCoord2f(0, 32); glVertex3f(-0.5,1,-0.5);
   glEnd();

   //  Undo transformations
   glPopMatrix();
   glDisable(GL_TEXTURE_2D);
}

// Default town base block. 1x1. 
static void GrassBlock(double x,double y,double z,
                  double dx, double dy, double dz,
                 int winter)
{


   //  Set specular color to white (based on examples)
   float white[] = {1,1,1,1};
   float black[] = {0,0,0,1};
   glMaterialf(GL_FRONT_AND_BACK,GL_SHININESS,shiny);
   glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,white);
   glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION,black);

   //  Save transformation
   glPushMatrix();
   //  Offset
   glTranslated(x,y,z);
   glScaled(dx,dy,dz);

   //  Enable textures
   glEnable(GL_TEXTURE_2D);
   glTexEnvi(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_MODULATE);
   glColor3f(1,1,1); //Use true texture color
   //  Top
   if(winter == 1){
      glBindTexture(GL_TEXTURE_2D,texture[7]);
   }
   else{
      glBindTexture(GL_TEXTURE_2D,texture[6]);
   }
   //Set normal vector
   glNormal3f( 0,+1, 0);
   //Build object

   drawSplitQuad(1, 32, 32);

   glBindTexture(GL_TEXTURE_2D,texture[4]);

   //  Bottom
   glNormal3f( 0,-1, 0);
   drawSplitQuad(0, 32, 32);


   //  Right
   glNormal3f(+1, 0, 0);
   glBegin(GL_QUADS);
   glTexCoord2f(0, 0); glVertex3f(+0.5,0,+0.5);
   glTexCoord2f(32, 0); glVertex3f(+0.5,0,-0.5);
   glTexCoord2f(0, 32); glVertex3f(+0.5,+1,-0.5);
   glTexCoord2f(32, 32); glVertex3f(+0.5,+1,+0.5);
   glEnd();

   //  Left
   glBegin(GL_QUADS);
   glNormal3f(-1, 0, 0);
   glTexCoord2f(0, 0); glVertex3f(-0.5,0,-0.5);
   glTexCoord2f(0, 32); glVertex3f(-0.5,0,+0.5);
   glTexCoord2f(32, 32); glVertex3f(-0.5,1,+0.5);
   glTexCoord2f(32, 0); glVertex3f(-0.5,1,-0.5);
   glEnd();

   //  Front

   glNormal3f(0, 0, +1);
   glBegin(GL_QUADS);
   glTexCoord2f(0, 0); glVertex3f(+0.5,0,+0.5);
   glTexCoord2f(32, 0); glVertex3f(-0.5,0,+0.5);
   glTexCoord2f(32, 32); glVertex3f(-0.5,1,+0.5);
   glTexCoord2f(0, 32); glVertex3f(+0.5,1,+0.5);
   glEnd();

   //  Back
   glNormal3f(0, 0, -1);
   glBegin(GL_QUADS);
   glTexCoord2f(0, 0); glVertex3f(-0.5,0,-0.5);
   glTexCoord2f(32, 0); glVertex3f(+0.5,0,-0.5);
   glTexCoord2f(32, 32); glVertex3f(+0.5,1,-0.5);
   glTexCoord2f(0, 32); glVertex3f(-0.5,1,-0.5);
   glEnd();

   //  Undo transformations
   glPopMatrix();
   glDisable(GL_TEXTURE_2D);
}

//Monomaterial block. Helper function to draw single-material chunks. 
static void MonoBlock(){
     //Top
   glNormal3f( 0,+1, 0);
   drawSplitQuad(1, 4, 4);

   //  Bottom
   glNormal3f( 0,-1, 0);
   drawSplitQuad(0, 1, 1);

   //  Right
   glNormal3f(+1, 0, 0);
   glBegin(GL_QUADS);
   glTexCoord2f(0, 0); glVertex3f(+0.5,0,+0.5);
   glTexCoord2f(32, 0); glVertex3f(+0.5,0,-0.5);
   glTexCoord2f(0, 32); glVertex3f(+0.5,+1,-0.5);
   glTexCoord2f(32, 32); glVertex3f(+0.5,+1,+0.5);
   glEnd();

   //  Left
   glBegin(GL_QUADS);
   glNormal3f(-1, 0, 0);
   glTexCoord2f(0, 0); glVertex3f(-0.5,0,-0.5);
   glTexCoord2f(0, 4); glVertex3f(-0.5,0,+0.5);
   glTexCoord2f(4, 4); glVertex3f(-0.5,1,+0.5);
   glTexCoord2f(4, 0); glVertex3f(-0.5,1,-0.5);
   glEnd();

   //  Front

   glNormal3f(0, 0, +1);
   glBegin(GL_QUADS);
   glTexCoord2f(0, 0); glVertex3f(+0.5,0,+0.5);
   glTexCoord2f(4, 0); glVertex3f(-0.5,0,+0.5);
   glTexCoord2f(4, 4); glVertex3f(-0.5,1,+0.5);
   glTexCoord2f(0, 4); glVertex3f(+0.5,1,+0.5);
   glEnd();

   //  Back
   glNormal3f(0, 0, -1);
   glBegin(GL_QUADS);
   glTexCoord2f(0, 0); glVertex3f(-0.5,0,-0.5);
   glTexCoord2f(4, 0); glVertex3f(+0.5,0,-0.5);
   glTexCoord2f(4, 4); glVertex3f(+0.5,1,-0.5);
   glTexCoord2f(0, 4); glVertex3f(-0.5,1,-0.5);
   glEnd();
}

//Block that represents sand
static void SandBlock(double x,double y,double z,
                  double dx, double dy, double dz){
   //  Set specular color to white (based on examples)
   float white[] = {1,1,1,1};
   float black[] = {0,0,0,1};
   glMaterialf(GL_FRONT_AND_BACK,GL_SHININESS,shiny);
   glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,white);
   glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION,black);

   //  Save transformation
   glPushMatrix();
   //  Offset
   glTranslated(x,y,z);
   glScaled(dx,dy,dz);

   //  Enable textures
   glEnable(GL_TEXTURE_2D);
   glTexEnvi(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_MODULATE);
   glColor3f(1,0.870,0.590); //Use true texture color
   //  Top
   glBindTexture(GL_TEXTURE_2D,texture[7]); //Use snow texture and new color

   MonoBlock();

   //  Undo transformations
   glPopMatrix();
   glDisable(GL_TEXTURE_2D);
 
}

//Block that represents water. Semi-transparent.
static void WaterBlock(double x,double y,double z,
                  double dx, double dy, double dz){
   //  Set specular color to white (based on examples)
   float white[] = {1,1,1,1};
   float black[] = {0,0,0,1};
   glMaterialf(GL_FRONT_AND_BACK,GL_SHININESS,shiny);
   glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,white);
   glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION,black);

   //  Save transformation
   glPushMatrix();
   //  Offset
   glTranslated(x,y,z);
   glScaled(dx,dy,dz);

   glColor4f(1,1,1, 0.5); //Use true texture color

   glEnable(GL_LIGHTING);
   glEnable(GL_BLEND);
   glBlendFunc(GL_SRC_ALPHA,GL_ONE);
   glDepthMask(0);


   //  Enable textures
   glEnable(GL_TEXTURE_2D);
   glTexEnvi(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_MODULATE);
   //  Draw water
   glBindTexture(GL_TEXTURE_2D,texture[13]); //Use water texture and new color

   MonoBlock();
   //  Undo transformations
   glPopMatrix();
   glDisable(GL_TEXTURE_2D);
}

//Block of Plastic
static void PlasticBlock(double x,double y,double z,
                  double dx, double dy, double dz, double th){
     //  Set specular color to white (based on examples)
   float white[] = {1,1,1,1};
   float black[] = {0,0,0,1};
   glMaterialf(GL_FRONT_AND_BACK,GL_SHININESS,shiny);
   glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,white);
   glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION,black);

   //  Save transformation
   glPushMatrix();
   //  Offset
   glTranslated(x,y,z);
   glRotated(th, 0, 1, 0);
   glScaled(dx,dy,dz);

   //  Enable textures
   glEnable(GL_TEXTURE_2D);
   glTexEnvi(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_MODULATE);
   glColor3f(1,1,1); //Use true texture color

   glBindTexture(GL_TEXTURE_2D,texture[11]);

   MonoBlock();

   //  Undo transformations
   glPopMatrix();

   glDisable(GL_TEXTURE_2D);
}

//A sphere that is either semi-transparent or has no lighting depending on if it's "on" or not. 
//Comes in pre-defined colors.
static void Spotlight(double x, double y, double z, double dx, double dy, double dz, int on, int color){
   glPushMatrix();
   //  Offset, scale and rotate
   glTranslated(x,y,z);
   glScaled(dx,dy,dz);
   //  White ball with yellow specular
   float yellow[]   = {1.0,1.0,0.0,1.0};
   float Emission[] = {0.0,0.0,0.01*emission,1.0};

   //  Enable textures
   glEnable(GL_TEXTURE_2D);
   glTexEnvi(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_MODULATE);


   glMaterialf(GL_FRONT,GL_SHININESS,shiny);
   glMaterialfv(GL_FRONT,GL_SPECULAR,yellow);
   glMaterialfv(GL_FRONT,GL_EMISSION,Emission);
   // If it's on, make it opaque, otherwise make it translucent.
   // As this is a light, it uses the "glass" texture of the windows
   glBindTexture(GL_TEXTURE_2D,texture[3]);

   if(!on){
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA,GL_ONE);
      glDepthMask(0);
   }
   else{
      glDisable(GL_LIGHTING);
   }

   if(color == 1){ //Red Light
      glColor4f(1,0,0, 0.9);
   }
   else if (color == 2){ //Yellow Light
      glColor4f(1,1,0, 0.9);
   }
   else if (color == 3){ //Red Light
      glColor4f(0,1,0, 0.9);
   }
   else{ //White Light
      glColor4f(1,1,1, 0.9);
   }

   //  Bands of latitude
   for (int ph=-90;ph<90;ph+=inc)
   {
      glBegin(GL_QUAD_STRIP);
      for (int th=0;th<=360;th+=2*inc)
      {
         Vertex(th,ph);
         Vertex(th,ph+inc);
      }
      glEnd();
   }
   glDisable(GL_BLEND);
   glEnable(GL_LIGHTING);
   glDepthMask(1);
   //  Undo transofrmations
   glPopMatrix();
}

static void MetalBlock(double x,double y,double z,
                  double dx, double dy, double dz){

   //  Set specular color to white (based on examples)
   float white[] = {1,1,1,1};
   float black[] = {0,0,0,1};
   glMaterialf(GL_FRONT_AND_BACK,GL_SHININESS,shiny);
   glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,white);
   glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION,black);

   //  Save transformation
   glPushMatrix();
   //  Offset
   glTranslated(x,y,z);
   glScaled(dx,dy,dz);

   //  Enable textures
   glEnable(GL_TEXTURE_2D);
   glTexEnvi(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_MODULATE);
   glColor3f(0.5,0.5,0.5); //Darken

   //Bind texture and draw
   glBindTexture(GL_TEXTURE_2D,texture[24]);
   MonoBlock();

   //  Undo transformations
   glPopMatrix();
   glDisable(GL_TEXTURE_2D);
}

// The frame of the Streetlight
static void StreetLight(double x, double y, double z, double dx, double dy, double dz, double th){

   //  Save transformation
   glPushMatrix();
   //  Offset
   glTranslated(x,y,z);
   glRotated(th, 0, 1, 0);
   glScaled(dx,dy,dz);

   MetalBlock(0, 0, 0, 0.5, 0.1, 0.5);
   MetalBlock(0, 0.1, 0, 0.1, 1.9, 0.1);
   MetalBlock(0, 2, 0.4, 0.1, 0.05, 1);

   glPopMatrix();
}

// The "light" of the streetlight. Rendered before other transparent things, since this can be opaque at night.
static void StreetLight2(double x, double y, double z, double dx, double dy, double dz, double th){

   //  Save transformation
   glPushMatrix();
   //  Offset
   glTranslated(x,y,z);
   glRotated(th, 0, 1, 0);
   glScaled(dx,dy,dz);

   Spotlight(0, 2, 0.69, 0.05, 0.05, 0.2, 0, 0);

   glPopMatrix();
}

// The frame of the Streetlight
static void TrafficLight(double x, double y, double z, double dx, double dy, double dz, double th){

   //  Save transformation
   glPushMatrix();
   //  Offset
   glTranslated(x,y,z);
   glRotated(th, 0, 1, 0);
   glScaled(dx,dy,dz);

   MetalBlock(0, 0, 0, 0.5, 0.1, 0.5);
   MetalBlock(0, 0.1, 0, 0.1, 1.4, 0.1);
   MetalBlock(0, 1.5, 0, 0.5, 1.5, 0.5);

   glPopMatrix();
}

void updateTrafficLights(){
   //Get new state based on time
   // Check if 20 seconds have passed since the last update
   if (abs(zh - last_update_time) >= 200) {
      // Update the last update time
      last_update_time = zh;

      // Increment the traffic light state using modulo to cycle it between 1, 2, and 3
      trafficState = (trafficState % 3) + 1;
   }
}


// The lights of the TrafficLight. 
static void TrafficLight2(double x, double y, double z, double dx, double dy, double dz, double th){

   //  Save transformation
   glPushMatrix();
   //  Offset
   glTranslated(x,y,z);
   glRotated(th, 0, 1, 0);
   glScaled(dx,dy,dz);

   switch(trafficState){
      case 1:
         Spotlight(0, 1.75, 0.1, 0.25, 0.25, 0.25, 1, 3);
         Spotlight(0, 2.25, 0.1, 0.25, 0.25, 0.25, 0, 2);
         Spotlight(0, 2.75, 0.1, 0.25, 0.25, 0.25, 0, 1);
         break;
      case 2:
         Spotlight(0, 1.75, 0.1, 0.25, 0.25, 0.25, 0, 3);
         Spotlight(0, 2.25, 0.1, 0.25, 0.25, 0.25, 1, 2);
         Spotlight(0, 2.75, 0.1, 0.25, 0.25, 0.25, 0, 1);
         break;
      case 3:
         Spotlight(0, 1.75, 0.1, 0.25, 0.25, 0.25, 0, 3);
         Spotlight(0, 2.25, 0.1, 0.25, 0.25, 0.25, 0, 2);
         Spotlight(0, 2.75, 0.1, 0.25, 0.25, 0.25, 1, 1);
         break;
   }

   glPopMatrix();
}

static void ShirtBlock(double x,double y,double z,
                  double dx, double dy, double dz, double th){
     //  Set specular color to white (based on examples)
   float white[] = {1,1,1,1};
   float black[] = {0,0,0,1};
   glMaterialf(GL_FRONT_AND_BACK,GL_SHININESS,shiny);
   glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,white);
   glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION,black);

   //  Save transformation
   glPushMatrix();
   //  Offset
   glTranslated(x,y,z);
   glRotated(th, 0, 1, 0);
   glScaled(dx,dy,dz);

   //  Enable textures
   glEnable(GL_TEXTURE_2D);
   glTexEnvi(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_MODULATE);
   glColor3f(1,1,1); //Use true texture color

   glBindTexture(GL_TEXTURE_2D,texture[22]);

   MonoBlock();
   //  Undo transformations
   glPopMatrix();

   glDisable(GL_TEXTURE_2D);
}

static void PantBlock(double x,double y,double z,
                  double dx, double dy, double dz, double th){
     //  Set specular color to white (based on examples)
   float white[] = {1,1,1,1};
   float black[] = {0,0,0,1};
   glMaterialf(GL_FRONT_AND_BACK,GL_SHININESS,shiny);
   glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,white);
   glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION,black);

   //  Save transformation
   glPushMatrix();
   //  Offset
   glTranslated(x,y,z);
   glRotated(th, 0, 1, 0);
   glScaled(dx,dy,dz);

   //  Enable textures
   glEnable(GL_TEXTURE_2D);
   glTexEnvi(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_MODULATE);
   glColor3f(1,1,1); //Use true texture color

   glBindTexture(GL_TEXTURE_2D,texture[23]);

   MonoBlock();

   //  Undo transformations
   glPopMatrix();

   glDisable(GL_TEXTURE_2D);
}

// Head of the man. It's a "camera" because it's a convenient place to put a light.
static void CameraHead(){
   glPushMatrix();
   glTranslated(0,2.45,0);

   double Cth = -headTh;

   glRotated(Cth, 0, 1, 0);
   glScaled(1,1,1);
   MetalBlock(0, -0.15, 0, 0.3, 0.3, 0.6);
   MetalBlock(0, 0.15, 0.05, 0.3, 0.05, 0.7);

   Spotlight(0, 0, 0.3, 0.15, 0.15, 0.05, 1, 0);
   glPopMatrix();
}

//Screen of the man's tablet.
static void GlowScreen(double x,double y,double z,
                  double dx, double dy, double dz){

   //  Set specular color to white (based on examples)
   float white[] = {1,1,1,1};
   float black[] = {0,0,0,1};
   glMaterialf(GL_FRONT_AND_BACK,GL_SHININESS,shiny);
   glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,white);
   glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION,black);

   //  Save transformation
   glPushMatrix();
   //  Offset
   glTranslated(x,y,z);
   glScaled(dx,dy,dz);

   glDisable(GL_TEXTURE_2D);
   glDisable(GL_LIGHTING);
   glTexEnvi(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_MODULATE);
   glColor3f(0.6,0.6,0.9); //blue screen

   //Bind texture and draw
   glBindTexture(GL_TEXTURE_2D,texture[24]);
   MonoBlock();

   //  Undo transformations
   glPopMatrix();
   glDisable(GL_TEXTURE_2D);
   glEnable(GL_LIGHTING);

}

static void Arms(double x,double y,double z, double angle){
   //  Save transformation
   glPushMatrix();
   //  Offset
   glTranslated(x,y,z);
   glRotated(angle, 1, 0, 0);

   ShirtBlock(0.75, -0.75, 0, 0.5, 1, 0.5, 0);
   ShirtBlock(-0.75, -0.75, 0, 0.5, 1, 0.5, 0);
   PlasticBlock(0.75, -0.8, 0, 0.4, 0.1, 0.4, 0);
   PlasticBlock(-0.75, -0.8, 0, 0.4, 0.1, 0.4, 0);

   //If the drone is being controlled, hold a tablet. 
   if(controllingDrone){
      MetalBlock(0, -0.8, 0, 1, 0.05, 0.5);
      GlowScreen(0, -0.75, 0, 0.7, 0.01, 0.45);
   }

   glPopMatrix();

}

//draw the man.
static void Man(double x,double y,double z,
                  double dx, double dy, double dz, double th){
   float white[] = {1,1,1,1};
   float black[] = {0,0,0,1};
   glMaterialf(GL_FRONT_AND_BACK,GL_SHININESS,shiny);
   glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,white);
   glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION,black);

   //  Save transformation
   glPushMatrix();
   //  Offset
   glTranslated(x,y,z);
   glRotated(th, 0, 1, 0);
   glScaled(dx,dy,dz);
   PantBlock(0.3, 0, 0, 0.5, 1, 0.5, 0);
   PantBlock(-0.3, 0, 0, 0.5, 1, 0.5, 0);
   PantBlock(0, 1, 0, 1, 0.3, 0.5, 0);
   ShirtBlock(0,1.3, 0, 1, 1, 0.8, 0);
   int armAngle = (controllingDrone) ? -60:0;
   Arms(0, 2, 0, armAngle);

   CameraHead();
   glPopMatrix();
}

//The "ball" camera. 
static void CameraBall(double x, double y, double z, double r){
   //  Save transformation
   glPushMatrix();
   //  Offset, scale and rotate
   glTranslated(x,y,z);
   glScaled(r,r,r);
   float yellow[]   = {1.0,1.0,0.0,1.0};
   float Emission[] = {0.0,0.0,0.01*emission,1.0};
   glColor3f(1,1,1);
   glMaterialf(GL_FRONT,GL_SHININESS,shiny);
   glMaterialfv(GL_FRONT,GL_SPECULAR,yellow);
   glMaterialfv(GL_FRONT,GL_EMISSION,Emission);


   glEnable(GL_TEXTURE_2D);
   glBindTexture(GL_TEXTURE_2D, texture[11]);

   //  Bands of latitude
   for (int ph=-90;ph<90;ph+=inc)
   {
      glBegin(GL_QUAD_STRIP);
      for (int th=0;th<=360;th+=2*inc)
      {
         Vertex(th,ph);
         Vertex(th,ph+inc);
      }
      glEnd();
   }
   //  Undo transofrmations
   glPopMatrix();
}

//The body of the drone.
static void DroneBody(){

   double thickness = 0.10;
   
   //Draw propeller arms
   PlasticBlock(0.5, 1 - thickness, 0.5, thickness, thickness, 1.7, 45);
   PlasticBlock(-0.5, 1 - thickness, 0.5, thickness, thickness, 1.7, -45);
   PlasticBlock(0.5, 1 - thickness, -0.5, thickness, thickness, 1.7, -45);
   PlasticBlock(-0.5, 1 - thickness, -0.5, thickness, thickness, 1.7, 45);

   PlasticBlock(0, 0.5, 0, 1, 1 - thickness - 0.5, 1, 0);

   //Draw navigation lights under arms
   //Always on, so opaque.
   Spotlight(1, 1-thickness, 1, 0.05, 0.05, 0.05, 1, 1);
   Spotlight(-1, 1-thickness, 1, 0.05, 0.05, 0.05, 1, 1);
   Spotlight(1, 1-thickness, -1, 0.05, 0.05, 0.05, 1, 3);
   Spotlight(-1, 1-thickness, -1, 0.05, 0.05, 0.05, 1, 3);

   //Draw landing gear
   PlasticBlock(0.5, 0, 0.5, thickness, 1 - thickness, thickness, 45);
   PlasticBlock(-0.5, 0, 0.5, thickness, 1 - thickness, thickness, 45);
   PlasticBlock(0.5, 0, -0.5, thickness, 1 - thickness, thickness, 45);
   PlasticBlock(-0.5, 0, -0.5, thickness, 1 - thickness, thickness, 45);
}

//The propellors of the drone.
static void DronePropellors(double x,double y,double z, double r){

   //  Set specular color to white (based on examples)
   float white[] = {1,1,1,1};
   float black[] = {0,0,0,1};
   glMaterialf(GL_FRONT_AND_BACK,GL_SHININESS,shiny);
   glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,white);
   glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION,black);

   glEnable(GL_TEXTURE_2D);
   //Draw translucent propellors
   //  Save transformation
   glPushMatrix();
   //  Offset, scale and rotate
   glTranslated(x,y,z);
   glRotated(90,1,0,0); //Transform, as original code drew it vertical
   glRotated(zh,0, 0, 1); //Make it SPIN
   glScaled(r,r,r);

   glColor4f(1,1,1, 0.8); //Use true texture color

   //Two-sided lighting for the propellor. 
   glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,1);
   glBlendFunc(GL_SRC_ALPHA,GL_ONE);
   glEnable(GL_BLEND);
   glDepthMask(0);


   glBindTexture(GL_TEXTURE_2D,texture[12]);
   glNormal3f(0,0,1);
   glBegin(GL_TRIANGLE_FAN);
   glTexCoord2f(0.5,0.5);
   glVertex3f(0,0,0);
   for (int k=0;k<=360;k+=10)
   {
      glTexCoord2f(0.5*Cos(k)+0.5,0.5*Sin(k)+0.5);
      glVertex3f(1*Cos(k),Sin(k),0);
   }

   glEnd();
   

   glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,0);
   glDisable(GL_BLEND);
   glDepthMask(1);

   //  Undo transofrmations
   glPopMatrix();
}

//Renders all 4 propellors as a "set" for easier movement.
static void DronePropellorSet(double x, double y, double z, double dx, double dy, double dz, double th){


   //  Save transformation
   glPushMatrix();
   //  Offset
   glTranslated(x,y,z);
   glRotated(th, 0, 1, 0);
   glScaled(dx,dy,dz);


   //Draw all 4 propellors.
   DronePropellors(1, 1.001, 1, 0.8);
   DronePropellors(1, 1.001, -1, 0.8);
   DronePropellors(-1, 1.001, 1, 0.8);
   DronePropellors(-1, 1.001, -1, 0.8);

   glPopMatrix();

}

//Combines the camera ball with the spotlight to make the full camera. 
static void BallCam(){
   //Draw sphere with spotlight disc. Uses global variables. 
   glPushMatrix();
   glTranslated(0,0.35,0);

   double Cth = -camPh;
   double Caz = -camTh;

   glRotated(Caz, 0, 1, 0);
   glRotated(Cth, 1, 0, 0);
   glScaled(1,1,1);

   CameraBall(0, 0, 0, 0.25);
   Spotlight(0, 0, 0.20, 0.15, 0.15, 0.05, 1, 0);
   glPopMatrix();

}

//Draw the drone, minus propellors. This is because propellors are transparent. 
static void Drone(double x, double y, double z, double dx, double dy, double dz, double th){

   //  Save transformation
   glPushMatrix();
   //  Offset
   glTranslated(x,y,z);
   glRotated(th, 0, 1, 0);
   glScaled(dx,dy,dz);
   //Draw body
   DroneBody();
   
   //Draw ball camera
   BallCam();
   glPopMatrix();
}

// Draw all things in the struct library.
static void drawThing(Object thing){
   if(strcmp(thing.type, "House") == 0){
      SolidHouse(thing.x, thing.y, thing.z, thing.scale, thing.scale, thing.scale, thing.rotation, 1, 1, 1, 2, 0);
   }
   else if(strcmp(thing.type, "Tree") == 0){
       SolidTree(thing.x, thing.y, thing.z, thing.scale, thing.scale, thing.scale, 0);
   }
   else if(strcmp(thing.type, "Streetlight") == 0){
      StreetLight(thing.x, thing.y, thing.z, thing.scale, thing.scale, thing.scale, thing.rotation);
   }
   else if(strcmp(thing.type, "Streetlight2") == 0){
      StreetLight2(thing.x, thing.y, thing.z, thing.scale, thing.scale, thing.scale, thing.rotation);
   }
   else if(strcmp(thing.type, "Trafficlight") == 0){
      TrafficLight(thing.x, thing.y, thing.z, thing.scale, thing.scale, thing.scale, thing.rotation);
   }
   else if(strcmp(thing.type, "Trafficlight2") == 0){
      TrafficLight2(thing.x, thing.y, thing.z, thing.scale, thing.scale, thing.scale, thing.rotation);
   }
   else if(strcmp(thing.type, "Shrub") == 0){

   }
   else if(strcmp(thing.type, "Man") == 0){

   }
}

//Draw a block on the map
static void drawMapBlock(int type, int x, int z){
   int drawX = x - 7;
   int drawZ = z - 7;

   if(type == 0){
      SandBlock(drawX, -1, drawZ, 1, 0.5, 1);
   }
   else if (type == 1){
      GrassBlock(drawX, -1, drawZ, 1, 1, 1, 0);
   }
   else if(type == 3){
      RoadBlock(drawX, -1, drawZ, 1, 1, 1, 0);
   }
   else if(type == 4){
      RoadBlock(drawX, -1, drawZ, 1, 1, 1, 90);
   }
   else if (type == 5){
      IntersectionBlock(drawX, -1, drawZ, 1, 1, 1);
   }
   else if(type == 6){
      SandBlock(drawX, -1, drawZ, 1, 1, 1);
   }
}

//Draw a transparent block on the map
static void drawMapBlockTransparent(int type, int x, int z){
   int drawX = x - 7;
   int drawZ = z - 7;

   if(type == 0){
      WaterBlock(drawX, -0.5, drawZ, 1, 0.5, 1);
   }
}

//Draw everything.
static void drawScene(){

   //Opaque terrain loop
   for (int i = 0; i < worldRows; i++) {
        for (int j = 0; j < worldCols; j++) {
            drawMapBlock(worldMap[i][j], i, j);
        }
    }

   //Opaque things
   for(int i = 0; i < opaqueThingsSize; i++){
      drawThing(opaqueThings[i]);
   }
   Drone(droneX, droneY, droneZ, droneScale, droneScale, droneScale, droneTh);
   Man(manX, manY, manZ, manScale, manScale, manScale, manTh);
   for (int i = 0; i < worldRows; i++) {
        for (int j = 0; j < worldCols; j++) {
            drawMapBlockTransparent(worldMap[i][j], i, j);
        }
    }
   
   //Transparent things
   for(int i = 0; i < transparentThingsSize; i++){
      drawThing(transparentThings[i]);
   }
   DronePropellorSet(droneX, droneY, droneZ, droneScale, droneScale, droneScale, droneTh);
}

/*
 *  Draw the drone's UI as an overlay. Must be called last!
 */
void DroneUI()
{
   glDisable(GL_LIGHTING);
   //  Save transform attributes (Matrix Mode and Enabled Modes)
   glPushAttrib(GL_TRANSFORM_BIT|GL_ENABLE_BIT);
   //  Save projection matrix and set unit transform
   glMatrixMode(GL_PROJECTION);
   glPushMatrix();
   glLoadIdentity();
   glOrtho(-asp,+asp,-1,1,-1,1);
   //  Save model view matrix and set to indentity
   glMatrixMode(GL_MODELVIEW);
   glPushMatrix();
   glLoadIdentity();
   //  Draw instrument panels with texture
   glColor3f(1,1,1);
   glEnable(GL_TEXTURE_2D);
   glBindTexture(GL_TEXTURE_2D,texture[20]);
   glBegin(GL_QUADS);
   glTexCoord2d(0,0);glVertex2f(-asp,-1);
   glTexCoord2d(1,0);glVertex2f(-1,-1);
   glTexCoord2d(1,1);glVertex2f(-1, 1);
   glTexCoord2d(0,1);glVertex2f(-asp, 1);
   glEnd();
   glBindTexture(GL_TEXTURE_2D,texture[21]);
   glBegin(GL_QUADS);
   glTexCoord2d(1,0);glVertex2f(asp,-1);
   glTexCoord2d(0,0);glVertex2f(1,-1);
   glTexCoord2d(0,1);glVertex2f(1, 1);
   glTexCoord2d(1,1);glVertex2f(asp, 1);
   glEnd();
   glDisable(GL_TEXTURE_2D);
   //  Display parameters
   glWindowPos2i(5,5);
   Print("Position=%f,%f ",
     droneX, droneZ);
   //  Reset model view matrix
   glPopMatrix();
   //  Reset projection matrix
   glMatrixMode(GL_PROJECTION);
   glPopMatrix();
   glEnable(GL_LIGHTING);
   //  Pop transform attributes (Matrix Mode and Enabled Modes)
   glPopAttrib();
}

/*
 *  A helpful screen on game controls. Draw last.
 */
void helpScreen()
{
   glDisable(GL_LIGHTING);
   //  Save transform attributes (Matrix Mode and Enabled Modes)
   glPushAttrib(GL_TRANSFORM_BIT|GL_ENABLE_BIT);
   //  Save projection matrix and set unit transform
   glMatrixMode(GL_PROJECTION);
   glPushMatrix();
   glLoadIdentity();
   glOrtho(-asp,+asp,-1,1,-1,1);
   //  Save model view matrix and set to indentity
   glMatrixMode(GL_MODELVIEW);
   glPushMatrix();
   glLoadIdentity();
   //  Draw help
   glColor3f(1,1,1);
   glEnable(GL_TEXTURE_2D);
   glBindTexture(GL_TEXTURE_2D,texture[19]);
   glBegin(GL_QUADS);
   glTexCoord2d(0,0);glVertex2f(-asp,-1);
   glTexCoord2d(1,0);glVertex2f(asp,-1);
   glTexCoord2d(1,1);glVertex2f(asp, 1);
   glTexCoord2d(0,1);glVertex2f(-asp, 1);
   glEnd();

   //  Reset model view matrix
   glPopMatrix();
   //  Reset projection matrix
   glMatrixMode(GL_PROJECTION);
   glPopMatrix();
   glEnable(GL_LIGHTING);
   //  Pop transform attributes (Matrix Mode and Enabled Modes)
   glPopAttrib();
}

//Draw the Sun and the Moon. 
void drawSunMoon(){
   glDisable(GL_FOG);
   float timeSlowed = glutGet(GLUT_ELAPSED_TIME)/100.0;

   //  Translate intensity to color vectors. These default to "night" mode.
   float Ambient[]   = {0.01*ambient ,0.01*ambient ,0.05*ambient ,1.0};
   float Diffuse[]   = {0.01*diffuse ,0.01*diffuse ,0.01*diffuse ,1.0};
   float Specular[]  = {0.01*specular,0.01*specular,0.01*specular,1.0};
   night = 1;
   //  Light position
   float yPos = distance*Sin(timeSlowed);

   //If it's day, change the ambient. 
   if(yPos > 0){
      Ambient[0] = 0.05 * ambient;
      Ambient[1] = 0.05 * ambient;
      night = 0;
   }

   //Light position
   float Position[]  = {distance*Cos(timeSlowed), yPos, ylight,1.0};

   //  Draw light position as ball (still no lighting here)
   glColor3f(1,1,1);
   glDisable(GL_LIGHTING); //Glowing Sun/Moon

   //  OpenGL should normalize normal vectors
   glEnable(GL_NORMALIZE);
   SunMoon(Position[0],Position[1],Position[2], 0.1, 0);
   SunMoon(-Position[0], -Position[1],-Position[2], 0.1, 1);

   //  Enable lighting
   glEnable(GL_LIGHTING);

   //  Location of viewer for specular calculations
   glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER,local);

   //  glColor sets ambient and diffuse color materials
   glColorMaterial(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE);
   glEnable(GL_COLOR_MATERIAL);

   //  Enable light 0
   glEnable(GL_LIGHT0);

   //  Set ambient, diffuse, specular components and position of light 0
   glLightfv(GL_LIGHT0,GL_AMBIENT ,Ambient);
   glLightfv(GL_LIGHT0,GL_DIFFUSE ,Diffuse);
   glLightfv(GL_LIGHT0,GL_SPECULAR,Specular);
   glLightfv(GL_LIGHT0,GL_POSITION,Position);
   glEnable(GL_FOG);

}

//Draw the spotlights 
void drawSpotLights(){
   //Set all spotlights
   //float Direction[] = {Cos(Th)*Sin(Ph),Sin(Th)*Sin(Ph),-Cos(Ph),0};
   float Direction[] = {0, -1, 0,0};
   float Exp = 20;
   float at0 = 100;
   float at1 = at0;
   float at2 = at1;

   glEnable(GL_LIGHTING); // Enable lighting

   glColorMaterial(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE);
   glEnable(GL_COLOR_MATERIAL);

   for (int i = 0; i < NUM_LIGHTS; i++) {
      if(night){
         glEnable(GL_LIGHT0 + i + 1); // Enable each light
         //  Set ambient, diffuse, specular components and position of light 0
         glLightfv(GL_LIGHT0 + i + 1,GL_AMBIENT ,light_colors[i]);
         glLightfv(GL_LIGHT0 + i + 1,GL_DIFFUSE ,light_colors[i]);
         glLightfv(GL_LIGHT0 + i + 1,GL_SPECULAR,light_colors[i]);
         glLightfv(GL_LIGHT0 + i + 1,GL_POSITION,light_positions[i]);
         //  Set spotlight parameters
         glLightfv(GL_LIGHT0 + i + 1,GL_SPOT_DIRECTION,Direction);
         glLightf(GL_LIGHT0 + i + 1,GL_SPOT_CUTOFF,10);
         glLightf(GL_LIGHT0 + i + 1,GL_SPOT_EXPONENT,Exp);
         //  Set attenuation
         glLightf(GL_LIGHT0 + i + 1,GL_CONSTANT_ATTENUATION ,at0/100.0);
         glLightf(GL_LIGHT0 + i + 1,GL_LINEAR_ATTENUATION   ,at1/100.0);
         glLightf(GL_LIGHT0 + i + 1,GL_QUADRATIC_ATTENUATION,at2/100.0);
      }
      else{
         glDisable(GL_LIGHT0 + i + 1); // Disable each light
      }

   }

   // Define light positions for the mobile lights (drone, man).
   GLfloat mobileLightPos[2][4] = {
      { droneX, droneY, droneZ, 1.0f },
      { manX, manY + 2.45 * manScale, manZ, 1.0f }
   };

   //Define light colors for the mobile lights
   GLfloat mobileLightColor[2][4] = {
      { 1.0f, 1.0f, 1.0f, 1.0f }, // White
      { 1.0f, 1.0f, 1.0f, 1.0f } // White
   };

   //Continue counter
   int i = NUM_LIGHTS;

   //Calculate the position of the drone and man.s
   float Direction2[] = {Sin(- camTh + droneTh), Sin(camPh), Cos(- camTh + droneTh), 0};
   float Direction3[] = {Sin(- headTh + manTh), 0, Cos(- headTh + manTh), 0};

   //If the spotlight is on:
   if(spotlight){
      //Similar, but our direction is the direction we look
      glEnable(GL_LIGHT0 + i + 1); // Turn Light On

      //  Set diffuse, specular components and position of light 0
      glLightfv(GL_LIGHT0 + i + 1,GL_DIFFUSE ,mobileLightColor[0]);
      glLightfv(GL_LIGHT0 + i + 1,GL_SPECULAR,mobileLightColor[0]);
      glLightfv(GL_LIGHT0 + i + 1,GL_POSITION,mobileLightPos[0]);
      //  Set spotlight parameters
      glLightfv(GL_LIGHT0 + i + 1,GL_SPOT_DIRECTION,Direction2);
      glLightf(GL_LIGHT0 + i + 1,GL_SPOT_CUTOFF,10);
      glLightf(GL_LIGHT0 + i + 1,GL_SPOT_EXPONENT,0);
      //  Set attenuation
      glLightf(GL_LIGHT0 + i + 1,GL_CONSTANT_ATTENUATION ,at0/100.0);
      glLightf(GL_LIGHT0 + i + 1,GL_LINEAR_ATTENUATION   ,at1/100.0);
      glLightf(GL_LIGHT0 + i + 1,GL_QUADRATIC_ATTENUATION,at2/100.0);
   }
   else{
      glDisable(GL_LIGHT0 + i + 1); // Turn light off
   }

   //Increment counter
   i++;

   if(flashlight){
   //Direction of light = direction we look in
      glEnable(GL_LIGHT0 + i + 1); // Enable each light

      //  Set diffuse, specular components and position of light 0
      
      glLightfv(GL_LIGHT0 + i + 1,GL_AMBIENT ,mobileLightColor[1]);
      glLightfv(GL_LIGHT0 + i + 1,GL_DIFFUSE ,mobileLightColor[1]);
      glLightfv(GL_LIGHT0 + i + 1,GL_SPECULAR,mobileLightColor[1]);
      
      glLightfv(GL_LIGHT0 + i + 1,GL_POSITION,mobileLightPos[1]);
      //  Set spotlight parameters
      glLightfv(GL_LIGHT0 + i + 1,GL_SPOT_DIRECTION,Direction3);
      glLightf(GL_LIGHT0 + i + 1,GL_SPOT_CUTOFF,30);
      glLightf(GL_LIGHT0 + i + 1,GL_SPOT_EXPONENT,0);
      //  Set attenuation
      glLightf(GL_LIGHT0 + i + 1,GL_CONSTANT_ATTENUATION ,at0/100.0);
      glLightf(GL_LIGHT0 + i + 1,GL_LINEAR_ATTENUATION   ,at1/100.0);
      glLightf(GL_LIGHT0 + i + 1,GL_QUADRATIC_ATTENUATION,at2/100.0);
   }
   else{
      glDisable(GL_LIGHT0 + i + 1); // Disable light
   }
}

//Get the distance to drone/man, in 2d
double getDistance(double x, double y, double z, int obj){
   if(obj == 1){
      return sqrt(pow(x - droneX, 2) + pow(z - droneZ, 2));
   }
   else if (obj == 2){
      return sqrt(pow(x - manX, 2) + pow(z - manZ, 2));
   }
   else{
      return -1;
   }
}

//Move drone away from (x, y, z).
void goAway(double x, double y, double z, int obj) {
   double cX = 0;
   double cY = 0;
   double cZ = 0;

   if(obj == 1){
      cX = droneX;
      cY = droneY;
      cZ = droneZ;
   }
   else if(obj == 2){
      cX = manX;
      cY = manY;
      cZ = manZ;
   }

   // Calculate the direction vector components
   double dx = cX - x;
   double dy = cY - y;
   double dz = cZ - z;
   
   // Calculate the magnitude (length) of the direction vector
   double magnitude = sqrt(dx * dx + dy * dy + dz * dz);
   
   // Check to avoid division by zero (in case the points are identical)
   if (magnitude != 0) {
      // Normalize the direction vector to get the unit vector
      dx = dx / magnitude;
      dy = dy / magnitude;
      dz = dz / magnitude;
   } else {
      // Handle the case where the points are identical (no direction)
      // Shouldn't happen, but if it happens, we just move the camera up by 1 unit. 
      dx = dz = 0.00;
      dy = dt;
   }

   if(obj == 1){
      droneX += dx * 2 * dt;
      droneY += dy * 2 * dt; 
      droneZ += dz * 2 * dt;
   }
   else if(obj == 2){
      manX += dx * 2 * dt;
      manY += dy * 2 * dt; 
      manZ += dz * 2 * dt;
   }

}

void handleCollisionMan(Object object){
   double distance = getDistance(object.x, object.y, object.z, 2);

   if(distance < object.radius*object.scale){
      goAway(object.x, object.y + object.height*object.scale/2, object.z, 2);
      handleCollisionMan(object);
   }   
}

void handleCollisionDrone(Object object){
   double distance = getDistance(object.x, object.y, object.z, 1);

   if((distance < object.radius*object.scale) && droneY < object.height*object.scale){
      goAway(object.x, object.y + object.height*object.scale/2, object.z, 1);
      handleCollisionDrone(object);
   }   
}

//Handle collision for the movable objects. Currently uses bounding cylinders.
void handleCollision(Object object){
   handleCollisionDrone(object);
   handleCollisionMan(object);
}

//Handle collision.
void collisionCheck(){
   for(int i = 0; i < opaqueThingsSize; i++){
      handleCollision(opaqueThings[i]);
   }

   if(droneY < 0){
      droneY = 0;
   }
   if(manY < 0){
      manY = 0;
   }
}

//Variables for Fog
float density = 0.3;
float fogColor1[4] = {0.3, 0.3, 0.5, 1.0};
float fogColor2[4] = {0.2, 0.2, 0.2, 1.0};

/*
 *  OpenGL (GLUT) calls this routine to display the scene
 */
void display()
{
   //  Erase the window and the depth buffer
   glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

   //  Enable Z-buffering in OpenGL
   glEnable(GL_DEPTH_TEST);

   //  Undo previous transformations
   glLoadIdentity();

   //Antialiasing
   glEnable(GL_LINE_SMOOTH);
   glEnable(GL_BLEND);
   glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
   glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

   updateTrafficLights();
   //If the Help screen is displayed, display it and skip everything else
   if(displayHelp == 1){
      helpScreen();
      //  Render the scene and make it visible
      ErrCheck("display");
      glFlush();
      glutSwapBuffers();
      return;
   }

   //Draw fog
   glEnable(GL_FOG);
   glFogi (GL_FOG_MODE, GL_EXP2);
   if(night){
      glFogfv (GL_FOG_COLOR, fogColor2);
   }
   else{
      glFogfv (GL_FOG_COLOR, fogColor1);
   }
   glHint (GL_FOG_HINT, GL_NICEST);
   glFogf (GL_FOG_DENSITY, density);

   //These are for the "ghost" debug camera. 
   double dx = Cos(th); 
   double dy = Sin(ph);
   double dz = Sin(th); 

   //Change dx, dy, dz if controlling the drone
   if(controllingDrone){
      dx = Sin(-th + droneTh); //Formerly Cos
      dy = Sin(ph);
      dz = Cos(-th + droneTh); //Formerly Sin
   }
   else if (controllingMan){
      dx = Sin(-th + manTh); //Formerly Cos
      dy = Sin(ph);
      dz = Cos(-th + manTh); //Formerly Sin
   }

   //Direction of view
   double Cx = Ex + dx;
   double Cy = Ey + dy;
   double Cz = Ez + dz;

   //"Up" direction
   double Ux = 0;
   double Uy = Cos(ph);
   double Uz = 0;

   //Update the drone's camera variables if it's being controlled
   if(controllingDrone){
      camPh = ph;
      camTh = th;
   }
   else if(controllingMan){
      headTh = th;
   }

   //Handle camera projections
   if(!debug){


      //1st person drone/man
      if(firstPerson){
         if(controllingDrone){
            Ex = droneX;
            Ey = droneY;
            Ez = droneZ;
         }
         else{
            Ex = manX;
            Ey = manY + 2.45 * manScale;
            Ez = manZ;
         }
         gluLookAt(Ex, Ey, Ez, Cx, Cy, Cz, Ux, Uy, Uz);
      }
      else{
         //Set eye position to opposite of 1st person view direction
         //ie. view from "behind".
         if(controllingDrone){ 
            Ex = droneX + +0.5*dim*Sin(th - droneTh)*Cos(ph);
            Ey = droneY - +0.5*dim        *Sin(ph);
            Ez = droneZ + -0.5*dim*Cos(th - droneTh)*Cos(ph);

            gluLookAt(Ex, Ey, Ez, droneX, droneY, droneZ, Ux, Uy, Uz);
         }
         else {
            Ex = manX + +0.5*dim*Sin(th - manTh)*Cos(ph);
            Ey = manY + 2.45 * manScale - + 0.5*dim        *Sin(ph);
            Ez = manZ + -0.5*dim*Cos(th - manTh)*Cos(ph);
            gluLookAt(Ex, Ey, Ez, manX, manY, manZ, Ux, Uy, Uz);
         }
      }
   }
   //"Ghost" camera in debug
   else{
      gluLookAt(Ex, Ey, Ez, Cx, Cy, Cz, Ux, Uy, Uz);
   }


   //  Flat or smooth shading
   glShadeModel(smooth ? GL_SMOOTH : GL_FLAT);

   //Render skybox and sun/mon
   SkyBox();
   drawSunMoon(); 


   if(!debug){
      //Check for collision
      collisionCheck();
      //Draw spotlights and scene.
      drawSpotLights();
      drawScene();
      //If the UI is active, draw it.
      if(drawUI == 1 && firstPerson == 1 && controllingDrone == 1){
         DroneUI();
      }
   }
   else{
      //Draw debug objects.
      switch(objectChosen)
      {
         case 0:
            SandBlock(0, 0, 0, 1, 1, 1);
            break;
         case 1:
            IntersectionBlock(0, 0, 0, 1, 1, 1);
            break;
         case 2:
            RoadBlock(0, 0, 0, 1, 1, 1, 90);
            break;
         case 3:
            TrafficLight(0, 0, 0, 1, 1, 1, -90);
            TrafficLight2(0, 0, 0, 1, 1, 1, -90);
            break;
         case 4:
            WaterBlock(0, 0, 0, 1, 1, 1);
            break;
         case 5:
            StreetLight(0, 0, 0, 1, 1, 1, -90);
            StreetLight2(0, 0, 0, 1, 1, 1, -90);
            break;
         case 6:
            SolidHouse(0, 0, 0, 0.5, 0.5, 0.5, -90, 1, 1, 1, 2, 0);
            break;
         case 7:
            SolidTree(0, 0, 0, 1, 1, 1, 0);
            break;
         case 8:
            Drone(0, 0, 0, 1, 1, 1, -90);
            DronePropellorSet(0, 0, 0, 1, 1, 1, -90);
            break;
         case 9:
            Man(0, 0, 0, 0.3, 0.3, 0.3, -90);
            break;
      }
   }


   //  Draw axes - no lighting from here on
   glDisable(GL_LIGHTING);
   glDisable(GL_FOG);
   glColor3f(1,1,1);
   if (axes)
   {
      const double len=3.0;  //  Length of axes
      glBegin(GL_LINES);
      glVertex3d(0.0,0.0,0.0);
      glVertex3d(len,0.0,0.0);
      glVertex3d(0.0,0.0,0.0);
      glVertex3d(0.0,len,0.0);
      glVertex3d(0.0,0.0,0.0);
      glVertex3d(0.0,0.0,len);
      glEnd();
      //  Label axes
      glRasterPos3d(len,0.0,0.0);
      Print("X");
      glRasterPos3d(0.0,len,0.0);
      Print("Y");
      glRasterPos3d(0.0,0.0,len);
      Print("Z");
      
   }
   //  Render the scene and make it visible
   ErrCheck("display");
   glFlush();
   glutSwapBuffers();
}

/*
 *  GLUT calls this routine when the window is resized
 */
void idle()
{
   //  Elapsed time in seconds
   double t = glutGet(GLUT_ELAPSED_TIME)/1000.0;
   zh = fmod(90*t,360.0);
   //  Tell GLUT it is necessary to redisplay the scene
   glutPostRedisplay();
}

/*
 *  GLUT calls this routine when an arrow key is pressed
 */
void special(int key,int x,int y)
{
   //  Right arrow key - increase angle by 5 degrees
   if (key == GLUT_KEY_RIGHT)
      th += 5;
   //  Left arrow key - decrease angle by 5 degrees
   else if (key == GLUT_KEY_LEFT)
      th -= 5;
   //  Up arrow key - increase elevation by 5 degrees
   else if (key == GLUT_KEY_UP)
      ph += 5;
   //  Down arrow key - decrease elevation by 5 degrees
   else if (key == GLUT_KEY_DOWN)
      ph -= 5;

   // Limit drone camera view to +5 degress, -85 degrees elevation and +/- 90 degrees side to side

   //  Keep angles to +/-360 degrees
   th %= 360;
   ph %= 360;
   //  Update projection
   Project(mode?fov:0,asp,dim);
   //  Tell GLUT it is necessary to redisplay the scene
   glutPostRedisplay();
}

/*
 *  GLUT calls this routine when a key is pressed
 */
void key(unsigned char ch,int x,int y)
{
   //  Exit on ESC
   if (ch == 27)
      exit(0);
   //  Reset view angle and all objects
   else if (ch == '0'){
      if(debug){
         Ex = -2;
         Ey = 1;
         Ez = 0;
         th = 0;
         ph = 0;
      }
      else{
         th = ph = 0;
         Ex = 0;
         Ey = 0.2;
         Ez = 0;

         droneX = 0;
         droneY = 1;
         droneZ = 0;
         manX = 0;
         manY = 0;
         manZ = 0;
      }
   }

   //  Change control mode
   else if (ch == 'm' || ch == 'M'){
      controllingDrone = 1 - controllingDrone;
      controllingMan = 1 - controllingMan;
   }

   else if (ch == 'p' || ch == 'P')
      firstPerson = 1-firstPerson;
   
   //Move Camera
   else if (ch == 'w' || ch == 'W'){
      if(debug){
         //Move "fowards"
         Ex += Cos(th)*dt;
         Ez += Sin(th)*dt;
      }

      if(controllingDrone){
         droneX += Sin(droneTh) * dt;
         droneZ += Cos(droneTh) * dt;
      }
      else if(controllingMan){
         manX += Sin(manTh) * dt;
         manZ += Cos(manTh) * dt;
      }
   }
   else if (ch == 's' || ch == 'S') {
      if(debug){
         //Move "backwards"
         Ex -= Cos(th)*dt;
         Ez -= Sin(th)*dt;
      }

      if(controllingDrone){
         droneX -= Sin(droneTh) * dt;
         droneZ -= Cos(droneTh) * dt;
      }
      else if(controllingMan){
         manX -= Sin(manTh) * dt;
         manZ -= Cos(manTh) * dt;
      }
   }
   //Use a +90 degree shift on azimuth for left/right movement
   //This does not affect Ey, so we do not change it
   else if (ch == 'a' || ch == 'A'){
      if(debug){
         Ex -= Cos((th + 90)%360)*dt;
         Ez -= Sin((th + 90)%360)*dt;
      }
      if(controllingDrone){
         droneX += Sin((droneTh + 90)%360) * dt;
         droneZ += Cos((droneTh + 90)%360) * dt;
      }
      else if(controllingMan){
         manX += Sin((manTh + 90)%360) * dt;
         manZ += Cos((manTh + 90)%360) * dt;    
      }
   }
   else if (ch == 'd' || ch == 'D') {
      if(debug){
         Ex += Cos((th + 90)%360)*dt;
         Ez += Sin((th + 90)%360)*dt;
      }
      if(controllingDrone){
         droneX -= Sin((droneTh + 90)%360) * dt;
         droneZ -= Cos((droneTh + 90)%360) * dt;
      }
      else if (controllingMan){
         manX -= Sin((manTh + 90)%360) * dt;
         manZ -= Cos((manTh + 90)%360) * dt;
      }
   }

   //Only for drone and ghost camera. 
   else if (ch == 'r' || ch == 'R') {
      if(debug){
      Ey += dt;
      }
      if(controllingDrone){
         droneY += dt;
      }
   }
   else if (ch == 'f' || ch == 'F') {
      if(debug){
      Ey -= dt;
      }
      if(controllingDrone){
         droneY -= dt;
      }
   }
   else if (ch == 'q' || ch == 'Q') {
      if(controllingDrone){
         droneTh += 5;
      }
      else if (controllingMan){
         manTh += 5;
      }
   }
   else if (ch == 'e' || ch == 'E') {
      if(controllingDrone){
         droneTh -= 5;
      }
      else if (controllingMan){
         manTh -= 5;
      }
   }
   else if (ch == 'h' || ch == 'H') {
      displayHelp = 1 - displayHelp;
   }
   else if (ch == 'l' || ch == 'L') {
      debug = 1 - debug;

      //Change controlled object
      if(debug){
         Ex = -2;
         Ey = 1;
         Ez = 0;
         th = 0;
         ph = 0;
         controllingDrone = 0;
         controllingMan = 0;
         for(int i = 0; i < 8; i++){
            glDisable(GL_LIGHT0 + i); // Disable each light to remove ghost lights
         }
      }
      else{
         controllingDrone = 1;
         controllingMan = 0;         
      }
   }
   else if (ch == 'g' || ch == 'G') {
      if(controllingDrone){
         spotlight = 1 - spotlight;
      }
      if(controllingMan){
         flashlight = 1 - flashlight;
      }
   }
   else if (ch == '[') {
      if(debug){
         objectChosen = (objectChosen + 1)%10;
      }
   }
   else if (ch == ']') {
      if(debug){
         objectChosen = (objectChosen + 9)%10;
      }
   }
   else if (ch == 't' || ch == 'T') {
      drawUI = 1 - drawUI;
   }


   //  Translate shininess power to value (-1 => 0)
   shiny = shininess<0 ? 0 : pow(2.0,shininess);
   //  Reproject
   Project(mode?fov:0,asp,dim);
   //  Animate if requested
   glutIdleFunc(move?idle:NULL);
   //  Tell GLUT it is necessary to redisplay the scene
   glutPostRedisplay();
}

/*
 *  GLUT calls this routine when the window is resized
 */
void reshape(int width,int height)
{
   //  Ratio of the width to the height of the window
   asp = (height>0) ? (double)width/height : 1;
   //  Set the viewport to the entire window
   glViewport(0,0, RES*width,RES*height);
   //  Set projection
   Project(mode?fov:0,asp,dim);
}

/*
 *  GLUT calls this routine when a mouse is moved
 */
void motion(int x,int y)
{
   //  Do only when move is set
   if (move)
   {
      //  Zoom in/out
      if (moveCam<0){
         fov -= (Y-y);

         //Prevent unreasonable zooms
         if(fov < 25){
            fov = 25;
         }
         if(fov > 90){
            fov = 90;
         }
      }
      // Pan camera
      else
      {
         th += (Cos(th)*(X-x) - Sin(th)*(Y-y))/5; //These are divided by 5/2 to slow the speed down
         ph += (Sin(th)*(X-x) + Cos(th)*(Y-y))/2;
         th %= 360;
         ph %= 360;
      }
      //  Remember location
      X = x;
      Y = y;
   }
   //  Update projection
   Project(mode?fov:0,asp,dim);
   //  Tell GLUT it is necessary to redisplay the scene
   glutPostRedisplay();
}

/*
 *  GLUT calls this routine when a mouse button is pressed or released
 */
void mouse(int key,int status,int x,int y)
{
   //  On button down, set 'move' and remember location
   if (status==GLUT_DOWN)
   {
      moveCam = (key==GLUT_LEFT_BUTTON) ? 1 : -1;
      X = x;
      Y = y;
   }
   //  On button up, unset move
   else if (status==GLUT_UP)
      moveCam = 0;

   //  Update projection
   Project(mode?fov:0,asp,dim);
   //  Tell GLUT it is necessary to redisplay the scene
   glutPostRedisplay();
}

/*
 *  Random numbers from min to max to the power p
 */
static float frand(float min,float max,float p)
{
   return pow(rand()/(float)RAND_MAX,p)*(max-min)+min;
}

/*
 * Initialize icosasphere locations
 */
void Init()
{
   for (int i=0;i<n;i++)
   {
      float th = frand(0,360,1);
      float ph = frand(-90,+90,1);
      float r  = frand(0.1,0.7,3);
      is[i].x = r*Sin(th)*Cos(ph);
      is[i].y = r*Cos(th)*Cos(ph);
      is[i].z = r*Sin(ph) + 1.0;
   }
}

/*
 *  Start up GLUT and tell it what to do
 */
int main(int argc,char* argv[])
{
   //  Initialize
   Init();
   //  Initialize GLUT
   glutInit(&argc,argv);
   //  Request double buffered, true color window with Z buffering at 600x600
   glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
   glutInitWindowSize(400,400);
   glutCreateWindow("Patrick Liu Final Project");
#ifdef USEGLEW
   //  Initialize GLEW
   if (glewInit()!=GLEW_OK) Fatal("Error initializing GLEW\n");
#endif
   //  Set callbacks
   glutDisplayFunc(display);
   glutReshapeFunc(reshape);
   glutSpecialFunc(special);
   glutKeyboardFunc(key);
   glutMouseFunc(mouse);
   glutMotionFunc(motion);
   glutIdleFunc(idle);
   
   //  Load textures
   texture[0] = LoadTexBMP("textures/house/door.bmp");
   texture[1] = LoadTexBMP("textures/house/roof.bmp");
   texture[2] = LoadTexBMP("textures/house/walls.bmp");
   texture[3] = LoadTexBMP("textures/house/window.bmp");
   texture[4] = LoadTexBMP("textures/nature/dirt.bmp");
   texture[5] = LoadTexBMP("textures/nature/fruit.bmp");
   texture[6] = LoadTexBMP("textures/nature/grass.bmp");
   texture[7] = LoadTexBMP("textures/nature/snow.bmp");
   texture[8] = LoadTexBMP("textures/nature/trunk.bmp");
   texture[9] = LoadTexBMP("textures/nature/leaves.bmp");
   texture[10] = LoadTexBMP("textures/roads/road.bmp");
   texture[11] = LoadTexBMP("textures/drone/dronebody.bmp");
   texture[12] = LoadTexBMP("textures/drone/propBlur.bmp");
   texture[13] = LoadTexBMP("textures/nature/water.bmp");
   texture[14] = LoadTexBMP("textures/nature/sun.bmp");
   texture[15] = LoadTexBMP("textures/nature/moon.bmp");
   texture[16] = LoadTexBMP("textures/nature/day.bmp");
   texture[17] = LoadTexBMP("textures/nature/night.bmp");
   texture[18] = LoadTexBMP("textures/roads/intersection.bmp");
   texture[19] = LoadTexBMP("textures/interface/help.bmp");
   texture[20] = LoadTexBMP("textures/interface/DroneUI1.bmp");
   texture[21] = LoadTexBMP("textures/interface/DroneUI2.bmp");
   texture[22] = LoadTexBMP("textures/man/shirt.bmp");
   texture[23] = LoadTexBMP("textures/man/pants.bmp");
   texture[24] = LoadTexBMP("textures/roads/metal.bmp");

   //  Pass control to GLUT so it can interact with the user
   ErrCheck("init");
   glutMainLoop();
   return 0;
}
