/**
 **********************************************************************************************************************
 * @file       sketch_4_Wall_Physics.pde
 * @author     Steve Ding, Colin Gallacher
 * @version    V4.1.0
 * @date       08-January-2021
 * @brief      wall haptic example using 2D physics engine 
 **********************************************************************************************************************
 * @attention
 *
 *
 **********************************************************************************************************************
 */



/* library imports *****************************************************************************************************/ 
import processing.serial.*;
import static java.util.concurrent.TimeUnit.*;
import java.util.concurrent.*;
/* end library imports *************************************************************************************************/  



/* scheduler definition ************************************************************************************************/ 
private final ScheduledExecutorService scheduler      = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/ 



/* device block definitions ********************************************************************************************/
Board             haplyBoard;
Device            widgetOne;
Mechanisms        pantograph;

byte              widgetOneID                         = 5;
int               CW                                  = 0;
int               CCW                                 = 1;
boolean           renderingForce                      = false;
/* end device block definition *****************************************************************************************/



/* framerate definition ************************************************************************************************/
long              baseFrameRate                       = 120;
/* end framerate definition ********************************************************************************************/ 



/* elements definition *************************************************************************************************/

/* Screen and world setup parameters */
float             pixelsPerCentimeter                 = 40.0;

/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

/* task space */
PVector           posEE                               = new PVector(0, 0);
PVector           fEE                                = new PVector(0, 0); 

/* World boundaries in centimeters */
FWorld            world;
float             worldWidth                          = 25.0;  
float             worldHeight                         = 15.0; 

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;


/* Initialization of wall */
FBox              wall;


/* Initialization of virtual tool */
HVirtualCoupling  s;
PImage            haplyAvatar;

/* end elements definition *********************************************************************************************/ 



/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */
  
  /* screen size definition */
  size(1000, 600);
  
  /* device setup */
  
  /**  
   * The board declaration needs to be changed depending on which USB serial port the Haply board is connected.
   * In the base example, a connection is setup to the first detected serial device, this parameter can be changed
   * to explicitly state the serial port will look like the following for different OS:
   *
   *      windows:      haplyBoard = new Board(this, "COM10", 0);
   *      linux:        haplyBoard = new Board(this, "/dev/ttyUSB0", 0);
   *      mac:          haplyBoard = new Board(this, "/dev/cu.usbmodem1411", 0);
   */ 
  haplyBoard          = new Board(this, "/dev/cu.usbmodem1422301", 0);
  widgetOne           = new Device(widgetOneID, haplyBoard);
  pantograph          = new Pantograph();
  
  widgetOne.set_mechanism(pantograph);
  
  widgetOne.add_actuator(1, CCW, 2);
  widgetOne.add_actuator(2, CW, 1);
 
  widgetOne.add_encoder(1, CCW, 241, 10752, 2);
  widgetOne.add_encoder(2, CW, -61, 10752, 1);
  
  
  widgetOne.device_set_parameters();
  
  
  /* 2D physics scaling and world creation */
  hAPI_Fisica.init(this); 
  hAPI_Fisica.setScale(pixelsPerCentimeter); 
  world               = new FWorld();
  
  
  
  /* creation of wall */
  //1
  wall                   = new FBox(7.0, 0.2);
  wall.setPosition(edgeTopLeftX+worldWidth/2.0, edgeTopLeftY+((2*worldHeight)/12.0));
  wall.setStatic(true);
  wall.setFill(159, 125, 35);
  world.add(wall);
  
  //2
    wall                   = new FBox(0.2, 3.0);
  wall.setPosition(edgeTopLeftX+13.5, edgeTopLeftY+4.1);
  wall.setStatic(true);
  wall.setFill(159, 125, 35);
  world.add(wall);
  
  //3
      wall                   = new FBox(0.2, 4.0);
  wall.setPosition(edgeTopLeftX+11.0, edgeTopLeftY+4.6);
  wall.setStatic(true);
  wall.setFill(159, 125, 35);
  world.add(wall);
  
  //4
  wall                   = new FBox(7.5, 0.2);
  wall.setPosition(edgeTopLeftX+7.35, edgeTopLeftY+6.5);
  wall.setStatic(true);
  wall.setFill(159, 125, 35);
  world.add(wall);
  
  //5
      wall                   = new FBox(0.2, 5);
  wall.setPosition(edgeTopLeftX+3.5, edgeTopLeftY+6.5);
  wall.setStatic(true);
  wall.setFill(159, 125, 35);
  world.add(wall);
  
  //6
        wall                   = new FBox(0.2, 3.5);
  wall.setPosition(edgeTopLeftX+6.0, edgeTopLeftY+2.5);
  wall.setStatic(true);
  wall.setFill(159, 125, 35);
  world.add(wall);
  
  //7
    wall                   = new FBox(3.0, 0.2);
  wall.setPosition(edgeTopLeftX+7.4, edgeTopLeftY+4.3);
  wall.setStatic(true);
  wall.setFill(159, 125, 35);
  world.add(wall);
  
  //8
  wall                   = new FBox(0.2, 2.3);
  wall.setPosition(edgeTopLeftX+3.5, edgeTopLeftY+1.9);
  wall.setStatic(true);
  wall.setFill(159, 125, 35);
  world.add(wall);
  
  //8.5 tricky
  wall                   = new FBox(0.2, 1);
  wall.setPosition(edgeTopLeftX+3.5, edgeTopLeftY+3.5);
  wall.setStatic(false);
  wall.setFill(0, 0, 0);
  world.add(wall);
  
  //9
   wall                   = new FBox(3.0, 0.2);
  wall.setPosition(edgeTopLeftX+20, edgeTopLeftY+((2*worldHeight)/12.0));
  wall.setStatic(true);
  wall.setFill(159, 125, 35);
  world.add(wall);
  
  //10
  wall                   = new FBox(0.2, 2.0);
  wall.setPosition(edgeTopLeftX+21.4, edgeTopLeftY+3.6);
  wall.setStatic(true);
  wall.setFill(159, 125, 35);
  world.add(wall);
  
  //11
     wall                   = new FBox(5.0, 0.2);
  wall.setPosition(edgeTopLeftX+21.75, edgeTopLeftY+6.0);
  wall.setStatic(true);
  wall.setFill(159, 125, 35);
  world.add(wall);
  
  //12
     wall                   = new FBox(0.2, 2.0);
  wall.setPosition(edgeTopLeftX+19.2, edgeTopLeftY+5.1);
  wall.setStatic(true);
  wall.setFill(159, 125, 35);
  world.add(wall);
  
  //13
  wall                   = new FBox(3.7, 0.2);
  wall.setPosition(edgeTopLeftX+17.45, edgeTopLeftY+4.2);
  wall.setStatic(true);
  wall.setFill(159, 125, 35);
  world.add(wall);
  
  //14
   wall                   = new FBox(0.2, 4.0);
  wall.setPosition(edgeTopLeftX+15.73, edgeTopLeftY+6.3);
  wall.setStatic(true);
  wall.setFill(159, 125, 35);
  world.add(wall);
  
  //15
  wall                   = new FBox(0.2, 4.0);
  wall.setPosition(edgeTopLeftX+13.5, edgeTopLeftY+9.5);
  wall.setStatic(true);
  wall.setFill(159, 125, 35);
  world.add(wall);
  
  //17
  wall                   = new FBox(3.0, 0.2);
  wall.setPosition(edgeTopLeftX+11.9, edgeTopLeftY+9);
  wall.setStatic(true);
  wall.setFill(159, 125, 35);
  world.add(wall);
  
  //18
   wall                   = new FBox(9.0, 0.2);
  wall.setPosition(edgeTopLeftX+14.8, edgeTopLeftY+11.5);
  wall.setStatic(true);
  wall.setFill(159, 125, 35);
  world.add(wall);
  
  //19
  wall                   = new FBox(0.2, 1.0);
  wall.setPosition(edgeTopLeftX+10.4, edgeTopLeftY+11.9);
  wall.setStatic(true);
  wall.setFill(159, 125, 35);
  world.add(wall);
  
  //20
  wall                   = new FBox(0.2, 2.0);
  wall.setPosition(edgeTopLeftX+19.4, edgeTopLeftY+10.6);
  wall.setStatic(true);
  wall.setFill(159, 125, 35);
  world.add(wall);
  
  //21
  wall                   = new FBox(0.2, 4.0);
  wall.setPosition(edgeTopLeftX+19.4, edgeTopLeftY+9.6);
  wall.setStatic(true);
  wall.setFill(159, 125, 35);
  world.add(wall);
  
  //22
  wall                   = new FBox(0.2, 6.6);
  wall.setPosition(edgeTopLeftX+21.8, edgeTopLeftY+10.95);
  wall.setStatic(true);
  wall.setFill(159, 125, 35);
  world.add(wall);
  
  //23
  wall                   = new FBox(0.2, 4);
  wall.setPosition(edgeTopLeftX+17.5, edgeTopLeftY+8);
  wall.setStatic(true);
  wall.setFill(159, 125, 35);
  world.add(wall);
  
  //24
  wall                   = new FBox(2.6, 0.2);
  wall.setPosition(edgeTopLeftX+20.6, edgeTopLeftY+7.6);
  wall.setStatic(true);
  wall.setFill(159, 125, 35);
  world.add(wall);
  
  //25
  wall                   = new FBox(5.5, 0.2);
  wall.setPosition(edgeTopLeftX+5.3, edgeTopLeftY+12.2);
  wall.setStatic(true);
  wall.setFill(159, 125, 35);
  world.add(wall);
  
  //26
  wall                   = new FBox(3, 0.2);
  wall.setPosition(edgeTopLeftX+6.5, edgeTopLeftY+9);
  wall.setStatic(true);
  wall.setFill(159, 125, 35);
  world.add(wall);
  
  //27
  wall                   = new FBox(0.2, 3.4);
  wall.setPosition(edgeTopLeftX+8, edgeTopLeftY+10.6);
  wall.setStatic(true);
  wall.setFill(159, 125, 35);
  world.add(wall);
  
  //28
  wall                   = new FBox(5.5, 0.2);
  wall.setPosition(edgeTopLeftX+5.3, edgeTopLeftY+12.2);
  wall.setStatic(true);
  wall.setFill(159, 125, 35);
  world.add(wall);
  
  //29
    wall                   = new FBox(4.5, 0.2);
  wall.setPosition(edgeTopLeftX+3, edgeTopLeftY+10.5);
  wall.setStatic(true);
  wall.setFill(159, 125, 35);
  world.add(wall);
  
    /* Finish Button */
  c2                  = new FCircle(0.2);
  c2.setPosition(worldWidth-2.5, edgeTopLeftY+worldHeight/2.0);
  c2.setFill(104,71,141);
  c2.setStaticBody(true);
  c2.setSensor(true);
  world.add(c2);
  
  /* Haptic Tool Initialization */
  s                   = new HVirtualCoupling((1)); 
  s.h_avatar.setDensity(4);  
  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 
 
  
  /* If you are developing on a Mac users must update the path below 
   * from "../img/Haply_avatar.png" to "./img/Haply_avatar.png" 
   */
  haplyAvatar = loadImage("./img/stick.png"); 
  haplyAvatar.resize((int)(hAPI_Fisica.worldToScreen(1)), (int)(hAPI_Fisica.worldToScreen(1)));
  s.h_avatar.attachImage(haplyAvatar); 


  /* world conditions setup */
  world.setGravity((0.0), (1000.0)); //1000 cm/(s^2)
  world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); 
  world.setEdgesRestitution(.4);
  world.setEdgesFriction(0.5);
  
  
  world.draw();
  
  
  /* setup framerate speed */
  frameRate(baseFrameRate);
  

  /* setup simulation thread to run at 1kHz */ 
  SimulationThread st = new SimulationThread();
  scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
}
/* end setup section ***************************************************************************************************/



/* draw section ********************************************************************************************************/
void draw(){
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  if(renderingForce == false){
    background(255);
    world.draw();
  }
}
/* end draw section ****************************************************************************************************/



/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable{
  
  public void run(){
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */
    
    renderingForce = true;
    
    if(haplyBoard.data_available()){
      /* GET END-EFFECTOR STATE (TASK SPACE) */
      widgetOne.device_read_data();
    
      angles.set(widgetOne.get_device_angles()); 
      posEE.set(widgetOne.get_device_position(angles.array()));
      posEE.set(posEE.copy().mult(200));  
    }
    
    s.setToolPosition(edgeTopLeftX+worldWidth/2-(posEE).x, edgeTopLeftY+(posEE).y-7); 
    
    
    s.updateCouplingForce();
    fEE.set(-s.getVirtualCouplingForceX(), s.getVirtualCouplingForceY());
    fEE.div(100000); //dynes to newtons
    
    torques.set(widgetOne.set_device_torques(fEE.array()));
    widgetOne.device_write_torques();
  
    world.step(1.0f/1000.0f);
  
    renderingForce = false;
  }
}
/* end simulation section **********************************************************************************************/



/* helper functions section, place helper functions here ***************************************************************/

/* end helper functions section ****************************************************************************************/
