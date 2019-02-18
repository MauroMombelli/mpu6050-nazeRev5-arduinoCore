public class ShowMap extends PApplet {
  float rotationAngle;
  float elevationAngle;
  
  float centerX;
  float centerY;
  float centerZ;
  
  Lines l;
  // -----------------------------------------------------------------
  public void settings() {
    size(1000, 600, "processing.opengl.PGraphics3D");
  }
  //PFont font;
  void setup() {
    //font = loadFont("DroidSerif-48.vlw");
    l = new Lines();
  }
  
  float camX = 0, camY = 0, camZ = 0;
  float addcamX = 0, addcamY = 0, addcamZ = 0;
  // -----------------------------------------------------------------
  void draw() {
    // background in the draw loop to make it animate rather than draw over itself
    background(0);
    lights();
    stroke(255);
   
    pushMatrix();
    translate(150, 0, 0);
    fill(255);
    box(100);
    popMatrix();
    
    l.draw();
   /*
    pushMatrix();
    translate(0, height/2-50, 0);
    fill(255, 0, 0);
    box(10);
    popMatrix();
    */
    updateCamera();
  }
  
  void updateCamera() {
    rotationAngle = -map(mouseX, 0, width, 0, TWO_PI);
    elevationAngle = map(mouseY, 0, height, 0, PI);
  
    centerX = cos(rotationAngle) * sin(elevationAngle);
    centerY = sin(rotationAngle) * sin(elevationAngle);
    centerZ = -cos(elevationAngle);
  
    float r = addcamX;//sqrt(addcamX*addcamX+addcamY*addcamY+addcamZ*addcamZ);  
    camX += r * centerX;
    camY += r * centerY;
    camZ += r * centerZ;
    
    r = addcamY;//sqrt(addcamX*addcamX+addcamY*addcamY+addcamZ*addcamZ);
    float centerX2 = cos(rotationAngle+PI/2) * sin(elevationAngle);
    float centerY2 = sin(rotationAngle+PI/2) * sin(elevationAngle);
    float centerZ2 = -cos(elevationAngle);
    camX += r * centerX2;
    camY += r * centerY2;
    camZ += r * centerZ2;
    camera(camX, camY, camZ, camX+centerX, camY+centerY, camZ+centerZ, 0, 0, 1);
    
    pushMatrix();
    textSize(32);
    fill(0, 102, 153);
    //rotateX(PI/2);
    //rotateY(PI/2);
    //rotateY(centerY);
    text("word", camX-centerX, camY-centerY, camZ-centerZ);
    popMatrix();
  }
  
  void keyPressed() {
    switch(key){
      case 'w':
      addcamX = 1;
      break;
      case 's':
      addcamX = -1;
      break;
      case 'a':
      addcamY = 1;
      break;
      case 'd':
      addcamY = -1;
      break;
    }
  }
  
  void keyReleased() {
    switch(key){
      case 'w':
      case 's':
      addcamX = 0;
      break;
      case 'a':
      case 'd':
      addcamY = 0;
      break;
    }
  }
}


class Lines {
  int line = 10;
  Module[] mods;
  
  Lines(){ 
    mods = new Module[line*3];
     
    int index = 0; 
    for (int z = 0; z < line; z++) {
      mods[index++] = new Module(1, 1, z);
    }
    for (int y = 0; y < line; y++) { 
      mods[index++] = new Module(1, y, 1);
    }
    for (int x = 0; x < line; x++) { 
      mods[index++] = new Module(x, 1, 1);
    }
  }
  
  void draw() { 
    for (int i = 0; i < line*3; i++) { 
      mods[i].draw();
    }
  }
}

class Module {
  int xOffset; 
  int yOffset; 
  int zOffset; 
  float x, y;
 
  // Contructor 
  Module(int xOffsetTemp, int yOffsetTemp, int zOffset) { 
    xOffset = xOffsetTemp; 
    yOffset = yOffsetTemp;
    this.zOffset = zOffset;
  }
 
  // Custom method for drawing the object 
  void draw() { 
    noFill(); 
    stroke(255); 
    strokeWeight(0.3); 
    pushMatrix();
    translate(xOffset, yOffset, zOffset);
    box(1);
    popMatrix();
  }
}
