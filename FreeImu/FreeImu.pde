import processing.serial.*;
import processing.opengl.*;
import java.util.Vector;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.io.ByteArrayInputStream;

Serial myPort;  // Create object from Serial class

GyroCalibration gyroCalibration = new GyroCalibration();

Vector3d offsetGyro = new Vector3d(19, -37, 23);

final String serialPort = "/dev/ttyUSB0"; // replace this with your serial port. On windows you will need something like "COM1".

//float [] q = new float [4];
float [] hq = null;
float [] Euler = new float [3]; // psi, theta, phi

PFont font;
final int VIEW_SIZE_X = 800, VIEW_SIZE_Y = 600;

DCMlogic dcm = new DCMlogic();

Arrow magne = new Arrow();

SegmentedLine positions = new SegmentedLine();

final int burst = 32;
int count = 0;

void myDelay(int time) {
  try {
    Thread.sleep(time);
  } catch (InterruptedException e) { }
}

boolean serialRunning = false;

public void settings() {
  size(800, 800, "processing.opengl.PGraphics3D");
}

void setup() 
{
  myPort = new Serial(this, serialPort, 115200);
  
  // The font must be located in the sketch's "data" directory to load successfully
  font = loadFont("DroidSerif-48.vlw"); 
  
  println("Waiting IMU.. "+serialPort);
  
  myPort.clear();
  
  while (!serialRunning) {
    myPort.write("v/r/n");
    myDelay(1000);
    println("asking data");
  }
  println("end ask data");
/*  
  String[] args = {"YourSketchNameHere"};
  ShowMap sa = new ShowMap();
  PApplet.runSketch(args, sa);
*/
  /*
  println(myPort.readStringUntil('\n'));
  myPort.write("q" + char(burst));
  myPort.bufferUntil('\n');
  */
}


float decodeFloat(String inString) {
  byte [] inData = new byte[4];
  
  if(inString.length() == 8) {
    inData[0] = (byte) unhex(inString.substring(0, 2));
    inData[1] = (byte) unhex(inString.substring(2, 4));
    inData[2] = (byte) unhex(inString.substring(4, 6));
    inData[3] = (byte) unhex(inString.substring(6, 8));
  }
      
  int intbits = (inData[3] << 24) | ((inData[2] & 0xff) << 16) | ((inData[1] & 0xff) << 8) | (inData[0] & 0xff);
  return Float.intBitsToFloat(intbits);
}

short acce[] = new short[3];
short gyro[] = new short[3];
short magn[] = new short[3];
float remote_quad[] = null;
static float gyroLsbToRad500dps = 131f / 0.0174532925;
int c2 = 70000;
void serialEvent(Serial p) {
  int c1;
  while(p.available() >= 18) {
    serialRunning = true;
    if (c2 == 70000){
      c1 = p.read();
      c2 = p.read();
    }else{
      c1 = c2;
      c2 = p.read();
    }
    if (c1 == c2){
      c2 = 70000;
      short arr[] = null;
      float quad[] = null;
      switch(c1){
        case 'Q':
          quad = new float[4];
          break;
        case 'A':
          //print("reading acce ");
          arr = acce;
          try {
            String text = System.currentTimeMillis()+","+arr[0]+","+arr[1]+","+arr[2]+"\n";
            Files.write(Paths.get(dataPath("")+"/acce.txt"), text.getBytes(), StandardOpenOption.CREATE, StandardOpenOption.APPEND);
          }catch (IOException e) {
              //exception handling left as an exercise for the reader
              e.printStackTrace();
          }
          break;
        case 'G':
          //print("reading gyro ");
          arr = gyro;
          gyroCalibration.add(arr[0], arr[1], arr[2]);
          Vector3f offset = gyroCalibration.getMean();
          arr[0] -= offset.vec[0];
          arr[1] -= offset.vec[1];
          arr[2] -= offset.vec[2];
          try {
            String text = System.currentTimeMillis()+","+arr[0]+","+arr[1]+","+arr[2]+"\n";
            Files.write(Paths.get(dataPath("")+"/gyro.txt"), text.getBytes(), StandardOpenOption.CREATE, StandardOpenOption.APPEND);
          }catch (IOException e) {
              //exception handling left as an exercise for the reader
              e.printStackTrace();
          }
          
          break;
        case 'M':
          //print("reading magne ");
          arr = magn;
          //println(arr[0]+" "+arr[1]+" "+arr[2]);
          float sum = arr[0] + arr[1] + arr[2];
          magne.pointTo(arr[0]/sum, arr[1]/sum, arr[2]/sum);
          break;
        default:
          println("unknown "+c1);
      }
      if (arr != null){
        arr[0] = arr[1] = arr[2] = 0;
        arr[0] = (short)( (p.read() & 0x00ff) | (p.read() & 0x00ff)<<8 );
        arr[1] = (short)( (p.read() & 0x00ff) | (p.read() & 0x00ff)<<8 );
        arr[2] = (short)( (p.read() & 0x00ff) | (p.read() & 0x00ff)<<8 );
        if (Math.abs(arr[0]) > 30000 || Math.abs(arr[1]) > 30000 || Math.abs(arr[0]) > 30000){
          println("possible sensor overflow "+c1);
        }
        //println(arr[0]+" "+arr[1]+" "+arr[2]);
        if ( 
          (acce[0] != 0 || acce[1] != 0 || acce[2] != 0) && 
          (gyro[0] != 0 || gyro[1] != 0 || gyro[2] != 0)  && 
          (magn[0] != 0 || magn[1] != 0 || magn[2] != 0)
        ){
          //dcm.FreeIMUUpdate(gyro[0] / gyroLsbToRad500dps, gyro[1] / gyroLsbToRad500dps, gyro[2] / gyroLsbToRad500dps, acce[0], acce[1], acce[2], magn[0], magn[1], magn[2]);
          dcm.FreeIMUUpdate(gyro[0] / gyroLsbToRad500dps, gyro[1] / gyroLsbToRad500dps, gyro[2] / gyroLsbToRad500dps, acce[0], acce[1], acce[2], 0.0f, 0.0f, 0.0f);
          //println("with magne");
          //positions.add(new float[]{dcm.posX, dcm.posY, dcm.posZ});
        }else if ( (acce[0] != 0 || acce[1] != 0 || acce[2] != 0) && (gyro[0] != 0 || gyro[1] != 0 || gyro[2] != 0) ){
          //only gyro
          //dcm.FreeIMUUpdate(gyro[0] / gyroLsbToRad500dps, gyro[1] / gyroLsbToRad500dps, gyro[2] / gyroLsbToRad500dps, 0, 0, 1, 0.0f, 0.0f, 0.0f);
          
          //only acce
          //dcm.FreeIMUUpdate(0.0f, 0.0f, 0.0f, (float)acce[0], (float)acce[1], (float)acce[2], 0.0f, 0.0f, 0.0f);
          
          //only magne
          //m.FreeIMUUpdate(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, (float)magn[0], (float)magn[1], (float)magn[2]);
          
          //gyro + acce
          dcm.FreeIMUUpdate(gyro[0] / gyroLsbToRad500dps, gyro[1] / gyroLsbToRad500dps, gyro[2] / gyroLsbToRad500dps, acce[0], acce[1], acce[2], 0.0f, 0.0f, 0.0f);
          
        }
      }
      if (quad != null){
        println("reading Q "+p.available());
        quad[0] = quad[1] = quad[2] = quad[3] = 0;
        byte[] inBuffer = myPort.readBytes(16);
        ByteBuffer buf = ByteBuffer.wrap(inBuffer);
        buf.order(ByteOrder.LITTLE_ENDIAN);
        try{
          println("Got Quaternion! " + inBuffer.length);
          quad[0] = buf.getFloat();
          quad[1] = buf.getFloat();
          quad[2] = buf.getFloat();
          quad[3] = buf.getFloat();
          remote_quad = quad;
        }catch(Exception e){
          println("excpetion reading float: " +e);
        }
      }
    }else{
      //print((char)c1);
      //print(c2);
    }
  }
}



void buildBoxShape() {
  //box(60, 10, 40);
  noStroke();
  beginShape(QUADS);
  
  //Z+ (to the drawing area)
  fill(#00ff00);
  vertex(-30, -5, 20);
  vertex(30, -5, 20);
  vertex(30, 5, 20);
  vertex(-30, 5, 20);
  
  //Z-
  fill(#0000ff);
  vertex(-30, -5, -20);
  vertex(30, -5, -20);
  vertex(30, 5, -20);
  vertex(-30, 5, -20);
  
  //X-
  fill(#ff0000);
  vertex(-30, -5, -20);
  vertex(-30, -5, 20);
  vertex(-30, 5, 20);
  vertex(-30, 5, -20);
  
  //X+
  fill(#ffff00);
  vertex(30, -5, -20);
  vertex(30, -5, 20);
  vertex(30, 5, 20);
  vertex(30, 5, -20);
  
  //Y-
  fill(#ff00ff);
  vertex(-30, -5, -20);
  vertex(30, -5, -20);
  vertex(30, -5, 20);
  vertex(-30, -5, 20);
  
  //Y+
  fill(#00ffff);
  vertex(-30, 5, -20);
  vertex(30, 5, -20);
  vertex(30, 5, 20);
  vertex(-30, 5, 20);
  
  endShape();
}


void drawCube() {  
  pushMatrix();
    translate(VIEW_SIZE_X/2, VIEW_SIZE_Y/2 + 50, 0);
    scale(5,5,5);
    
    // a demonstration of the following is at 
    // http://www.varesano.net/blog/fabio/ahrs-sensor-fusion-orientation-filter-3d-graphical-rotating-cube
    rotateZ(-Euler[2]);
    rotateX(-Euler[1]);
    rotateY(-Euler[0]);
    
    buildBoxShape();
    
  popMatrix();
}

void drawCube2() {
  if (remote_quad != null){
    float angle[] = new float[3];
    quaternionToEuler(remote_quad, angle);
    pushMatrix();
      translate(VIEW_SIZE_X/4, VIEW_SIZE_Y/4 + 50, 0);
      scale(2,2,2);
      
      // a demonstration of the following is at 
      // http://www.varesano.net/blog/fabio/ahrs-sensor-fusion-orientation-filter-3d-graphical-rotating-cube
      rotateZ(-angle[2]);
      rotateX(-angle[1]);
      rotateY(-angle[0]);
      
      buildBoxShape();
      
    popMatrix();
  }
}


void draw() {
  background(#000000);
  fill(#ffffff);
  
  if(hq != null) { // use home quaternion
    quaternionToEuler(quatProd(hq, dcm.getQ()), Euler);
    text("Disable home position by pressing \"n\"", 20, VIEW_SIZE_Y - 30);
  } else {
    quaternionToEuler(quatConjugate(dcm.getQ()), Euler);
    text("Point FreeIMU's X axis to your monitor then press \"h\"", 20, VIEW_SIZE_Y - 30);
  }
  
  textFont(font, 20);
  textAlign(LEFT, TOP);
  text("Q:\n" + dcm.getQ()[0] + "\n" + dcm.getQ()[1] + "\n" + dcm.getQ()[2] + "\n" + dcm.getQ()[3], 20, 20);
  if (remote_quad != null){
    text("R Q:\n" + remote_quad[0] + "\n" + remote_quad[1] + "\n" + remote_quad[2] + "\n" + remote_quad[3], 200, 20);
  }
  text("Euler Angles:\nYaw (psi)  : " + degrees(Euler[0]) + "\nPitch (theta): " + degrees(Euler[1]) + "\nRoll (phi)  : " + degrees(Euler[2]), 20, 200);
  text("gyroCalibration: " + gyroCalibration, 400, 20);
  
  drawCube();
  drawCube2();
  magne.drawarrow();
  
  positions.draw();
  //myPort.write("q" + 1);
}


void keyPressed() {
  if(key == 'h') {
    println("pressed h");
    // set hq the home quaternion as the quatnion conjugate coming from the sensor fusion
    hq = quatConjugate(dcm.getQ());
  }
  else if(key == 'n') {
    println("pressed n");
    hq = null;
  }
  
  if(key == 'g') {
    println("pressed g");
    // set hq the home quaternion as the quatnion conjugate coming from the sensor fusion
    gyroCalibration.reset();
  }
}

// See Sebastian O.H. Madwick report 
// "An efficient orientation filter for inertial and intertial/magnetic sensor arrays" Chapter 2 Quaternion representation

void quaternionToEuler(float [] q, float [] euler) {
  euler[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1); // psi
  euler[1] = -asin(2 * q[1] * q[3] + 2 * q[0] * q[2]); // theta
  euler[2] = atan2(2 * q[2] * q[3] - 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1); // phi
}

float [] quatProd(float [] a, float [] b) {
  float [] q = new float[4];
  
  q[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
  q[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
  q[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
  q[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
  
  return q;
}

// returns a quaternion from an axis angle representation
float [] quatAxisAngle(float [] axis, float angle) {
  float [] q = new float[4];
  
  float halfAngle = angle / 2.0;
  float sinHalfAngle = sin(halfAngle);
  q[0] = cos(halfAngle);
  q[1] = -axis[0] * sinHalfAngle;
  q[2] = -axis[1] * sinHalfAngle;
  q[3] = -axis[2] * sinHalfAngle;
  
  return q;
}

// return the quaternion conjugate of quat
float [] quatConjugate(float [] quat) {
  float [] conj = new float[4];
  
  conj[0] = quat[0];
  conj[1] = -quat[1];
  conj[2] = -quat[2];
  conj[3] = -quat[3];
  
  return conj;
}

public class SegmentedLine {
  Vector<float[]> points = new Vector();
  
  public void add(float p[]) {
    points.add(p);
  }
  
  public void draw() {
    for (int i = 0; i < points.size(); i++){
      float[] p = points.get(i);
      pushMatrix();
      translate(p[0]+100, p[1]+100, p[2]+100);
      sphere(4);
      popMatrix();
    }
  }
}

public class Arrow {
  public float versor[] = new float[3];
 
  public void pointTo(float x, float y, float z){
    versor[0] = x;
    versor[1] = y;
    versor[2] = z;
  }
 
  public void drawarrow() {
    stroke(0,0,255);
    pushMatrix();
      translate(100, 500, 0);

      float x = versor[0];
      float y = versor[1];
      float z = versor[2];
      
      PVector d = new PVector(x, y, z);
      PVector u = new PVector(0, 0, 1);
      PVector w0 = new PVector(-y, x, 0);
      PVector u0 = w0.cross(u);
      
      double distW0 = Math.sqrt(x*x + y*y);
      double distU0 = Math.sqrt(u0.x*u0.x + u0.y*u0.y);
      
      double heading = Math.atan2(y,x);
      double pitch = Math.asin(z);
      double roll = Math.atan2( w0.dot(u),  u0.dot(u) / distW0 * distU0);
      double dist = Math.sqrt(x*x + y*y + z*z);

      rotateZ( (float)heading );
      rotateX( (float)roll );
      rotateY( (float)pitch );
      
      buildBoxShape();
      
    popMatrix();
  }
  
  void buildBoxShape() {
    //box(60, 10, 40);
    noStroke();
    beginShape(QUADS);
    
    //Z+ (to the drawing area)
    fill(#00ff00); //green
    vertex(0, 0, 5);
    vertex(30, 0, 5);
    vertex(30, 5, 5);
    vertex(0, 5, 5);
    
    //Z-
    fill(#0000ff); //blue
    vertex(0, 0, 0);
    vertex(30, 0, 0);
    vertex(30, 5, 0);
    vertex(0, 5, 0);
    
    //X-
    fill(#ff0000); //ref
    vertex(0, 0, 0);
    vertex(0, 0, 5);
    vertex(0, 5, 5);
    vertex(0, 5, 0);
    
    //X+
    fill(#ffff00); //yellow
    vertex(30, 0, 0);
    vertex(30, 0, 5);
    vertex(30, 5, 5);
    vertex(30, 5, 0);
    
    //Y-
    fill(#ff00ff); //magenta
    vertex(0, 0, 0);
    vertex(30, 0, 0);
    vertex(30, 0, 5);
    vertex(0, 0, 5);
    
    //Y+
    fill(#00ffff); //cyan
    vertex(0, 5, 0);
    vertex(30, 5, 0);
    vertex(30, 5, 5);
    vertex(0, 5, 5);
    
    endShape();
  }
}
