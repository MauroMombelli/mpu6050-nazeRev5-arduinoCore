class GyroCalibration{
  double sumX = 0;
  double sumY = 0;
  double sumZ = 0;
  long num = 0;
  
  Vector3f min = new Vector3f();
  Vector3f max = new Vector3f();
  
  Vector3f tmp = new Vector3f();
  
  void add(float x, float y, float z){
    tmp.vec[0] = x;
    tmp.vec[1] = y;
    tmp.vec[2] = z;
    add(tmp);
  }
  long resetMin = millis();
  long resetMax = millis();
  void add(Vector3f v){
    if (millis() - resetMin > 1000){
      min.reset();
    }
    if (millis() - resetMax > 1000){
      max.reset();
    }
    /*
    if (v.lenght() > 100){
      println("ignored "+v);
      return;
    }*/
    sumX += v.vec[0];
    sumY += v.vec[1];
    sumZ += v.vec[2];
    if (num == 0){
      min = new Vector3f(v);
      max = new Vector3f(v);
    }
    for (int i = 0; i < 3; i++){
      if (v.vec[i] > max.vec[i]){
        max.vec[i] = v.vec[i];
        resetMax = millis();
      }
      if (v.vec[i] < min.vec[i]){
        min.vec[i] = v.vec[i];
        resetMin = millis();
      }
    }
    num++;
  }
  
  float gyroLsbToDeg500dps = 0.0174532925;
  Vector3f getMean(){
    return new Vector3f((int)(sumX/num)*gyroLsbToDeg500dps, (int)(sumY/num)*gyroLsbToDeg500dps, (int)(sumZ/num)*gyroLsbToDeg500dps);
  }
  
  void reset(){
    sumX = sumY = sumZ = num = 0;
    min.reset();
    max.reset();
  }
  
  
  public String toString(){
    return "mean: "+getMean()+ "\nmin: "+min+"\nmax: "+max + " to rad:"+gyroLsbToDeg500dps;
  }
}
