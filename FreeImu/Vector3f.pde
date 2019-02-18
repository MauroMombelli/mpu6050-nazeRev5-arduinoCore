class Vector3f{
  float vec[] = new float[3];
    
  Vector3f(){
  }
  
  Vector3f(Vector3f v){
    for (int i = 0; i < 3; i++){
      vec[i] = v.vec[i];
    }
  }
  
  Vector3f(float x, float y, float z){
    vec[0] = x;
    vec[1] = y;
    vec[2] = z;
  }
  
  public String toString(){
    return "("+vec[0]+","+vec[1]+","+vec[2]+") "+lenght();
  }
  
  public float lenght(){
    return sqrt(vec[0]*vec[0]+vec[1]*vec[1]+vec[2]*vec[2]);
  }
  
  public void reset(){
    vec[0] = vec[1] = vec[2] = 0;
  }
}
