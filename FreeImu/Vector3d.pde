class Vector3d{
  int vec[] = new int[3];
    
  Vector3d(){
  }
  
  Vector3d(Vector3d v){
    for (int i = 0; i < 3; i++){
      vec[i] = v.vec[i];
    }
  }
  
  Vector3d(int x, int y, int z){
    vec[0] = x;
    vec[1] = y;
    vec[2] = z;
  }
  
  public String toString(){
    return "("+vec[0]+","+vec[1]+","+vec[2]+")";
  }
}
