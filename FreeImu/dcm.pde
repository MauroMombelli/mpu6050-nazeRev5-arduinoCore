public class DCMlogic {
  float sampleFreq = 100;
  public float q0 = 1, q1 = 0, q2 = 0, q3 = 0;

  public float accelerationX = 0, accelerationY = 0, accelerationZ = 0;
  public float speedX = 0, speedY = 0, speedZ = 0;
  public float posX = 0, posY = 0, posZ = 0;
  //float twoKp = 2.0f * 1f;
  //float twoKi = 2.0f * 0.0f;
  private float KP_M = 2.0f;
  private float KP_A = 2.0f;  
  private float KI_M = 0;


  //long lastUp = System.nanoTime();
  long lastFreqUp=System.currentTimeMillis(), count=-1, countG=0, countM=0,countA=0;
  private float sampleFreqG=1600;
  private float sampleFreqA=1600;
  private float sampleFreqM=70;

  /*
  private float integralFBxA;
  private float integralFByA;
  private float integralFBzA;
  */
  
  private float integralFBxM = 0;
  private float integralFByM = 0;
  private float integralFBzM = 0;
  
  private long lastUpdateNs = 0;
  
  private long lastLog = 0;
  
  private long start = System.currentTimeMillis();

  private boolean valid(float x, float y, float z){
    if (x == 0.0f && y == 0.0f && z == 0.0f){
      return false;
    }
    
    if ( Float.isNaN(x) || Float.isNaN(y) || Float.isNaN(z) )
      return false;
      
    return true;
  }
  
  public void FreeIMUUpdate(float gx, float gy, float gz, float rawAx, float rawAy, float rawAz, float mx, float my, float mz){
    
    /* DINAMIC FREQUENCY! */
    if (count == -1){ //just the first time!
      lastFreqUp = System.currentTimeMillis();
    }
    count ++;
    if (System.currentTimeMillis()-lastFreqUp>=1000){
      System.out.println("Frequenza: "+count+" G: "+countG+" A: "+countA+" M: "+countM );
      sampleFreq = count;
      sampleFreqA = countA;
      sampleFreqM = countM;
      sampleFreqG = countG;
      count=countG=countA=countM=0;
      lastFreqUp = System.currentTimeMillis();
    }
    /* END DINAMIC FREQUENCY! */
    
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;
    float qa, qb, qc;

    // Auxiliary variables to avoid repeated arithmetic
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    // Use magnetometer measurement only when valid (avoids NaN in magnetometer normalisation)
    float halfwx=0, halfwy=0, halfwz=0;
    
    if( valid(mx, my, mz) ) {
      float hx, hy, bx, bz;
      
      countM++;
      
      // Normalise magnetometer measurement
      recipNorm = invSqrt(mx * mx + my * my + mz * mz);
      mx *= recipNorm;
      my *= recipNorm;
      mz *= recipNorm;

      // Reference direction of Earth's magnetic field
      hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
      hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
      bx = (float) Math.sqrt(hx * hx + hy * hy);
      bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

      // Estimated direction of magnetic field
      halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
      halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
      halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);
      
      /*
      float norm = invSqrt(halfwx*halfwx+halfwy*halfwy+halfwz*halfwz);
      halfwx*=norm;
      halfwy*=norm;
      halfwz*=norm;
      
      
      float freqMagne = (1.0f / sampleFreqM);
      integralFBxM += halfex * freqMagne * KI_M;
      integralFByM += halfey * freqMagne * KI_M;
      integralFBzM += halfez * freqMagne * KI_M;
      */
      
      halfex += KP_M * (my * halfwz - mz * halfwy);// * freqMagne + integralFBxM;
      halfey += KP_M * (mz * halfwx - mx * halfwz);// * freqMagne + integralFByM;
      halfez += KP_M * (mx * halfwy - my * halfwx);// * freqMagne + integralFBzM;
      
      if ( Float.isNaN(halfex) || Float.isNaN(halfey) || Float.isNaN(halfez) ){
        println("magne nan halfe");
      }
    }

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    float halfvx=0, halfvy=0, halfvz=0;
    if( valid(rawAx, rawAy, rawAz) ) {
      countA++;
      
      // Normalise accelerometer measurement
      recipNorm = invSqrt(rawAx * rawAx + rawAy * rawAy + rawAz * rawAz);
      
      if ( Float.isNaN(recipNorm) ){
        println("acce nan a recipNorm " +recipNorm);
      }
      float ax = rawAx * recipNorm;
      float ay = rawAy * recipNorm;
      float az = rawAz * recipNorm;
      if ( Float.isNaN(ax) || Float.isNaN(ay) || Float.isNaN(az) ){
        println("acce nan a pre " +halfvx+ " "+halfvy+" "+halfvz);
      }

      // Estimated direction of gravity
      halfvx = q1q3 - q0q2;
      halfvy = q0q1 + q2q3;
      halfvz = q0q0 - 0.5f + q3q3;
      if ( Float.isNaN(halfvx) || Float.isNaN(halfvy) || Float.isNaN(halfvz) ){
        println("acce nan halfv " +halfvx+ " "+halfvy+" "+halfvz);
      }
      
      halfex += KP_A * (ay * halfvz - az * halfvy) * (1.0f / sampleFreqA);
      halfey += KP_A * (az * halfvx - ax * halfvz) * (1.0f / sampleFreqA);
      halfez += KP_A * (ax * halfvy - ay * halfvx) * (1.0f / sampleFreqA);
      
      if ( Float.isNaN(halfex) || Float.isNaN(halfey) || Float.isNaN(halfez) ){
        println("acce nan halfe " +ax+ " "+ay+" "+az+" "+sampleFreqA + " " + (ay * halfvz - az * halfvy) + " " + (1.0f / sampleFreqA));
      }else{
        //println("acce halfe " +ax+ " "+ay+" "+az+" "+halfex + " " + halfey + " " + halfez);
      }
    }
    
    countG++;
    // Integrate rate of change of quaternion
    gx *= (1.0f / sampleFreqG);   // pre-multiply common factors
    gy *= (1.0f / sampleFreqG);
    gz *= (1.0f / sampleFreqG);
    
    if (Float.isNaN(gx) || Float.isNaN(gy) || Float.isNaN(gz)){
      println("gyro1 nan");
    }
      
    /*
    // Compute and apply integral feedback if enabled  
    if(twoKi > 0.0f) {
      integralFBx += twoKi * halfex;  // integral error scaled by Ki
      integralFBy += twoKi * halfey;
      integralFBz += twoKi * halfez;
      gx += integralFBx;  // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    }
    */
    
    if ( Float.isNaN(halfex) || Float.isNaN(halfey) || Float.isNaN(halfez) ){
      println("final nan halfe");
    }
    
    // Apply error feedback
    gx += halfex;
    gy += halfey;
    gz += halfez;
    
    if (Float.isNaN(gx) || Float.isNaN(gy) || Float.isNaN(gz)){
      println("gyro2 nan");
    }

    if (Float.isNaN(q0) || Float.isNaN(q1) || Float.isNaN(q2) || Float.isNaN(q3)){
      println("quat0 nan");
    }
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);
    if (Float.isNaN(q0) || Float.isNaN(q1) || Float.isNaN(q2) || Float.isNaN(q3)){
      println("quat1 nan");
    }

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
    if (Float.isNaN(q0) || Float.isNaN(q1) || Float.isNaN(q2) || Float.isNaN(q3)){
      println("quat2 nan");
      exit();
    }
    
    //------------------NOW WE ESTIOMATE THE MOTION
    if (System.currentTimeMillis()-start < 2000){
      return; //ignore first seconds
    }
    
    // Estimated direction of gravity, normalized
    float estGx = q1q3 - q0q2;
    float estGy = q0q1 + q2q3;
    float estGz = q0q0 - 0.5f + q3q3;
    if ( Float.isNaN(estGx) || Float.isNaN(estGy) || Float.isNaN(estGz) ){
      println("acce nan halfv " +estGx+ " "+estGy+" "+estGz);
    }
    // Normalized gravity to 9.8m/s
    estGx *= 9.8 * 2;
    estGy *= 9.8 * 2;
    estGz *= 9.8 * 2;
    
    //estimate real acceleration
    float divider = 0.00122194513;
    accelerationX = rawAx * divider - estGx;
    accelerationY = rawAy * divider - estGy;
    accelerationZ = rawAz * divider - estGz;
    
    int estGxraw = (int)(estGx / divider);
    int estGyraw = (int)(estGy / divider);
    int estGzraw = (int)(estGz / divider);
    
    accelerationX = rawAx - estGxraw;
    accelerationY = rawAy - estGyraw;
    accelerationZ = rawAz - estGzraw;
    
    /*
    System.out.println("est gravity: "+estGx+" "+estGy+" "+estGz );
    System.out.println("raw acce: "+rawAx+" "+rawAy+" "+rawAz);
    
    
    */
    
    if (abs(accelerationX) < 100){
      accelerationX = 0;
    }
    if (abs(accelerationY) < 100){
      accelerationY = 0;
    }
    if (abs(accelerationZ) < 100){
      accelerationZ = 0;
    }
    
    
    if (lastUpdateNs == 0)
      lastUpdateNs = System.nanoTime();
    long now = System.nanoTime();
    float deltaT = (now - lastUpdateNs) / 1000000000.0;
    lastUpdateNs = now;
    
    float moveX = accelerationX * deltaT;
    float moveY = accelerationY * deltaT;
    float moveZ = accelerationZ * deltaT;
    
    /*
    //rotate movement according to current rotation
    float moveRot[] = rotateVector(moveX, moveY, moveZ);
    //moveRot[0] MUST be 0 as it is irrational part
    speedX += moveRot[1];
    speedY += moveRot[2];
    speedZ += moveRot[3];
    */
    speedX += moveX;
    speedY += moveY;
    speedZ += moveZ;
    
    posX += speedX * deltaT;
    posY += speedY * deltaT;
    posZ += speedZ * deltaT;
    
    
    if (System.currentTimeMillis() - lastLog > 100){
      lastLog = System.currentTimeMillis();
      //println("> est acce: "+accelerationX+" "+accelerationY+" "+accelerationZ+" deltaT " + deltaT);
      //println("m/s acce: "+rawAx+" "+rawAy+" "+rawAz);
      //println("> speed: "+moveX+" "+moveY+" "+moveZ );
      //println("> speed: "+moveRot[1]+" "+moveRot[2]+" "+moveRot[3] );
      //println("> speed: "+speedX+" "+speedY+" "+speedZ );
      if (accelerationX == 0 && accelerationY == 0 && accelerationZ == 0){
        speedX = speedY = speedZ = 0;
      }
      //println("> pos: "+posX+" "+posY+" "+posZ );
      //println(">>> posizione: "+posX+" "+posY+" "+posZ );
    }
    
    
    /*
    //estimate the centripedal acceleration cused by rotation
    float r = ;
    float centAccX = (gx * gx * r * r) / (r);
    */
  }
  
  float[] quaternionMult(float r[], float q[]){
    return new float[] {r[0]*q[0]-r[1]*q[1]-r[2]*q[2]-r[3]*q[3],
            r[0]*q[1]+r[1]*q[0]-r[2]*q[3]+r[3]*q[2],
            r[0]*q[2]+r[1]*q[3]+r[2]*q[0]-r[3]*q[1],
            r[0]*q[3]-r[1]*q[2]+r[2]*q[1]+r[3]*q[0]};
  }

  float[] rotateVector(float x, float y, float z){
    float r[] = new float[]{0, x, y, z}; 
    float q_conj[] = new float[]{q0,-q1,-q2,-q3};
    float q[] = new float[]{q0, q1, q2, q3};
    return quaternionMult(quaternionMult(q,r),q_conj);
  }
  
  // ---------------------------------------------------------------------------------------------------
  // Fast inverse square-root
  // See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
  // also this is needed because otherwise we could get a NAN (it's also a bit
  // faster i guess)
  strictfp float invSqrt(float x) {
    /*
    float xhalf = 0.5f * x;
    int i = Float.floatToRawIntBits(x); // convert integer to keep the
    // representation IEEE 754

    i = 0x5f3759df - (i >> 1);
    x = Float.intBitsToFloat(i);
    x = x * (1.5f - xhalf * x * x);

    return x;*/
    return (float) (1/Math.sqrt(x));
  }

  public void update(float x, float y, float z, float x2, float y2, float z2, float x3, float y3, float z3) {
    FreeIMUUpdate(x, y, z, x2, y2, z2, x3, y3, z3);
  }
  
  public float[] getQ(){
    return new float[] {q0, q1, q2, q3};
  }
}
