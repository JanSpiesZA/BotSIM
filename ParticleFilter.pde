void resample()
{
  float W = 0.0;      //Sum of all the particles weights
  float beta = 0.0;
  float wm = 0.0;
  int index = int(random(maxParticles));  
  float alpha = 0.0;    //Normalised weight using all the added prob values of the particles
  Robot tempParticles[] = new Robot[maxParticles];  
  
  for(int k=0; k < maxParticles; k++)
  {
    tempParticles[k] = new Robot("PARTICLE");
  }  
  
  //Determines the biggest importance weight (prob)  
  for (int k = 0; k < maxParticles; k++)
  {      
    if (particles[k].prob > wm) 
    {
      wm = particles[k].prob;      
    }
  }  
   
  for (int i = 0; i < maxParticles; i++)
  {
   beta += random(0, 2*wm);   
   while (beta > particles[index].prob)
   {      
    beta -= particles[index].prob;
    index = (index + 1) % maxParticles;
   }   
   tempParticles[i].set(particles[index].x, particles[index].y, particles[index].heading);
   tempParticles[i].setNoise(noiseForward, noiseTurn, noiseSense);
  }
  
  ////Normalise the prob by dividing prob by the sum of all probs (W) and saving the value to alpha
  ////Calculates the sum of all the probabilities
  //for (int k=0; k < maxParticles; k++)
  //{
  //  W += particles[k].prob;    
  //}
  //for (int k=0; k < maxParticles; k++)
  //{
  //  particles[k].prob = particles[k].prob / W;
  //}
  
  particles = tempParticles;
  
  //arrayCopy(tempParticles, particles);
  
  
  //for (int k = 0; k < maxParticles; k++)
  //{
  //  println(k + ": "+particles[k].xPos + " : "+ particles[k].yPos +" : " + particles[k].noiseForward);
  //}
  
  //for (int k = 0; k < maxParticles; k++)
  //{
  //  fill(0,0,255);
  //  ellipse(tempParticles[k].xPos, tempParticles[k].yPos, 20,20);    
  //}
}    