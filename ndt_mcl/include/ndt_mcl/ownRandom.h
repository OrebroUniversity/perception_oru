/**
 * A helper class for calculating random numbers
 * @author Jari Saarinen (jari.p.saarinen@gmail.com)
 */ 

#ifndef _OWN_RANDOM_H_
#define _OWN_RANDOM_H_

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include <vector>

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
/// Class for random number generation
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
class ownRandom{
		public:
				float *normalRandomCache;
				bool isOk;
				int size;
				
				ownRandom(){
						isOk=false;
						size = 0;
						allocate(10000);
						srand( (unsigned)time( NULL ) ); ///< set new seed
				}
				~ownRandom(){
				
				}
				///Allocate the memory for cache
				void allocate(int m_size){
						if(size != 0){ 
								if(normalRandomCache)free(normalRandomCache);
						}
						 
						normalRandomCache = (float *) malloc(sizeof(float)*m_size);
						isOk=true;
						size = m_size;
				}
				
				void fillCache(){
						if(size > 0 && normalRandomCache != NULL){
								for(int i=0;i<size;i++){
										normalRandomCache[i] = normalRandom(); 
										isOk = true;     
								}
						}else{
								allocate(10000);
								fillCache();
						}
				}
				/**
				* Gets the uniform random number from cache 
				* 
				*/
				float getCachedUniformRandom(){
						int rv;
						float scale;
						
						if(!isOk) fillCache();
						
						scale = (float) size / (float)RAND_MAX;
						rv = rand(); 
						rv = (int) ((float) rv * scale);
						if(rv >=size){
								fprintf(stderr,"ERROR in getCachedUniformRandom(). rv=%d\n",rv);
								rv = size-1; ///< Just for precaution
						}
						return normalRandomCache[rv];
				}
				
				/**
				 * Returns a (pseudo) random number between 0 and 1.
				 **/
				float uniformRandom(void){
						return ( ((float) rand() / (float) RAND_MAX));
				}
				
				/**
				* Returns normally distributed random number
				* Zero mean and unit variance.
				*/
				float normalRandom()
				{
						static float gset;
						static int randomStored = 0;
						float fac,rsq,v1,v2;

						if(randomStored){
								return gset;
						}
    
						do {
								v1=2.0*uniformRandom()-1.0; //pick two uniform numbers in the square extending
																						//from -1 to +1 in each direction, 
								v2=2.0*uniformRandom()-1.0;
								rsq=v1*v1+v2*v2;		//see if they are in the unit circle,
						} while (rsq >= 1.0 || rsq == 0.0); //if not try again
	
						fac=sqrt(-2.0*log(rsq)/rsq);
						gset=v1*fac; // can be used also as normal distributed random number!!
						return v2*fac;
				}
				
				/**
				* Returns a sample from given covariance "ellipse"
				 * cov[0] = X - variance
				 * cov[1] = XY -covariance
				 * cov[2] = Y - variance
				 * @return xRand and yRand the random value  
				*/
				void covRandom(float& xRand, float& yRand, float cov[3]){
						float a,b,c;
						float eig1,eig2;
						float vx1,vy1,vx2,vy2;
						float d;
						float x,y;
						
						xRand = normalRandom();
						yRand = normalRandom();
						// calculate eigen values
						a = 1.0;
						b = -(cov[0]+cov[2]);
						c = cov[0]*cov[2] - cov[1]*cov[1];

						eig1 = (-b + sqrt(b*b-4*a*c))/(2*a);
						eig2 = (-b - sqrt(b*b-4*a*c))/(2*a);

						// calculate eigen vectors
						vx1 = cov[1];
						vy1 = eig1 - cov[0];
						d = sqrt(vx1*vx1+vy1+vy1);
						vx1 /= d;
						vy1 /= d;

						vx2 = cov[1];
						vy2 = eig2 - cov[0];
						d = sqrt(vx2*vx2+vy2+vy2);
						vx2 /= d;
						vy2 /= d;
	
						// Transform the random variable according to the covariance
						x = xRand * sqrt(eig1);
						y = yRand * sqrt(eig2);
	
						xRand = x * vx1 + y * vx2;
						yRand = x * vy1 + y * vy2;
				}
};
#endif

