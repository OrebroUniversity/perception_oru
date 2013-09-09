/**
* This is an implementation of a particle filter
* Short howto:
 * Create  -> CParticleFilter filt;
 * Init    -> filt.initializeNormalRandom(init_pose, variance,size_of_filter); ///also uniform init
 * Predict -> filt.predict(delta_movement, std_of_noise)
 
 * Next Calculate the likehood using some likelihood function, which updates  
 * LikelihoodFunction(&filt) where you update the particle likelihood manually 
 * (filt.Particles[i].lik) or then you calculate the likelihood vector and use
 * updateLikelihood(float *lik) - function
 * 
 * Next make the normalization -> filt.normalize();
 * 
 * Now you can go back to predict or then you can resample the distribution by 
 * calling filt.SIRupdate();
 * @author Jari Saarinen (jari.p.saarinen@gmail.com)
*/
#ifndef C_PARTICLE_FILTER_H_
#define C_PARTICLE_FILTER_H_

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include <vector>
#include <Eigen/Core>

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



//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
/// Definitions for pose and scan (uses namespace mcl)
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
namespace mcl{
    /**
		 * Definition of scan for Scan Match interface
		 * Uses the polar coordinate format (r,a) pairs. 
		 * Natural for sensor coordinates and used as sensor coordinates here!
		 */
		struct scan{
				public:
						float *r;  ///< Distance measurement vector [m]
						float *a;  ///< Angle measurement of distance measurement [rad] 
						int N;     ///< The number of measurements in vector
						scan(){
								N=0;
								r=NULL;
								a=NULL;
						}
						scan(const scan &s) {
						   *this = s;
						}
						scan &operator=(const scan &s) {
								s.copy(*this);
								return *this;
						}/*
						~scan(){
								if(r)free(r);
								if(a)free(a);
								r=NULL;a=NULL;
						}*/
	   			void allocate(int n){
							if(r) free(r);
							if(a) free(a);
								r = (float *) malloc(n*sizeof(float));
								a = (float *) malloc(n*sizeof(float));
								N=n;
						}
						void copy(struct scan &new_scan) const {
								new_scan.r = (float *) malloc(N*sizeof(float));
								new_scan.a = (float *) malloc(N*sizeof(float));
								new_scan.N = N;
								memcpy(new_scan.r,r,N*sizeof(float));
								memcpy(new_scan.a,a,N*sizeof(float));
						}
						void set(float *rr,float *aa){
							if(N<=0) return;
								for(int i=0;i<N;i++){
										r[i]=rr[i];
										a[i]=aa[i];
								}
						}
		};
	/**
		 * ScanMatch pose definition
		 * 2D-pose x,y and heading 
	 */
		struct pose{
				public:
						float x; ///<[m]
						float y; ///<[m]
						float a; ///<[rad]
						pose(){
								x=0;
								y=0;
								a=0;
						}
						pose(float _x,float _y,float _a){
							x=_x;y=_y;a=_a;
						}
						void set(pose &p){
								x=p.x;
								y=p.y;
								a=p.a;
						}
						void set(float xx,float yy, float aa){
								x=xx;
								y=yy;
								a=aa;
						}
						/** 
						* Calculates differential movement based on two global positions
						* i.e. odometric positions 
						*/
						void setToDifferentialPose(pose odo_cur,pose odo_ref){					
								float ddx,ddy,dist,alpha;
								pose tmp;
			         ///Calculates the differential movement in odometry frame of reference
								ddx = odo_cur.x - odo_ref.x;
								ddy = odo_cur.y - odo_ref.y;
								alpha = atan2(ddy, ddx);
								dist = sqrt(ddx*ddx+ddy*ddy);
	
								tmp.x = dist * cos( alpha - odo_ref.a );
								tmp.y = dist * sin( alpha - odo_ref.a );
								tmp.a = (float)fmod((float)(odo_cur.a - odo_ref.a), (float)(2.0*M_PI)); 
								if(tmp.a < 0) tmp.a += 2*(float)M_PI;
								x = tmp.x;
								y= tmp.y;
								a = tmp.a;		
			
						}
							/**
						 * Integrates new position from this position and differential position 
						 * @param diff the differential position to be added to this position
						 * @return pose New integrated position
							 **/
						const pose integrateDifferential(const pose diff){
							pose result;
							float l,phii;
		    
							l = sqrt(diff.x*diff.x + diff.y*diff.y);
							phii = atan2(diff.y,diff.x);
				
							result.x = x + (float)cos(a+phii)*l;
							result.y = y + (float)sin(a+phii)*l;

		    // Update angle
							result.a = (float)fmod((float)(diff.a + a),(float)( 2*M_PI));
		    // fmod accepts also negative values
							if(result.a < 0) result.a += 2*(float)M_PI;
							return result; 
						}
						void to2PI(){
								a = (float)fmod((float)(a),(float)( 2*M_PI));
								if(a < 0) a += 2*(float)M_PI;
						}
						void toPI(){
							int cnt=0;
								if(a>M_PI) while(a>M_PI){ 
									a-=2.0*M_PI;
									cnt++;
									//if(cnt%10000==0) fprintf(stderr,"Loopping alot a=%.2f cnt=%d\n",a,cnt);
								}
								else if(a<-M_PI) while(a<-M_PI){ 
									a+=2.0*M_PI;
									cnt++;
									//if(cnt%10000==0) fprintf(stderr,"Loopping alot a=%.2f cnt=%d\n",a,cnt);
								}
						}
						
						~pose(){}
		
		};
		

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
/// Definition of one particle
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
struct TPoseParticle{
		public:
				float x;	// X - coordinate of the particle
				float y;	// Y - coordinate of the particle
				float a;	// Heading
				float p;	// Probability of the particle 
				float lik;	// Likelihood of the particle
				TPoseParticle(){
						x=0;
						y=0;
						a=0;
						p=0;
						lik=0;
				}
				void to2PI(){
						a = (float)fmod((float)(a),(float)( 2*M_PI));
						if(a < 0) a += 2*(float)M_PI;
				}
				void toPI(){
						if(a>M_PI) while(a>M_PI) a-=2.0*M_PI;
						else if(a<-M_PI) while(a<-M_PI) a+=2.0*M_PI;
				}
};



//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
/// The Particle filter
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

class CParticleFilter{
		public:
				TPoseParticle *Particles;	       ///< Particle distribution
				float Lik;					             ///< Likelihood of the distribution
				float outLiers;			             ///< Percentage of the outlier measurements in distribution
				int NumOfParticles;			         ///< Number of particles in distribution
				int size;					               ///< reserved memory Size (number of reserved particles!) 
				mcl::pose average;                ///< Distribution mean
				mcl::pose variance;              ///< Distribution variance
				bool isAvgSet;                   ///< Tells if the distribution mean has been calculated
				ownRandom myrand;                 ///< The random number class
				
				CParticleFilter();               ///<default constructor
				~CParticleFilter();              ///<destructor
				/**
				 * Allocates memory for the distribution
				 * @p num_particles The amount of particles you want to reserve
				*/
				void allocate(int num_particles);
				/**
				* Frees the reserved memory
				*/
				void myfree();
				
				/**
				* Get the mean values of the distribution
				* @returns mcl::pose
				*/
				mcl::pose getDistributionMean(bool doWeighting=false);
				/**
				* Computes the best indices and returns the average
				* @param N The best particles
				* @param M Option to randomize the worst M
				* @param vx variance x
				* @param vy variance y
				* @param va variance a
				*/
				mcl::pose averageOverNBestAndRandomize(int N, int M=0,float vx=0,float vy=0,float va=0);
				/**
				* Calculate the variance of the distribution
				*/
				Eigen::Matrix3d getDistributionVariances();
				
				/**
				* Initializes the filter by sampling from normal distribution with 
				* mean in @p0 and variance defined by @variance
				*/
				void initializeNormalRandom(mcl::pose p0, mcl::pose variance,int size);
				
				/**
				* Initializes the distribution using uniform distribution
				* The box Pmin and Pmax sets the borders and dP sets the step size
				* @p Pmin the minimum values
				* @p Pmax maximum values
				* @p dP step size
				**/
				void initializeUniform(mcl::pose Pmin, mcl::pose Pmax, mcl::pose dP);
				
				/**
				 * Performs the Sample Importance Resampling (SIR) algorithm for the distribution
				 * The algorithm chooses the best particles (with respect to the probability) and 
				 * resamples these. 
				 * 
				 * You should have updated the likelihoods and normalized the distribution before running this
				 * Also, it might be smart not to run this in every iteration, since the distribution looses accuracy
				 * due to the "discretation"
				 **/
				void SIRUpdate();
				
				/**
				* Performs the normalization step
				* i.e. according to updated likelyhoods the probability of each 
				* particle is calculated and the whole distribution gets 
				* probablity of 1
				*/
				void normalize();
				
				/**
				* Performs the prediction step
				* Moves the cloud according to estimated movement(@dp) and adds noise according to @std
				* Assumes independed noise (i.e the x,y,a are not correlated, which is not really true)
				* @param dP relative movement between the scans
				* @param std The standard deviation of the noise that is independently added to each component
				**/
				void predict(mcl::pose dP, mcl::pose std);
				
				/**
				* Performs prediction step with system noise covariance matrix
				* Moves the particles according to @dP and samples the noise 
				* Note that the angle noise is not correlated with the position
				* using covariance matrix @Q[4]
				* @p dP relative motion between steps
				* @p Q[4] Uncertainty as covariance (Var(X) Var(XY) Var(Y) STD(A)!!)
				*/
				void predict(mcl::pose dP, float Q[4]);
				
				/**
				* You Can use this to update the vector of likelihoods
				* Most probably you will use this update step so that
				* you give the whole filter to some likelihood function, which 
				* updates the public data TPoseParticle *Particles directly
				* however, this is here if you like to use it :)
				* @param *lik the vector of likelihoods for the particles 
				*		(has to be the same size as the number of particles)
				*/
				void updateLikelihood(float *lik);
				
				/**
				* Resizes the distribution so that n particles
				* from indices *ind will be saved
				* NOTE:: The memory will not be reallocated
				
				* @p n the size of *ind and the "new" vector
				*/
				void resize(int n);
				
				/**
				* Prints some status data
				*/
				void print();
				void saveToFile(int Fileind);
								
private:
		TPoseParticle *tmp;	///< Particle distribution for SIR (Maintained for efficiency reasons here)
		/**
		 * Heap Sort algorithm
		 * @param *indx The index of values after sort
		 * @param N The number of values
		 **/
		void hpsrt(int * indx);
		
};
}
#endif

