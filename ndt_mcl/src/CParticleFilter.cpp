/**
* The cpp file for the particle filter
*/

#include "CParticleFilter.h"

using namespace mcl;

CParticleFilter::CParticleFilter(){
		Particles=NULL;
		Lik=0;
		outLiers=0;
		NumOfParticles=0;
		size=0;
		isAvgSet = false;
}
CParticleFilter::~CParticleFilter(){
		//myfree();
}
				
/**
 * Allocates memory for the distribution. You should do this before using the filter
 * @p num_particles The amount of particles you want to reserve
 */
void CParticleFilter::allocate(int num_particles){
		Particles = (TPoseParticle *) malloc(sizeof(TPoseParticle)*num_particles);
		//fprintf(stderr,"CParticleFilter::allocate():: Called and reserved %d bytes memory = %d particles\n", sizeof(TPoseParticle)*num_particles,num_particles );
		if(!Particles){
				fprintf(stderr,"Failed to reserve memory - exiting\n");
				exit(1);
		}
		
		tmp = (TPoseParticle *) malloc(sizeof(TPoseParticle)*num_particles);
		
		if(!tmp){
				fprintf(stderr,"Failed to reserve memory - exiting\n");
				exit(1);
		}
		
		size = num_particles;
		NumOfParticles=0;
		outLiers = 0;
		Lik=0;
}
/**
* Frees the reserved memory
*/
void CParticleFilter::myfree(){
		if(Particles) free(Particles);
		if(tmp) free(tmp);
		size = 0; 
		NumOfParticles = 0;
}
				
/**
* Get the mean values of the distribution
* @returns mcl::pose
*/
mcl::pose CParticleFilter::getDistributionMean(bool doWeighting){
		double sumX=0,sumY=0,sumA=0;
		int i;
		mcl::pose pos;
		double sumW=0;			
		isAvgSet = true;
						
		double dsum_a=0;
		
		double ax=0,ay=0;	
		if(doWeighting){
			for(i=0;i<NumOfParticles;i++){
				sumX += (double)Particles[i].p*(double)Particles[i].x;
				sumY += (double)Particles[i].p*(double)Particles[i].y;
				ax += (double)Particles[i].p * cos(Particles[i].a);
				ay += (double)Particles[i].p * sin(Particles[i].a);		
				sumW += (double)Particles[i].p; 
		}
			
			if(fabs(sumA)>10000){ 
				fprintf(stderr,"GANSKA ISOA (%.2f %.2f %.2f)\n",sumX,sumY,sumA);
				//usleep(1000*1000*10);
			}
			if( fabs(sumW-1.0)> 0.01){ 
					fprintf(stderr,"getDistributionMean::SUMW=%.2f\n",sumW); exit(1);
			}
			pos.set(sumX,sumY,atan2(ay,ax));
			//pos.toPI();
		}else{ 
			for(i=0;i<NumOfParticles;i++){
				sumX += Particles[i].x;
				sumY += Particles[i].y;
				ax += cos(Particles[i].a);
				ay += sin(Particles[i].a);
				
				
			}
			if(NumOfParticles==0) fprintf(stderr,"CParticleFilter::getDistributionMean():: WTF!!!!\n");
			pos.x = sumX / (float) NumOfParticles;
			pos.y = sumY / (float) NumOfParticles;
			pos.a =  atan2(ay,ax);
		}
		pos.toPI();				
		average = pos;
						
		return pos;
}
				
/**
* Calculate the variance of the distribution
*    //xx xy xa//
*    //xy yy ya//
*    //xa ya aa//
*/
Eigen::Matrix3d CParticleFilter::getDistributionVariances(){
		mcl::pose avg;
		int i;
						
		if(isAvgSet) avg = average;
		else avg = getDistributionMean();
						  
		double xx = 0, yy=0, xy=0, aax=0, aay=0;
		double ax = cos(avg.a);
		double ay = sin(avg.a);
		double w2 = 0;
		
				
		for(i=0;i<NumOfParticles;i++){
				xx += Particles[i].p*(Particles[i].x - avg.x)*(Particles[i].x - avg.x);
				yy += Particles[i].p*(Particles[i].y - avg.y)*(Particles[i].y - avg.y);
				xy += Particles[i].p*(Particles[i].x - avg.x)*(Particles[i].y - avg.y);
				
				aax += Particles[i].p*(cos(Particles[i].a) - ax)*(cos(Particles[i].a) - ax);
				aay += Particles[i].p*(sin(Particles[i].a) - ay)*(sin(Particles[i].a) - ay);
				w2 += Particles[i].p * Particles[i].p;
		}
		
		if(w2 == 1.0){
			fprintf(stderr,"CParticleFilter::getDistributionVariances -- w2=%lf Should not happen!\n",w2);
			w2 = 0.99;
		}
		double wc = 1.0 / (1.0 - w2);

		Eigen::Matrix3d cov;
		cov << wc*xx , wc*xy , 0,
					 wc*xy , wc*yy , 0,
					 0     , 0     , atan2(wc*aay, wc*aax);
		
		return cov;
}




/**
 * Initializes the filter by sampling from normal distribution with 
 * mean in @p0 and variance defined by @variance
 */
void CParticleFilter::initializeNormalRandom(mcl::pose p0, mcl::pose variance,int size){
		int i;
		
		if(this->size != size){
				this->allocate(size);
		}
		for(i=0;i<size;i++){
				Particles[i].x = p0.x + myrand.normalRandom() * variance.x;
				Particles[i].y = p0.y + myrand.normalRandom() * variance.y;
				Particles[i].a = p0.a + myrand.normalRandom() * variance.a;
				Particles[i].lik = 1.0;
				Particles[i].p = 1.0 / size;
				Particles[i].toPI();
		}
		NumOfParticles = size;
		isAvgSet = false;
}

/**
 * Initializes the distribution using uniform distribution
 * The box Pmin and Pmax sets the borders and dP sets the step size
 * @p Pmin the minimum values
 * @p Pmax maximum values
 * @p dP step size
 **/
void CParticleFilter::initializeUniform(mcl::pose Pmin, mcl::pose Pmax, mcl::pose dP){
		int Nx,Ny,Na; ///< number of particles for each component
		int i,j,k;
		
		if(dP.x==0 || dP.y==0 || dP.a==0){
				fprintf(stderr,"initializeUniform():: Invalid argument\n");
				return;
		}
		
		Nx = (int)((Pmax.x - Pmin.x)/dP.x);
		Ny = (int)((Pmax.y - Pmin.y)/dP.y);
		Na = (int)((Pmax.a - Pmin.a)/dP.a);
		///If something stupid should happen above
		if(Nx == 0) Nx = 1;
		if(Ny == 0) Ny = 1;
		if(Na == 0) Na = 1;
		
		fprintf(stderr,"initializeUniform()::Allocating (%d * %d * %d) =  %d particles\n",Nx,Ny,Na,Nx*Ny*Na);
		this->allocate(Nx*Na*Ny);
		
		for(i=0;i<Nx;i++){
				for(j=0;j<Ny;j++){
						for(k=0;k<Na;k++){
								Particles[NumOfParticles].x = Pmin.x + dP.x*i;
								Particles[NumOfParticles].y = Pmin.y + dP.y*j;
								Particles[NumOfParticles].a = Pmin.a + dP.a*k;
								Particles[NumOfParticles].lik = 1.0;
								Particles[NumOfParticles].p = 1.0 / (Nx*Ny*Na);
								Particles[NumOfParticles].toPI();
								NumOfParticles++;
						}
				}
		}
		isAvgSet = false;
}
/**
* Performs the Sample Importance Resampling (SIR) algorithm for the distribution
* The algorithm chooses the best particles (with respect to the probability) and 
* resamples these. 
* 
* You should have updated the likelihoods and normalized the distribution before running this
* Also, it might be smart not to run this in every iteration, since the distribution looses accuracy
* due to the "discretation"
**/
void CParticleFilter::SIRUpdate(){
		TPoseParticle *tmp2;
		float U=0,Q=0;
		int i=0,j=0,k=0;
	
		U = myrand.uniformRandom() / (float) NumOfParticles;
		//fprintf(stderr,"SIRUpdate()::U=%.6f\n",U);
		while(U < 1.0){
		
				if(Q>U){
						U += 1.0/NumOfParticles;
						
						if(k>=NumOfParticles || i>=NumOfParticles){
								while(i<NumOfParticles){
										tmp[i]=Particles[NumOfParticles-1];
										tmp[i].p = 1.0 / NumOfParticles;
										i++;
								}
								//fprintf(stderr,"ERROR: SIRupdate:: Invalid index k='%d' or i='%d'\n",k,i);
								tmp2 = Particles;
								Particles = tmp;
								tmp = tmp2;
								return;        
						}
						tmp[i]=Particles[k];
						tmp[i].p = 1.0 / NumOfParticles;
						i++;
				}
				else{
						j++;
						k=j;
						if(j>=NumOfParticles){
								while(i<NumOfParticles){
									tmp[i]=Particles[NumOfParticles-1];
									tmp[i].p = 1.0 / NumOfParticles;
									i++;
								}
								//fprintf(stderr,"ERROR: SIRupdate:: Invalid index j='%d' \n",j);
								tmp2 = Particles;
								Particles = tmp;
								tmp = tmp2;
								return;        
						}
						Q += Particles[j].p;
						///j++; ///WAS HERE until 30.7.2008
						
						if(j==NumOfParticles){
								while(i<NumOfParticles){
										tmp[i]=Particles[k-1];
										tmp[i].p = 1.0 / NumOfParticles;
										i++;
								}
								tmp2 = Particles;
								Particles = tmp;
								tmp = tmp2;
								return;
						}
				}
		}//While
		while(i<NumOfParticles){
			  if(k>=NumOfParticles) k=NumOfParticles-1;
				tmp[i]=Particles[k];
				tmp[i].p = 1.0 / NumOfParticles;
				i++;
		}
		isAvgSet = false;
		tmp2 = Particles;
		Particles = tmp;
		tmp = tmp2;
}

/**
 * Performs the normalization step
 * i.e. according to updated likelyhoods the probability of each 
 * particle is calculated and the whole distribution gets 
 * probablity of 1
 */
void CParticleFilter::normalize(){
		int i;
		double summ=0;
		isAvgSet = false;
		for(i=0;i<NumOfParticles;i++){
				Particles[i].p *= Particles[i].lik; 
				summ+=Particles[i].p;
		}
		if(summ != 0){
			for(i=0;i<NumOfParticles;i++){
				Particles[i].p = Particles[i].p/summ;
			}
		}else{
			for(i=0;i<NumOfParticles;i++){
				Particles[i].p = 1.0/NumOfParticles;
			}
		}
}


/**
 * Performs the prediction step
 * Moves the cloud according to estimated movement(@dp) and adds noise according to @std
 * Assumes independed noise (i.e the x,y,a are not correlated, which is not really true)
 * @param dP relative movement between the scans
 * @param std The standard deviation of the noise that is independently added to each component
**/
void CParticleFilter::predict(mcl::pose dP, mcl::pose std){
		int i;
		float dx=0.0,dy=0.0,dl=0.0;
		float a=0.0;
				
		//dl = sqrt(dP.x*dP.x+dP.y*dP.y);
		//a  = atan2(dP.y,dP.x);
		
		/**
		* Modified 25.9.2008 to induce the noise into the differential movement
		* instead of into the trajectory in world frame
		**/
		float dxe,dye; ///<estimates
		for(i=0;i<NumOfParticles;i++){
			///Generate noise from normal distribution
			dxe = dP.x + myrand.normalRandom()* std.x;
			dye = dP.y + myrand.normalRandom()* std.y;
			
			dl = sqrt(dxe*dxe+dye*dye);
			a  = atan2(dye,dxe);

			dx = dl * cos(Particles[i].a +a );
			dy = dl * sin(Particles[i].a +a );
		
			Particles[i].x = Particles[i].x + dx;
			Particles[i].y = Particles[i].y + dy;
			Particles[i].a = Particles[i].a + dP.a + myrand.normalRandom()* std.a;
			Particles[i].toPI();
		}
		isAvgSet = false;
}

/**
 * Performs prediction step with system noise covariance matrix
 * Moves the particles according to @dP and samples the noise 
 * Note that the angle noise is not correlated with the position
 * using covariance matrix @Q[4]
 * @p dP relative motion between steps
 * @p Q[4] Uncertainty as covariance (Var(X) Var(XY) Var(Y) STD(A)!!)
 * FIXME: NOT TESTED 
 */
void CParticleFilter::predict(mcl::pose dP, float Q[4]){
		int i;
		float dx,dy,dl;
		float cov[3];
		float xRand,yRand;
		float a,b,c;
		float eig1=1,eig2=1;
		float vx1,vy1,vx2,vy2;
		float d;
		float x,y;

	// calculate eigen values
		cov[0] = Q[0];
		cov[1] = Q[1];
		cov[2] = Q[2];
	
		a = 1.0;
		b = -(cov[0]+cov[2]);
		c = cov[0]*cov[2] - cov[1]*cov[1];
	
		if( (b*b-4*c)>=0){
				eig1 = (-b + sqrt(b*b-4*a*c))/(2*a);
				eig2 = (-b - sqrt(b*b-4*a*c))/(2*a);
		
		// calculate eigen vectors
				if(eig1 != eig2){
						vx1 = cov[1];
						vy1 = eig1 - cov[0];
						d = sqrt(vx1*vx1+vy1*vy1);
						vx1 /= d;
						vy1 /= d;
			
						vx2 = cov[1];
						vy2 = eig2 - cov[0];
						d = sqrt(vx2*vx2+vy2*vy2);
						vx2 /= d;
						vy2 /= d;
				}
				else{
						vx1 = 1.0;
						vx2 = 0.0;
						vy1 = 0.0;
						vy2 = 1.0;
				}
		
		}
		else{
				fprintf(stderr,"IMAGINARY EIGEN VALUES\n");
				eig1=1;
				eig2=1;
		
				vx1 = 1.0;
				vx2 = 0.0;
				vy1 = 0.0;
				vy2 = 1.0;
		}
		// Transform the random variable according to the covariance
	
		dl = sqrt(dP.x*dP.x+dP.y*dP.y);
	
		for(i=0;i<NumOfParticles;i++){
				xRand = myrand.normalRandom();
				yRand = myrand.normalRandom();
		
				x = xRand * sqrt(eig1);
				y = yRand * sqrt(eig2);
		
				xRand = x * vx1 + y * vx2;
				yRand = x * vy1 + y * vy2;

				dx = dl * cos(Particles[i].a + dP.a);
				dy = dl * sin(Particles[i].a + dP.a);
		
				///Transform the correlated noise according to the heading of the particle
				///FIXME: Remember to test the validity as this is not tested 
				float covX,covY,covA,covL;
				covL = sqrt(xRand*xRand + yRand*yRand);
				covA = atan2(yRand,xRand);
				covX = covL * cos(Particles[i].a+covA);
				covY = covL * sin(Particles[i].a+covA);
		
				Particles[i].x = Particles[i].x + dx + covX;
				Particles[i].y = Particles[i].y + dy + covY;
				Particles[i].a = Particles[i].a + dP.a + myrand.normalRandom()* Q[3];
				Particles[i].toPI();
		}
		isAvgSet = false;
}

/**
 * You Can use this to update the vector of likelihoods
 * Most probably you will use this update step so that
 * you give the whole filter to some likelihood function, which 
 * updates the public data TPoseParticle *Particles directly
 * however, this is here if you like to use it :)
 * @param *lik the vector of likelihoods for the particles 
 *		(has to be the same size as the number of particles)
 */
void CParticleFilter::updateLikelihood(float *lik){
		int i;
		for(i=0;i<NumOfParticles;i++){
				Particles[i].lik = lik[i];
		}
		isAvgSet = false;
}

/**
 * Prints some status data
 */
void CParticleFilter::print(){
		getDistributionMean();
		getDistributionVariances();
		fprintf(stderr,"Filter:: size=%d, AVG:(%.1f,%.1f,%.1f), VAR:(%.1f,%.1f,%.1f)\n",size, average.x,
						average.y,average.a, variance.x,variance.y,variance.a);
}



/**
 * Resizes the distribution so that n particles
 * from indices *ind will be saved
 * NOTE:: The memory will not be reallocated
 * @p n the size of the "new" vector
 */
void CParticleFilter::resize(int n){
		int i;
		TPoseParticle *tmp2;
		int *ind = (int *) malloc(NumOfParticles * sizeof(int));
		for(i=0;i<NumOfParticles;i++) ind[i] = i;
		hpsrt(ind); ///< find indices for the best ones
		for(i=0;i<10;i++) fprintf(stderr,"%d ",ind[i]);
		
		fprintf(stderr,"The best p=%f and worst %f \n",Particles[ind[0]].p,Particles[ind[NumOfParticles-1]].p);
		
		for(i=0;i<n;i++){
				tmp[i] = Particles[ind[NumOfParticles-1-i]];
		}
		
		
		tmp2 = Particles;
		Particles = tmp;
		tmp = tmp2;	
		NumOfParticles = n;	
		free(ind);
		fprintf(stderr,"Filter resized. New size=%d. The Pb=%f Pw=%f\n",n,Particles[0].p,Particles[NumOfParticles-1].p);
		
}

mcl::pose CParticleFilter::averageOverNBestAndRandomize(int N, int M,float vx,float vy,float va){
	
	if(N>=NumOfParticles){
		return getDistributionMean(true);
	}
	mcl::pose avg;
	if(N<=0) return avg;
	int *ind = (int *) malloc(NumOfParticles * sizeof(int));
	
	for(int i=0;i<NumOfParticles;i++) ind[i] = i;
	hpsrt(ind); ///< find indices for the best ones
	avg.x = 0;
	avg.y = 0;
	avg.a = 0;
	float ax=0,ay=0;
	
	for(int i=0;i<N;i++){
			avg.x+=Particles[ind[i]].x;
			avg.y+=Particles[ind[i]].y;
			ax += cos(Particles[ind[i]].a);
			ay += sin(Particles[ind[i]].a);
	}
	
	avg.x/=N;
	avg.y/=N;
	avg.a = atan2(ay,ax);
	
	if(M>0 && M<NumOfParticles){
		for(int i=NumOfParticles-1;i>NumOfParticles-M;i--){
			Particles[ind[i]].x = avg.x + myrand.normalRandom() * vx;
			Particles[ind[i]].y = avg.y + myrand.normalRandom() * vy;
			Particles[ind[i]].a = avg.a + myrand.normalRandom() * va;
		}
		
	}
	
	
	free(ind);
	return avg;
}

/**
 * Heap Sort algorithm
 * @param *indx The index of values after sort
 * @param N The number of values
 **/
void CParticleFilter::hpsrt(int * indx){
		unsigned long i,ir,j,k,ira;
		float rra;
		float *ra = (float *) malloc(NumOfParticles*sizeof(float));
		int n = NumOfParticles;
		//Arrange according to the probability
		for(i=0;i<NumOfParticles;i++) ra[i] = Particles[i].p; 
						
		if (n < 1) return;
		k=(n >> 1);
		ir=n-1;
		for (;;) 
		{
				if(k > 0)
				{ 
						rra=ra[--k];
						ira=indx[k];
				}
				else 
				{
						rra=ra[ir];
						ira=indx[ir];
						
						ra[ir]=ra[0];
						indx[ir]=indx[0];
						if (--ir == 0) 
						{
								ra[0]=rra; 
								indx[0]=ira; 
								break;
						}
				}
				i=k;
				j=k+1;
				while (j <= ir)
				{
						if (j < ir && ra[j] < ra[j+1]) j++;
						if (rra < ra[j])
						{
								ra[i]=ra[j];
								indx[i]=indx[j];  
								i=j;
								j <<= 1;
						} else break;
				}
				ra[i]=rra;
				indx[i]=ira;
		}
		free(ra);
}

void CParticleFilter::saveToFile(int Fileind){
		char name[20];
		FILE *f;
		
		sprintf(name,"particle%d.tx",Fileind);
		
		f= fopen(name,"wt");
		for(int i=0;i<NumOfParticles;i++){
				fprintf(f,"%.3f %.3f %.3f\n",Particles[i].x,Particles[i].y,Particles[i].a);
		}
		fclose(f);
}




