/*
 * Implementation for the 6dof pose particle filtering
 */ 

/**
 * Initializes the filter using normally distributed random variables with given means (m-values) and standard deviations (v-values)
 */ 
void ParticleFilter3D::initializeNormalRandom(unsigned int NumParticles, double mx, double my, double mz, double mroll, double mpitch, double myaw,
																									 double vx, double vy, double vz, double vroll, double vpitch, double vyaw)
{
	for(unsigned int i=0;i<NumParticles;i++){
		
		double x = mx + myrand.normalRandom() * vx;
		double y = my + myrand.normalRandom() * vy;
		double z = mz + myrand.normalRandom() * vz;
		
		double roll 	= mroll + myrand.normalRandom() * vroll;
		double pitch = mpitch + myrand.normalRandom() * vpitch;
		double yaw 	= myaw + myrand.normalRandom() * vyaw;
		
		PoseParticle P(x,y,z,roll,pitch,yaw);
		P.lik = 1.0;
		P.p = 1.0 / (double) NumParticles;
		pcloud.push_back(P);
	} 
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
void ParticleFilter3D::SIRUpdate(){
		std::vector<PoseParticle> tmp;
		tmp.resize(pcloud.size());
		double U=0,Q=0;
		int i=0,j=0,k=0;
		
		int NumOfParticles = pcloud.size();
		U = myrand.uniformRandom() / (double) NumOfParticles;
		//fprintf(stderr,"SIRUpdate()::U=%.6f\n",U);
	
		
		while(U < 1.0){
		
				if(Q>U){ ///<-- Replicate the particle
						U += 1.0/(double)NumOfParticles;
						
						/// Check for index error 
						if(k>=NumOfParticles || i>=NumOfParticles){
								fprintf(stderr,"SIR error i=%d k=%d N=%d",i,k,NumOfParticles);
								break; ///Leave the loop 
						}
						tmp[i]=pcloud[k];
						tmp[i].p = 1.0 / (double)NumOfParticles;
						i++;
				}
				else{ ///Moving on
						j++;
						k=j;
						
						if(j>=NumOfParticles){ ///Index exceeded
								//fprintf(stderr,"SIR error(2) i=%d k=%d N=%d",i,k,NumOfParticles);
								break; ///Leave the loop     
						}
						Q += pcloud[j].p; ///< add the weight to cumulative sum
				}
		}//While
		
		if(i<(NumOfParticles-1)) fprintf(stderr,"SIR error(3) i=%d k=%d N=%d\n",i,k,NumOfParticles);
		while(i<NumOfParticles){ ///Make sure that the vector is filled
			  if(k>=NumOfParticles) k=NumOfParticles-1;
				tmp[i]=pcloud[k];
				tmp[i].p = 1.0 / NumOfParticles;
				i++;
		}
		
		pcloud = tmp;
}

/**
 * Performs the normalization step
 * i.e. according to updated likelyhoods the probability of each 
 * particle is calculated and the whole distribution gets 
 * probablity of 1
 */
void ParticleFilter3D::normalize(){
		int i;
		double summ=0;
		
		for(unsigned i=0;i<pcloud.size();i++){
				pcloud[i].p *= pcloud[i].lik; 
				summ+=pcloud[i].p;
		}
		if(summ > 0){
			for(i=0;i<pcloud.size();i++){
				pcloud[i].p = pcloud[i].p/summ;
			}
		}else{
			for(i=0;i<pcloud.size();i++){
				pcloud[i].p = 1.0/(double)pcloud.size();
			}
		}
}

void ParticleFilter3D::predict(Eigen::Affine3d Tmotion, double vx, double vy, double vz, double vroll, double vpitch, double vyaw){
	Eigen::Vector3d tr = Tmotion.translation();
	Eigen::Vector3d rot = Tmotion.rotation().eulerAngles(0,1,2);
	
	for(unsigned int i=0;i<pcloud.size();i++){
		double x = tr[0] + myrand.normalRandom() * vx;
		double y = tr[1] + myrand.normalRandom() * vy;
		double z = tr[2] + myrand.normalRandom() * vz;
		
		double roll 	= rot[0] + myrand.normalRandom() * vroll;
		double pitch = rot[1] + myrand.normalRandom() * vpitch;
		double yaw 	= rot[2] + myrand.normalRandom() * vyaw;
		
		pcloud[i].T = pcloud[i].T *(xyzrpy2affine(x,y,z,roll,pitch,yaw));
	}
}

Eigen::Affine3d ParticleFilter3D::getMean(){
	double mx=0, my=0,mz=0;
	//Eigen::Quaternion<double> qm;
	double roll_x = 0, roll_y=0;
	double pitch_x = 0, pitch_y=0;
	double yaw_x = 0, yaw_y=0;
	
	
	
	for(unsigned int i=0;i<pcloud.size();i++){		
			//Eigen::Quaternion<double> q(pcloud[i].T.rotation());
			//qm=qm+pcloud[i].p * q;
			Eigen::Vector3d tr = pcloud[i].T.translation();
			mx += pcloud[i].p * tr[0];
			my += pcloud[i].p * tr[1];
			mz += pcloud[i].p * tr[2];
			
			//Get as euler
			Eigen::Vector3d rot = pcloud[i].T.rotation().eulerAngles(0,1,2);
			roll_x+=pcloud[i].p*cos(rot[0]); 
			roll_y+=pcloud[i].p*sin(rot[0]);
			
			pitch_x+=pcloud[i].p*cos(rot[1]); 
			pitch_y+=pcloud[i].p*sin(rot[1]);
			
			yaw_x+=pcloud[i].p*cos(rot[2]); 
			yaw_y+=pcloud[i].p*sin(rot[2]);
	}
	return xyzrpy2affine(mx,my,mz, atan2(roll_y,roll_x), atan2(pitch_y,pitch_x), atan2(yaw_y,yaw_x));
	
	
	//qm.normalize();
	//Eigen::Matrix3d m;
	//m = qm.toRotationMatrix();
	//Eigen::Translation3d v(mx,my,mz);
}



