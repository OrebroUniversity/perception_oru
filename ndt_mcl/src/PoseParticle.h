/**
 * A helper class for handling one 6D pose particle.
 * @author Jari Saarinen (jari.p.saarinen@gmail.com)
 */ 

#ifndef _POSE_PARTICLE_H_
#define _POSE_PARTICLE_H_

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
/**
 * Definition for 6D pose particle 
 *  
 */
class PoseParticle{
		public:
				Eigen::Affine3d T; ///<-- 6D pose represented as Eigen::Affine3d
				double p;					/// Probability of the particle <-- =Weight 
				double lik;				/// Likelihood of the particle <-- =Measurement likelihood
				
				PoseParticle(){
						p=0;
						lik=0;
				}
				PoseParticle(double x, double y, double z, double roll, double pitch, double yaw){
					this->set(x,y,z,roll,pitch,yaw);
				}
				
				
				/**
				 * Set the value with (x,y,z) and (roll, pitch, yaw) values
				 */ 
				void set(double x, double y, double z, double roll, double pitch, double yaw){
					Eigen::Matrix3d m;
					m = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
																* Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
																* Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
					
					Eigen::Translation3d v(x,y,z);
					T = v*m;
				}
				/**
				 * Get the value as RPY
				 */ 
				void getRPY(double &roll, double &pitch, double &yaw){
						Eigen::Vector3d rot = T.rotation().eulerAngles(0,1,2);
						roll=rot[0];
						pitch=rot[1];
						yaw = rot[2];
				}
				
				/**
				 * Get the value as x,y,z
				 */ 
				void getXYZ(double &x, double &y, double &z){
					Eigen::Vector3d tr = T.translation();
					x = tr[0];
					y = tr[1];
					z = tr[2];
				}
				
				
				
			
				
};

#endif
