#include <stdlib.h>
#include <iostream>
#include <assert.h>
#include <ndt_visualisation/NDTVizGlut.hh>

using namespace std;

int main(int argc, char *argv[]) 
{
    std::cout << "test program" << std::endl;

     NDTVizGlut glut3d;
     glut3d.win_run(&argc, argv);
     {
         NDTVizGlutEllipsoid* el = new NDTVizGlutEllipsoid(1., 1., false, 40);
         el->setPos(Eigen::Vector3d(1., 1., 1.));
         Eigen::Matrix3d cov = Eigen::Matrix3d::Zero(3,3);
         cov(0,0) = 1.;
         cov(1,1) = 1.5;
         cov(2,2) = 0.5;
         el->setCov(cov);
         glut3d.addObject(el);
     }
     
     {
         NDTVizGlutEllipsoid* el = new NDTVizGlutEllipsoid(1., 1., true, 40);
         el->setPos(Eigen::Vector3d(4., 3., 1.));
         Eigen::Matrix3d cov = Eigen::Matrix3d::Zero(3,3);
         cov(0,0) = 0.1;
         cov(1,1) = 0.5;
         cov(2,2) = 0.001;
         el->setCov(cov);
         glut3d.addObject(el);
     }
     
     NDTVizGlutPointCloudColor pts;
     NDTVizGlutSetOfLines lines;
     for (size_t i = 0; i < 1000; i++) {
         pts.push_back(i*0.01, 2*sin(i*0.01), cos(i*0.002), i*0.001, 1, 1.);
         lines.push_back(-i*0.01, 2*sin(i*0.01), cos(i*0.002), i*0.001, 1, 1.);
     }
     lines.push_back(0, 0, 0, 1, 1, 1);
     lines.push_back(2, 2, 2, 3, 3, 3);
        
     glut3d.addObject(&pts);
     glut3d.addObject(&lines);
     int counter = 0;
     while(true)
     {
	  {
	       glut3d.process_events();
	   
               usleep(1000);
               for (size_t i = 0; i < 1000; i++) {
                   pts.getPoint(i).pos[1] = sin(counter*0.001);
               }
               glut3d.repaint();
	  }
          counter++;
     }
     exit(1);
}

