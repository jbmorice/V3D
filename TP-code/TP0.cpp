#include <iostream>

#include <visp/vpDebug.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageSimulator.h>
#include <visp/vpDisplayX.h>


using namespace std ;



int main()
{
  vpImage<vpRGBa> Icamera(300,400,0);
  vpImage<vpRGBa> Iimage(876,1200);
  
 
  vpImageIo::read(Iimage,"../data/big-sleep.jpg") ;
  

  double L = 0.600 ;
  double l = 0.438;
  // Initialise the 3D coordinates of the Iimage corners
  vpColVector X[4];
  for (int i = 0; i < 4; i++) X[i].resize(3);
  // Top left corner
  X[0][0] = -L;
  X[0][1] = -l;
  X[0][2] = 0;
  
  // Top right corner
  X[1][0] = L;
  X[1][1] = -l;
  X[1][2] = 0;
  
  // Bottom right corner
  X[2][0] = L;
  X[2][1] = l;
  X[2][2] = 0;
  
  //Bottom left corner
  X[3][0] = -L;
  X[3][1] = l;
  X[3][2] = 0;
  


  vpImageSimulator sim;
  sim.init(Iimage, X);
  vpCameraParameters cam(800.0, 800.0, 200, 150);
  
  cam.printParameters() ;

  vpHomogeneousMatrix  c1gMo(0,0,2,  vpMath::rad(0),vpMath::rad(0),0) ;

  sim.setCameraPosition(c1gMo);
  
  sim.getImage(Icamera,cam);

  vpDisplayX d1(Icamera) ;
  vpDisplay::display(Icamera) ;
  vpDisplay::flush(Icamera) ;
  vpDisplay::getClick(Icamera) ;


  
  cout << "Image I1g " <<endl ;
  cout << c1gMo << endl ;
  vpImageIo::write(Icamera,"I1g.jpg") ;
  vpImageIo::write(Icamera,"I1g.pgm") ;

  vpHomogeneousMatrix c1dMo(0.1,0,2, vpMath::rad(0),vpMath::rad(0),0) ;

  Icamera = 0 ;
  sim.setCameraPosition(c1dMo);
  sim.getImage(Icamera,cam);  
  vpDisplay::display(Icamera) ;
  vpDisplay::flush(Icamera) ;
  vpDisplay::getClick(Icamera) ;


  cout << "Image I1d " <<endl ;
  cout << c1dMo << endl ;
  vpImageIo::write(Icamera,"I1d.jpg") ;
  vpImageIo::write(Icamera,"I1d.pgm") ;


  //----------------------------------------------------------------------------

  Icamera = 0 ;
  cam.initPersProjWithoutDistortion(800.0, 600.0, 200, 150);
  cam.printParameters() ;


  vpHomogeneousMatrix c2gMo(0,0,1.5,  vpMath::rad(0),vpMath::rad(0),0) ;
  sim.setCameraPosition(c2gMo);
  
  sim.getImage(Icamera,cam);

  vpDisplay::display(Icamera) ;
  vpDisplay::flush(Icamera) ;
  vpDisplay::getClick(Icamera) ;
  cout << "Image I2g " <<endl ;
  cout << c2gMo << endl ;
  vpImageIo::write(Icamera,"I2g.jpg") ;
  vpImageIo::write(Icamera,"I2g.pgm") ;


//--------------------
  Icamera = 0 ;
  vpHomogeneousMatrix c2dMo(0.1,0.1,1.5,  vpMath::rad(10),vpMath::rad(10),vpMath::rad(10)) ;
  sim.setCameraPosition(c2dMo);

  sim.getImage(Icamera,cam);  
  vpDisplay::display(Icamera) ;
  vpDisplay::flush(Icamera) ;
  vpDisplay::getClick(Icamera) ;
  cout << "Image I2g " <<endl ;
  cout << c2dMo << endl ;
  vpImageIo::write(Icamera,"I2d.jpg") ;
  vpImageIo::write(Icamera,"I2d.pgm") ;



  return 0;
}
