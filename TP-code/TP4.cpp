#include <iostream>

#include <visp/vpDebug.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageSimulator.h>
#include <visp/vpDisplayX.h>
#include <visp/vpKeyPointSurf.h>
#include <visp/vpHomography.h>


using namespace std ;

void AfficheAppariement(vpImage<unsigned char> &I1,  
			vpImage<unsigned char> &I2, 
			vpImage<unsigned char> &I, 
			vpKeyPointSurf &surf)

{

  for (int i =0 ; i < I1.getRows() ; i++)
    for (int j = 0 ; j < I1.getCols() ; j++)
      {
	I[i][j] = I1[i][j] ;
	I[i][j+I1.getCols()] = I2[i][j] ;
      }
  vpDisplay::display(I) ;

  vpDisplay::flush(I) ;
  int nb =  surf.matchPoint(I2);

  vpImagePoint p1[nb], p2[nb];
  
  
  for(unsigned int i=0; i<nb; i++)
    {
      surf.getMatchedPoints(i, p1[i], p2[i]);
      char s[10] ;
      sprintf(s,"%d",i) ;
      p2[i].set_u(p2[i].get_u()+I1.getCols()) ;
      vpDisplay::displayCharString(I,p1[i],s,vpColor::red) ;
      vpDisplay::displayCharString(I,p2[i],s,vpColor::red) ;
      vpDisplay::displayLine(I,p1[i],p2[i],vpColor::yellow) ;
      
    } 

  vpDisplay::flush(I) ;
}



void DLT()
{

}



int main()
{
  vpImage<unsigned char> I1(300,400,0);
  vpImage<unsigned char> I2(300,400,0);
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


  // I1g
  vpHomogeneousMatrix  c1gMo(0,0,2,  vpMath::rad(0),vpMath::rad(0),0) ;
  sim.setCameraPosition(c1gMo);
  sim.getImage(I1,cam);
  cout << "Image I1g " <<endl ;
  cout << c1gMo << endl ;

  // I1d
  vpHomogeneousMatrix c1dMo(0.1,0,2, 
			    vpMath::rad(0),vpMath::rad(0),vpMath::rad(25)) ; //0.1,0,2, vpMath::rad(0),vpMath::rad(0),0) ;
  sim.setCameraPosition(c1dMo);
  sim.getImage(I2,cam);  
  cout << "Image I1d " <<endl ;
  cout << c1dMo << endl ;
 
  vpHomogeneousMatrix cgMcd = c1gMo * c1dMo.inverse() ;
  vpMatrix K = cam.get_K() ;

  vpDisplayX dg(I1,10,10,"I1") ;
  vpDisplay::display(I1) ;
  vpDisplay::flush(I1) ;

  vpDisplayX dd(I2,450,10,"I2") ;
  vpDisplay::display(I2) ;
  vpDisplay::flush(I2) ;

  // Image resultat
  vpImage<unsigned char> I ;
  I.resize(I1.getRows(), I1.getCols()*2) ;
  vpDisplayX d(I,10,400,"I") ;

  vpKeyPointSurf surf;
  surf.setDescriptorType(vpKeyPointSurf::extendedDescriptor) ; 	
  //First grab the reference image Irefrence
  surf.setHessianThreshold(10000) ;

  //Build the reference SURF points.
  surf.buildReference(I1);
  
  AfficheAppariement(I1,  I2, I, surf) ;

  //Then grab another image which represents the current image Icurrent

  //Match points between the reference points and the SURF points computed in the current image.
  int nb =  surf.matchPoint(I2);
  cout << "Nombre de points mis en correspondance " <<  nb <<  endl ;
  //Display the matched points
  surf.display(I1, I2);
  vpImagePoint p1[nb], p2[nb];

  if(nb >= 0){ // ... add paired points to vectPts
    
    for(unsigned int i=0; i<nb; i++)
      {
	surf.getMatchedPoints(i, p1[i], p2[i]);
	char s[10] ;
	sprintf(s,"%d",i) ;
	cout << i <<"  "  << p1[i].get_u() <<"  " << p1[i].get_v() <<"  " ;
	  cout <<  p2[i].get_u() <<"  " << p2[i].get_v() << endl;
	vpDisplay::displayCharString(I1,p1[i],s,vpColor::yellow) ;
	vpDisplay::displayCharString(I2,p2[i],s,vpColor::yellow) ;
      }

  }



  

  return 0;
}
