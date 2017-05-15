#include <iostream>

#include <visp/vpDebug.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageSimulator.h>
#include <visp/vpDisplayX.h>
#include <visp/vpColor.h>


using namespace std ;



int main()
{
  vpImage<unsigned char> Ig(300,400,0);
  vpImage<unsigned char> Id(300,400,0);
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
  vpHomogeneousMatrix  gMo(0,0,2,  vpMath::rad(0),vpMath::rad(0),0) ;
  sim.setCameraPosition(gMo);
  sim.getImage(Ig,cam);
  cout << "Image I1g " <<endl ;
  cout << "gMo " << endl ;
  cout << gMo << endl ;

  // I1d
  vpHomogeneousMatrix dMo(0.1,0,2, vpMath::rad(0),vpMath::rad(0),vpMath::rad(0)) ; 

  sim.setCameraPosition(dMo);
  sim.getImage(Id,cam);  
  cout << "Image I1d " <<endl ;
  cout << "dMo " << endl ;
  cout << dMo << endl ;
 

  vpDisplayX dg(Ig,10,10,"Ig") ;
  vpDisplay::display(Ig) ;
  vpDisplay::flush(Ig) ;

  vpDisplayX dd(Id,10,10,"Id") ;
  vpDisplay::display(Id) ;
  vpDisplay::flush(Id) ;

  vpHomogeneousMatrix gMd = gMo * dMo.inverse(); // Matrice de passage entre les deux cameras

  vpTranslationVector gtd;
  gMd.extract(gtd);
  vpRotationMatrix gRd;
  gMd.extract(gRd);

  vpMatrix gFd = cam.get_K_inverse().transpose() * gtd.skew() * gRd * cam.get_K_inverse();

  std::cout << "matrice\n" << gFd << std::endl;

  double a = gFd[0][0];
  double b = gFd[0][1];
  double c = gFd[0][2];

  vpImagePoint pd ; 

  for (int i=0 ; i < 5 ; i++)
    {
      cout << "Click point number " << i << endl ;
      vpDisplay::getClick(Id, pd) ;
      
      
      vpDisplay::displayCross(Id,pd,5,vpColor::red) ;


      // Calcul du lieu geometrique
      //....

      vpMatrix p(3, 1);
      p[0][0] = pd.get_u();
      p[1][0] = pd.get_v();
      p[2][0] = 1;

      vpMatrix Deg = gFd * p;

      std::cout << Deg << std::endl;


      // Initization of points
      int py1 = 0;
      int py2 = Id.getCols();
      int px1 = 0;
      int px2 = 0; 

      for(int j1 = -1000; j1 < 1000; j1++)
      {
        if(Deg[0][0]*py1 + Deg[1][0] * j1 +Deg[0][2] == 0)
        {
          std::cout << "j1 find" << j1 << std::endl;
          px1 = j1;
        }
      }

      for(int j2 = -1000; j2 < 1000; j2++)
      {
        if(Deg[0][0]*py2 + Deg[1][0] * j2 +Deg[0][2] == 0)
        {
          std::cout << "j2 find" << j2 << std::endl;
          px2 = j2;
        }
      }

      // Affichage dans Ig
      
      vpDisplay::displayLine(Ig,px1,py1,px2,py2, vpColor::red) ;

      vpDisplay::flush(Id) ;
      vpDisplay::flush(Ig) ;
    }

  // exemple de code pour sauvegarder une image avec les plan overlay
  vpImage<vpRGBa> Icol ;
  vpDisplay::getImage(Id,Icol) ;
  vpImageIo::write(Icol,"resultat.jpg") ;
  vpImageIo::write(Id,"I1g.jpg") ;

  


  vpDisplay::getClick(Id) ;
  cout << "OK " << endl ;

  vpDisplay::close(Id) ;
  vpDisplay::close(Ig) ;

  

  return 0;
}
