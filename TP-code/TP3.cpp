#include <iostream>

#include <visp/vpDebug.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageSimulator.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDot.h>


using namespace std ;

// Calculer l'homographie aHb a partir des coordonnees des point p1 et p2
void DLT(unsigned int n,
	 vpImagePoint *p1,
	 vpImagePoint *p2,
	 vpMatrix &H12)
{
  
  // NBPTMIN points son necessaire ; remplace le 1e6 par ce nombre
#define NBPTMIN 1e6 
  if(n<    NBPTMIN )
  {
    cout << "there must be at least " << NBPTMIN <<  " points in the both images\n" <<endl  ;
    throw ;
  }

  // ...

}


int main()
{
  vpImage<unsigned char> I1;
  vpImage<unsigned char> I2;
  vpImage<vpRGBa> Iimage(876,1200);
  
 
  vpImageIo::read(I1,"../data/I1.pgm") ;
  vpImageIo::read(I2,"../data/I2.pgm") ;



  vpCameraParameters cam(800.0, 800.0, 200, 150);
  cam.printParameters() ;

  vpDisplayX d1(I1,10,10,"I1") ;
  vpDisplay::display(I1) ;
  vpDisplay::flush(I1) ;

  vpDisplayX d2(I2,450,10,"I2") ;
  vpDisplay::display(I2) ;
  vpDisplay::flush(I2) ;

  int nb = 5;
  vpImagePoint p1[nb], p2[nb];
  
  // clicker 5 point sur l'image I2 ; recuperer leur coordonnees
  for(unsigned int i=0; i<nb; i++)
    {
      vpDisplay::getClick(I1, p1[i]) ;
      vpDot d ;
      d.initTracking(I1,p1[i]) ;
      d.track(I1,p1[i]) ;
      char s[10] ;
      sprintf(s,"%d",i) ;
      vpDisplay::displayCross(I1,p1[i],10,vpColor::blue) ;
      vpDisplay::displayCharString(I1,p1[i],s,vpColor::red) ;
      vpDisplay::flush(I1) ;
    }
  
  // clicker 5 point sur l'image I1 ; recuperer leur coordonnees
  // faites attention a les clicker dans le meme ordre
  for(unsigned int i=0; i<nb; i++)
    {
      vpDisplay::getClick(I2, p2[i]) ;
      vpDot d ;
      d.initTracking(I2,p2[i]) ;
      d.track(I2,p2[i]) ;
      char s[10] ;
      sprintf(s,"%d",i) ;
      vpDisplay::displayCross(I2,p2[i],10,vpColor::green) ;
      vpDisplay::displayCharString(I2,p2[i],s,vpColor::red) ;
      vpDisplay::flush(I2) ;
    }
  

  // Calculer l'homographie
  vpMatrix H12 ;
  DLT(nb,p1, p2, H12) ;

  cout << "Homographie H12 : " <<endl ; 
  cout << H12 << endl ;

  //Verification 
  double residue =0 ;
  for (int i=0 ; i < nb ; i++) 
    {
      // Connaissant le formule permettant le transfert des points p2 dans p1
      // Calculer les coordonnées des point p1 connaissant p2 et dHg
      vpImagePoint p1_calcule  ;
      //    p1_calcule  = ... ;

      // en deduire l'erreur sur commise sur chaque point et 
      // afficher un cercle de rayon 10 fois cette erreur
      double r ;
      //      r = ... ;
      cout << "point " <<i << "  " << r <<endl ;;
      double rayon ;
      rayon = sqrt(r)*10 ; if (rayon < 10) rayon =10 ;
      vpDisplay::displayCircle(I1,p1_calcule,rayon,vpColor::green) ; ;
    }


  vpDisplay::flush(I1) ;
  vpImage<vpRGBa> Ic ;
  vpDisplay::getImage(I1,Ic) ;
  vpImageIo::write(Ic,"resultat.jpg") ;

  vpDisplay::getClick(I1) ;

  vpDisplay::close(I2) ;
  vpDisplay::close(I1) ;


  

  return 0;
}
