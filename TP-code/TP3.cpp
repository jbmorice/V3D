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
#define NBPTMIN 5
  if(n < NBPTMIN)
  {
    cout << "there must be at least " << NBPTMIN <<  " points in the both images\n" <<endl;
    throw;
  }

  vpMatrix A(n * 2, 9);

  for(int i = 0; i < n; i++){
	  A[2 * i][0] = 0;
	  A[2 * i][1] = 0;
	  A[2 * i][2] = 0;
	  A[2 * i][3] = -1 * p2[i].get_u();
  	  A[2 * i][4] = -1 * p2[i].get_v();
	  A[2 * i][5] = -1 * 1;
	  A[2 * i][6] = p1[i].get_v() * p2[i].get_u();
  	  A[2 * i][7] = p1[i].get_v() * p2[i].get_v();
	  A[2 * i][8] = p1[i].get_v() * 1;

	  A[2 * i + 1][0] = 1 * p2[i].get_u();
  	  A[2 * i + 1][1] = 1 * p2[i].get_v();
	  A[2 * i + 1][2] = 1 * 1;
	  A[2 * i + 1][3] = 0;
	  A[2 * i + 1][4] = 0;
	  A[2 * i + 1][5] = 0;
	  A[2 * i + 1][6] = -1 * p1[i].get_u() * p2[i].get_u();
  	  A[2 * i + 1][7] = -1 * p1[i].get_u() * p2[i].get_v();
	  A[2 * i + 1][8] = -1 * p1[i].get_u() * 1;
  }

  vpMatrix V(9, 1);
  vpColVector D;
  A.svd(D, V);
  std::cout << "taille D : " << D.getRows() << "x" << D.getCols() << std::endl;

  int minColIndex = 0;
  for(int i = 1; i < D.getRows(); i++) {
   if(D[i] < D[minColIndex]) {
    minColIndex = i;
   }
  }

std::cout << "min col index = " << minColIndex << std::endl;

  H12.resize(3, 3);
  int k = 0;
  for(int i = 0; i < 3; i++) {
  	for(int j = 0; j < 3; j++) {
  		H12[i][j] = V[k][minColIndex];
  		k++;
  	}
}

  std::cout << "H12 calculee" << '\n';
}

void transfer(const vpImage<unsigned char>& I1, const vpImage<unsigned char>& I2, const vpMatrix H12, vpImage<unsigned char>& result)
{
	result.resize(I1.getRows(), I1.getCols(), 0);
	for (int i = 0; i < I1.getRows(); i++)
	{
		for (int j = 0; j < I1.getCols(); j++)
		{
			result[i][j] = I1[i][j];
		}
	}

	for (int i = 0; i < I1.getRows(); i++)
	{
		for (int j = 0; j < I1.getCols(); j++)
		{
			vpColVector P2(3);
		    vpColVector P1(3);

		    P2[0] = j;
		    P2[1] = i;
		    P2[2] = 1;

			P1 = H12 * P2;

			int k = P1[1] / P1[2];
			int l = P1[0] / P1[2];

			if( k >= 0 && k < I1.getRows() && l >= 0 && l < I1.getCols())
				result[k][l] = (I2[i][j] + I1[k][l]) / 2;
		}
	}
}

int main()
{
  vpImage<unsigned char> I1;
  vpImage<unsigned char> I2;
  vpImage<vpRGBa> Iimage(876,1200);


  vpImageIo::read(I1, "../data/I1.pgm") ;
  vpImageIo::read(I2, "../data/I2.pgm") ;



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
  DLT(nb, p1, p2, H12) ;

  cout << "Homographie H12 : " << endl ;
  cout << H12 << endl ;

  //Verification
  double residue = 0 ;
  for (int i = 0 ; i < nb ; i++)
    {
      // Connaissant le formule permettant le transfert des points p2 dans p1
      // Calculer les coordonn�es des point p1 connaissant p2 et dHg
	  vpMatrix p1_mat(3, 1);
	  vpMatrix p2_mat(3, 1);
	  p2_mat[0][0] = p2[i].get_u();
  	  p2_mat[1][0] = p2[i].get_v();
	  p2_mat[2][0] = 1;
	  p1_mat = H12 * p2_mat;

	  vpImagePoint p1_calcule;
	  p1_calcule.set_u(p1_mat[0][0] / p1_mat[2][0]);
  	  p1_calcule.set_v(p1_mat[1][0] / p1_mat[2][0]);

      // en deduire l'erreur sur commise sur chaque point et
      // afficher un cercle de rayon 10 fois cette erreur
      double r = vpImagePoint::distance(p1_calcule, p1[i]);

      std::cout << "point " << i << "  " << r << std::endl;
      double rayon ;
      rayon = sqrt(r)*10 ; if (rayon < 10) rayon = 10;
      vpDisplay::displayCircle(I1, p1_calcule, rayon, vpColor::green);
    }

  vpDisplay::flush(I1);
  vpImage<vpRGBa> Ic;
  vpDisplay::getImage(I1,Ic);
  vpImageIo::write(Ic, "resultat.jpg");

  vpDisplay::getClick(I1);

  vpImage<unsigned char> Iresult;
  transfer(I1, I2, H12, Iresult);

  vpDisplayX dresult(Iresult, 450, 450, "Resultat fusion");
  vpDisplay::display(Iresult);
  vpDisplay::flush(Iresult);
  vpImageIo::write(Iresult, "resultat_fusion.jpg");

  vpDisplay::getClick(Iresult);

  vpDisplay::close(I2);
  vpDisplay::close(I1);




  return 0;
}
