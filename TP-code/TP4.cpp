#include <iostream>

#include <visp/vpDebug.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageSimulator.h>
#include <visp/vpDisplayX.h>
#include <visp/vpHomography.h>

using namespace std ;

void getCorrespondances(vpImagePoint* p1, vpImagePoint* p2)
{
    p1[0].set_u(117.5130997);    p1[0].set_v(62.34123611);    p2[0].set_u(202.841095);     p2[0].set_v(36.29648209);
    p1[1].set_u(84.06044006);    p1[1].set_v(67.55551147);    p2[1].set_u(169.5350189);    p2[1].set_v(26.80556679);
    p1[2].set_u(80.27194214);    p1[2].set_v(111.0672302);    p2[2].set_u(147.9641113);    p2[2].set_v(64.5475769);
    p1[3].set_u(342.6855164);    p1[3].set_v(199.8661346);    p2[3].set_u(63.4621048);     p2[3].set_v(68.28819275);
    p1[4].set_u(302.6676636);    p1[4].set_v(226.6687317);    p2[4].set_u(300.4017639);    p2[4].set_v(263.6835022);
    p1[5].set_u(101.5870972);    p1[5].set_v(63.0242424);     p2[5].set_u(187.8421478);    p2[5].set_v(29.56011963);
    p1[6].set_u(153.4119415);    p1[6].set_v(91.05652618);    p2[6].set_u(222.968277);     p2[6].set_v(77.2434845);
    p1[7].set_u(190.6780548);    p1[7].set_v(110.7231598);    p2[7].set_u(247.8312683);    p2[7].set_v(110.4263763);
    p1[8].set_u(302.8087463);    p1[8].set_v(133.9337616);    p2[8].set_u(339.9194641);    p2[8].set_v(178.880661);
    p1[9].set_u(162.7279968);    p1[9].set_v(276.4970398);    p2[9].set_u(152.7050171);    p2[9].set_v(248.9367065);
    p1[10].set_u(151.0850067);   p1[10].set_v(36.12360764);   p2[10].set_u(244.672287);    p2[10].set_v(25.44586563);
    p1[11].set_u(171.7740173);   p1[11].set_v(53.67162704);   p2[11].set_u(256.0083618);   p2[11].set_v(49.99362183);
    p1[12].set_u(116.7895355);   p1[12].set_v(74.19098663);   p2[12].set_u(196.8202972);   p2[12].set_v(45.97808456);
    p1[13].set_u(104.2023163);   p1[13].set_v(83.85998535);   p2[13].set_u(181.4200439);   p2[13].set_v(50.26084518);
    p1[14].set_u(84.71365356);   p1[14].set_v(190.8507233);   p2[14].set_u(300.4017639);   p2[14].set_v(263.6835022);
    p1[15].set_u(138.8526764);   p1[15].set_v(273.5761719);   p2[15].set_u(131.6974182);   p2[15].set_v(236.8515778);
    p1[16].set_u(167.2081451);   p1[16].set_v(96.59983063);   p2[16].set_u(233.1238556);   p2[16].set_v(88.96112061);
}

void afficheAppariement(vpImage<unsigned char> &I1,
			vpImage<unsigned char> &I2,
			vpImage<unsigned char> &I,
			vpImagePoint* p1,
			vpImagePoint* p2,
			int nb)
{

  for (int i =0 ; i < I1.getRows() ; i++)
    for (int j = 0 ; j < I1.getCols() ; j++)
      {
	I[i][j] = I1[i][j] ;
	I[i][j+I1.getCols()] = I2[i][j] ;
      }
  vpDisplay::display(I) ;

  vpDisplay::flush(I) ;

  for(unsigned int i=0; i<nb; i++)
    {
      char s[10] ;
      sprintf(s,"%d",i) ;
      p2[i].set_u(p2[i].get_u()+I1.getCols()) ;
      vpDisplay::displayCharString(I,p1[i],s,vpColor::red) ;
      vpDisplay::displayCharString(I,p2[i],s,vpColor::red) ;
      vpDisplay::displayLine(I,p1[i],p2[i],vpColor::yellow) ;

    }

  vpDisplay::flush(I) ;
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

			if(k >= 0 && k < I1.getRows() && l >= 0 && l < I1.getCols())
				result[k][l] = (I2[i][j] + I1[k][l]) / 2;
		}
	}
}

void DLT(unsigned int n, vpImagePoint *p1, vpImagePoint *p2, vpMatrix &H12)
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

  int minColIndex = 0;
  for(int i = 1; i < D.getRows(); i++) {
   if(D[i] < D[minColIndex]) {
    minColIndex = i;
   }
  }

  H12.resize(3, 3);
  int k = 0;

  for(int i = 0; i < 3; i++) {
  	for(int j = 0; j < 3; j++) {
  		H12[i][j] = V[k][minColIndex];
  		k++;
  	}
}

}


std::vector<int> generateRandomSubset(int subsetSize, int fullSetSize)
{
	std::vector<int> res;
	for (int i = 0; i < subsetSize; i++) {
		int r = rand() % fullSetSize;
    	if(std::find(res.begin(), res.end(), r) == res.end()){
			res.push_back(r);
		}
		else {
			i--;
		}
	}
	return res;
}

double computeSubsetError(std::vector<int> subset, vpImagePoint* p1, vpImagePoint* p2, vpMatrix& H12)
{
	double error = 0;
	for(int i = 0; i < subset.size(); i++) {
        vpColVector P1(3);
        vpColVector P2(3);

		P2[0] = p2[i].get_u();
		P2[1] = p2[i].get_v();
		P2[2] = 1;

		P1 = H12 * P2;

		vpImagePoint p1_calcule;

		p1_calcule.set_u(P1[0] / P1[2]);
		p1_calcule.set_v(P1[1] / P1[2]);

		error += sqrt(vpImagePoint::distance(p1[subset[i]], p1_calcule));

	}

	return error / subset.size();
}

int checkFullSetError(vpImagePoint* p1, vpImagePoint* p2, int fullSetSize, vpMatrix& H12, double threshold, std::vector<bool>& goodPoints)
{
    std::cout << "----------" << std::endl;
    std::cout << "Full set error" << std::endl;
    std::cout << "----------" << std::endl;

	int count = 0;

	for(int i = 0; i < fullSetSize; i++) {
        vpColVector P1(3);
        vpColVector P2(3);

		P2[0] = p2[i].get_u();
		P2[1] = p2[i].get_v();
		P2[2] = 1;

		P1 = H12 * P2;

		vpImagePoint p1_calcule;

		p1_calcule.set_u(P1[0] / P1[2]);
		p1_calcule.set_v(P1[1] / P1[2]);

		double error = sqrt(vpImagePoint::distance(p1[i], p1_calcule));

        std::cout << "Point " << i << " error : " << error << std::endl;

		if(error < threshold) {
            std::cout << "Passes threshold of " << threshold << std::endl;
            goodPoints[i] = true;
			count++;
        }
	}

	return count;
}

std::vector<bool> ransac(vpImagePoint* p1, vpImagePoint* p2, vpMatrix& H12, int subsetSize, int fullSetSize, int iterations, double subsetErrorThreshold, double individualErrorThreshold)
{
    std::cout << "====================" << std::endl;
    std::cout << "Ransac" << std::endl;
    std::cout << "====================" << std::endl;

	std::vector<bool> goodPoints(fullSetSize, false);
	std::vector<int> previousSubset;

    int goodPointsNumber = 0;
	int maxGoodPointsNumber = 0;

    H12.resize(3, 3, 0);

	for(int i = 0; i < iterations; i++) {
        std::cout << "--------------------" << std::endl;
        std::cout << "Ransac iteration number " << i << std::endl;
        std::cout << "--------------------" << std::endl;

		std::vector<int> subset = generateRandomSubset(subsetSize, fullSetSize);
        std::cout << "Subset generated " << std::endl;

        vpImagePoint temp_p1[subsetSize];
        vpImagePoint temp_p2[subsetSize];

        for (int i = 0; i < subsetSize; i++) {
            temp_p1[i] = p1[subset[i]];
            temp_p2[i] = p2[subset[i]];
        }

        std::cout << "DLT on subset " << std::endl;
		DLT(subsetSize, temp_p1, temp_p2, H12);

		double subsetError = computeSubsetError(subset, p1, p2, H12);

		if(subsetError < subsetErrorThreshold) {
            std::cout << "Subset error inferior to threshold of " << subsetErrorThreshold << std::endl;

            std::vector<bool> tempGoodPoints(fullSetSize, false);
        	goodPointsNumber = checkFullSetError(p1, p2, fullSetSize, H12, individualErrorThreshold, tempGoodPoints);

            std::cout << "New good points : " << goodPointsNumber << std::endl;
            std::cout << "Previous good points : " << maxGoodPointsNumber << std::endl;
        	if(goodPointsNumber > maxGoodPointsNumber) {
                std::cout << "Better" << std::endl;

        		maxGoodPointsNumber = goodPointsNumber;
                goodPoints = tempGoodPoints;
        	}
            else {
                std::cout << "Worse" << std::endl;
            }
        }
        else {
            std::cout << "Subset error superior to threshold of " << subsetErrorThreshold << std::endl;
        }

	}

    std::cout << "Final points selected : ";
    for(int i = 0; i < goodPoints.size(); i++) {
        if(goodPoints[i]) {
            std::cout << i << " ";
        }
    }
    std::cout << std::endl;

    std::cout << "DLT on selected points" << std::endl;

    vpImagePoint selectedP1[maxGoodPointsNumber];
    vpImagePoint selectedP2[maxGoodPointsNumber];
    int count = 0;

    for(int i = 0; i < goodPoints.size(); i++) {
        if(goodPoints[i]) {
            selectedP1[count] = p1[i];
            selectedP2[count] = p2[i];
            count++;
        }
    }

    DLT(goodPointsNumber, selectedP1, selectedP2, H12);

    return goodPoints;
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
    vpDisplay::display(I1);
    vpDisplay::flush(I1);

    vpDisplayX dd(I2, 450, 10, "I2") ;
    vpDisplay::display(I2);
    vpDisplay::flush(I2);

    // Image resultat
    vpImage<unsigned char> I;
    I.resize(I1.getRows(), I1.getCols() * 2) ;
    vpDisplayX d(I, 10, 400, "I");

    int nb = 17;
    vpImagePoint p1[nb], p2[nb];
    getCorrespondances(p1, p2);
    afficheAppariement(I1, I2, I, p1, p2, nb);

    if(nb >= 0){ // ... add paired points to vectPts

        for(unsigned int i=0; i<nb; i++) {
            char s[10] ;
            sprintf(s,"%d",i) ;
            cout << i <<"  "  << p1[i].get_u() <<"  " << p1[i].get_v() <<"  " ;
            cout <<  p2[i].get_u() <<"  " << p2[i].get_v() << endl;
            vpDisplay::displayCharString(I1,p1[i],s,vpColor::yellow);
            vpDisplay::displayCharString(I2,p2[i],s,vpColor::yellow);
        }

    }
/*
    // DLT classique
    // ============
    vpMatrix H12 ;
    DLT(17, p1, p2, H12) ;

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

        vpDisplayX dresult(Iresult, 450, 450, "Transfer result");
        vpDisplay::display(Iresult);
        vpDisplay::flush(Iresult);
        vpImageIo::write(Iresult, "transfer_result.jpg");

        vpDisplay::getClick(Iresult);

        vpDisplay::close(I2);
        vpDisplay::close(I1);

        //==========
*/
    // Ransac
    //===========
    vpMatrix H12;
    std::vector<bool> goodPoints = ransac(p1, p2, H12, 5, 17, 20, 50, 10);

    // Affichage resultat
    for (int i=0 ; i < nb ; i++) {
        if(goodPoints[i]) {
            vpImagePoint p1_calcule  ;

            vpColVector P2(3);
            vpColVector P1(3);

            P2[0] = p2[i].get_u();
            P2[1] = p2[i].get_v();
            P2[2] = 1;

            P1 = H12*P2;

            P1[0]/=P1[2];
            P1[1]/=P1[2];
            P1[2]/=P1[2];

            p1_calcule.set_u(P1[0]);
            p1_calcule.set_v(P1[1]);

            double r ;
            r = vpImagePoint::distance(p1[i],p1_calcule) ;

            cout << "point " << i << "  " << sqrt(r) <<endl;
            double rayon ;
            rayon = sqrt(r)*10 ; if (rayon < 10) rayon =10 ;
            vpDisplay::displayCircle(I1,p1_calcule,rayon,vpColor::green);
        }
    }
    vpDisplay::flush(I1);
    vpImage<vpRGBa> Ic;
    vpDisplay::getImage(I1,Ic);
    vpImageIo::write(Ic, "resultat.jpg");

    vpImage<unsigned char> Iresult;
    transfer(I1, I2, H12, Iresult);

    vpDisplayX dresult(Iresult, 450, 450, "Transfer result");
    vpDisplay::display(Iresult);
    vpDisplay::flush(Iresult);
    vpImageIo::write(Iresult, "transfer_result.jpg");

    vpDisplay::getClick(I);

    vpDisplay::close(I) ;
    vpDisplay::close(I2) ;
    vpDisplay::close(I1) ;
    vpDisplay::close(Iresult) ;

    return 0;
}
