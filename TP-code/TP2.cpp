#include <iostream>

#include <visp/vpDebug.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageSimulator.h>
#include <visp/vpDisplayX.h>
#include <visp/vpImageConvert.h>
#include <math.h>

using namespace std ;

void displayImage(vpImage<unsigned char> img, int posX, int posY, const char *title)
{
    vpDisplayX d(img, posX, posY, title);
    vpDisplay::display(img);
    vpDisplay::flush(img);
    vpDisplay::getClick(img);
    vpDisplay::close(img);
}

void winnerTakesAll(const vpImage<unsigned char> & img_1, const vpImage<unsigned char> & img_2, vpImage<float> & disparity_map)
{
    disparity_map.resize(img_1.getHeight(), img_1.getWidth());

    for(int i = 0; i < img_1.getHeight(); i++) {
      for(int j = 0; j < img_1.getWidth(); j++) {
        float crit = abs(img_2[i][0] - img_1[i][j]);
        float disparity = 0;

        for(int k = 1; k < img_2.getWidth(); k++) {
          float new_crit = abs(img_2[i][k] - img_1[i][j]);

          if(new_crit < crit) {
            crit = new_crit;
            disparity = k;

          }

        }

        disparity_map[i][j] = disparity;

      }
    }
}

int main()
{
  vpImage<unsigned char> img_1;
  vpImage<unsigned char> img_2;

  vpImageIo::read(img_1, "../data/tsukuba-l.jpg");
  vpImageIo::read(img_2, "../data/tsukuba-r.jpg");

  vpImage<float> disparity_map;
  winnerTakesAll(img_1, img_2, disparity_map);

  vpImage<unsigned char> disparity_map_uchar;
  vpImageConvert::convert(disparity_map, disparity_map_uchar);
  displayImage(disparity_map_uchar, 0, 0, "Disparity Map");
  vpImageIo::write(disparity_map_uchar, "disparity_map.png") ;

  return 0;
}
