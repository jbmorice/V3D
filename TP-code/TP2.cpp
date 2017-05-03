#include <iostream>

#include <visp/vpDebug.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageSimulator.h>
#include <visp/vpDisplayX.h>
#include <visp/vpImageConvert.h>
#include <visp/vpImageFilter.h>
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

float SSD(const vpImage<unsigned char> & img_1, const vpImage<unsigned char> & img_2, int i, int j, int u, vpMatrix kernel)
{
    int rows = kernel.getRows();
    int cols = kernel.getCols();
    int heigth = img_1.getHeight();
    int width = img_1.getCols();

    double sum = 0;
    int m = 0;
    int n = 0;

    for(int k = -rows / 2; k <= rows / 2; k++)
    {
        n = 0;
        for(int l = -cols / 2; l <= cols / 2; l++)
        {
            if(i + k < 0 || i + k >= heigth)
                sum += 0;

            else if(j + l < 0 || j + l >= width)
                sum += 0;

            else
                sum += kernel[m][n] * pow((img_2[i + k][u + l] - img_1[i + k][j + l]), 2);
            n++;

            return sum;

      }
    }
}

void computeDisparityAC(vpImage<unsigned char> & img_1, vpImage<unsigned char> & img_2, vpMatrix & kernel, vpImage<float> & disparity_map)
{

  for(int i = 0; i < img_1.getHeight(); i++) {
    for(int j = 0; j < img_1.getWidth(); j++) {

      float crit =  SSD(img_1, img_2, i, j, 0, kernel);
      float disparity = 0;

      for(int k = 1; k < img_2.getWidth(); k++) {

        float new_crit = SSD(img_1, img_2, i, j, k, kernel);

        if(new_crit < crit) {
          crit = new_crit;
          disparity = k;
        //   disparity = abs(j - k);

        }
      }

      disparity_map[i][j] = disparity;

    }
  }
}

void computeDisparityWTA(vpImage<unsigned char> & img_1, vpImage<unsigned char> & img_2, vpImage<float> & disparity_map)
{

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

vpMatrix generateGaussianKernel(int size, float sigma) {
    int gaussSize = (size + 1) / 2;
    double gauss[gaussSize];
    vpImageFilter::getGaussianKernel(gauss, size, sigma);

    vpMatrix kernel1D(1, size);
    for(int i = 0; i < size; i++) {
        if(i < gaussSize - 1) {
            kernel1D[0][i] = gauss[(gaussSize - 1) - i];
        }
        if(i == gaussSize - 1) {
            kernel1D[0][i] = gauss[0];
        }
        if(i > gaussSize - 1) {
            kernel1D[0][i] = gauss[i - (gaussSize - 1)];
        }

    }

    return kernel1D.transpose() * kernel1D;

}

int main()
{
  vpImage<unsigned char> img_1;
  vpImage<unsigned char> img_2;

  vpImageIo::read(img_1, "../data/tsukuba-l.jpg");
  vpImageIo::read(img_2, "../data/tsukuba-r.jpg");

  vpImage<float> disparity_map;
  disparity_map.resize(img_1.getHeight(), img_1.getWidth());

  // computeDisparityWTA(img_1, img_2, disparity_map);

  vpMatrix kernel = generateGaussianKernel(3, 1);
  computeDisparityAC(img_1,img_2, kernel, disparity_map);

  vpImage<unsigned char> disparity_map_uchar;
  vpImageConvert::convert(disparity_map, disparity_map_uchar);
  displayImage(disparity_map_uchar, 0, 0, "Disparity Map");
  vpImageIo::write(disparity_map_uchar, "disparity_map.png");

  return 0;
}
