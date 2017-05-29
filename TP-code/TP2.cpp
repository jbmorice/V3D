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

float SSD(const vpImage<unsigned char> & img_1, const vpImage<unsigned char> & img_2, int i, int j, int k, vpMatrix kernel)
{
    int rows = kernel.getRows();
    int cols = kernel.getCols();
    double sum = 0;
    for(int m = -rows / 2; m <= rows / 2; m++){
        for(int n = -cols / 2; n <= cols / 2; n++){
            // Zero padding if window goes beyond image borders
            if(i + m < 0 || i + m >= img_1.getHeight()) {
                sum += 0;
            }
            else if(j + n < 0 || j + n >= img_1.getWidth()) {
                sum += 0;
            }
            else {
                // k ? i + n ? i + n + k ?
                sum += kernel[m + rows / 2][n + rows / 2] * pow(img_2[i + m][n + k] - img_1[i + m][j + n], 2);
            }
        }
    }

    return sum;
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
          disparity = abs(j - k);

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

vpMatrix generateNormalKernel(int size) {
    vpMatrix kernel(size, size);
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            kernel[i][j] = 1;
        }
    }

    return kernel;

}

int main()
{
  vpImage<unsigned char> img_1;
  vpImage<unsigned char> img_2;

  vpImageIo::read(img_1, "../data/scene_l.pgm");
  vpImageIo::read(img_2, "../data/scene_r.pgm");

  vpImage<float> disparity_map;
  disparity_map.resize(img_1.getHeight(), img_1.getWidth());

  std::cout << "WTA" << std::endl;

  computeDisparityWTA(img_1, img_2, disparity_map);

  vpImage<unsigned char> disparity_map_uchar;
  vpImageConvert::convert(disparity_map, disparity_map_uchar);
  //displayImage(disparity_map_uchar, 0, 0, "Disparity Map");
  vpImageIo::write(disparity_map_uchar, "disparity_map_wta.png");

  for(int i = 0; i <= 9; i++) {
      if(i % 2 == 1) {
          std::cout << "Normal kernel " << std::to_string(i) << std::endl;

          vpMatrix kernel = generateNormalKernel(i);
          computeDisparityAC(img_1, img_2, kernel, disparity_map);

          vpImageConvert::convert(disparity_map, disparity_map_uchar);
          //displayImage(disparity_map_uchar, 0, 0, "Disparity Map");
          vpImageIo::write(disparity_map_uchar, "disparity_map_normal_kernel_" + std::to_string(i) + ".png");

          for(int j = 1; j <= 10; j += 3) {
              std::cout << "Gaussian kernel " << std::to_string(i) << " " << std::to_string(j) << std::endl;

              vpMatrix kernel = generateGaussianKernel(i, j);
              computeDisparityAC(img_1, img_2, kernel, disparity_map);

              vpImageConvert::convert(disparity_map, disparity_map_uchar);
              //displayImage(disparity_map_uchar, 0, 0, "Disparity Map");
              vpImageIo::write(disparity_map_uchar, "disparity_map_gaussian_kernel_" + std::to_string(i) + "_" + std::to_string(j) + ".png");

          }

      }
  }

  return 0;
}
