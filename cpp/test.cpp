#include <stdio.h>
#include <stdlib.h>

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <conio.h>
#include <iostream>


#include <Eigen/Dense>

using namespace std;

#define Range Eigen::Vector2f
#define RangeMax() y()
#define RangeMin() x()

class SDF{
public:
	SDF(int width, int height){
		data = (float *)malloc(width*height*sizeof(float));
		dim = 2;
		rows = height;
		cols = width;
		display_window = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
	}

	void Render(Range range){

		if(dim == 2){
			float *data_ptr = data;
			char *img_ptr = display_window->imageData;

			float scale = 255.0f/(range.RangeMax() - range.RangeMin());

			for(int i = 0; i < rows; i++){
				for(int j = 0; j < cols; j++, data_ptr++, img_ptr++){
					*img_ptr = (char)((*data_ptr) > range.RangeMax() ? 255 : 
					( (*data_ptr) < range.RangeMin() ? 0 : scale*((*data_ptr) - range.RangeMin())));
					
				}
			}

			cvNamedWindow("render", CV_WINDOW_AUTOSIZE);
			cvShowImage("render", display_window);
			cvWaitKey(0);
		}

	}

	//yea yea yea
	float *Iterator(){ 
		return data;
	}


	float operator()(int x, int y) const { return data[y*cols + x]; }
	float &operator()(int x, int y){ return data[y*cols + x]; }

	int GetRows() { return rows; }
	int GetCols() { return cols; }

private:
	IplImage *display_window;
	int rows, cols;
	int dim;
	float *data;
};
float clamp(float val, float min, float max){
	return val < min ? min : (val > max ? max : val);
}
void Create2DSphereSDF(SDF &sdf){
	Eigen::Vector2f center(sdf.GetCols()/2, sdf.GetRows()/2);
	
	for(int i = 0; i < sdf.GetRows(); i++){
		for(int j = 0; j < sdf.GetCols(); j++){
			Eigen::Vector2f point(j, i);

			//sdf(j, i) = clamp( (point - center).norm() - 100, -10, 10);
			sdf(j, i) = (point - center).norm() - 100;
			
			
		}
	}
}

void IntegrateSDF2D(SDF &sdf, float **integral){
	
	float *sdf_ptr = sdf.Iterator();

	for(int i = 0; i < sdf.GetRows(); i++){
		for(int j = 0; j < sdf.GetCols(); j++, sdf_ptr++){
			if(i == 0 && j == 0){
				integral[i][j] = fabs(*sdf_ptr);
			}else if(i == 0){
				integral[i][j] = fabs(*sdf_ptr) + integral[i][j-1];
			}else if(j == 0){
				integral[i][j] = fabs(*sdf_ptr) + integral[i-1][j];
			}else{
				integral[i][j] = fabs(*sdf_ptr) + integral[i-1][j] + integral[i][j-1] - integral[i-1][j-1];
			}

		}
	}

	/*for(int i = 0; i < sdf.GetRows(); i++){
		for(int j = 0; j < sdf.GetCols(); j++, sdf_ptr++){
			if(i == 0 && j == 0){
				integral[i][j] = *sdf_ptr;
			}else if(i == 0){
				integral[i][j] = *sdf_ptr + integral[i][j-1];
			}else if(j == 0){
				integral[i][j] = *sdf_ptr + integral[i-1][j];
			}else{
				integral[i][j] = *sdf_ptr + integral[i-1][j] + integral[i][j-1] - integral[i-1][j-1];
			}

		}
	}
*/


}

float BoxIntegral(float **integral, Eigen::Vector2d top_left, Eigen::Vector2d btm_right){
	return integral[(int)btm_right.y()][(int)btm_right.x()] - integral[(int)btm_right.y()][(int)top_left.x()]
	- integral[(int)top_left.y()][(int)btm_right.x()] + integral[(int)top_left.y()][(int)top_left.x()];
}


float YBoxFilterResponse(float **integral, int x, int y){
	return	BoxIntegral( integral, Eigen::Vector2d(x-2, y-4), Eigen::Vector2d(x+2,y-2))
			-2* BoxIntegral( integral, Eigen::Vector2d(x-2, y-1), Eigen::Vector2d(x+2,y+1))
			+ BoxIntegral( integral, Eigen::Vector2d(x-2, y+2), Eigen::Vector2d(x+2,y+4));
}

float XBoxFilterResponse(float **integral, int x, int y){
	return	BoxIntegral( integral, Eigen::Vector2d(x-4, y-2), Eigen::Vector2d(x-2,y+2))
			-2* BoxIntegral( integral, Eigen::Vector2d(x-1, y-2), Eigen::Vector2d(x+1,y+2))
			+ BoxIntegral( integral, Eigen::Vector2d(x+2, y-2), Eigen::Vector2d(x+4,y+2));
}

float XYBoxFilterResponse(float **integral, int x, int y){
	return	0	+BoxIntegral( integral, Eigen::Vector2d(x-3, y-3), Eigen::Vector2d(x-1,y-1))
				+BoxIntegral( integral, Eigen::Vector2d(x+1, y+1), Eigen::Vector2d(x+3,y+3))
				-BoxIntegral( integral, Eigen::Vector2d(x-3, y+1), Eigen::Vector2d(x-1,y+3))
				-BoxIntegral( integral, Eigen::Vector2d(x+1, y-3), Eigen::Vector2d(x+3,y-1));
}

void ConvolveBoxFilters(SDF &sdf, float **integral){

	float **temp;
	temp = (float **)malloc(512*sizeof(float *));
	for(int i = 0; i < 512; i++)
		temp[i] = (float *)malloc(512*sizeof(float));

	float max = -99999, min = 9999;
	for(int i = 4; i < sdf.GetRows()-4; i++){
		for(int j = 4; j < sdf.GetCols()-4; j++){
			float xy = XYBoxFilterResponse(integral, j, i);
			float response = XBoxFilterResponse(integral, j, i) * YBoxFilterResponse(integral, j, i) - pow(0.9*xy,2);
			
			if(response > max)
				max = response;
			if(response < min)
				min = response;

			temp[i][j] = response;
		}
	}

	IplImage *img = cvCreateImage(cvSize(512,512), IPL_DEPTH_8U, 1);
	float scale = 255.0f / (max - min);

	for(int i = 4; i < 512-4; i++){
		for(int j = 4; j < 512-4; j++){
			img->imageData[i*img->widthStep + j] = (char)((int)(scale * (temp[i][j] - min)));
		}
	}

	cvNamedWindow("integral", CV_WINDOW_AUTOSIZE);
	cvShowImage("integral", img);
	cvWaitKey(0);

	printf("%f %f\n", max, min);

}


int main(){
	SDF sdf(512,512);
	Create2DSphereSDF(sdf);

//	sdf.Render(Range(-10,10));

	float **integral;
	integral = (float **)malloc(512*sizeof(float *));
	for(int i = 0; i < 512; i++)
		integral[i] = (float *)malloc(512*sizeof(float));

	IntegrateSDF2D(sdf, integral);
	ConvolveBoxFilters(sdf, integral);

}