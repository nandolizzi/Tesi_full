#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv\cv.h"
#include "opencv\highgui.h"

#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;

/// Global Variables
const int min_intersection_slider_max = 1000;
int min_intersection_slider= 1;
int min_intersection= 2;

const int min_length_slider_max = 500;
int min_length = 2;
int min_length_slider = 1;

const int max_gap_slider_max = 200;
int max_gap = 2;
int max_gap_slider = 1;

const int theta_slider_max = 360;
int min_theta = 2;
int theta_slider = 1;

int thresh = 50;
int max_thresh = 255;
RNG rng(12345);

int erosion_elem = 0;
int erosion_size = 0;
int dilation_elem = 0;
int dilation_size = 0;
int const max_elem = 2;
int const max_kernel_size = 35;

const char* window_name = "detected lines";

/// Matrices to store images
Mat src;
Mat dst;
Mat cdst;
Mat prova;
Mat prova_1;
Mat erosion_dst, dilation_dst;
vector<Vec4i> lines;


void refresh_image(int t_res, int min_int, int min_len, int max_g);
void refresh_image_final(int t_res, int min_int, int min_len, int max_g);

void on_trackbar_intersection(int, void *);
void on_trackbar_length(int, void *);
void on_trackbar_gap(int, void*);
void on_trackbar_theta(int, void*);
/** @function Dilation and Erosion */
void Erosion(int, void*);
void Dilation(int, void*);

void Mat2txt(const string filename, const Mat matrix);

int main(int argc, char** argv)
{
	/* Copio il nome del file da aprire che passo come secondo argomento dalla 
		linea di comando. */
	const char* filename = argv[1];

	src = imread(filename, 0);
	if (src.empty())
	{
		std::cout << "can not open " << filename << endl;
		return -1;
	}

	cv::namedWindow("source", WINDOW_NORMAL);
	cv::imshow("source", src);

	Canny(src, dst, 50, 200, 3);
	cvtColor(dst, prova, CV_GRAY2BGR);
	cvtColor(dst, cdst, CV_GRAY2BGR);

	/*Crete window for erosion*/
	/*namedWindow("Erosion Demo", WINDOW_NORMAL);

	/// Create Erosion Trackbar
	createTrackbar("Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", "Erosion Demo",
		&erosion_elem, max_elem,
		Erosion);

	createTrackbar("Kernel size:\n 2n +1", "Erosion Demo",
		&erosion_size, max_kernel_size,
		Erosion);

	Erosion(0, 0);*/

	//waitKey();

	theta_slider = theta_slider_max;
	min_intersection_slider = min_intersection_slider_max;
	min_length_slider = min_length_slider_max;
	max_gap_slider = max_gap_slider_max;
	
	
	cv::HoughLinesP(dst, lines, 1, CV_PI / theta_slider, min_intersection_slider, min_length_slider, max_gap_slider);
	for (size_t i = 0; i < lines.size(); i++)
	{
		Vec4i l = lines[i];
		line(prova, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 3, CV_AA);
	}

	//namedWindow("prova_1", WINDOW_NORMAL);

	prova.copyTo(cdst);
	cv::namedWindow(window_name, WINDOW_NORMAL);
	cv::imshow(window_name, cdst);

	cv::createTrackbar("Intersection", window_name, &min_intersection_slider, min_intersection_slider_max, on_trackbar_intersection);
	cv::createTrackbar("Length", window_name, &min_length_slider, min_length_slider_max, on_trackbar_length);
	cv::createTrackbar("Gap", window_name, &max_gap_slider, max_gap_slider_max, on_trackbar_gap);
	cv::createTrackbar("Theta*pi ", window_name, &theta_slider, theta_slider_max, on_trackbar_theta);

	cv::waitKey();

	// close the window
	cvDestroyWindow("Source");
	cvDestroyWindow(window_name);
	refresh_image_final(theta_slider, min_intersection_slider, min_length_slider, max_gap_slider);

	cv::waitKey();

	/* window for dilation */
	
	cv::namedWindow("Dilation Demo", WINDOW_NORMAL);
	//cvMoveWindow("Dilation Demo", src.cols, 0);

	/// Create Dilation Trackbar
	cv::createTrackbar("Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", "Dilation Demo",
		&dilation_elem, max_elem,
		Dilation);

	cv::createTrackbar("Kernel size:\n 2n +1", "Dilation Demo",
		&dilation_size, max_kernel_size,
		Dilation);

	/// Default start
	Dilation(0, 0);

	cv::waitKey();

	//waitKey();

	/* Copy image matrix inside vector to write a .txt file*/

	string fili_filename = "..\\..\\outputs\\matrice_fili.txt";
	string corridor_filename = "..\\..\\outputs\\matrice_corridoio.txt";
	string active_corridor_filename = "..\\..\\..\\elaborazione_dati_lidar\\data\\matrice_corridoio_attivo.txt";

	cvtColor(prova_1, prova_1, CV_RGB2GRAY);
	cvtColor(dilation_dst, dilation_dst, CV_RGB2GRAY);
	Mat active_corridor = dilation_dst - prova_1;

	/*Mat2txt(fili_filename, prova_1);*/
	Mat2txt(corridor_filename, dilation_dst);
	Mat2txt(active_corridor_filename, active_corridor);

	return 0;
}

void refresh_image(int t_res, int min_int, int min_len, int max_g)
{
	cv::HoughLinesP(dst, lines, 1, CV_PI / t_res, min_int, min_len, max_g);
	for (size_t i = 0; i < lines.size(); i++)
	{
		Vec4i l = lines[i];
		line(cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 3, CV_AA);
	}
	imshow(window_name, cdst);
	return;
};
void refresh_image_final(int t_res, int min_int, int min_len, int max_g)
{
	prova_1 = 0;
	HoughLinesP(dst, lines, 1, CV_PI / t_res, min_int, min_len, max_g);
	for (size_t i = 0; i < lines.size(); i++)
	{
		Vec4i l = lines[i];
		line(prova_1, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 3, CV_AA);
	}
	namedWindow("contours_source", WINDOW_NORMAL);
	imshow("contours_source", prova_1);
	return;
};

void on_trackbar_intersection(int, void *)
{
	prova.copyTo(cdst);
	if (min_intersection_slider <= 1)
	{
		cout << "Trackbar value too small; please insert an higher value!\nI'll show the image obtained for a trackbar value of 1!\n";
		refresh_image(theta_slider, 1, min_length_slider, max_gap_slider);
		/* Uso l'assoluto per evitare di avere i buchi e di considerare, in quel modo,
		anche i pixel relativi ai fili. */
		prova_1 = abs(cdst - prova);
		//prova_1 = cdst - prova;
		//imshow("prova_1", prova_1);
		return;
	}

	if (min_intersection_slider < min_intersection_slider_max)
	{
		refresh_image(theta_slider, min_intersection_slider, min_length_slider, max_gap_slider);
		prova_1 = abs(cdst - prova);
		//prova_1 = cdst - prova;
		return;
	}
	else
	{
		refresh_image(theta_slider, min_intersection_slider_max, min_length_slider, max_gap_slider);
		prova_1 = abs(cdst - prova);
		//prova_1 = cdst - prova;
		return;
	}
	prova_1 = abs(cdst - prova);
	//prova_1 = cdst - prova;
	return;
}
void on_trackbar_length(int, void *)
{
	prova.copyTo(cdst);
	if (min_length_slider <= 1)
	{
		cout << "Trackbar value too small; please insert an higher value!\nI'll show the image obtained for a trackbar value of 1!\n";
		refresh_image(theta_slider, min_intersection_slider, 1, max_gap_slider);
		/* Uso l'assoluto per evitare di avere i buchi e di considerare, in quel modo,
		anche i pixel relativi ai fili. */
		prova_1 = abs(cdst - prova);
		//prova_1 = cdst - prova;
		return;
	}

	if (min_length_slider < min_length_slider_max)
	{
		refresh_image(theta_slider, min_intersection_slider, min_length_slider, max_gap_slider);
		prova_1 = abs(cdst - prova);
		//prova_1 = cdst - prova;
		return;
	}
	else
	{
		refresh_image(theta_slider, min_intersection_slider, min_length_slider_max, max_gap_slider);
		prova_1 = abs(cdst - prova);
		//prova_1 = cdst - prova;
		return;
	}
	prova_1 = abs(cdst - prova);
	//prova_1 = cdst - prova;
	return;
}
void on_trackbar_gap(int, void*)
{
	prova.copyTo(cdst);
	if (max_gap_slider <= 1)
	{
		cout << "Trackbar value too small; please insert an higher value!\nI'll show the image obtained for a trackbar value of 1!\n";
		refresh_image(theta_slider, min_intersection_slider, min_length_slider, 1);
		/* Uso l'assoluto per evitare di avere i buchi e di considerare, in quel modo,
		anche i pixel relativi ai fili. */
		prova_1 = abs(cdst - prova);
		//prova_1 = cdst - prova;
		return;
	}

	if (max_gap_slider < max_gap_slider_max)
	{
		refresh_image(theta_slider, min_intersection_slider, min_length_slider, max_gap_slider);
		prova_1 = abs(cdst - prova);
		//prova_1 = cdst - prova;
		return;
	}
	else
	{
		refresh_image(theta_slider, min_intersection_slider, min_length_slider, max_gap_slider_max);
		prova_1 = abs(cdst - prova);
		//prova_1 = cdst - prova;
		return;
	}
	prova_1 = abs(cdst - prova);
	//prova_1 = cdst - prova;
	return;
}
void on_trackbar_theta(int, void*)
{
	prova.copyTo(cdst);
	if (theta_slider <= 1)
	{
		cout << "Trackbar value too small; please insert an higher value!\nI'll show the image obtained for a trackbar value of 1!\n";
		refresh_image(1, min_intersection_slider, min_length_slider, max_gap_slider);
		/* Uso l'assoluto per evitare di avere i buchi e di considerare, in quel modo,
		anche i pixel relativi ai fili. */
		prova_1 = abs(cdst - prova);
		//prova_1 = cdst - prova;
		return;
	}

	if (theta_slider < theta_slider_max)
	{
		cout << "Theta: " << CV_PI / theta_slider << endl;
		refresh_image(theta_slider, min_intersection_slider, min_length_slider, max_gap_slider);
		prova_1 = abs(cdst - prova);
		//prova_1 = cdst - prova;
		return;
	}
	else
	{
		refresh_image(theta_slider_max, min_intersection_slider, min_length_slider, max_gap_slider);
		cout << "Theta: " << CV_PI / theta_slider_max << endl;
		prova_1 = abs(cdst - prova);
		//prova_1 = cdst - prova;
		return;
	}
	prova_1 = abs(cdst - prova);
	//prova_1 = cdst - prova;
	return;
}

void Erosion(int, void*)
{
	int erosion_type;
	if (erosion_elem == 0){ erosion_type = MORPH_RECT; }
	else if (erosion_elem == 1){ erosion_type = MORPH_CROSS; }
	else if (erosion_elem == 2) { erosion_type = MORPH_ELLIPSE; }

	Mat element = getStructuringElement(erosion_type,
		Size(2 * erosion_size + 1, 2 * erosion_size + 1),
		Point(erosion_size, erosion_size));

	/// Apply the erosion operation
	erode(dst, erosion_dst, element);
	imshow("Erosion Demo", erosion_dst);
}
/** @function Dilation */
void Dilation(int, void*)
{
	int dilation_type;
	if (dilation_elem == 0){ dilation_type = MORPH_RECT; }
	else if (dilation_elem == 1){ dilation_type = MORPH_CROSS; }
	else if (dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }

	Mat element = getStructuringElement(dilation_type,
		Size(2 * dilation_size + 1, 2 * dilation_size + 1),
		Point(dilation_size, dilation_size));
	/// Apply the dilation operation
	dilate(prova_1, dilation_dst, element);
	imshow("Dilation Demo", dilation_dst);
}

void Mat2txt(const string filename, const Mat matrix)
{
	FILE *fw = fopen(filename.c_str(), "w");

	if (fw == NULL)
	{
		cout << "Error! Empty file!" << endl;
		fclose(fw);
		return;
	}
	
	for (int i = 0; i < matrix.rows; i++)
	{
		for (int j = 0; j < matrix.cols; j++)
		{
			if ((int)matrix.at<unsigned char>(i, j) != 0)
			{
				//fprintf (fw, "%d %d %d\n", i, j, 1);
				fprintf(fw, "%d\n",1);
			}
			else
				fprintf(fw, "%d\n",0);
				//fprintf(fw, "%d %d %d\n", i, j, 0);
		}
	}

	fclose(fw);
	return;
};