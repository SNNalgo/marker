#include "num_extract.hpp"

int main (int argc , char *argv[]){
    Mat img = imread(argv[1],1);
    //Scalar lower(29,92,114);
    //Scalar higher(37,256,256);
    Scalar lower(0,92,114);
    Scalar higher(74,256,256);
    Mat img2 = Mat::zeros( img.size(), CV_8UC3 );
    cvtColor(img,img2,CV_BGR2HSV);
    Mat output;
    inRange(img2 , lower , higher , output);
    vector<Mat>dst;
    Num_Extract Num1;
    Num1.extract(output,img);
	for(int i = 0 ; i<Num1.dst.size() ; i++){
		dst.push_back(Num1.dst[i]);
	}
    cout << dst.size()<<endl;

}
