//
// Created by laptop2 on 8/21/17.
//

#include "opencv2/core/core.hpp"

#include "opencv2/ml/ml.hpp"
#include <fstream>

#include "../include/data.h"
using namespace cv;
using namespace std;


int CountLines(char *filename)
{
    ifstream ReadFile;
    long long int n=0;
    string tmp;
    ReadFile.open(filename,ios::in);//ios::in 表示以只读的方式读取文件
    if(ReadFile.fail())//文件打开失败:返回0
    {
        return 0;
    }
    else//文件存在
    {
        while(getline(ReadFile,tmp,'\n'))
        {
            n++;
        }
        ReadFile.close();
     //   sleep(5);
        return n;
    }
}

inline bool scan_lf(double &num)
{
    char in;double Dec=0.1;
    bool IsN=false,IsD=false;
    in=getchar();
    if(in==EOF) return false;
    while(in!='-'&&in!='.'&&(in<'0'||in>'9'))
        in=getchar();
    if(in=='-'){IsN=true;num=0;}
    else if(in=='.'){IsD=true;num=0;}
    else num=in-'0';
    if(!IsD){
        while(in=getchar(),in>='0'&&in<='9'){
            num*=10;num+=in-'0';}
    }
    if(in!='.'){
        if(IsN) num=-num;
        return true;
    }else{
        while(in=getchar(),in>='0'&&in<='9'){
            num+=Dec*(in-'0');Dec*=0.1;
        }
    }
    if(IsN) num=-num;
    return true;
}



data data_train;

int main( int argc, char** argv )
{


   //
    // char filename[512]="/home/laptop2/work_space/intern_ws/o3d/test_ws/train_fea_40.txt";
    char filename[512]=
          //  "/home/laptop2/work_space/intern_ws/o3d/test_ws/train_20_feat.txt";
           //   "/home/laptop2/work_space/intern_ws/o3d/test_ws/train_20_feat_backup.txt";
         "/home/laptop2/work_space/intern_ws/o3d/test_ws/txt_dataset/train_20_feat_people.txt";


    long long int LINES=CountLines(filename);
    cerr<<"The number of lines is :"<<LINES<<endl;
    long long int row=LINES ,col=8;

   // float training_data[row][col-1];
   // float lables[row];

  //
    data_train.init_array(LINES);





//    std::string file_open=
//   // "/home/laptop2/work_space/intern_ws/o3d/test_ws/txt_dataset/train_20_feat.txt";
//         // "/home/laptop2/work_space/intern_ws/o3d/test_ws/train_fea_40.txt";
//          // "/home/laptop2/work_space/intern_ws/o3d/test_ws/train_20_feat.txt";
//
//          //  "/home/laptop2/work_space/intern_ws/o3d/test_ws/txt_dataset/train_20_feat.txt";
//
//            "/home/laptop2/work_space/intern_ws/o3d/test_ws/train_feature.txt";


/*
    int j_num_fea =0;
    std::string ss1;
    long int num_total_lines=0;
    long long int LINES[5];
    long long int start_lines=0;
   // long long int row=CountLines(filename);
    int row=600000, col=8;

    float training_data[row][col-1];
    float lables[row];



    while(j_num_fea<1) {

        char szName[100000];// = {'\0'};

        try {
            sprintf(szName,
                    // "/home/laptop2/work_space/intern_ws/o3d/test_ws/txt_dataset/test_read/test%d.txt",
                    "/home/laptop2/work_space/intern_ws/o3d/test_ws/txt_dataset/test_read/feature%d.txt",
                    j_num_fea);

        }
        catch (exception e) {
            cerr <<"Error"<<endl;
        }




        LINES[j_num_fea]=CountLines(szName);
        cerr<<"The number of lines is :"<<LINES[j_num_fea]<<endl;
        cerr<<"The value of the szname is :"<<szName<<std::endl;



      //  ifstream fin(szName); //read the training dataset.

        freopen(

                //  "/home/laptop2/work_space/intern_ws/o3d/test_ws/txt_dataset/train_20_feat.txt",
               // "/home/laptop2/work_space/intern_ws/o3d/test_ws/train_feature.txt",

                szName,
                "r",stdin);




        for(int i=start_lines;i< start_lines+ LINES[j_num_fea] ;i++){
            for(int j=0;j<col;j++) {
                if(j<col-1){
                  //  fin >> training_data[i][j];

                scanf("%f",&training_data[i][j]);
                cerr<<"the number of the training data is :"<< training_data[i][j]<<endl;


                }else{
                    scanf("%f", &lables[i]);//The training lables

                    // fin >> lables[i];//The training lables

                }
            }
        }
     //   fin.close();


        start_lines+=LINES[j_num_fea];
        cerr<<"The number of the start_lines is :"<<start_lines<<endl;
        j_num_fea++;

    }

*/

/*
    for(int i=0;i<9;i++){
        for(int j=0;j<7;j++){

          std::cout<<std::fixed<<training_data[i][j]<<" ";
        }
        std::cout<<std::endl;
        std::cout<<lables[i]<<std::endl;
    }


*/

/*

    ifstream fin;

    fin.open (file_open.c_str(), std::fstream::in);
    if(!fin.is_open())
        std::cerr<<"Fail to open the file !"<<std::endl;
        //return ;
    double s=0;
    int count=0;
    int row_l=0;
    int col_l=0;
    while(fin>>s){

       // std::cout<<"The value of s is :"<<s<<std::endl;
        if(count<7){

            training_data[row_l][col_l++]=s;

            count++;
        } else {

            lables[row_l]=s;
          // std::cout<<"The value of  lables[row]is :"<< lables[row_l]<<std::endl;
            row_l++;
            count=0;
            col_l=0;
        }
    }

*/


    ifstream fin(
            //"/home/laptop2/work_space/intern_ws/o3d/test_ws/train_20_feat.txt"

            "/home/laptop2/work_space/intern_ws/o3d/test_ws/train_20_feat_backup.txt"

    );

    float tmp;
    for(long long int i=0;i<row;i++){
        for( int j=0;j<col;j++) {
            if(j<col-1){

                fin>>tmp;
               // fin >> training_data[i][j];
                data_train.add_train_data(i,j,tmp);
               // cerr<<"     the value of the  add_train_data is :"<< tmp<<endl;
            }else{

                fin >>tmp;
                //fin>>lables[i];//The training lables
                data_train.add_lables(i,tmp);

            }
        }
    }
    fin.close();





/*
   freopen(

         //  "/home/laptop2/work_space/intern_ws/o3d/test_ws/txt_dataset/train_20_feat.txt",
           "/home/laptop2/work_space/intern_ws/o3d/test_ws/train_feature.txt",


           "r",stdin);

    for(int i=0;i<row;i++){
        for(int j=0;j<col;j++) {
            if(j<col-1){
                scanf("%f",&training_data[i][j]);
                cerr<<"The value of the training_data is:"<< training_data[i][j]<<endl;
            }else{
                scanf("%f", &lables[i]);//The training lables
                cerr<<"The value of the lable is:"<< lables[i]<<endl;
            }
        }
    }


*/





    CvMat trainingDataCvMat = cvMat( row, col-1, CV_32FC1, data_train.training_data );

    CvMat responsesCvMat = cvMat( row, 1, CV_32FC1, data_train.lables );

    CvRTParams params= CvRTParams(10,500, 0, false,5, 0, true, 0, 100, 0, CV_TERMCRIT_ITER );

    CvERTrees etrees;
    etrees.train(&trainingDataCvMat, CV_ROW_SAMPLE, &responsesCvMat,
                 Mat(), Mat(), Mat(), Mat(),params);


    Mat impo= etrees.get_var_importance();
    // float train_error=etrees.get_train_error();
    std::cerr<<"The value of the var_importance is :"<<impo<<std::endl;
    //std::cerr<<"The training error is :"<<etrees.get_train_error()<<std::endl;



    //*******************************testing*******************************************
//    double sampleData[4]={0.197799, 0.797150, 0.005051 ,0.994949 };
//    Mat sampleMat(4, 1, CV_32FC1, sampleData);
//    float r = etrees.predict(sampleMat);
//    cout<<endl<<"result:  "<<r<<endl;


    //   CvMat testingdataset = cvMat( row, col-1, CV_32FC1, training_data );


    char filename_test[512]="/home/laptop2/work_space/intern_ws/o3d/test_ws/test_200_309.txt";

    //"/home/laptop2/work_space/intern_ws/o3d/test_ws/txt_dataset/feature_box.txt";
   // "/home/laptop2/work_space/intern_ws/o3d/test_ws/txt_dataset/feature_chair_subset.txt";

    int LINES_test=CountLines(filename_test);

    cerr<<"The number of testing data lines is :"<<LINES_test<<endl;

    row=LINES_test ,col=8;     //n行 m列


    float testing_data[row][col-1];
    float testing_lables[row];
    ifstream fin_test(
            "/home/laptop2/work_space/intern_ws/o3d/test_ws/test_200_309.txt"
          //  "/home/laptop2/work_space/intern_ws/o3d/test_ws/txt_dataset/feature_box.txt"

           // "/home/laptop2/work_space/intern_ws/o3d/test_ws/txt_dataset/feature_chair_subset.txt"

    );  //read_the test dataset
    for(int i=0;i<row;i++){
        for(int j=0;j<col;j++) {
            if(j<col-1){
                fin_test >> testing_data[i][j];
            }else{
                fin_test >> testing_lables[i];
            }
        }
    }
    fin_test.close();


    Mat testing_dataCvMat= Mat( row, col-1, CV_32FC1, testing_data );
    Mat testing_lablesCvMat = Mat( row, 1, CV_32FC1, testing_lables );

    std::cerr<<"The size of the testing_dataCvMat is :"<<testing_dataCvMat.rows
             <<" "<<testing_dataCvMat.cols<<std::endl;
    std::cerr<<"The size of the testing_lablesCvMat is:"<< testing_lablesCvMat.rows
             <<" "<<testing_lablesCvMat.cols <<std::endl;


    //calculate the errors.
    double test_hr = 0;
    double recall=0;
    for (int i=0; i<LINES_test; i++)
    {
        double r;
        Mat sample = testing_dataCvMat.rowRange(i,i+1).clone();
        //  std::cout<<"The value  of the testing_dataCvMat("<<i<<") is:"<<sample<<std::endl;

        r = etrees.predict(sample);
        if(r==1200){
            recall++;
        }
        std::cout<<"The value of r is :"<<i<<" "<<r<<" "<<testing_lablesCvMat.at<float>(i,0) <<std::endl;
        r = fabs((double)r - testing_lablesCvMat.at<float>(i,0)) <= FLT_EPSILON ? 1 : 0;
        test_hr += r;
    }

    recall/=LINES_test;
    test_hr /=LINES_test;

    cerr<<"The accuracy rate is :"<<test_hr<<endl;
    cerr<<"The number of the recall is :"<<recall<<endl;
    //get the  the variable importance vector...




    return 0;
}