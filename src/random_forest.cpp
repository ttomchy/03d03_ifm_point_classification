//
// Created by laptop2 on 8/21/17.
//

#include "opencv2/core/core.hpp"

#include "opencv2/ml/ml.hpp"
#include <fstream>

#include "../include/data.h"
using namespace cv;
using namespace std;


long long int CountLines(char *filename)
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


long long int total_lines(const int num_of_files,const string & str1,const string & str2){

    int j_num_fea =0;
    //  std::string ss1;
    long long int num_total_lines=0;
    long long int LINES[5];

    char szName[100] = {'\0'};

    while(j_num_fea<num_of_files) {

        try {
            sprintf(szName,
                    "%s%d%s",
                    str1.c_str(),
                    j_num_fea,
                    str2.c_str()
            );

        }
        catch (exception e) {
            cerr <<"Error"<<endl;
        }


        LINES[j_num_fea]=CountLines(szName);
        cerr<<"The number of lines is :"<<LINES[j_num_fea]<<endl;
        cerr<<"The value of the szname is :"<<szName<<std::endl;

        num_total_lines += LINES[j_num_fea];
        j_num_fea++;

    }

    return num_total_lines;
}





void read_train_data(int num_of_files, long long int num_total_lines,int col_feature ,
                     vector<vector<float> > &training_array,vector<float> &label_class,
                     const string & str1,const string & str2,
                     int test_files,const string & str1_test,const string & str2_test,
                     vector<vector<float> >&test_array,vector<float> test_lables)
{

    char szName[100] = {'\0'};
    long long int start_lines=0;
    long long int LINES[5];


    int j_num_fea=0;


    label_class.resize(num_total_lines);
    training_array.resize(num_total_lines);
    for(int i=0;i<num_total_lines;i++) {

        training_array[i].resize(col_feature);

    }


    while(j_num_fea<num_of_files){

        try {

            sprintf(szName,
                    "%s%d%s",
                    str1.c_str(),
                    j_num_fea,
                    str2.c_str()
            );



        }
        catch (exception e) {
            cerr <<"Error"<<endl;
        }


        LINES[j_num_fea]=CountLines(szName);


        ifstream fin(szName); //read the training dataset.

        for(long long int i=start_lines;i< start_lines+ LINES[j_num_fea] ;i++){
            for(int j=0;j<8;j++) {
                if(j<7){

                    fin>>training_array[i][j];

                }else{

                    fin>>label_class[i];

                }
            }
        }

        fin.close();
        start_lines+=LINES[j_num_fea];
        cerr<<"The number of the start_lines is :"<<start_lines<<endl;

        j_num_fea++;
    }



    Mat trainingDataMat(num_total_lines,7,CV_32FC1);
    for(int i=0; i<trainingDataMat.rows; ++i){
        for(int j=0; j<trainingDataMat.cols; ++j) {
            trainingDataMat.at<float>(i, j) = training_array.at(i).at(j);
        }
    }


    Mat labelsMat(label_class);
    labelsMat = labelsMat.reshape(1,label_class.size());


    CvRTParams params_2= CvRTParams(10,5000, 0, false,5, 0, true, 0, 100, 0, CV_TERMCRIT_ITER );

    CvRTrees* rtree = new CvRTrees;
    rtree->train(trainingDataMat, CV_ROW_SAMPLE, labelsMat,
                 Mat(), Mat(),  Mat(), Mat(), params_2);



/*

    CvMat trainingDataCvMat = cvMat( row, col-1, CV_32FC1, data_train.training_data );

   // CvMat trainingDataCvMat=CvMat(data_train.array);

    CvMat responsesCvMat = cvMat( row, 1, CV_32FC1, data_train.lables );

    CvRTParams params= CvRTParams(10,500, 0, false,5, 0, true, 0, 100, 0, CV_TERMCRIT_ITER );

    CvERTrees etrees;
    etrees.train(&trainingDataCvMat, CV_ROW_SAMPLE, &responsesCvMat,
                 Mat(), Mat(), Mat(), Mat(),params);


    Mat impo= etrees.get_var_importance();
    // float train_error=etrees.get_train_error();
    std::cerr<<"The value of the var_importance is :"<<impo<<std::endl;
    //std::cerr<<"The training error is :"<<etrees.get_train_error()<<std::endl;

*/

    //*******************************testing*******************************************
  // long long int LINES_test= total_lines(test_files,str1_test.c_str(),str2_test.c_str());

    char sme[100] = {'\0'};
    int j_num_test=0;


    while(j_num_test<test_files){

        try {
            sprintf(sme,"%s%d%s", str1_test.c_str(), j_num_test, str2_test.c_str());
          //  sprintf(sme, "/home/laptop2/work_space/intern_ws/o3d/test_ws/txt_dataset/test/test8.txt");
        }
        catch (exception e) {
            cerr <<"Error"<<endl;
        }

       cerr<<"the sme is :"<<sme<<endl;
        // LINES[j_num_test]=CountLines(sme);
       long long int num_of_test_line =CountLines(sme);
      //  cerr<<"The number of testing data lines is :"<<num_of_test_line<<endl;



//       // vector<vector<float> >().swap(training_array);
        training_array.clear();
        training_array.shrink_to_fit();

        label_class.clear();
        label_class.shrink_to_fit();
//
//        cerr<<"The size of the training_array is:"<<training_array.size()<<endl;
//        cerr<<"The capacity of the training_array is:"<<training_array.capacity()<<endl;



        test_array.resize(num_of_test_line);
        for(int i=0;i<num_of_test_line;i++) {

            test_array[i].resize(col_feature);
        }

        cerr<<"The size of the test_array is:"<<test_array.size()<<endl;
      //  cerr<<"The capacity of the test_array is:"<<test_array.capacity()<<endl;




        test_lables.resize(num_of_test_line);

        cerr<<"The size of the test_lables is:"<<test_lables.size()<<endl;
    //    cerr<<"The capacity of the test_lables is:"<<test_lables.capacity()<<endl;

//
//        trainingDataMat.release();
//        labelsMat.release();
//
//        cerr<<"THe size of the  trainingDataMat is :"<<trainingDataMat.rows<<" "<<trainingDataMat.cols<<endl;



       // cerr<<"THhe size of the trainingData_testMat is :"<<trainingData_testMat.rows<<" "<<trainingData_testMat.cols<<endl;
      //  cerr<<"THhe size of the labels_testMat is :"<<labels_testMat.rows<<" "<<labels_testMat.cols<<endl;



        ifstream fin(sme);

        for(long long int i=0;i<num_of_test_line ;i++){
            for(int j=0;j<8;j++) {
                if(j<7){

                    fin>>test_array[i][j];

                }else{

                    fin>>test_lables[i];

                }
            }
        }
        fin.close();


        Mat trainingData_testMat(num_of_test_line,7,CV_32FC1);
        for(int i=0; i<num_of_test_line; ++i){
            for(int j=0; j<7; ++j) {
                trainingData_testMat.at<float>(i, j) = test_array.at(i).at(j);
            }
        }

        Mat labels_testMat(test_lables);
        labels_testMat = labels_testMat.reshape(1,test_lables.size());


        double test_hr = 0;
        double recall=0;
        double recall_1500=0;
        double recall_1600=0;
        double recall_1700=0;
        double recall_1800=0;
        double recall_1100=0;
        double recall_1300=0;
        double recall_1400=0;
        double recall_1900=0;
        double recall_1200=0;
        float  lable_temp=test_lables[1];

        for (int i=0; i< num_of_test_line; i++)
        {
            double r=0;
            Mat sample = trainingData_testMat.rowRange(i,i+1).clone();
            //std::cout<<"The value  of the testing_dataCvMat("<<i<<") is:"<<sample<<std::endl;

            r = rtree->predict(sample);

            if(r==1200){
                recall++;
            }

       //   if(  int(lable_temp==1200)){
              if(r==1500){
                  recall_1500++;
              } else if(r==1600)
              {
                  recall_1600++;
              } else if(r==1800){
                  recall_1800++;
              }
              else if(r==1700){
                  recall_1700++;
              }
              else if(r==1400){
                  recall_1400++;
              }

              else if(r==1300){
                  recall_1300++;
              }
              else if(r==1100){
                  recall_1100++;
              }
              else if(r==1900){
                  recall_1900++;
              }
              else if(r==1200){
                  recall_1200++;
              }
      //    }

                r = fabs((float)r - labels_testMat.at<float>(i,0)) <= FLT_EPSILON ? 1 : 0;
                test_hr += r;

        }


      //  if(int (lable_temp)==1200)
    //    {

            test_hr /= num_of_test_line;
            cerr<<"The accuracy rate is :"<<test_hr<<endl;
            recall_1100/=num_of_test_line;
            recall_1200/=num_of_test_line;
            recall_1300/=num_of_test_line;
            recall_1400/=num_of_test_line;
            recall_1500/=num_of_test_line;
            recall_1600/=num_of_test_line;
            recall_1700/=num_of_test_line;
            recall_1800/=num_of_test_line;

            cerr<<"The misclass of the wall is :"<<recall_1100<<endl;
            cerr<<"The misclass of the target is :"<<recall_1200<<endl;
            cerr<<"The misclass of the chair is :"<<recall_1300<<endl;
            cerr<<"The misclass of the people is :"<<recall_1400<<endl;
            cerr<<"The misclass of the bottle is :"<<recall_1500<<endl;
            cerr<<"The misclass of the box   is :"<<recall_1600<<endl;
            cerr<<"The misclass of the flower is :"<<recall_1700<<endl;
            cerr<<"The misclass of the garbage is :"<<recall_1800<<endl;
            cerr<<"The misclass of the column is :"<<recall_1900<<endl;



     //   }
     //   else{

         //   test_hr /= num_of_test_line;
         //   cerr<<"The accuracy rate is :"<<test_hr<<endl;
          //  recall/= num_of_test_line;
            cerr<<"The number of the recall is :"<<recall<<endl;

       // }


        test_hr=0;
        recall=0;
        num_of_test_line=0;

        switch(int(lable_temp)){
            case 1100:
                cerr<<"1100 is the wall"<<endl;
                break;

            case 1200:
               cerr<<"1200 is the target"<<endl;
                break;
            case 1300:
                cerr<<"1300 is the chair "<<endl;
                break;
            case 1400:
                cerr<<"1400 is the people "<<endl;
                break;
            case 1500:
                cerr<<"1500 is the bottle"<<endl;
                break;
            case 1600:
                cerr<<"1600 is the box "<<endl;
                break;
            case 1700:
                cerr<<"1700 is the flower "<<endl;
                break;
            case 1800:
                cerr<<"1800 is the garbage "<<endl;
                break;
            case 1900:
                cerr<<"1900 is the column "<<endl;
                break;

            default:
                cerr<<"Wrong lable!!!"<<endl;
                break;
        }

        j_num_test++;
    }


}


vector<vector<float> > training_array;
vector<float> label_class;

vector<vector<float> >test_array;

vector<float> test_lables;


int main( int argc, char** argv )
{




    int num_of_files=10;
    int num_of_test_files=11;

    string s1=
           // "/home/laptop2/work_space/intern_ws/o3d/test_ws/txt_dataset/train_20_feat_chair";
   "/home/laptop2/work_space/intern_ws/o3d/test_ws/txt_dataset/training/train_";
    string s2=".txt";

    long long int num_total_lines=total_lines(num_of_files,s1,s2);


    cerr<<"The number of total lines is :"<<num_total_lines<<endl;

    std::string s1_test=
            //"/home/laptop2/work_space/intern_ws/o3d/test_ws/txt_dataset/feature_box";
           //  "/home/laptop2/work_space/intern_ws/o3d/test_ws/test_200_309";
   "/home/laptop2/work_space/intern_ws/o3d/test_ws/txt_dataset/test/test";
     //  /home/laptop2/work_space/intern_ws/o3d/test_ws/txt_dataset/test/test
    read_train_data(num_of_files,num_total_lines,7,training_array,label_class,s1,s2,num_of_test_files,s1_test,s2,test_array,test_lables);



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

/*
    ifstream fin(

           "/home/laptop2/work_space/intern_ws/o3d/test_ws/txt_dataset/train_20_feat_chair.txt"
    );

    float tmp;
    for(long long int i=0;i<row;i++){
        for( int j=0;j<col;j++) {
            if(j<col-1){

                fin>>tmp;
                training_array[i][j]=tmp;
               // cerr<<"     the value of the  add_train_data is :"<< tmp<<endl;
            }else{

                fin >>tmp;
                label_class[i]=tmp;

            }
        }
    }
    fin.close();

 */



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

    return 0;
}