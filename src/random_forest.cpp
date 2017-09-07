//
// Created by laptop2 on 8/21/17.
//

#include "opencv2/core/core.hpp"

#include "opencv2/ml/ml.hpp"
#include <fstream>

#include "../include/data.h"
#include "../include/features.h"
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
            sprintf(szName,"%s%d%s", str1.c_str(), j_num_fea, str2.c_str());
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


    CvRTParams params_2= CvRTParams(10,6500, 0, false,5, 0, true, 0, 100, 0, CV_TERMCRIT_ITER );

    CvRTrees* rtree = new CvRTrees;
    rtree->train(trainingDataMat, CV_ROW_SAMPLE, labelsMat,
                 Mat(), Mat(),  Mat(), Mat(), params_2);

    rtree->save("train_model.xml");

    training_array.clear();
    training_array.shrink_to_fit();

    label_class.clear();
    label_class.shrink_to_fit();
    labelsMat.release();
    trainingDataMat.release();




    ///*******************************testing*******************************************
 //  long long int LINES_test= total_lines(test_files,str1_test.c_str(),str2_test.c_str());
//
//    cerr<<"Now starting load files!!!"<<endl;
//
//
//    CvRTrees* rtree = new CvRTrees;
//
//    rtree->load("/home/laptop2/work_space/intern_ws/o3d/test_ws/train_model.xml");
//
//    if( rtree->get_tree_count() == 0 )
//    {
//       cerr<<"Could not read the classifier"<<endl;
//
//    }


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
        double recall_2000=0,recall_2100,recall_2200,recall_2300,recall_2400;
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
              else if(r==2000){
                  recall_2000++;
              }
              else if(r==2100){
                  recall_2100++;
              }
              else if(r==2200){
                  recall_2200++;
              }
              else if(r==2300){
                  recall_2300++;
              }
              else if(r==2400){
                  recall_2400++;
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

            recall_1300/=num_of_test_line;
            recall_1400/=num_of_test_line;
            recall_1500/=num_of_test_line;
            recall_1600/=num_of_test_line;
            recall_1700/=num_of_test_line;
            recall_1800/=num_of_test_line;


            recall_1200/=num_of_test_line;

            recall_2000/=num_of_test_line;
            recall_2100/=num_of_test_line;
            recall_2200/=num_of_test_line;
            recall_2300/=num_of_test_line;
            recall_2400/=num_of_test_line;

            cerr<<"The misclass of the wall is :"<<recall_1100<<endl;

            cerr<<"The misclass of the target is :"<<recall_1200<<endl;
            cerr<<"The misclass of the people is :"<<recall_1400<<endl;

/*
            cerr<<"The misclass of the chair is :"<<recall_1300<<endl;

            cerr<<"The misclass of the bottle is :"<<recall_1500<<endl;
            cerr<<"The misclass of the box   is :"<<recall_1600<<endl;
            cerr<<"The misclass of the flower is :"<<recall_1700<<endl;
            cerr<<"The misclass of the garbage is :"<<recall_1800<<endl;
            cerr<<"The misclass of the column is :"<<recall_1900<<endl;
*/

            cerr<<"The misclass of the diff small bottle:"<<recall_2000<<endl;
            cerr<<"The misclass of the diff flower :"<<recall_2100<<endl;
            cerr<<"The misclass of the diff garbage :"<<recall_2200<<endl;
            cerr<<"The misclass of  the diff bottle is :"<<recall_2300<<endl;
            cerr<<"The misclass of the diff chair is :"<<recall_2400<<endl;





        //   }
     //   else{

         //   test_hr /= num_of_test_line;
         //   cerr<<"The accuracy rate is :"<<test_hr<<endl;
          //  recall/= num_of_test_line;
            cerr<<"The number of the recall points is :"<<recall/num_of_test_line<<endl;

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


            case 2000:
                cerr<<"2000 is the diff small bottle"<<endl;
                break;

            case 2100:
                cerr<<"2100 is the diff flower"<<endl;
                break;

            case 2200:
                cerr<<"2200 is the diff garbage"<<endl;
                break;

            case 2300:
                cerr<<"2300 is the diff bottle"<<endl;
                break;

            case 2400:
                cerr<<"2400 is the diff chair"<<endl;
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

    int num_of_files=7;
    int num_of_test_files=8;

    string s1=
   //"/home/laptop2/work_space/intern_ws/o3d/test_ws/txt_dataset/training/train_";
    "/home/laptop2/work_space/intern_ws/o3d/test_ws/txt_dataset/scale/train/feature_diff_";
 // "/home/laptop2/work_space/intern_ws/o3d/test_ws/txt_dataset/scale/0_06/train/feature_d_";
    string s2=".txt";

    long long int num_total_lines=total_lines(num_of_files,s1,s2);


    cerr<<"The number of total lines is :"<<num_total_lines<<endl;

    std::string s1_test=
            //"/home/laptop2/work_space/intern_ws/o3d/test_ws/txt_dataset/feature_box";
           //  "/home/laptop2/work_space/intern_ws/o3d/test_ws/test_200_309";
 //  "/home/laptop2/work_space/intern_ws/o3d/test_ws/txt_dataset/test/test";
  //  "/home/laptop2/work_space/intern_ws/o3d/test_ws/txt_dataset/diff_scale/feature_diff_scale_target";
     //  /home/laptop2/work_space/intern_ws/o3d/test_ws/txt_dataset/test/test
   "/home/laptop2/work_space/intern_ws/o3d/test_ws/txt_dataset/scale/test/feature_diff_test";
      //      "/home/laptop2/work_space/intern_ws/o3d/test_ws/txt_dataset/scale/0_06/test/feature_d_t_";
    read_train_data(num_of_files,num_total_lines,7,training_array,label_class,s1,s2,num_of_test_files,s1_test,s2,test_array,test_lables);



    return 0;
}