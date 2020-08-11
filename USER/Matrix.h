//
// Created by Frank Young on 2020/4/5.
//

#ifndef __MATRIX_H
#define __MATRIX_H
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <initializer_list>

using namespace std;

//Matrix矩阵类
class Matrix
{
public:
    Matrix(uint8_t row,uint8_t col)//构造函数
    {
        m=row;
        n=col;
        uint8_t i=0, j=0;
        data=new float*[row];
        for(i=0;i<row;i++)
            data[i]=new float[col];
        for(i=0;i<m;i++){
            for(j=0;j<n;j++)
                data[i][j]=0.0;
        }//矩阵所有元素清零
    } //构造M行N列的矩阵

    Matrix()//构造函数
    {
        m=1;
        n=1;
        int i,j;
        data=new float*[1];
        for(i=0;i<1;i++)
            data[i]=new float[1];
        for(i=0;i<m;i++){
            for(j=0;j<n;j++)
                data[i][j]=0.0;
        }//矩阵所有元素清零
    } //构造M行N列的矩阵

    Matrix(initializer_list<initializer_list<float>>);//构造函数
    Matrix(const Matrix &src) //拷贝构造函数
    {
        m=src.m;
        n=src.n;
        int i,j;
        data=new float*[m];//动态建立二维数组
        for(i=0;i<m;i++)
            data[i]=new float[n];
        for(i=0;i<m;i++){
            for(j=0;j<n;j++)
                data[i][j]=src.data[i][j];
        }//动态数组赋值


    }

    ~Matrix()//析构函数
    {
        for(int i=0;i<m;i++)
            delete []data[i];
        delete []data;
    }
    Matrix& operator = (const Matrix &src);//重载"="运算符
    Matrix operator * (const Matrix &m2);
    Matrix operator + (const Matrix &m2);
    Matrix operator - (const Matrix &m2);
    float* operator [] (uint8_t num);
    static Matrix cat(uint8_t dem, const Matrix &src_A, const Matrix &src_B);
    void display();

private:
    float **data;
    uint8_t m,n;//矩阵的行数，列数
};//类定义结束


#endif //TEST_MATRIX_H
