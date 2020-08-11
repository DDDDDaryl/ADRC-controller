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

//Matrix������
class Matrix
{
public:
    Matrix(uint8_t row,uint8_t col)//���캯��
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
        }//��������Ԫ������
    } //����M��N�еľ���

    Matrix()//���캯��
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
        }//��������Ԫ������
    } //����M��N�еľ���

    Matrix(initializer_list<initializer_list<float>>);//���캯��
    Matrix(const Matrix &src) //�������캯��
    {
        m=src.m;
        n=src.n;
        int i,j;
        data=new float*[m];//��̬������ά����
        for(i=0;i<m;i++)
            data[i]=new float[n];
        for(i=0;i<m;i++){
            for(j=0;j<n;j++)
                data[i][j]=src.data[i][j];
        }//��̬���鸳ֵ


    }

    ~Matrix()//��������
    {
        for(int i=0;i<m;i++)
            delete []data[i];
        delete []data;
    }
    Matrix& operator = (const Matrix &src);//����"="�����
    Matrix operator * (const Matrix &m2);
    Matrix operator + (const Matrix &m2);
    Matrix operator - (const Matrix &m2);
    float* operator [] (uint8_t num);
    static Matrix cat(uint8_t dem, const Matrix &src_A, const Matrix &src_B);
    void display();

private:
    float **data;
    uint8_t m,n;//���������������
};//�ඨ�����


#endif //TEST_MATRIX_H
