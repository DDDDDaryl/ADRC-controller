//
// Created by Frank Young on 2020/4/5.
//

#include "Matrix.h"
Matrix& Matrix::operator =(const Matrix &src) //重载"="运算符
{
    if(this == &src)
        return *this;
    int i,j;
    for(i=0;i<m;i++)
        delete [] data[i];
    delete [] data;
    m=src.m;n=src.n;
    data=new float*[m];//动态建立二维数组
    for(i=0;i<m;i++)
        data[i]=new float[n];
    for(i=0;i<m;i++)
        for(j=0;j<n;j++)
            data[i][j]=src.data[i][j];
    return *this;
}

//矩阵*运算符重载
Matrix Matrix::operator *(const Matrix &m2)//矩阵乘法的实现
{
    Matrix m3(this->m,m2.n);
    if(this->m * this->n == 1){
        Matrix m4(m2.m,m2.n);
        for(int i=0; i<m4.m; ++i){
            for(int j=0; j<m4.n; ++j){
                m4.data[i][j] = this->data[0][0]*m2.data[i][j];
            }
        }
        return m4;
    }
    else if (m2.m*m2.n == 1){
        Matrix m4(this->m,this->n);
        for(int i=0; i<m4.m; ++i){
            for(int j=0; j<m4.n; ++j){
                m4.data[i][j] = m2.data[0][0]*this->data[i][j];
            }
        }
        return m4;
    }
    else if(this->n!=m2.m)
    {
        printf("Operator* ERROR.\r\n");
        exit(0);
    }
    int i,j,k;
    for(i=0;i<this->m;i++)
        for(j=0;j<m2.n;j++)
        {
            for(k=0;k<this->n;k++)
            {
                m3.data[i][j]+=this->data[i][k]*m2.data[k][j];
            }

        }
    return m3;
}

//显示矩阵元素
void Matrix::display()
{
    int i,j;
    for(i=0;i<m;i++)
    {
        for(j=0;j<n;j++)
        {
            printf("%#1.6f\t", data[i][j]);
        }
        printf("\r\n");
    }
}

Matrix Matrix::operator+(const Matrix &m2) {
    Matrix m3(this->m,this->n);
    if(this->m!=m2.m||this->n!=m2.n){
        printf("Matrix Matrix::operator+(const Matrix &m2) ERROR.");
        exit(0);
    }
    for(int i=0; i<this->m; ++i){
        for(int j=0; j<this->n; ++j){
            m3.data[i][j] = this->data[i][j] + m2.data[i][j];
        }
    }
    return m3;
}

Matrix Matrix::operator-(const Matrix &m2) {
    Matrix m3(this->m,this->n);
    if(this->m!=m2.m||this->n!=m2.n){
        printf("Matrix Matrix::operator-(const Matrix &m2) ERROR.");
        exit(0);
    }
    for(int i=0; i<this->m; ++i){
        for(int j=0; j<this->n; ++j){
            m3.data[i][j] = this->data[i][j] - m2.data[i][j];
        }
    }
    return m3;
}

Matrix::Matrix(initializer_list<initializer_list<float>> init) {
    m = init.size();
    n = init.begin()->size();
    const initializer_list<float> *it_i = init.begin();
    const float *it_j;
    data = new float* [m];
    for(int i=0; i<m; ++i, ++it_i){
        data[i] = new float[n];
        it_j = it_i->begin();
        for(int j=0; j<n; ++j, ++it_j){
            data[i][j] = *it_j;
        }
    }
}

Matrix Matrix::cat(uint8_t dem, const Matrix &src_A, const Matrix &src_B) {
    if(dem == 1){
        if(src_A.m != src_B.m){
            return Matrix(1,1);
        }
        Matrix tmp(src_A.m, src_A.n+src_B.n);
        for(uint8_t i=0; i<tmp.n; ++i){
            for(uint8_t j=0; j<tmp.m && i<src_A.n; ++j){
                tmp.data[j][i] = src_A.data[j][i];
            }
            for(uint8_t j=0; j<tmp.m && i>=src_A.n; ++j){
                tmp.data[j][i] = src_B.data[j][i-src_A.n];
            }
        }
        return tmp;
    }
    else if (dem ==2){
        if(src_A.n != src_B.n){
            return Matrix(1,1);
        }
        Matrix tmp(src_A.m+src_B.m, src_A.n);
        for(uint8_t i=0; i<tmp.m; ++i){
            for(uint8_t j=0; j<tmp.n && i<src_A.m; ++j){
                tmp.data[i][j] = src_A.data[i][j];
            }
            for(uint8_t j=0; j<tmp.n && i>=src_A.m; ++j){
                tmp.data[i][j] = src_B.data[i-src_A.m][j];
            }
        }
        return tmp;
    }
    else{
        return Matrix(1,1);
    }
}

float* Matrix::operator [] (uint8_t num){
    return this->data[num];
}

const float* Matrix::operator [] (uint8_t num) const {
    return this->data[num];
}