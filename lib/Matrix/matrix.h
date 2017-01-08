#ifndef MATRIX_H
#define MATRIX_H

#include "assert.h"
#include <iostream>
#include <stdexcept>
#include <math.h>
#include <cstring>
#define MAX_MATRIX_SIZE 64

class Matrix {

 public:
  // Constructor
  Matrix();
  // Alt Constructor
  Matrix(int l, int h, double d[]); 
  Matrix( const Matrix& other );
  Matrix& operator=( const Matrix& other );

  void printMatrix();  

  double& operator()(const int, const int);

  Matrix add(const Matrix);

  Matrix& operator+( const Matrix& ); 

  Matrix subtract(const Matrix);

  Matrix& operator-( const Matrix& );
  
  Matrix multiply(Matrix);

  Matrix& operator*( const Matrix& );

  Matrix transpose();
    
  Matrix inverse();
    
  int getLength();
  
  void setLength(int);
  
  int getHeight();
  
  void setHeight(int);

 private:
  
  int length;
  int height;
  double data[MAX_MATRIX_SIZE];

};


#endif


