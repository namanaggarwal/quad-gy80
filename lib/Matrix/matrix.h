#ifndef MATRIX_H
#define MATRIX_H

#include "assert.h"
#include "Arduino.h"

#include <math.h>


#define MAX_MATRIX_SIZE 100

class Matrix {
 public: 
  // Constructor
  Matrix();  
  // Alt Constructors  
  Matrix(int h, int w, double d[]);
  Matrix(int h, int w);
  Matrix( const Matrix& other );  
  Matrix& operator=( const Matrix& other ); 

  
  void printMatrix();
  
  double& operator()( const int, const int );
  double& operator()( const int );
  
  Matrix add( const Matrix );  
  Matrix& operator+( const Matrix& );
  
  Matrix subtract( const Matrix );
  Matrix& operator-( const Matrix& );
  
  Matrix multiply(Matrix);

  Matrix divide(Matrix);

  Matrix& operator*( const Matrix& );

  Matrix transpose();
  Matrix inverse();

  Matrix concatenate( const Matrix, int axis=0 );
    
  int getWidth();
  void setWidth(int);
  int getHeight();
  void setHeight(int);

 private:
  int width;
  int height;
  double data[MAX_MATRIX_SIZE];
   
};

#endif

