#include "matrix.h"

Matrix::Matrix() {
  length = 1;
  height = 1;
  data[0] = 1;
}

Matrix::Matrix(int l, int h, double d[]) {
  length = l;
  height = h;
  for(int i = 0; i < l*h; i++) {
    data[i] = d[i];
  }
}

Matrix::Matrix(const Matrix& other) {
  this->length = other.length;
  this->height = other.height;
  std::memcpy(this->data, other.data, length*height*sizeof(double));
}

Matrix& Matrix::operator=( const Matrix& other ) {
  this->length = other.length;
  this->height = other.height;
  std::memcpy(this->data, other.data, length*height*sizeof(double));
  return *this;
}

// Default Print Function
void Matrix::printMatrix() {
  std::cout << "| ";
  double curr_val;
  for(int i = 0; i < (length*height); i++) {
    curr_val = roundf(data[i]*100)/100.0;
    if(curr_val == 0) {
      std::cout << abs(curr_val) << " ";
    }
    else std::cout << curr_val << " ";
      if(((i + 1)% length) == 0) {
	std::cout << "|";
	if((i+1) != (length*height))
	  std::cout<< std::endl << "| ";
	else
	  std::cout << std::endl << std::endl;
      }
  }
}
  
// Indexing allows for assignment and reference
double& Matrix::operator()(const int nRow, const int nCol)
{
  if(nRow < length && nCol < height) {
    return data[((nRow-1)*length+(nCol-1))];
  }
  else {
    throw std::runtime_error("Out of range");
  }
}
 
 
  /* Matrix Arithmetic */

Matrix Matrix::add(const Matrix A) {
  //check that A and B have identical dimensions
  assert(length == A.length);
  assert(height == A.height);
  Matrix Result = A;
  for(int i = 0; i < A.length*A.height; i++) {
    Result.data[i] = A.data[i] + data[i];
  }
  return Result;
}

 // Addition Operator Overload
Matrix& Matrix::operator+( const Matrix& other ) {
  Matrix temp = *this;
  for(int i = 0; i < length*height; i++) {
    this->data[i] = temp.data[i] + other.data[i];
  }

  return *this;
}

Matrix Matrix::subtract(const Matrix A) {
  //check that A and B have identical dimensions
  assert(length == A.length);
  assert(height == A.height);
  Matrix Result = A;
  for(int i = 0; i < A.length * A.height; i++) {
    Result.data[i] = data[i] - A.data[i];
  }
  return Result;
}

Matrix& Matrix::operator-( const Matrix& other ) {
  Matrix temp = *this;
  for(int i = 0; i < length*height; i++) {
    this->data[i] = temp.data[i] + other.data[i];
  }
  return *this;
}
  
 
Matrix Matrix::multiply(Matrix right) {
	
  //check that matrices have appropriate dimensions
  assert(height == right.length);
  	
  // create temporary array to load into result Matrix

  double data_temp[length*right.height];
  double sum = 0;
  int result_count = 0;
  	
  for(int i = 0; i < height; i++) {
    for(int k = 0; k < right.length; k++) {
      sum = 0;
      for (int j = 0; j < right.height; j++) {
        //sum += L[i * Lw + j] * R[j*Rw + k];
        sum += this->data[i * length + j] * right.data[j*right.length + k];
      } 
      data_temp[result_count] = sum;
      result_count++;
    }
  }
  	
  Matrix r = Matrix(right.length, height, data_temp);
  return r;
  	
}

Matrix& Matrix::operator*( const Matrix& other ) {
  double sum = 0;
  int result_count = 0;

  for(int i = 0; i < height; i++) {
    for(int k = 0; k < other.length; k++) {
      sum = 0;
      for(int j = 0; j < other.height; j++) {
	sum += this->data[i*length + j] * other.data[j*other.length + k];
      }
      this->data[result_count] = sum;
      result_count++;
    }
  }
  return *this;
}
      
  
Matrix Matrix::transpose() {
  double transpose[length*height];
  for (int i = 0; i < length; i++) {
    for(int j = 0; j < height; j++) {
      transpose[(j*length) + i] = data[(i*length) + j]; //write rows as columns and columns as rows
    }
  }
  Matrix r = Matrix(length, height, transpose);
  return r;
}
  
Matrix Matrix::inverse() {
  int pivot_row;
  double temp;
  int pivots[length];

  Matrix I = *this;
  
  for (int d = 0; d < length; d++) {
    //find pivot row, or row with the biggest value in the current column
    temp = 0;
    for(int i = d; i < length; i++) {
      if(abs(I.data[i*length+d]) >= temp)
	{
	  temp = abs(I.data[i*length+d]);
	  pivot_row = i;
	}
    }
    
    // Singular matrices are non-invertible
    if (I.data[(pivot_row*length)+d] == 0.0) {
      std::cerr << "Singular matrix detected!!!" << std::endl;
      return I;
    }

    // pivot
    if (pivot_row != d) {
      for (int j = 0; j < length; j++) {
	temp = I.data[(d*length)+j];
	I.data[(d*length)+j] = I.data[(pivot_row*length)+j];
	I.data[(pivot_row*length)+j] = temp;
      }
    }

    pivots[d] = pivot_row;
    temp = 1.0/I.data[(d*length)+d];
    I.data[(d*length)+d] = 1.0;

    //row reduction
    for(int j = 0; j < length; j++) {
      I.data[(d*length)+j] = I.data[(d*length)+j]*temp;
    }

    //elimination
    for (int i = 0; i < length; i++) {
      if(i != d) {
	temp = I.data[(i*length)+d];
	I.data[(i*length)+d] = 0.0;
	for (int j = 0; j < length; j++) {
	  I.data[(i*length)+j] = I.data[(i*length)+j] - I.data[(d*length)+j] * temp;
	}
      }
    }
  }

  //undo pivot row swaps
  for (int d = length-1; d >= 0; d--) {
    if(pivots[d] != d)
      {
	for (int i = 0; i < length; i++)
	  {
	    temp = I.data[(i*length) + d];
	    I.data[(i*length)+d] = I.data[(i*length)+pivots[d]];
	    I.data[(i*length)+pivots[d]] = temp;
	  }
      }
  }

  return I;
  
}
  
int Matrix::getLength() { return length; }
void Matrix::setLength(int nl) { length = nl; }
int Matrix::getHeight() { return height; }
void Matrix::setHeight(int nh) { height = nh; }


	 
	    
	    
    
