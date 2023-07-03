/**
 * Dynamic systems library base class.
 *
 * Giorgio Manca <giorgio.manca.97@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * June 30, 2023
 */

#include <polynomial_kit/polynomial.hpp>

namespace PolynomialKit 
{
  template <typename T>
  Polynomial<T>::Polynomial() {
    reserve(1);
    poly_(0,0) = T();
  }

  template <typename T>
  Polynomial<T>::Polynomial(const T& value) {
    reserve(1);
    poly_(0,0) = value;
  }

  template <typename T>
  Polynomial<T>::Polynomial(const MatrixX<T>& matrix) {
    if(matrix.rows() > 2 && matrix.cols() > 2) {
      std::invalid_argument("Argument is not a vector.");
    }

    if(matrix.rows() >= 1 && matrix.cols() >= 1) {
      if(matrix.rows() > 1) {
        reserve(matrix.rows());
        poly_ = MatrixX<T>::Map(matrix.data(), 1, matrix.rows());
      } else {
        reserve(matrix.cols());
        poly_ = MatrixX<T>::Map(matrix.data(), 1, matrix.cols());
      }
    }
  }

  template <typename T>
  Polynomial<T>::~Polynomial() {}

  template <typename T>
  int Polynomial<T>::degree() {
    return size_-1;
  }

  template <typename T>
  unsigned int Polynomial<T>::size() {
    return size_;
  }

  template <typename T>
  unsigned int Polynomial<T>::capacity() {
    return capacity_;
  }
  
  template <typename T>
  void Polynomial<T>::reserve(unsigned int capacity, bool force) {
    if(capacity == 0) {
      std::invalid_argument("capacity cannot be zero");
    }
    if(force || capacity > capacity_) {
      poly_.conservativeResize(1, capacity);
      capacity_ = capacity;
      if(capacity_ < size_) {
        size_ = capacity_;
      }
    }
  }
  
  template <typename T>
  void Polynomial<T>::clean() {
    this->poly_(0, seq(this->size(), this->capacity()-1)) = T();
  }

  template <typename T>
  void Polynomial<T>::trim() {
    unsigned int index = size_-1;
    while(index > 0) {
      if(poly_(0, index) != 0) {
        break;
      }
      index--;
    }
    capacity_ = index+1;
    size_ = capacity_;
    poly_.conservativeResize(1, size_);
  }

  template <typename T>
  void Polynomial<T>::set(unsigned int degree, const T& coeff) {
    if(coeff != T()){
      if(degree >= capacity_) {
        reserve(degree+1);
      }
      if(degree >= size_) {
        size_ = degree+1;
      }
      poly_(0, degree) = coeff;
    }
  }

  template <typename T>
  T Polynomial<T>::get(unsigned int degree) {
    if(degree < size_){
      return poly_(0, degree);
    } else {
      return T();
    }
  }

  template <typename T>
  T Polynomial<T>::coeff(unsigned int degree) {
    return get(degree);
  }

  template <typename T>
  T Polynomial<T>::eval(const T& point) {
    T state = point;
    T value = (*this)[0];
    for(int i = 1; i < this->degree(); i++) {
      value = (*this)[i] * state;
      state = state * state;
    }
    return value;
  }

  template <typename T>
  VectorX<T> Polynomial<T>::row_vector(unsigned int mindegree) {
    return row_matrix(mindegree);
  }

  template <typename T>
  VectorX<T> Polynomial<T>::col_vector(unsigned int mindegree) {
    return col_matrix(mindegree);
  }
  
  template <typename T>
  MatrixX<T> Polynomial<T>::row_matrix(unsigned int mindegree) {
    if(mindegree >= size_) {
      return MatrixX<T>::Map(poly_.data(), 1, poly_.cols()).conservativeResize(1, mindegree+1);
    } else {
      return MatrixX<T>::Map(poly_.data(), 1, poly_.cols());
    }
  }

  template <typename T>
  MatrixX<T> Polynomial<T>::col_matrix(unsigned int mindegree) {
    if(mindegree >= size_) {
      return MatrixX<T>::Map(poly_.data(), poly_.cols(), 1).conservativeResize(1, mindegree+1);
    } else {
      return MatrixX<T>::Map(poly_.data(), poly_.cols(), 1);
    }
  }

  template <typename T>
  Polynomial<T>& Polynomial<T>::operator=(const Polynomial<T>& other) {
    this->reserve(other.size());
    this->poly_(0, seq(0, other.degree())) = other.poly_(0, seq(0, other.degree()));
    return *this;
  }

    template <typename T>
  Polynomial<T>& Polynomial<T>::operator+=(const Polynomial<T>& other) {
    unsigned int degree = 0;
    this->reserve(other.size());
    if(other.size() > this->size()) {
      degree = other.degree();
      this->size_ = other.size();
    } else {
      degree = this->degree();
    }
    this->poly_(0, seq(0, degree)) += other.poly_(0, seq(0, degree));
    while(this->size_ > 1 && this->poly_(0, this->size_-1) == T()) {
      this->size_--;
    }
    return *this;
  }

  template <typename T>
  Polynomial<T>& Polynomial<T>::operator-=(const Polynomial<T>& other) {
    unsigned int degree = 0;
    this->reserve(other.size());
    if(other.size() > this->size()) {
      degree = other.degree();
      this->size_ = other.size();
    } else {
      degree = this->degree();
    }
    this->poly_(0, seq(0, degree)) -= other.poly_(0, seq(0, degree));
    while(this->size_ > 1 && this->poly_(0, this->size_-1) == T()) {
      this->size_--;
    }
    return *this;
  }

  template <typename T>
  Polynomial<T>& Polynomial<T>::operator*=(const Polynomial<T>& other) {
    unsigned int size = this->degree() + other.degree() + 1;
    MatrixX<T> res = MatrixX<T>(1, size);
    for(unsigned int i = 0; i < this->size(); i++) {
      for(unsigned int j = 0; j < this->size(); j++) {
        res.poly_(0, i+j) += this->poly_(0, i) * this->poly_(0, j);
      }
    }
    this->reserve(size);
    this->size_ = size;
    this->poly_(0, seq(0, size-1)) = res;
    return *this;
  }

  template <typename T>
  Polynomial<T>& Polynomial<T>::operator^=(unsigned int p) {
    Polynomial<T> square = *this;
    bool first = true;
    while(true) {
      if(p % 2 == 1) {
        if(first) {
          this = square;
          first = false;
        } else {
          this *= square;
        }
      }
      p /= 2;
      if(p == 0) {
        break;
      }
      square *= square;
    }
    return *this;
  }

  template <typename T>
  Polynomial<T>& Polynomial<T>::operator<<=(unsigned int s) {
    this->size_++;
    this->reserve(this->size());
    this->poly_(0, seq(1, this->degree())) = this->poly_(0, seq(0, this->degree()-1));
    this->poly_(0, 0) = T();
    return *this;
  }
  
  template <typename T>
  Polynomial<T>& Polynomial<T>::operator>>=(unsigned int s) {
    this->size_--;
    this->poly_(0, seq(0, this->degree()-1)) = this->poly_(0, seq(1, this->degree()));
    return *this;
  }
  
  template <typename T>
  bool Polynomial<T>::operator==(const Polynomial<T>& other) {
    if(this->degree() == other.degree()) {
      return this->poly_(0, seq(0, this->degree())) == other.poly_(0, seq(0, other.degree()));
    } else {
      return false;
    }
  }

  template <typename T>
  Polynomial<T> Polynomial<T>::operator+() {
    Polynomial<T> res;
    res.capacity_ = this->capacity_;
    res.size_ = this->size_;
    res.poly_ = this->poly_;
    return res;
  }
  
  template <typename T>
  Polynomial<T> Polynomial<T>::operator-() {
    Polynomial<T> res;
    res.capacity_ = this->capacity_;
    res.size_ = this->size_;
    res.poly_ = -this->poly_;
    return res;
  }

  template <typename T>
  Polynomial<T> Polynomial<T>::operator+(const Polynomial<T>& other) {
    Polynomial<T> res = *this;
    res += other;
    return res;
  }

  template <typename T>
  Polynomial<T> Polynomial<T>::operator-(const Polynomial<T>& other) {
    Polynomial<T> res = *this;
    res -= other;
    return res;
  }

  template <typename T>
  Polynomial<T> Polynomial<T>::operator*(const Polynomial<T>& other) {
    Polynomial<T> res = *this;
    res *= other;
    return res;
  }

  template <typename T>
  Polynomial<T> Polynomial<T>::operator^(unsigned int p) {
    Polynomial<T> res = *this;
    res ^= p;
    return res;
  }

  template <typename T>
  Polynomial<T> Polynomial<T>::operator<<(unsigned int s) {
    Polynomial<T> res = *this;
    res <<= s;
    return res;
  }

  template <typename T>
  Polynomial<T> Polynomial<T>::operator>>(unsigned int s) {
    Polynomial<T> res = *this;
    res >>= s;
    return res;
  }
}