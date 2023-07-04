/**
 * Dynamic systems library base class.
 *
 * Giorgio Manca <giorgio.manca.97@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * June 30, 2023
 */

#include <polynomial_kit/polynomial.hpp>
#include <iostream>

namespace PolynomialKit 
{
  template <typename T>
  Polynomial<T>::Polynomial() {
    reserve(1);
    size_ = 1;
    poly_(0,0) = T();
  }

  template <typename T>
  Polynomial<T>::Polynomial(const T& value) {
    reserve(1);
    size_ = 1;
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
        size_ = matrix.rows();
        poly_ = MatrixX<T>::Map(matrix.data(), 1, matrix.rows());
      } else {
        reserve(matrix.cols());
        size_ = matrix.cols();
        poly_ = MatrixX<T>::Map(matrix.data(), 1, matrix.cols());
      }
    }
  }

  template <typename T>
  Polynomial<T>::Polynomial(const Polynomial<T>& other) {
    reserve(other.size());
    size_ = capacity_;
    poly_ = other.poly_(0, seq(0, other.degree()));
  }

  template <typename T>
  Polynomial<T>::~Polynomial() {}

  template <typename T>
  inline unsigned int Polynomial<T>::degree() const {
    return size_-1;
  }

  template <typename T>
  inline unsigned int Polynomial<T>::size() const {
    return size_;
  }

  template <typename T>
  inline unsigned int Polynomial<T>::capacity() const {
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
  void Polynomial<T>::regrade(unsigned int degree) {
    if(degree > this->degree()) {
      reserve(degree+1);
    }
    size_ = degree + 1;
  }

  template <typename T>
  void Polynomial<T>::reset(unsigned int degree, bool clean) {
    for(unsigned int i = degree; i < clean ? capacity_ : size_; i++) {
      poly_(0, i) = T();
    }
  }

  template <typename T>
  void Polynomial<T>::clean() {
    for(unsigned int i = this->size_; i < capacity_; i++) {
      poly_(0, i) = T();
    }
  }

  template <typename T>
  void Polynomial<T>::trim() {
    if(capacity_ > size_) {
      capacity_ = size_;
      poly_.conservativeResize(1, size_);
    }
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
  T Polynomial<T>::get(unsigned int degree) const {
    if(degree < size_){
      return poly_(0, degree);
    } else {
      return T();
    }
  }

  template <typename T>
  T Polynomial<T>::coeff(unsigned int degree) const {
    return get(degree);
  }

  template <typename T>
  T Polynomial<T>::eval(const T& point) const {
    T x = point;
    T y = poly_(0, 0);
    for(unsigned int i = 1; i < size_; i++) {
      y += poly_(0, i) * x;
      x *= point;
    }
    return y;
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
    MatrixX<T> res = MatrixX<T>(1, std::max(size_, mindegree+1));
    res(0, seq(0, degree())) = poly_(0, seq(0, degree()));
    return res;
  }

  template <typename T>
  MatrixX<T> Polynomial<T>::col_matrix(unsigned int mindegree) {
    MatrixX<T> res = MatrixX<T>(std::max(size_, mindegree+1), 1);
    res(seq(0, degree()), 0) = poly_(0, seq(0, degree())).transpose();
    return res;
  }

  template <typename T>
  Polynomial<T>& Polynomial<T>::operator=(const Polynomial<T>& other) {
    this->reserve(other.size());
    this->size_ = other.size();
    this->poly_(0, seq(0, other.degree())) = other.poly_(0, seq(0, other.degree()));
    return *this;
  }

    template <typename T>
  Polynomial<T>& Polynomial<T>::operator+=(const Polynomial<T>& other) {
    this->reserve(other.size());
    if(other.size() > this->size()) {
      this->poly_(0, seq(0, this->degree())) += other.poly_(0, seq(0, this->degree()));
      this->poly_(0, seq(this->size(), other.degree())) = other.poly_(0, seq(this->size(), other.degree()));
      this->size_ = other.size();
    } else {
      this->poly_(0, seq(0, other.degree())) += other.poly_(0, seq(0, other.degree()));
    }
    while(this->size_ > 1 && this->poly_(0, this->size_-1) == T()) {
      this->size_--;
    }
    return *this;
  }

  template <typename T>
  Polynomial<T>& Polynomial<T>::operator-=(const Polynomial<T>& other) {
    this->reserve(other.size());
    if(other.size() > this->size()) {
      this->poly_(0, seq(0, this->degree())) -= other.poly_(0, seq(0, this->degree()));
      this->poly_(0, seq(this->size(), other.degree())) = -other.poly_(0, seq(this->size(), other.degree()));
      this->size_ = other.size();
    } else {
      this->poly_(0, seq(0, other.degree())) -= other.poly_(0, seq(0, other.degree()));
    }
    while(this->size_ > 1 && this->poly_(0, this->size_-1) == T()) {
      this->size_--;
    }
    return *this;
  }

  template <typename T>
  Polynomial<T>& Polynomial<T>::operator*=(const Polynomial<T>& other) {
    unsigned int size = this->degree() + other.degree() + 1;
    MatrixX<T> res = MatrixX<T>::Zero(1, size);
    for(unsigned int i = 0; i < this->size(); i++) {
      for(unsigned int j = 0; j < other.size(); j++) {
        res(0, i+j) += this->poly_(0, i) * other.poly_(0, j);
      }
    }
    this->reserve(size);
    this->size_ = size;
    this->poly_(0, seq(0, this->degree())) = res;
    return *this;
  }

  template <typename T>
  Polynomial<T>& Polynomial<T>::operator^=(unsigned int p) {
    if(p == 0) {
      *this = Polynomial<T>(std::pow(T(), T()));
    } else if (p > 1) {
      Polynomial<T> square(*this);
      bool first = true;
      while(true) {
        if(p % 2 == 1) {
          std::cout << 1 << std::endl;
          if(first) {
            *this = square;
            first = false;
          } else {
            *this *= square;
          }
        } else {
          std::cout << 0 << std::endl;
        }
        p /= 2;
        if(p == 0) {
          break;
        }
        square *= square;
      }
    }    
    return *this;
  }

  template <typename T>
  Polynomial<T>& Polynomial<T>::operator<<=(unsigned int s) {
    if(s > 0) {
      this->size_ += s;
      this->reserve(this->size_);
      for(unsigned int i = this->degree(); i >= s; i--) {
        this->poly_(0, i) = this->poly_(0, i-s);
      }
      for(unsigned int i = 0; i < s; i++) {
        this->poly_(0, i) = T();
      }
    }
    return *this;
  }
  
  template <typename T>
  Polynomial<T>& Polynomial<T>::operator>>=(unsigned int s) {
    if(s > 0) {
      this->size_ -= s;
      for(unsigned int i = 0; i < this->size(); i++) {
        this->poly_(0, i) = this->poly_(0, i+s);
      }
    }
    return *this;
  }
  
  template <typename T>
  bool Polynomial<T>::operator==(const Polynomial<T>& other) const {
    if(this->degree() == other.degree()) {
      return this->poly_(0, seq(0, this->degree())) == other.poly_(0, seq(0, other.degree()));
    } else {
      return false;
    }
  }

  template <typename T>
  Polynomial<T> Polynomial<T>::operator+() const {
    return Polynomial<T>(this->poly_);
  }
  
  template <typename T>
  Polynomial<T> Polynomial<T>::operator-() const {
    return Polynomial<T>(-this->poly_);
  }

  template <typename T>
  Polynomial<T> Polynomial<T>::operator+(const Polynomial<T>& other) const {
    Polynomial<T> res(this->poly_);
    res += other;
    return res;
  }

  template <typename T>
  Polynomial<T> Polynomial<T>::operator-(const Polynomial<T>& other) const {
    Polynomial<T> res(this->poly_);
    res -= other;
    return res;
  }

  template <typename T>
  Polynomial<T> Polynomial<T>::operator*(const Polynomial<T>& other) const {
    Polynomial<T> res(this->poly_);
    res *= other;
    return res;
  }

  template <typename T>
  Polynomial<T> Polynomial<T>::operator^(unsigned int p) const {
    Polynomial<T> res(this->poly_);
    res ^= p;
    return res;
  }

  template <typename T>
  Polynomial<T> Polynomial<T>::operator<<(unsigned int s) const {
    Polynomial<T> res(this->poly_);
    res <<= s;
    return res;
  }

  template <typename T>
  Polynomial<T> Polynomial<T>::operator>>(unsigned int s) const {
    Polynomial<T> res(this->poly_);
    res >>= s;
    return res;
  }

  template class Polynomial<double>;
  template class Polynomial<float>;
  template class Polynomial<long>;
  template class Polynomial<int>;
  template class Polynomial<short>;

  template class Polynomial<std::complex<double>>;
  template class Polynomial<std::complex<float>>;
  template class Polynomial<std::complex<long>>;
  template class Polynomial<std::complex<int>>;
  template class Polynomial<std::complex<short>>;
}