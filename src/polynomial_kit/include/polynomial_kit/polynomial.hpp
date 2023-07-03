/**
 * Dynamic systems library base class.
 *
 * Giorgio Manca <giorgio.manca.97@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * June 30, 2023
 */

#ifndef POLYNOMIAL_KIT__DYNAMIC_SYSTEM_HPP_
#define POLYNOMIAL_KIT__DYNAMIC_SYSTEM_HPP_

#include "visibility_control.h"

#include <cmath>
#include <Eigen/Core>

using namespace Eigen;


namespace PolynomialKit 
{
  template <typename T = double>
  class POLYNOMIAL_KIT_PUBLIC Polynomial {
    public:
      Polynomial();
      Polynomial(const T& value);
      Polynomial(const MatrixX<T>& matrix);
      ~Polynomial();

      unsigned int degree() const;
      unsigned int size() const;
      unsigned int capacity() const;
      void reserve(unsigned int capacity, bool force = false);
      void clean();
      void trim();

      void set(unsigned int degree, const T& coeff);
      T get(unsigned int degree) const;
      T coeff(unsigned int degree) const;
      T eval(const T& value) const;
      
      VectorX<T> row_vector(unsigned int mindegree = 0);
      VectorX<T> col_vector(unsigned int mindegree = 0);
      MatrixX<T> row_matrix(unsigned int mindegree = 0);
      MatrixX<T> col_matrix(unsigned int mindegree = 0);

      Polynomial& operator=(const Polynomial& other);
      
      Polynomial& operator+=(const Polynomial& other);
      Polynomial& operator-=(const Polynomial& other);
      Polynomial& operator*=(const Polynomial& other);
      Polynomial& operator^=(unsigned int p);
      Polynomial& operator<<=(unsigned int s);
      Polynomial& operator>>=(unsigned int s);

      bool operator==(const Polynomial& other) const;

      Polynomial operator+() const;
      Polynomial operator-() const; 

      Polynomial operator+(const Polynomial& other) const;
      Polynomial operator-(const Polynomial& other) const;
      Polynomial operator*(const Polynomial& other) const;
      Polynomial operator^(unsigned int p) const;
      Polynomial operator<<(unsigned int s) const;
      Polynomial operator>>(unsigned int s) const;

      template <typename U>
      inline explicit operator Polynomial<U>() const {
        return Polynomial<U>(this->poly_.template cast<U>());
      }

      friend std::ostream &operator<<(std::ostream &output, const Polynomial<T> &poly) {
        for(unsigned int i = 0; i < poly.size(); i++) {
          output << poly.poly_(0, i);
          if(i < poly.degree()) {
            output << " ";
          }
        }
        return output;  
      }

    private:
      MatrixX<T> poly_ = MatrixX<T>(1,1);
      unsigned int size_ = 1;
      unsigned int capacity_ = 1;
  };

  using Polynomiald = Polynomial<double>;
  using Polynomialf = Polynomial<float>;
  using Polynomiall = Polynomial<long>;
  using Polynomiali = Polynomial<int>;
  using Polynomials = Polynomial<short>;
  
  using Polynomialdc = Polynomial<std::complex<double>>;
  using Polynomialfc = Polynomial<std::complex<float>>;
  using Polynomiallc = Polynomial<std::complex<long>>;
  using Polynomialic = Polynomial<std::complex<int>>;
  using Polynomialsc = Polynomial<std::complex<short>>;
}

#endif  //POLYNOMIAL_KIT__DYNAMIC_SYSTEM_HPP_