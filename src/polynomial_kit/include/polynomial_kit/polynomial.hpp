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

      int degree();
      unsigned int size();
      unsigned int capacity();
      void reserve(unsigned int capacity, bool force = false);
      void clean();
      void trim();

      void set(unsigned int degree, const T& coeff);
      T get(unsigned int degree);
      T coeff(unsigned int degree);
      T eval(const T& value);
      
      VectorX<T> row_vector(unsigned int mindegree = 0);
      VectorX<T> col_vector(unsigned int mindegree = 0);
      MatrixX<T> row_matrix(unsigned int mindegree = 0);
      MatrixX<T> col_matrix(unsigned int mindegree = 0);

      Polynomial& operator=(const Polynomial& other);
      
      Polynomial& operator+=(const Polynomial& other);  // Sum
      Polynomial& operator-=(const Polynomial& other);  // Subtraction
      Polynomial& operator*=(const Polynomial& other);  // Multiplication
      Polynomial& operator^=(unsigned int p);         // Power
      Polynomial& operator<<=(unsigned int s);        // Shift left  (increase degree)
      Polynomial& operator>>=(unsigned int s);        // Shift right (decrease degree)

      bool operator==(const Polynomial& other);

      Polynomial operator+();
      Polynomial operator-(); 

      Polynomial operator+(const Polynomial& other);  // Sum
      Polynomial operator-(const Polynomial& other);  // Subtraction
      Polynomial operator*(const Polynomial& other);  // Multiplication
      Polynomial operator^(unsigned int p);           // Power
      Polynomial operator<<(unsigned int s);          // Shift left  (increase degree)
      Polynomial operator>>(unsigned int s);          // Shift right (decrease degree)

      template <typename U>
      inline explicit operator Polynomial<U>() const {
        return Polynomial<U>(this->poly_.cast<U>());
      }

    private:
      MatrixX<T> poly_ = MatrixX<T>(1,1);
      unsigned int size_ = 1;
      unsigned int capacity_ = 1;
  };

  typedef Polynomial<double> Polynomiald;
  typedef Polynomial<float> Polynomialf;
  typedef Polynomial<long> Polynomiall;
  typedef Polynomial<int> Polynomiali;
  typedef Polynomial<short> Polynomials;
  
  typedef Polynomial<std::complex<double>> Polynomialdc;
  typedef Polynomial<std::complex<float>> Polynomialfc;
  typedef Polynomial<std::complex<long>> Polynomiallc;
  typedef Polynomial<std::complex<int>> Polynomialic;
  typedef Polynomial<std::complex<short>> Polynomialsc;
}

#endif  //POLYNOMIAL_KIT__DYNAMIC_SYSTEM_HPP_