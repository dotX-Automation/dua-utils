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
      Polynomial(T val);
      Polynomial(VectorX<T> vector);
      Polynomial(MatrixX<T> matrix);

      ~Polynomial();

      VectorX<T> row_vector();
      VectorX<T> col_vector();
      
      MatrixX<T> row_matrix();
      MatrixX<T> col_matrix();

      Polynomial& operator=(const Polynomial& other);
      
      T& operator[](std::size_t idx);
      const T& operator[](std::size_t idx) const;

      T operator()(T x) const;

      bool operator==(const Polynomial& other);

      Polynomial operator+();
      Polynomial operator-(); 

      Polynomial operator+(const Polynomial& other);  // Sum
      Polynomial operator-(const Polynomial& other);  // Subtraction
      Polynomial operator*(const Polynomial& other);  // Multiplication
      Polynomial operator/(const Polynomial& other);  // Division whole part
      Polynomial operator%(const Polynomial& other);  // Division remainder
      Polynomial operator^(const unsigned int& p);    // Power
      Polynomial operator<<(const unsigned int& s);   // Shift left  (increase degree)
      Polynomial operator>>(const unsigned int& s);   // Shift right (decrease degree)

      Polynomial& operator+=(const Polynomial& rhs);  // Sum
      Polynomial& operator-=(const Polynomial& rhs);  // Subtraction
      Polynomial& operator*=(const Polynomial& rhs);  // Multiplication
      Polynomial& operator/=(const Polynomial& rhs);  // Division whole part
      Polynomial& operator%=(const Polynomial& rhs);  // Division remainder
      Polynomial& operator^=(const unsigned int& p);  // Power
      Polynomial& operator<<=(const unsigned int& s); // Shift left  (increase degree)
      Polynomial& operator>>=(const unsigned int& s); // Shift right (decrease degree)
    
    private:
      MatrixX<T> poly;
  };
}

#endif  //POLYNOMIAL_KIT__DYNAMIC_SYSTEM_HPP_