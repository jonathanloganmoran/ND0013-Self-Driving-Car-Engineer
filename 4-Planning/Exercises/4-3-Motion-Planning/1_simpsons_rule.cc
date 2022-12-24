/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: October 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/

/*
In this exercise we will calculate the value of the integral:
x(s)= integral(cos(theta(s)) ds) from a (s=0) to b (s=sf).
To do so since this integral does NOT have a closed-form solution we will use
Simpson's Rule.
The only things you need are:
    1) The definition of Theta(s) for a Cubic Spiral, which is:
        theta(s) = th_0 + a3*s^4/4 + a2*s^3/3 + a1*s^2/2 + a0*s
        NOTE: th_0 will be always 0 in this exercise.
    2) Simpson's Rule equation:
    f(x) = dx / 3.0 * (f(x0) + 4.0 * f(x1) + 2.0 * f(x2) + 4.0 * f(x3) + 2.0 *
    f(x4) + ... + func[n]);
    dx = b-a/n = sf/n
    xi = a + i*dx = i*dx
Follow the "TODO"s to complete this exercise
*/

#include <array>
#include <cmath>
#include <iostream>
#include <vector>

double calc_theta(const double s, const std::array<double, 4>& a) {
  // TODO - What is Theta(s)?
  // theta(s) = a3*s^4/4 + a2*s^3/3 + a1*s^2/2 + a0*s
  return ...;  // FIX THIS
}

double IntegrateBySimpson(const std::vector<double>& func, const double dx,
                          const std::size_t n) {
  if (n == 0) {
    return 0.0;
  }

  double sum_even = 0.0;
  double sum_odd = 0.0;
  for (std::size_t i = 1; i < n; ++i) {
    if ((i & 1) != 0) {
      sum_odd += func[i];
    } else {
      sum_even += func[i];
    }
  }
  // TODO - Put together Simpson's Rule here.
  // Remember that the formula for Simpson's Rule is:
  // delta_x/3*(f(x0) + 4f(x1) + 2f(x2) + 4f(x3) + 2f(x4) + ... + 4f(xn-1) +
  // f(xn))
  //
  double result = ...;

  return result;  // FIX THIS
}

std::vector<double> generate_f_s0_sn(const double delta_s,
                                     const std::array<double, 4>& a,
                                     const std::size_t n) {
  std::vector<double> f_s(n + 1);
  for (size_t i = 0; i <= n; ++i) {
    // TODO - What is s_i (or x_i on Simpsons Rule equation)
    // f(x) = dx / 3.0 * (f(x0) + 4.0 * f(x1) + 2.0 * f(x2) + 4.0 * f(x3) + 2.0
    // * f(x4) + ... + func[n]);
    // dx = sf/n;
    // xi = i*dx;
    // HINT: s_i = i * delta_s;
    double s_i = ...;  // FIX THIS

    double theta_i = calc_theta(s_i, a);
    f_s[i] = std::cos(theta_i);
  }
  return f_s;
}

// ******* MAIN FUNCTION ********
int main(int argc, const char* argv[]) {
  //
  double sg = 10.0;  // lenght of the spiral
  size_t n = 9;      // We'll use a n=9 Simpson's Rule
  double delta_s = sg / n;

  // Spiral "a" Coefficients: K(s)= a3*s^3 + a2*s^2 + a1*s + a0;
  // a=[a0,a1,a2,a3]
  std::array<double, 4> a{0.0, 0.045, 0.0225, -0.0027};

  // Function to integrate: f(s) = cos(theta(s))
  std::vector<double> f_s0_sn = generate_f_s0_sn(delta_s, a, n);

  double integral_result = IntegrateBySimpson(f_s0_sn, delta_s, n);

  double expected = 2.3391428316;
  std::cout << (fabs(expected - integral_result) < 0.00001 ? "PASS" : "FAIL")
            << std::endl;
  // std::cout << "Result: " << integral_result << std::endl;

  return 0;