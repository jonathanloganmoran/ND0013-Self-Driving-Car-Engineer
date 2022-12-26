/* ------------------------------------------------------------------------------
 * Lesson "4.3: Motion Planning"
 * Authors     : Munir Jojo-Verge.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Implements the Simpson's Rule numerical approximation
 *                       formula needed to compute the integrals of the cubic
 *                       spiral with respect to $x$ and $y$.
 * ----------------------------------------------------------------------------
 */

#include "1_simpsons_rule.h"


/* Returns the $theta(s)$ equation evaluated at point `s`.
 *
 * The polynomial function of the cubic spiral is described w.r.t. the
 * equation $\theta(s)$ such that $\theta_{0} = 0$, which results in:
 * $ \theta_{s} = a_{3}*s^{4}/4 + a_{2}*s^{3}/3 + a_{1}*s^{2}/2 + a_{0}*s $.  
 *
 * @param    s    Point at which to evaluate $theta(s)$.
 * @param    a    Vector of coefficient values of $theta(s)$.
 * @returns  Equation $theta(s)$ evaluated at point `s` with coefficients `a`.
 */
double calc_theta(
    const double s, 
    const std::array<double, 4>& a
) {
  double theta_s = 0;
  // Loop over all coefficients of the polynomial formed by the cubic spiral
  // Here we assume $theta_{0} = 0$
  for (int i = 0; i < a.size(); ++i) {
    double theta_i = a[i] * std::pow(s, i + 1) / (i + 1);
    theta_s += theta_i;
  }
  return theta_s;
}


/* Implements the Simpson's 1/3 rule numerical integration technique.
 *
 * For this problem (polynomial of the cubic spiral), the formula for the
 * Simpson's 1/3 rule computed here is given as:
 * $ (\Delta{x}/3) * (f(x_{0}) + 4*f(x_{1}) + 2*f(x_{2}) + ... + 4*f(x_{n+1}) + f(x_{n})) $.
 * 
 * @param    func     Equation of the integrand $f(s)$.
 * @param    dx       Integration step-size.
 * @param    n        Number of subintervals to divide [a, b], should be even.
 * @returns  result   Integral approximated with the Simpson's 1/3 rule.
 */
double IntegrateBySimpson(
    const std::vector<double>& func, 
    const double dx,
    const std::size_t n
) {
  if (n == 0) {
    return 0.0;
  }
  double sum_even = 0.0;
  double sum_odd = 0.0;
  for (std::size_t i = 1; i < n; ++i) {
    // Check if term is odd
    if ((i & 1) != 0) {
      sum_odd += func[i];
    } 
    else {
      sum_even += func[i];
    }
  }
  double result = (4.0 * sum_odd + 2.0 * sum_even + func[0] + func[n]);
  result *= (dx / 3.0);
  return result;
}


/* Computes the integrand, i.e., the function of the cubic spiral to integrate.
 *
 * The time-step $\Delta{x}$ is given as $\Delta{x} = \frac{b - a}{n}$ where
 * $n$ is the number of subintervals dividing the limits of integration.
 * We compute the $x_{i}$ in the Simpson's 1/3 rule as $x_{i} = i * \Delta{x}$.
 *
 * @param    delta_s  Integration interval, i.e., numerator of the step-size.
 * @param    a        Set of coefficients of the cubic spiral.
 * @param    n        Number of subintervals to divide [a, b], should be even.
 * @returns  f_s      Integrand $f(s)$.
 */
std::vector<double> generate_f_s0_sn(
    const double delta_s,
    const std::array<double, 4>& a,
    const std::size_t n
) {
  std::vector<double> f_s(n + 1);
  for (size_t i = 0; i <= n; ++i) {
    double s_i = i * delta_s;
    double theta_i = calc_theta(s_i, a);
    f_s[i] = std::cos(theta_i);
  }
  return f_s;
}