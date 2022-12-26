/* ------------------------------------------------------------------------------
 * Lesson "4.3: Motion Planning"
 * Authors     : Munir Jojo-Verge.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Header file for the Simpson's Rule numerical
 *                       approximation formula needed to compute the integrals
 *                       of the cubic spiral with respect to $x$ and $y$.
 * ----------------------------------------------------------------------------
 */

#include <array>
#include <cmath>
#include <iostream>
#include <vector>


// Returns the $theta(s)$ equation evaluated at point `s`.
double calc_theta(
    const double s, 
    const std::array<double, 4>& a
);


// Implements the Simpson's 1/3 rule numerical integration technique.
double IntegrateBySimpson(
    const std::vector<double>& func, 
    const double dx,
    const std::size_t n
);


// Computes the integrand, i.e., the function of the cubic spiral to integrate.
std::vector<double> generate_f_s0_sn(
    const double delta_s,
    const std::array<double, 4>& a,
    const std::size_t n
);