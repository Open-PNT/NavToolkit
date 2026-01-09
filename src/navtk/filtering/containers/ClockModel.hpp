#pragma once

namespace navtk {
namespace filtering {

/**
 * A set of Allan variance parameters that describe the relative effects
 * of white, flicker and FOGM noise processes on an oscillator.
 *
 * (1) Brown, R. G., & Hwang, P. Y. (1997). Introduction
 * to Random Signals and Applied Kalman Filtering, by Brown, Robert Grover.;
 * Hwang, Patrick YC New York: Wiley, c1997. Chapter 11 page 430, third
 * edition.
 *
 * (2) Time and Frequency: Theory and Fundamentals, Byron E. Blair, Editor,
 * NBS Monograph 140, May 1974.
 *
 * Following Ref (2), Allan variance values are parameterized by a sample
 * period \f$ \tau \f$ and calculated by sampling phase measurements at
 * \f$ \tau \f$ intervals, converting these to fractional frequency
 * deviations, then differencing and averaging over the sample period,
 * summing the squared differences and scaling. That is, if the nominal
 * frequency of an oscillator is \f$ f_0 \f$ and the angular frequency
 * offset (rad/sec) at time \f$ t \f$ is \f$ \psi (t) \f$ then the
 * fractional frequency deviation \f$ y(t) \f$ at time t is:
 *
 * \f$ y(t) = \frac{\psi(t)}{2\pi f_0} \f$
 *
 * Then for a given sampling period \f$ \tau \f$, the normalized difference is:
 *
 * \f$ \overline{y}_k(\tau) = \frac{y(t_k + \tau) - y(t_k)}{\tau} \f$
 *
 * Then for M values of \f$ \overline{y}_k(\tau) \f$, the Allan variance
 * is approximately (due to the finite number of samples):
 *
 * \f$ \sigma^2_y(\tau) \approx \frac{1}{2(M-1)}\sum^{M-1}_{k=1}(\overline{y}_{k+1} -
 * \overline{y}_k)^2 \f$
 *
 * Allan variances are normally derived from log plots of the above values
 * vs. \f$ \tau \f$.
 *
 * Predefined clock model parameters come from Ref (1).
 */
struct ClockModel {
	/**
	 * Allan variance coefficient \f$ h_{0} \f$, parameterizing the effects
	 * of a white noise on the frequency (random walk of phase).
	 */
	double h_0;
	/**
	 * Allan variance coefficient \f$ h_{-1} \f$, parameterizing the effects
	 * of a flicker noise on the frequency.
	 */
	double h_m1;
	/**
	 * Allan variance coefficient \f$ h_{-2} \f$, parameterizing the effects
	 * of a random walk on the frequency.
	 */
	double h_m2;

	/**
	 * q3 value for the third state
	 */
	double q3;
};

/** Rubidium clock model. */
constexpr ClockModel RUBIDIUM_CLOCK = {2e-20, 7e-24, 4e-29, 0};
/** Ovenized crystal clock model. */
constexpr ClockModel OVENIZED_CRYSTAL_CLOCK = {8e-20, 2e-21, 4e-23, 0};
/** Compensated crystal clock model. */
constexpr ClockModel COMPENSATED_CRYSTAL_CLOCK = {2e-19, 7e-21, 2e-20, 0};

}  // namespace filtering
}  // namespace navtk
