#pragma once

#include <memory>

#include <navtk/aspn.hpp>
#include <navtk/filtering/GenXhatPFunction.hpp>
#include <navtk/filtering/containers/ClockModel.hpp>
#include <navtk/filtering/containers/StandardDynamicsModel.hpp>
#include <navtk/filtering/stateblocks/StateBlock.hpp>
#include <navtk/navutils/math.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace filtering {

/**
 * enum that toggles between different Q models.
 */
enum class ClockChoice {

	/**
	 * 'QD' as given in Brown and Hwang with flicker terms \f$ h_{-1} \f$ included.
	 *
	 * For two states, the matrix is as follows:
	 * \f$ \begin{bmatrix} \frac{h_0}{2}dt + 2h_{-1}dt^2 + \frac{2}{3}\pi^2h_{-2}
	 * dt^3 & \pi^2h_{-2}dt^2 \\ \pi^2h_{-2}dt^2 & 4h_{-1} + 2\pi^2h_{-2}dt
	 * \end{bmatrix} \f$
	 *
	 *
	 * Estimates clock error and drift using three Allan variance coefficients.
	 * enum that toggles between different Q models. Please note that for three states, the
	 * matrix below is added to its corresponding expanded two-state matrix with an extra row and
	 * column of zeroes:
	 *
	 * \f$ \begin{bmatrix} \frac{Q_3dt^5}{20} & \frac{Q_3dt^4}{8} & \frac{Q_3dt^3}{6}
	 * \\ \frac{Q_3dt^4}{8} & \frac{Q_3dt^3}{6} & \frac{Q_3dt^2}{2}
	 * \\ \frac{Q_3dt^3}{6} & \frac{Q_3dt^2}{2} & Q_3dt
	 * \end{bmatrix} \f$
	 *
	 * References:
	 * (1) Brown, R. G., & Hwang, P. Y. (1997). Introduction
	 * to Random Signals and Applied Kalman Filtering, by Brown, Robert Grover.;
	 * Hwang, Patrick YC New York: Wiley, c1997. Chapter 11 page 430, third
	 * edition.
	 *
	 * (2) Relationship between Allan Variances and Kalman Filter Parameters.
	 * A.J. Van Dierendonck, J.B. McGraw and R. G. Brown.
	 *
	 * (3) Time and Frequency: Theory and Fundamentals, Byron E. Blair, Editor,
	 * NBS Monograph 140, May 1974.
	 *
	 * States:
	 *
	 * 0 - The clock's bias (seconds).
	 *
	 * 1 - The clock's average drift over propagation period (seconds/seconds).
	 *
	 * This clock bias model assumes that clock errors are the result of 3
	 * separate noise processes acting upon the frequency of the clock- a
	 * white noise source, a 'flicker' noise, and a random walk. The
	 * propagation of these states is simple, but the calculation of the noise
	 * covariance matrix is difficult with the flicker noise term included.
	 * Reference (2) gives the matrix as (see ClockModel for coefficient definitions):
	 *
	 * \f$ \begin{bmatrix}
	 * \frac{h_0}{2}\Delta t + 2 h_{-1}\Delta t^2 + \frac{2}{3}\pi^2 h_{-2}\Delta t^3 &
	 * 2h_{-1}\Delta t
	 * + \pi^2 h_{-2}\Delta t^2 \\ 2h_{-1}\Delta t + \pi^2 h_{-2}\Delta t^2 & \frac{h_0}{2\Delta t}
	 * + 2h_{-1} + \frac{8}{3} \pi^2h{-2}\Delta t \end{bmatrix}\f$
	 *
	 * Ref (1) cites Ref (2) but states that the result is incorrect, giving instead:
	 *
	 * \f$ \begin{bmatrix}
	 * \frac{h_0}{2}\Delta t + 2h_{-1}\Delta t^2 + \frac{2}{3}\pi^2h_{-2}\Delta t^3 & h_{-1}\Delta t
	 * + \pi^2h_{-2}\Delta t^2 \\ h_{-1}\Delta t + \pi^2h_{-2}\Delta t^2 & \frac{h_0}{2\Delta t} +
	 * 4h_{-1}
	 * + \frac{8}{3}\pi^2h_{-2}\Delta t \end{bmatrix} \f$
	 *
	 * However, attempting to validate yields a different result. Following along
	 * with Ref (2), the correlation function is given as:
	 *
	 * \f$ R_{xy}(t, \tau) = \int^t_0 h_x(u)h_y(u + \tau)\delta u; \tau \geq 0 \f$
	 *
	 * With the covariance at 0 correlation time:
	 *
	 * \f$ R_{xy}(t) = \int^t_0 h_x(u)h_y(u)\delta u \f$
	 *
	 * where \f$ h_n(t) \f$ is the impulse response functions for the various noise sources.
	 *
	 * The impulse responses for the clock bias process (first state) are given:
	 *
	 * \f$ h_{x_0}(t) = \sqrt{\frac{h_0}{2}}\mu(t) \f$
	 *
	 * \f$ h_{x_{-1}}(t) = 2\sqrt{h_{-1}t} \f$
	 *
	 * \f$ h_{x_{-2}}(t) = \pi\sqrt{2h_{-2}}t \f$
	 *
	 * where \f$ \mu(t) \f$ is the unit step function.
	 *
	 * The second state models drift in the first state. Modeling the instantaneous
	 * value of this process is problematic, so it is instead redefined as
	 * the average fractional frequency:
	 *
	 * \f$ \overline{y}(t) = \frac{x(t + \Delta t) - x(t)}{\Delta t} \f$
	 *
	 * In other words, it is the average value of the frequency drift (in sec/sec)
	 * over some sample time \f$ \Delta t\f$, which for our purposes is the
	 * filter propagation interval.
	 *
	 * The covariance we are looking for is \f$ cov[x(t), \overline{y}(t)]\f$.
	 * Plugging each \f$ h_{x_n} \f$ into the above equation to to solve for \f$ h_{\overline{y}_n}
	 * \f$:
	 *
	 * \f$ h_{\overline{y}_0}(t) = \frac{\sqrt{\frac{h_0}{2}}\mu(t + \Delta t) -
	 * \sqrt{\frac{h_0}{2}}\mu(t)}{\Delta t} = 0\f$
	 *
	 * \f$ h_{\overline{y}_{-1}}(t) = \frac{ 2\sqrt{h_{-1}(t + \Delta t)} - 2\sqrt{h_{-1}t} }{\Delta
	 * t} =  \frac{2 \sqrt{h_{-1}}}{\Delta t} (\sqrt{t + \Delta t}\sqrt{t}) \f$
	 *
	 * \f$ h_{\overline{y}_{-2}}(t) = \frac{ \pi\sqrt{2h_{-2}}(t + \Delta t) - \pi\sqrt{2h_{-2}}t
	 * }{\Delta t} = \pi \sqrt{2h_{-2}}\f$
	 *
	 * The covariance matrix is (all \f$ h_{k_n} \f$ are uncorrelated with \f$ h_{k_m} \f$ when \f$
	 * n \neq m \f$ ):
	 *
	 * \f$ \begin{bmatrix}
	 * \sum_{n=0}^2 \int^t_0 [h_{x_n}(\rho)]^2 \delta \rho & \sum_{n=0}^2 \int^t_0
	 * h_{x_n}(\rho)h_{\overline{y}_n}(\rho) \delta \rho\\ \sum_{n=0}^2 \int^t_0
	 * h_{x_n}(\rho)h_{\overline{y}_n}(\rho)\delta\rho & \sum_{n=0}^2 \int^t_0
	 * [h_{\overline{y}_n}(\rho)]^2 \delta \rho \end{bmatrix} \f$
	 *
	 * We start with the variances:
	 *
	 * \f$ \sigma^2_{x_0} = \int^t_0 [\sqrt{\frac{h_0}{2}} \mu(\rho)]^2 \delta \rho = \frac{h_0}{2}
	 * \int^t_0 1 \delta\rho = \frac{h_0 t}{2} \f$
	 *
	 * \f$ \sigma^2_{x_{-1}} = \int^t_0 [2\sqrt{h_{-1}\rho}]^2\delta \rho = 4h_{-1}\int^t_0\rho
	 * \delta\rho = 2h_{-1}t^2 \f$
	 *
	 * \f$ \sigma^2_{x_{-2}} = \int^t_0 [\pi\sqrt{2h_{-2}}\rho]^2\delta \rho = 2\pi^2h_{-2}\int^t_0
	 * \rho^2\delta\rho = \frac{2\pi^2h_{-2}t^3}{3} \f$
	 *
	 * \f$ \sigma^2_{\overline{y}_0} = \int^t_0 [0]^2 \delta \rho = C \f$
	 *
	 * \f$ \sigma^2_{\overline{y}_{-1}} = \int^t_0 [\frac{2\sqrt{h_{-1}} }{\Delta t} (\sqrt{\rho +
	 * \Delta t} - \sqrt{\rho})]^2 \delta \rho =
	 * \frac{4h_{-1}}{\Delta t^2}\int^t_0[2\rho + \Delta t - 2\sqrt{\rho + \Delta t} \sqrt{\rho}]
	 * \delta
	 * \rho \\
	 * = \frac{4h_{-1}}{\Delta t^2}[t^2 + t\Delta t] - \frac{8h_{-1}}{\Delta t^2}\int^t_0\sqrt{\rho
	 * + \Delta t}\sqrt{\rho} \delta \rho \f$
	 *
	 * The remaining integral term is thorny; possible solutions include approximating
	 * the integral as a truncated series, numerical methods like Simpson's rule, or
	 * considering the steady state where \f$ t \gg \Delta t \f$ such that
	 * \f$ \sqrt{\rho + \Delta t} \approx \sqrt{\rho} \f$, which results in:
	 *
	 * \f$ \frac{4h_{-1}}{\Delta t^2}[t^2 + t\Delta t] - \frac{4h_{-1}}{\Delta t^2} t^2 =
	 * \frac{4h_{-1}}{\Delta t}t \f$
	 *
	 * \f$ \sigma^2_{\overline{y}_{-2}} = \int^t_0 [\pi\sqrt{2h_{-2}}]^2 \delta \rho = 2\pi^2h_{-2}t
	 * \f$
	 *
	 * And finally the cross terms:
	 *
	 * \f$ \sigma_{xy_0}\sigma(t) = \int^t_0 \sqrt{\frac{h_0}{2}}\mu(\rho) * 0 \delta\rho = C \f$
	 *
	 * \f$ \sigma_{xy_{-1}}(t) = \int^t_0 2\sqrt{h_{-1}\rho} \frac{2 \sqrt{h_{-1}} (\sqrt{\rho +
	 * \Delta
	 * t} - \sqrt{\rho}) }{\Delta t}\delta \rho \\
	 * = \frac{4h_{-1}}{\Delta t}\int^t_0\sqrt{\rho}(\sqrt{\rho+ \Delta t} - \sqrt{\rho}) \delta
	 * \rho \\
	 * = \frac{4h_{-1}}{\Delta t}\int^t_0\sqrt{\rho}\sqrt{\rho + \Delta t} - \rho \delta \rho \\
	 * = -\frac{2h_{-1}t^2}{\Delta t} + \frac{4h_{-1}}{\Delta t}\int^t_0\sqrt{\rho}\sqrt{\rho +
	 * \Delta t}\delta \rho \f$
	 *
	 * Clearly, this integral has the same issue as the previous one. Again using the steady-state
	 * approximation results in:
	 *
	 * \f$ -\frac{2h_{-1}t^2}{\Delta t} + \frac{4h_{-1}}{\Delta t}\int^t_0 \rho \delta \rho =
	 * -\frac{2h_{-1}t^2}{\Delta t} + \frac{2h_{-1}t^2}{\Delta t} = 0 \f$
	 *
	 * \f$ \sigma_{xy_{-2}}(t) = \int^t_0 \pi\sqrt{2h_{-2}}\rho * \pi\sqrt{2h_{-2}} \delta\rho =
	 * 2\pi^2h_{-2}\int^t_0\rho\delta\rho = \pi^2h_{-2}t^2 \f$
	 *
	 * Finally, we put the results together giving:
	 *
	 * \f$ \begin{bmatrix}
	 * \frac{h_0 t}{2} + 2h_{-1}t^2 + \frac{2}{3}\pi^2h_{-2}t^3 & \pi^2h_{-2}t^2 \\
	 * \pi^2h_{-2}t^2 & \frac{4h_{-1}}{\Delta t}t + 2\pi^2h_{-2}t
	 * \end{bmatrix} \f$
	 *
	 */
	QD,

	/**
	 * 'QD' as given in Brown and Hwang but with flicker \f$ h_{-1} \f$ terms removed.
	 *
	 * For two states, the matrix is as follows:
	 * \f$ \begin{bmatrix} \frac{h_0}{2}dt + \frac{2}{3}\pi^2h_{-2}dt^3 & \pi^2h_{-2}
	 * dt^2 \\ \pi^2h_{-2}dt^2 & \frac{h_0}{2dt} + \frac{8}{3}\pi^2h_{-2}dt
	 * \end{bmatrix} \f$
	 *
	 */
	QD1,

	/**
	 * Same as 'QD3' but with a flicker term added to the cross terms.
	 *
	 * For two states, the matrix is as follows:
	 * \f$ \begin{bmatrix} \frac{h_0}{2}dt +\frac{2}{3}\pi^2h_{-2}dt^3 & h_{-1}dt + \pi^2
	 * h_{-2}dt^2 \\ h_{-1}dt + \pi^2h_{-2}dt^2 & 2\pi^2h_{-2}dt \end{bmatrix}
	 * \f$
	 *
	 */
	QD2,

	/**
	 * Standard Qd model with Q defined as:
	 * \f$ \begin{bmatrix} S_f & 0 \\ 0 & S_g\end{bmatrix} \f$
	 *
	 * where \f$ S_f \f$ and \f$ S_g \f$​ are given in Brown and Hwang as \f$ S_f = \frac{h_0}{2}
	 * \f$ and \f$ S_g = 2\pi^2h_{-2} \f$
	 *
	 * For two states, the matrix is as follows:
	 * \f$ \begin{bmatrix} \frac{h_0}{2}dt + \frac{2}{3}\pi^2h_{-2}dt^3 & \pi^2h_{-2}
	 * dt^2
	 * \\ \pi^2h_{-2}dt^2 & 2\pi^2h_{-2}dt \end{bmatrix} \f$
	 *
	 */
	QD3
};

/**
 * Clock Error State Block with a variety of Q implementations in 2 or 3 states
 */
class ClockBiasesStateBlock : public StateBlock<> {
public:
	/**
	 * @param label The label uniquely identifying this particular set of states.
	 * @param model Allan variance coefficient set to use.
	 * @param choice The Q being used
	 * @param model_frequency_dot False if 2 states and true is 3 states needed
	 */
	ClockBiasesStateBlock(const std::string& label,
	                      ClockModel model,
	                      ClockChoice choice       = navtk::filtering::ClockChoice::QD,
	                      bool model_frequency_dot = false);

	/**
	 * Create a copy of the StateBlock with the same properties.
	 *
	 * @return A shared pointer to a copy of the StateBlock.
	 */
	not_null<std::shared_ptr<StateBlock<>>> clone() override;

	/**
	 * Generate a complete description of how to propagate this state block forward in time, given a
	 * current estimate to linearize about.
	 *
	 * @param time_from Time time we are propagating from.
	 * @param time_to The time to propagate to.
	 *
	 * @return The Dynamics which describe the non-linear propagation of this state block.
	 */
	DynamicsModel generate_dynamics(GenXhatPFunction,
	                                aspn_xtensor::TypeTimestamp time_from,
	                                aspn_xtensor::TypeTimestamp time_to) override;

protected:
	/** Allan variance coefficient set **/
	ClockModel model;

	/** Oft-used value in Qd calculation **/
	const double pi_sq = navutils::PI * navutils::PI;

	/** enum for clock toggling **/
	ClockChoice choice;
};

}  // namespace filtering
}  // namespace navtk
