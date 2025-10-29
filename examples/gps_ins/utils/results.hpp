#pragma once

#include <xtensor/generators/xrandom.hpp>

#include <navtk/aspn.hpp>
#include <navtk/filtering/fusion/StandardFusionEngine.hpp>
#include <navtk/filtering/processors/DirectMeasurementProcessor.hpp>
#include <navtk/filtering/stateblocks/Pinson15NedBlock.hpp>
#include <navtk/filtering/stateblocks/StateBlock.hpp>
#include <navtk/inertial/BufferedImu.hpp>
#include <navtk/inertial/Inertial.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/tensors.hpp>
#include <navtk/utils/interpolation.hpp>
#include <utils/exampleutils.hpp>

namespace navtk {
namespace exampleutils {

/** TruthPlotResults struct stores the truth
 * data from TruthResults type struct.
 * TruthPlotResults is used for storing plotting
 * data after the log file has been completely
 * processed.
 * navtk::Vector and navtk::Matrix type is necessary
 * for plotting.
 */
struct TruthPlotResults {
	Vector time;         //!< time tags for plotting truth data
	Vector lat;          //!< latitude
	Vector lon;          //!< longitude
	Vector alt;          //!< altitude
	Vector vel_n;        //!< velocity north (m/s)
	Vector vel_e;        //!< velocity east (m/s)
	Vector vel_d;        //!< velocity down (m/s)
	Vector att_r;        //!< roll attitude (rad)
	Vector att_p;        //!< pitch attitude (rad)
	Vector att_y;        //!< yaw attitude (rad)
	Matrix quat_matrix;  //!< quaternions for truth tilt calculations
};

/** TruthResults struct stores the truth
 * data from the log file data processing.
 * std::vector is used to store the data
 * as the final size of the data vectors
 * is unknown. std::vector can increase
 * in size as the data is processed. Also
 * allows the vectors already to be separately
 * prepared for interpolation.
 */
struct TruthResults {
	std::vector<double> time;   //!< time tags
	std::vector<double> lat;    //!< latitude
	std::vector<double> lon;    //!< longitude
	std::vector<double> alt;    //!< altitude
	std::vector<double> vel_n;  //!< velocity north (m/s)
	std::vector<double> vel_e;  //!< velocity east (m/s)
	std::vector<double> vel_d;  //!< velocity down (m/s)
	std::vector<double> att_r;  //!< roll attitude (rad)
	std::vector<double> att_p;  //!< roll pitch (rad)
	std::vector<double> att_y;  //!< roll yaw (rad)
};

/**
 * Indicates whether implementation is loosely coupled GPS/INS
 * or tightly coupled GPS/INS.
 */
enum class CouplingType {
	// loosely coupled INS/GPS
	LOOSE = 1,
	// tightly coupled INS/GPS
	TIGHT = 2
};

/**
 * Stores the filter update results,
 * and calculates the corrected latitude, longitude, and
 * altitude, RPY, and velocity NED.
 * @param type indicates whether implementation is loosely coupled or tightly coupled.
 * type = LOOSE for loosely coupled, type = TIGHT for tightly coupled.
 * @param engine Fusion engine
 * @param ins current INS position, velocity, RPY state
 * @param block_label vector, element 0 is string label for the "pinson15" state block,
 * if performing a tightly coupled filter, element 0 is clock bias state block string label
 * @param filter_data pinson15 state block results and clock bias state block
 * results when type is set to tightly coupled
 * results corresponding index for each vector:
 * 0-2: latitude, longitude, altitude
 * 3-5: velocity north, velocity east, velocity down (m/s)
 * 6-8: RPY, attitude (rad)
 * 9-11: NED position error (m)
 * 12-14: NED velocity error (m/s)
 * 15-17: NED tilt error (rad)
 * 18-20: Accelerometer X-Y-Z bias (m/s^2)
 * 21-23: Gyro X-Y-Z bias (rad/s)
 * When type is set to tightly coupled:
 * 24-25: clock bias and clock drift (m, m/s)
 * @param sigma_data standard deviation of the pinson15 state block results
 * and clock bias state block when type is set to tightly coupled
 * sigma_data corresponding index:
 * 0-2: NED position sigma
 * 3-5: NED velocity sigma
 * 6-8: NED tilt sigma
 * 9-11: Accelerometer X-Y-Z sigma
 * 12-14: Gyro X-Y-Z sigma
 * When type is set to tightly coupled:
 * 15-16: clock bias and clock drift sigma
 * @param ins_data INS object state results
 * ins_data corresponding index:
 * 0-2: INS latitude, longitude, altitude
 * 3-5: INS velocity NED
 * 6-8: INS RPY
 * @param C_s_to_b sensor to body DCM, defaults to identity matrix.
 * @param time_tags processed filter state time tags in std::vector form.
 */
void generate_output(CouplingType type,
                     filtering::StandardFusionEngine &engine,
                     const inertial::Inertial &ins,
                     const std::vector<std::string> &block_label,
                     std::vector<Vector> &filter_data,
                     std::vector<Vector> &sigma_data,
                     std::vector<Vector> &ins_data,
                     std::vector<double> &time_tags,
                     const Matrix3 C_s_to_b = eye(3));

/**
 * generate_output() overload that works with a BufferedImu. See docs on other version.
 */
void generate_output(CouplingType type,
                     filtering::StandardFusionEngine &engine,
                     const inertial::BufferedImu &ins,
                     const std::vector<std::string> &block_label,
                     std::vector<Vector> &filter_data,
                     std::vector<Vector> &sigma_data,
                     std::vector<Vector> &ins_data,
                     std::vector<double> &time_tags,
                     const Matrix3 C_s_to_b = eye(3));

/**
 * Collects the pseudorange bias state estimates from the after each SinglePointPseudorangeProcessor
 * measurement update iteration.
 * @param pr_bias_output stores the pseudorange bias estimates into a map structure. The PRN is key.
 * The second element of the map holds a pair of vectors. The first vector is time tags, the second
 * vector is pseudorange bias measurements.
 * @param block_labels The string labels for the pseudorange bias state blocks. The labels are
 * formatted `"pr_bias_sv_"` + `PRN_NUMBER`. For example, for PRN 32, the state block label would be
 * `"pr_bias_sv_32"`.
 * @param engine Fusion engine.
 */
void generate_pr_bias_output(
    std::map<int, std::pair<std::vector<double>, std::vector<double>>> &pr_bias_output,
    std::vector<std::string> block_labels,
    filtering::StandardFusionEngine &engine);

/**
 * Updates the auxData of the Pinson15 state block and
 * propagates the filter forward in time to match the
 * measurement time stamp.
 * @param ins current INS position, velocity, and RPY state
 * @param engine Fusion engine
 * @param measurement_time current measurement time
 * @param f_ned the specific force for current INS state
 * @param pinson15_label the pinson15 stateblock string label
 **/
void propagate_filter(const inertial::Inertial &ins,
                      filtering::StandardFusionEngine &engine,
                      const aspn_xtensor::TypeTimestamp &measurement_time,
                      const Vector3 f_ned,
                      const std::string pinson15_label);

/**
 * propagate_filter() overload that works with a BufferedImu. See docs on other version.
 *
 * @param ignore_imu_lag When true, if \p ins doesn't have a solution that
 * covers measurement_time, just use the latest available
 */
void propagate_filter(const inertial::BufferedImu &ins,
                      filtering::StandardFusionEngine &engine,
                      const aspn_xtensor::TypeTimestamp &measurement_time,
                      const std::string pinson15_label,
                      const bool ignore_imu_lag = false);

/**
 * Applies filter error states NED position, velocity, and tilt feedback
 * to the INS state (position, velocity, and RPY).
 * @param ins current INS position, velocity, and RPY state
 * @param out_var last generate_output() filter update position, velocity,
 * tilt results
 * @param feedback_time time tag for feedback errors
 **/
void apply_feedback_pinson15(inertial::Inertial &ins,
                             const std::vector<Vector> &out_var,
                             const aspn_xtensor::TypeTimestamp &feedback_time);

/**
 * Converts the std::vector<Vector> to
 * navtk::Matrix form. It is assumed that each
 * navtk::Vector object inside data
 * is of the same length.
 * @param data std::vector holding navtk::Vector objects
 * @return Matrix of the converted vector<Vector>
 **/
Matrix std_to_navtk(const std::vector<Vector> &data);

/**
 * Converts the std::vector<double> to
 * navtk::Vector to be used for plotting.
 * @param data std::vector holds double objects
 * @return Vector of converted vector objects
 */
Vector std_to_navtk(const std::vector<double> &data);

/** Calculates the truth tilt value using quaternions to avoid
 * discontinuities that occur in Euler angles. The function makes the following assumptions:
 * 1) For ins_results, the inertial roll, pitch, yaw are located at row 6, 7, 8 respectively,
 * of ins_results.
 * 2) For filter_results, the tilt NED are located at row 15, 16, 17 respectively, of
 * filter_results.
 * @param truth struct containing the truth message data (time of validity, LLH, NED velocity, and
 * attitude (RPY))
 * @param truth_interp truth data interpolated to the filter result time tags
 * @param filter_results filtered data results generated in generate_output()
 * @param ins_results INS position, velocity, and RPY state results
 * @param time_tags time tags of the filter, INS, and sigma results, and the truth_interp data
 *
 * @return pair, first element is Matrix of true tilts, second element is
 * Matrix of the delta tilts between the truth and filter tilt results
 */
std::pair<Matrix, Matrix> calculate_truth_tilts(const TruthResults &truth,
                                                TruthPlotResults &truth_interp,
                                                const Matrix &filter_results,
                                                const Matrix &ins_results,
                                                const std::vector<double> &time_tags);

/**
 * Calculates the position profile of the truth and filter results.
 * @param filter_results stores the state of the filter results
 * @param truth_interp truth data interpolated to the resultant filter time tags
 *
 * @return pair, first element is delta ned position from initial position for truth,
 * the second element is delta ned position from initial position for the filter results.
 */
std::pair<Matrix, Matrix> calculate_position_profile(const Matrix &filter_results,
                                                     const TruthPlotResults &truth_interp);

/**
 * Interpolates the truth position and velocity and converts results to navtk type for
 * plotting.
 * @param truth struct of truth message data: time valid, LLH, NED velocity, and attitude
 * (RPY)
 * @param interp_time vector of resultant filter time tags used for interpolating the truth data
 *
 * @return pair, first element is interpolated truth results converted to TruthPlotResults,
 * second element is the unused indexes from interp_time for interpolating truth.
 */
std::pair<std::vector<size_t>, TruthPlotResults> convert_truth(
    const TruthResults &truth, const std::vector<double> interp_time);
/**
 * Takes in the current truth message data of time valid, LLH,
 * NED velocity, and attitude and stores this data to a struct
 * of type TruthResults.
 * @param truth struct storing the truth message data of time valid, LLH, NED velocity, and attitude
 * (RPY)
 * @param data Vector containing truth time valid, LLH, NED velocity, and attitude (RPY)
 */
void store_truth(TruthResults &truth, const Vector &data);

/**
 * Get PVA and force/rate from inertial, potentially working around minor time issues.
 *
 * @param ins BufferedImu to extract solution from
 * @param t Desired time of solution
 * @param ignore_imu_lag When true, substitute the latest available solution for the solution at
 *  time \p t if no solution at \p t exists.
 *
 * @return Pair of best match of PVA nd force/rates given function arguments. Will be pair of
 *  nullptrs if \p ignore_imu_lag is false and no solution at \p t is available.
 */
std::pair<std::shared_ptr<aspn_xtensor::MeasurementPositionVelocityAttitude>,
          std::shared_ptr<aspn_xtensor::MeasurementImu>>
get_inertial_aux(const inertial::BufferedImu &ins,
                 const aspn_xtensor::TypeTimestamp &t,
                 const bool ignore_imu_lag);

}  // namespace exampleutils
}  // namespace navtk
