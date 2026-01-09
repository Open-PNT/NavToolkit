#include <navtk/filtering/processors/BiasedRangeProcessor.hpp>

#include <navtk/aspn.hpp>
#include <navtk/errors.hpp>
#include <navtk/factory.hpp>
#include <navtk/filtering/utils.hpp>
#include <navtk/navutils/navigation.hpp>

using aspn_xtensor::MeasurementRangeToPoint;
using navtk::navutils::delta_lat_to_north;
using navtk::navutils::delta_lon_to_east;
using std::pow;
using std::sqrt;

namespace navtk {
namespace filtering {

BiasedRangeProcessor::BiasedRangeProcessor(std::string label,
                                           const std::string &position_label,
                                           const std::string &bias_label)
    : MeasurementProcessor(std::move(label), std::vector<std::string>{position_label, bias_label}) {
}

BiasedRangeProcessor::BiasedRangeProcessor(std::string label,
                                           std::vector<std::string> state_block_labels)
    : MeasurementProcessor(std::move(label), std::move(state_block_labels)) {

	if (get_state_block_labels().size() != 2) {
		log_or_throw<std::invalid_argument>(
		    "BiasedRangeProcessor must be provided with exactly 2 state block labels.");
	}
}

std::shared_ptr<StandardMeasurementModel> BiasedRangeProcessor::generate_model(
    std::shared_ptr<aspn_xtensor::AspnBase> measurement, GenXhatPFunction gen_x_and_p_func) {

	std::shared_ptr<MeasurementRangeToPoint> data =
	    std::dynamic_pointer_cast<MeasurementRangeToPoint>(measurement);
	if (data == nullptr) {
		log_or_throw<std::invalid_argument>(
		    "Measurement is not of correct type (MeasurementRangeToPoint). Unable to perform "
		    "update.");
		return nullptr;
	}

	auto h = [data = data](const Vector &x) {
		auto remote_point = data->get_remote_point();
		auto latitude     = remote_point.get_position1();
		auto longitude    = remote_point.get_position2();
		auto altitude     = remote_point.get_position3();
		if (isnan(latitude) || isnan(longitude) || isnan(altitude))
			log_or_throw(
			    "Some components of remote point's position in MeasurementRangeToPoint are null "
			    "(NaN)");
		return Vector{sqrt(pow(delta_lat_to_north(x[0] - latitude, latitude, altitude), 2) +
		                   pow(delta_lon_to_east(x[1] - longitude, latitude, altitude), 2) +
		                   pow(x[2] - altitude, 2)) +
		              x[3]};
	};

	// Ignoring feature location variance
	Vector z = {data->get_obs()};
	Matrix R = {{data->get_variance()}};

	auto xhat_p = gen_x_and_p_func(get_state_block_labels());
	if (xhat_p == nullptr) {
		return nullptr;
	}
	Matrix H = calc_numerical_jacobian(h, xhat_p->estimate);
	return std::make_shared<StandardMeasurementModel>(z, h, H, R);
}

not_null<std::shared_ptr<MeasurementProcessor<>>> BiasedRangeProcessor::clone() {
	return std::make_shared<BiasedRangeProcessor>(*this);
}

}  // namespace filtering
}  // namespace navtk
