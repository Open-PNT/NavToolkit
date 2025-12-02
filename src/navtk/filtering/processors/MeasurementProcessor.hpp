#pragma once

#include <memory>
#include <string>
#include <vector>

#include <spdlog/spdlog.h>

#include <navtk/aspn.hpp>
#include <navtk/filtering/GenXhatPFunction.hpp>
#include <navtk/filtering/containers/EstimateWithCovariance.hpp>
#include <navtk/filtering/containers/SampledMeasurementModel.hpp>
#include <navtk/filtering/containers/StandardMeasurementModel.hpp>
#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace filtering {

/**
 * A description of the way a raw measurement from a sensor relates to a set of states.
 *
 * MeasurementProcessors are used by a fusion engine (such as
 * navtk::filtering::StandardFusionEngine), along with one or several navtk::filtering::StateBlock
 * instances and a navtk::filtering::FusionStrategy. StateBlocks allow the fusion engine to predict
 * how states will change over time, via dynamics equations. MeasurementProcessors give the fusion
 * engine the equations needed to relate measurements to the states. The fusion strategy is
 * responsible for providing the fusion engine with the general algorithms used to combine the
 * information from StateBlocks and MeasurementProcessors.
 *
 * MeasurementProcessors process raw sensor data into estimated states suitable for a filter to use.
 * Each sensor should define a MeasurementProcessor which it gives to the fusion engine for later
 * use. When a measurement is received by the fusion engine from that sensor, it will find the
 * corresponding MeasurementProcessor and call MeasurementProcessor#generate_model to process the
 * measurement. The output of this function call (z,h,H,R) will then be used by the fusion engine to
 * call the the StandardFusionEngine::update method and update the filter estimate/error covariance.
 *
 * @tparam z The measurement vector
 * @tparam h The function mapping states to measurements (`h`) that accepts the estimate `xhat` as
 * an argument and returns the predicted measurement `zhat`
 * @tparam H the matrix providing approximate mapping of states to measurements; the Jacobian matrix
 * of `h`
 * @tparam R the corresponding measurement covariance matrix
 */
template <typename ModelType = StandardMeasurementModel>
class MeasurementProcessor {
public:
	/**
	 * A subclass of `std::runtime_error` that can be thrown to differentiate between manual throws
	 * and unexpected runtime errors.
	 */
	class GenerateModelError : public std::runtime_error {
	public:
		/**
		 * Creates an instance of GenerateModelError.
		 *
		 * @param what Text describing the error.
		 */
		GenerateModelError(const std::string& what) : std::runtime_error(what) {}
	};

public:
	/**
	 * Disabled default constructor.
	 */
	MeasurementProcessor() = delete;

	/**
	 * Constructs a MeasurementProcessor that updates the states for a single StateBlock.
	 *
	 * @param label The unique identifier for this MeasurementProcessor.
	 * @param state_block_label The value to store in #state_block_labels.
	 */
	MeasurementProcessor(std::string label, std::string state_block_label)
	    : state_block_labels(1, state_block_label), label(std::move(label)) {}

	/**
	 * Constructs a MeasurementProcessor that updates the states for one or more StateBlocks.
	 *
	 * @param label The unique identifier for this MeasurementProcessor.
	 * @param state_block_labels The value to store in #state_block_labels.
	 */
	MeasurementProcessor(std::string label, std::vector<std::string> state_block_labels)
	    : state_block_labels(std::move(state_block_labels)), label(std::move(label)) {}

	/**
	 * Custom copy constructor which creates a deep copy.
	 *
	 * @param processor The MeasurementProcessor to copy.
	 */
	MeasurementProcessor(const MeasurementProcessor& processor) {
		label              = processor.label;
		state_block_labels = processor.state_block_labels;
	}

	virtual ~MeasurementProcessor() = default;

	/**
	 * Generates a shared pointer of `ModelType`. `ModelType` contains the parameters
	 * required for a filter update:
	 *  - the measurement vector (`z`)
	 *  - the function mapping states to measurements (`h`) that accepts `xhat` as an argument and
	 *    returns `zhat`
	 *  - the matrix providing approximate mapping of states to measurements (`H`); the Jacobian
	 *    matrix of `h`
	 *  - the corresponding measurement covariance matrix (`R`)
	 *
	 * @param measurement The raw measurement received from the sensor.
	 * @param gen_x_and_p_func A function that will generate `xhat` (a vector of estimates for the
	 * concatenated states specified in #state_block_labels) and `P` (the covariance matrix
	 * corresponding to xhat) for the listed state block labels when called.
	 * @return A shared pointer of `ModelType` with the parameters required for a filter update. In
	 * situations where application of usability metrics results in insufficient data, or when an
	 * error occurs during generation, `nullptr` will be returned instead. In general, the
	 * `ModelType` contains:
	 *  - z: An Mx1 vector of measurement values
	 *  - h: A function which maps states to measurements (as zhat=h(xhat)) accepting `xhat` as
	 *       an argument and producing an Mx1 vector
	 *  - H: an MxN matrix relating `xhat` to z (as zhat = H*xhat, approximately); the Jacobian of h
	 *  - R: MxM covariance matrix for measurement vector z
	 */
	virtual std::shared_ptr<ModelType> generate_model(
	    std::shared_ptr<aspn_xtensor::AspnBase> measurement, GenXhatPFunction gen_x_and_p_func) = 0;

	/**
	 * Receive and use arbitrary aux data sent from the sensor. This method will be called by the
	 * fusion engine when the fusion engine receives aux data from a
	 * StandardFusionEngine::give_measurement_processor_aux_data call. The default implementation
	 * logs a warning that the measurement processor does not use the given type of aux data.
	 *
	 * If your MeasurementProcessor needs to receive aux data, this function needs to be
	 * implemented to be able to ingest your desired type(s) of aspn_xtensor::AspnBase from the
	 * incoming vector. Your override should use `std::dynamic_pointer_cast` or similar to check the
	 * message type of each message in the incoming AspnBaseVector against the specific
	 * aspn_xtensor::AspnBase subclasses you can support.
	 */
	virtual void receive_aux_data(const AspnBaseVector&) {
		spdlog::warn("The measurement processor labeled {} does not utilize this type of aux data.",
		             label);
	};

	/**
	 * Create a copy of the MeasurementProcessor with the same properties.
	 *
	 * @return A shared pointer to a copy of the MeasurementProcessor. A unique pointer is not used
	 * here because of an issue with the Python bindings:
	 * https://github.com/pybind/pybind11/issues/673
	 */
	virtual not_null<std::shared_ptr<MeasurementProcessor<ModelType>>> clone() = 0;

	/**
	 * @return The unique identifier for this MeasurementProcessor.
	 */
	std::string get_label() const { return label; }

	/**
	 * @return The list of state blocks that are associated with measurements passed into this
	 * processor.
	 */
	virtual std::vector<std::string> get_state_block_labels() const { return state_block_labels; }

protected:
	/**
	 * A list of state blocks that are associated with measurements passed into this processor. This
	 * list specifies which state estimates will be used within #generate_model and what the return
	 * of #generate_model must be with respect to (i.e. the `h(x)` will be this list of state
	 * blocks).
	 */
	std::vector<std::string> state_block_labels;

private:
	/// A unique identifier for this MeasurementProcessor.
	std::string label;
};

/**
 * Defines StandardMeasurementProcessor to be a MeasurementProcessor which uses a
 * StandardMeasurementModel.
 */
typedef MeasurementProcessor<StandardMeasurementModel> StandardMeasurementProcessor;

/**
 * Defines SampledMeasurementProcessor to be a MeasurementProcessor which uses a
 * SampledMeasurementModel.
 */
typedef MeasurementProcessor<SampledMeasurementModel> SampledMeasurementProcessor;

}  // namespace filtering
}  // namespace navtk
