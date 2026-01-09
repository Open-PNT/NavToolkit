#pragma once

#include <navtk/aspn.hpp>
#include <navtk/filtering/containers/MeasurementBufferBase.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
namespace filtering {

/**
 * A buffer for storing and accessing 3-dimensional measurements.
 */
class MeasurementBuffer3d : public MeasurementBufferBase<Vector3, Matrix3> {
public:
	using MeasurementBufferBase::MeasurementBufferBase;
};

}  // namespace filtering
}  // namespace navtk
