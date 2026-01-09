#include <utility>
#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <navtk/magnetic/MagnetometerCalibration.hpp>
#include <navtk/magnetic/MagnetometerCalibrationCaruso2d.hpp>
#include <navtk/magnetic/MagnetometerCalibrationEllipse2d.hpp>
#include <navtk/magnetic/MagnetometerCalibrationScaleFactorBias.hpp>
#include <navtk/magnetic/magnetic.hpp>
#include <navtk/tensors.hpp>

#include "binding_helpers.hpp"

using namespace pybind11::literals;

namespace mag = navtk::magnetic;

using mag::MagnetometerCalibration;
using mag::MagnetometerCalibrationCaruso2d;
using mag::MagnetometerCalibrationEllipse2d;
using mag::MagnetometerCalibrationScaleFactorBias;
using navtk::Matrix;
using navtk::Vector;

template <class MagnetometerCalibrationBase = MagnetometerCalibration>
class PyMagnetometerCalibration : public MagnetometerCalibrationBase {
public:
	using MagnetometerCalibrationBase::MagnetometerCalibrationBase;

	void generate_calibration(const Matrix& mag) override {
		PYBIND11_OVERRIDE_PURE(void, MagnetometerCalibrationBase, generate_calibration, mag);
	}

	Vector apply_calibration(const Vector& mag) const override {
		PYBIND11_OVERRIDE_PURE(Vector, MagnetometerCalibrationBase, apply_calibration, mag);
	}
};

template <class MagnetometerCalibrationScaleFactorBiasBase = MagnetometerCalibrationScaleFactorBias>
class PyMagnetometerCalibrationScaleFactorBias
    : public PyMagnetometerCalibration<MagnetometerCalibrationScaleFactorBiasBase> {
public:
	using PyMagnetometerCalibration<
	    MagnetometerCalibrationScaleFactorBiasBase>::PyMagnetometerCalibration;

	Vector apply_calibration(const Vector& mag) const override {
		PYBIND11_OVERRIDE(
		    Vector, MagnetometerCalibrationScaleFactorBiasBase, apply_calibration, mag);
	}
};

template <class MagnetometerCalibrationCaruso2dBase = MagnetometerCalibrationCaruso2d>
class PyMagnetometerCalibrationCaruso2d
    : public PyMagnetometerCalibrationScaleFactorBias<MagnetometerCalibrationCaruso2dBase> {
public:
	using PyMagnetometerCalibrationScaleFactorBias<
	    MagnetometerCalibrationCaruso2dBase>::PyMagnetometerCalibrationScaleFactorBias;

	void generate_calibration(const Matrix& mag) override {
		PYBIND11_OVERRIDE(void, MagnetometerCalibrationCaruso2dBase, generate_calibration, mag);
	}
};

template <class MagnetometerCalibrationEllipse2dBase = MagnetometerCalibrationEllipse2d>
class PyMagnetometerCalibrationEllipse2d
    : public PyMagnetometerCalibrationScaleFactorBias<MagnetometerCalibrationEllipse2dBase> {
public:
	using PyMagnetometerCalibrationScaleFactorBias<
	    MagnetometerCalibrationEllipse2dBase>::PyMagnetometerCalibrationScaleFactorBias;

	void generate_calibration(const Matrix& mag) override {
		PYBIND11_OVERRIDE(void, MagnetometerCalibrationEllipse2dBase, generate_calibration, mag);
	}
};

void add_magnetic_functions(pybind11::module& m) {
	m.doc() = "General utilities for magnetic calibration and conversions";

	CLASS(MagnetometerCalibration, PyMagnetometerCalibration<>)
	CTOR_NODOC_DEFAULT
	METHOD(MagnetometerCalibration, generate_calibration, "mag"_a)
	METHOD(MagnetometerCalibration, apply_calibration, "mag"_a)
	CDOC(MagnetometerCalibration);

	CLASS(MagnetometerCalibrationScaleFactorBias,
	      MagnetometerCalibration,
	      PyMagnetometerCalibrationScaleFactorBias<>)
	CTOR_NODOC_DEFAULT
	METHOD(MagnetometerCalibrationScaleFactorBias, apply_calibration, "mag"_a)
	METHOD_VOID(MagnetometerCalibrationScaleFactorBias, get_calibration_params)
	METHOD(MagnetometerCalibrationScaleFactorBias, set_calibration_params, "sf"_a, "b"_a)
	CDOC(MagnetometerCalibrationScaleFactorBias);

	CLASS(MagnetometerCalibrationCaruso2d,
	      MagnetometerCalibrationScaleFactorBias,
	      PyMagnetometerCalibrationCaruso2d<>)
	CTOR_NODOC_DEFAULT
	METHOD(MagnetometerCalibrationCaruso2d, generate_calibration, "mag"_a)
	CDOC(MagnetometerCalibrationCaruso2d);

	CLASS(MagnetometerCalibrationEllipse2d,
	      MagnetometerCalibrationScaleFactorBias,
	      PyMagnetometerCalibrationEllipse2d<>)
	CTOR(MagnetometerCalibrationEllipse2d, bool, "calibrate_caruso"_a = false)
	METHOD(MagnetometerCalibrationEllipse2d, generate_calibration, "mag"_a)
	CDOC(MagnetometerCalibrationEllipse2d);

	NAMESPACE_FUNCTION(mag_to_heading, mag, "mag_x"_a, "mag_y"_a, "magnetic_declination"_a = 0.0)
}
