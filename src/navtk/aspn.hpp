#pragma once

#include <vector>

#ifdef NAVTK_PYTHON_TENSOR
#	include <aspn23/xtensor_py/aspn_xtensor.hpp>
#else
#	include <aspn23/xtensor/aspn_xtensor.hpp>
#endif

#include <navtk/tensors.hpp>

typedef std::vector<std::shared_ptr<aspn_xtensor::AspnBase>> AspnBaseVector;

#ifndef NEED_DOXYGEN_EXHALE_WORKAROUND

#	include <spdlog/fmt/bundled/ostream.h>

// Define a custom formatter so fmt (via spdlog) can format ASPN types and enums.

template <>
struct fmt::formatter<aspn_xtensor::TypeTimestamp> {
	constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }

	template <typename FormatContext>
	constexpr auto format(const aspn_xtensor::TypeTimestamp& input, FormatContext& ctx) const {
		return fmt::format_to(ctx.out(), "{}", fmt::streamed(input));
	}
};


template <>
struct fmt::formatter<Aspn23MeasurementVelocityReferenceFrame> {
	constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }

	template <typename FormatContext>
	constexpr auto format(const Aspn23MeasurementVelocityReferenceFrame& input,
	                      FormatContext& ctx) const {
		switch (input) {
		case ASPN23_MEASUREMENT_VELOCITY_REFERENCE_FRAME_ECI:
			return fmt::format_to(ctx.out(), "ASPN23_MEASUREMENT_VELOCITY_REFERENCE_FRAME_ECI");
		case ASPN23_MEASUREMENT_VELOCITY_REFERENCE_FRAME_ECEF:
			return fmt::format_to(ctx.out(), "ASPN23_MEASUREMENT_VELOCITY_REFERENCE_FRAME_ECEF");
		case ASPN23_MEASUREMENT_VELOCITY_REFERENCE_FRAME_NED:
			return fmt::format_to(ctx.out(), "ASPN23_MEASUREMENT_VELOCITY_REFERENCE_FRAME_NED");
		case ASPN23_MEASUREMENT_VELOCITY_REFERENCE_FRAME_SENSOR:
			return fmt::format_to(ctx.out(), "ASPN23_MEASUREMENT_VELOCITY_REFERENCE_FRAME_SENSOR");
		}
		return fmt::format_to(ctx.out(), "Unknown enum value");
	}
};

template <>
struct fmt::formatter<Aspn23MeasurementAttitude3DReferenceFrame> {
	constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }

	template <typename FormatContext>
	constexpr auto format(const Aspn23MeasurementAttitude3DReferenceFrame& input,
	                      FormatContext& ctx) const {
		switch (input) {
		case ASPN23_MEASUREMENT_ATTITUDE_3D_REFERENCE_FRAME_ECI:
			return fmt::format_to(ctx.out(), "ASPN23_MEASUREMENT_ATTITUDE_3D_REFERENCE_FRAME_ECI");
		case ASPN23_MEASUREMENT_ATTITUDE_3D_REFERENCE_FRAME_ECEF:
			return fmt::format_to(ctx.out(), "ASPN23_MEASUREMENT_ATTITUDE_3D_REFERENCE_FRAME_ECEF");
		case ASPN23_MEASUREMENT_ATTITUDE_3D_REFERENCE_FRAME_NED:
			return fmt::format_to(ctx.out(), "ASPN23_MEASUREMENT_ATTITUDE_3D_REFERENCE_FRAME_NED");
		}
		return fmt::format_to(ctx.out(), "Unknown enum value");
	}
};
template <>
struct fmt::formatter<Aspn23MeasurementPositionVelocityAttitudeReferenceFrame> {
	constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }

	template <typename FormatContext>
	constexpr auto format(const Aspn23MeasurementPositionVelocityAttitudeReferenceFrame& input,
	                      FormatContext& ctx) const {
		switch (input) {
		case ASPN23_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_REFERENCE_FRAME_ECI:
			return fmt::format_to(
			    ctx.out(), "ASPN23_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_REFERENCE_FRAME_ECI");
		case ASPN23_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_REFERENCE_FRAME_GEODETIC:
			return fmt::format_to(
			    ctx.out(),
			    "ASPN23_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_REFERENCE_FRAME_GEODETIC");
		}
		return fmt::format_to(ctx.out(), "Unknown enum value");
	}
};
template <>
struct fmt::formatter<Aspn23MeasurementPositionReferenceFrame> {
	constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }

	template <typename FormatContext>
	constexpr auto format(const Aspn23MeasurementPositionReferenceFrame& input,
	                      FormatContext& ctx) const {
		switch (input) {
		case ASPN23_MEASUREMENT_POSITION_REFERENCE_FRAME_ECI:
			return fmt::format_to(ctx.out(), "ASPN23_MEASUREMENT_POSITION_REFERENCE_FRAME_ECI");
		case ASPN23_MEASUREMENT_POSITION_REFERENCE_FRAME_GEODETIC:
			return fmt::format_to(ctx.out(),
			                      "ASPN23_MEASUREMENT_POSITION_REFERENCE_FRAME_GEODETIC");
		}
		return fmt::format_to(ctx.out(), "Unknown enum value");
	}
};
template <>
struct fmt::formatter<Aspn23TypeSatnavTimeTimeReference> {
	constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }

	template <typename FormatContext>
	constexpr auto format(const Aspn23TypeSatnavTimeTimeReference& input,
	                      FormatContext& ctx) const {
		switch (input) {
		case ASPN23_TYPE_SATNAV_TIME_TIME_REFERENCE_TIME_GPS:
			return fmt::format_to(ctx.out(), "ASPN23_TYPE_SATNAV_TIME_TIME_REFERENCE_TIME_GPS");
		case ASPN23_TYPE_SATNAV_TIME_TIME_REFERENCE_TIME_GALILEO:
			return fmt::format_to(ctx.out(), "ASPN23_TYPE_SATNAV_TIME_TIME_REFERENCE_TIME_GALILEO");
		case ASPN23_TYPE_SATNAV_TIME_TIME_REFERENCE_TIME_BEIDOU:
			return fmt::format_to(ctx.out(), "ASPN23_TYPE_SATNAV_TIME_TIME_REFERENCE_TIME_BEIDOU");
		case ASPN23_TYPE_SATNAV_TIME_TIME_REFERENCE_TIME_GLONASS:
			return fmt::format_to(ctx.out(), "ASPN23_TYPE_SATNAV_TIME_TIME_REFERENCE_TIME_GLONASS");
		}
		return fmt::format_to(ctx.out(), "Unknown enum value");
	}
};
#endif
