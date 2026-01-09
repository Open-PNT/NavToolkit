#include <gps_ins/utils/plot.hpp>
#include <navtk/navutils/navigation.hpp>

using aspn_xtensor::TypeTimestamp;
using std::get;
using std::pair;
using std::vector;
using xt::all;
using xt::view;


namespace navtk {
namespace exampleutils {

void plot_acc_gyro_err(const Matrix &filter_results,
                       const Matrix &sigma_results,
                       const Vector &time_tags) {
	// Plot the accelerometer/gyroscope bias estimates
	vector<int> bias_index  = {18, 21};  // acc, gyro
	vector<int> sigma_index = {9, 12};
	int num_plots           = 2;
	// Import plotting tools from Python
	// pybind11::scoped_interpreter guard{true, argc, argv};
	auto pyplot         = pybind11::module::import("matplotlib.pyplot");
	auto figure         = pyplot.attr("figure");
	auto plot           = pyplot.attr("plot");
	auto subplot        = pyplot.attr("subplot");
	auto xlabel         = pyplot.attr("xlabel");
	auto ylabel         = pyplot.attr("ylabel");
	auto suptitle       = pyplot.attr("suptitle");
	auto legend         = pyplot.attr("legend");
	auto show           = pyplot.attr("show");
	auto grid           = pyplot.attr("grid");
	auto time_tags_plot = eval(view(time_tags, all()) - time_tags[0]);
	// Plot results
	vector<std::string> title_labels(
	    {"Accelerometer Bias Error State Estimates", "Gyro Bias Error State Estimates"});

	vector<std::string> y_labels({"Bias in (m/s^2)", "Bias in (rad/s)"});

	for (int k = 0; k < num_plots; k++) {
		figure();
		suptitle(title_labels[k]);

		subplot(311);
		plot(time_tags_plot, eval(view(filter_results, bias_index[k], all())), "k");
		plot(time_tags_plot,
		     eval(view(filter_results, bias_index[k], all()) +
		          view(sigma_results, sigma_index[k], all())),
		     "-r");
		plot(time_tags_plot,
		     eval(view(filter_results, bias_index[k], all()) -
		          view(sigma_results, sigma_index[k], all())),
		     "-r");
		grid();
		auto legend_x = pybind11::list(2);
		legend_x[0]   = "X";
		legend_x[1]   = "estimate +/- X Sigma";
		legend(legend_x);
		ylabel(y_labels[k]);

		subplot(312);
		plot(time_tags_plot, eval(view(filter_results, bias_index[k] + 1, all())), "b");
		plot(time_tags_plot,
		     eval(view(filter_results, bias_index[k] + 1, all()) +
		          view(sigma_results, sigma_index[k] + 1, all())),
		     "-y");
		plot(time_tags_plot,
		     eval(view(filter_results, bias_index[k] + 1, all()) -
		          view(sigma_results, sigma_index[k] + 1, all())),
		     "-y");
		grid();
		auto legend_y = pybind11::list(2);
		legend_y[0]   = "Y";
		legend_y[1]   = "estimate +/- Y Sigma";
		legend(legend_y);
		ylabel(y_labels[k]);

		subplot(313);
		plot(time_tags_plot, eval(view(filter_results, bias_index[k] + 2, all())), "g");
		plot(time_tags_plot,
		     eval(view(filter_results, bias_index[k] + 2, all()) +
		          view(sigma_results, sigma_index[k] + 2, all())),
		     "-m");
		plot(time_tags_plot,
		     eval(view(filter_results, bias_index[k] + 2, all()) -
		          view(sigma_results, sigma_index[k] + 2, all())),
		     "-m");
		xlabel("Time (s)");
		ylabel(y_labels[k]);

		auto legend_z = pybind11::list(2);
		legend_z[0]   = "Z";
		legend_z[1]   = "estimate +/- Z Sigma";
		legend(legend_z);
		grid();
	}
}

void plot_pos_vel_profile(const Matrix &filter_results,
                          const Matrix &d_ned_true,
                          const Matrix &d_ned_filt,
                          const TruthPlotResults &truth_interp,
                          const Vector &time_tags) {
	int vned_index      = 3;
	auto pyplot         = pybind11::module::import("matplotlib.pyplot");
	auto figure         = pyplot.attr("figure");
	auto plot           = pyplot.attr("plot");
	auto subplot        = pyplot.attr("subplot");
	auto xlabel         = pyplot.attr("xlabel");
	auto ylabel         = pyplot.attr("ylabel");
	auto suptitle       = pyplot.attr("suptitle");
	auto legend         = pyplot.attr("legend");
	auto grid           = pyplot.attr("grid");
	auto time_tags_plot = eval(view(time_tags, all()) - time_tags[0]);
	// Position Profile
	figure();
	suptitle("Position Profile (Relative to Starting Point)");

	subplot(311);
	plot(time_tags_plot, eval(view(d_ned_true, 0, all())), "k");
	plot(time_tags_plot, eval(view(d_ned_filt, 0, all())), "y-");
	ylabel("Position (meters)");
	grid();
	auto legend_n = pybind11::list(2);
	legend_n[0]   = "dNorth (true)";
	legend_n[1]   = "dNorth (estimated)";
	legend(legend_n);

	subplot(312);
	plot(time_tags_plot, eval(view(d_ned_true, 1, all())), "b");
	plot(time_tags_plot, eval(view(d_ned_filt, 1, all())), "c-");
	ylabel("Position (meters)");
	grid();
	auto legend_e = pybind11::list(2);
	legend_e[0]   = "dEast (true)";
	legend_e[1]   = "dEast (estimated)";
	legend(legend_e);

	subplot(313);
	plot(time_tags_plot, eval(view(d_ned_filt, 2, all())), "m-");
	plot(time_tags_plot, eval(view(d_ned_true, 2, all())), "g");
	ylabel("Position (meters)");
	grid();
	auto legend_d = pybind11::list(2);
	legend_d[0]   = "dAlt (true)";
	legend_d[1]   = "dAlt (estimated)";
	legend(legend_d);

	xlabel("Time (s)");

	// Velocity profile
	figure();
	suptitle("Velocity Profile");

	subplot(311);
	plot(time_tags_plot, truth_interp.vel_n, "k");
	plot(time_tags_plot, eval(view(filter_results, vned_index, all())), "y-");
	ylabel("Velocity (m/s)");
	grid();
	legend_n    = pybind11::list(2);
	legend_n[0] = "N True";
	legend_n[1] = "N Estimated";
	legend(legend_n);

	subplot(312);
	plot(time_tags_plot, truth_interp.vel_e, "b");
	plot(time_tags_plot, eval(view(filter_results, (++vned_index), all())), "c-");
	ylabel("Velocity (m/s)");
	grid();
	legend_e    = pybind11::list(2);
	legend_e[0] = "E True";
	legend_e[1] = "E Estimated";
	legend(legend_e);

	subplot(313);
	plot(time_tags_plot, truth_interp.vel_d, "g");
	plot(time_tags_plot, eval(view(filter_results, (++vned_index), all())), "m-");
	ylabel("Velocity (m/s)");
	grid();
	legend_d    = pybind11::list(2);
	legend_d[0] = "D True";
	legend_d[1] = "D Estimated";
	legend(legend_d);

	xlabel("Time (s)");
}

void plot_trajectory(const Matrix &d_ned_true, const Matrix &d_ned_filt) {
	auto pyplot = pybind11::module::import("matplotlib.pyplot");
	auto figure = pyplot.attr("figure");
	auto plot   = pyplot.attr("plot");
	auto xlabel = pyplot.attr("xlabel");
	auto ylabel = pyplot.attr("ylabel");
	auto title  = pyplot.attr("title");
	auto legend = pyplot.attr("legend");
	auto grid   = pyplot.attr("grid");
	// Plot the trajectory, with origin at starting location
	figure();
	plot(eval(view(d_ned_true, 1, all())), eval(view(d_ned_true, 0, all())), "k");
	plot(eval(view(d_ned_filt, 1, all())), eval(view(d_ned_filt, 0, all())), "m-");
	xlabel("Easting (m)");
	ylabel("Northing (m)");
	title("Trajectory");
	auto legend_labels = pybind11::list(2);
	legend_labels[0]   = "True";
	legend_labels[1]   = "Filter";
	legend(legend_labels);
	grid();
}

void plot_delta_results(const Matrix &filter_results,
                        const Matrix &sigma_results,
                        const TruthPlotResults &truth_interp,
                        const Matrix &delta_tilts,
                        const Vector &time_tags) {
	int lat_index  = 0;
	int vned_index = 3;
	int truth_size = time_tags.size();
	// Import plotting tools from Python
	auto pyplot         = pybind11::module::import("matplotlib.pyplot");
	auto figure         = pyplot.attr("figure");
	auto subplot        = pyplot.attr("subplot");
	auto plot           = pyplot.attr("plot");
	auto xlabel         = pyplot.attr("xlabel");
	auto ylabel         = pyplot.attr("ylabel");
	auto title          = pyplot.attr("title");
	auto suptitle       = pyplot.attr("suptitle");
	auto legend         = pyplot.attr("legend");
	auto grid           = pyplot.attr("grid");
	auto time_tags_plot = eval(view(time_tags, all()) - time_tags[0]);
	// Plot results
	vector<std::string> state_names({"North Pos Error",
	                                 "East Pos Error",
	                                 "Down Pos Error",
	                                 "North Vel Error",
	                                 "East Vel Error",
	                                 "Down Vel Error",
	                                 "North Tilt Error",
	                                 "East Tilt Error",
	                                 "Down Tilt Error"});
	vector<std::string> figure_titles({"Position Error", "Velocity Error", "Tilt Error"});

	vector<std::string> state_y_labels(
	    {"meters", "meters", "meters", "m/s", "m/s", "m/s", "radians", "radians", "radians"});
	// get the conversion factors
	auto lat_factor = navutils::delta_lat_to_north(1, truth_interp.lat[0], truth_interp.alt[0]);
	auto lon_factor = navutils::delta_lon_to_east(1, truth_interp.lat[0], truth_interp.alt[0]);
	// calculate the delta between truth and filter estimate
	// find matching time tags
	Matrix delta_pos_vel = zeros(6, truth_size);
	view(delta_pos_vel, 0, all()) =
	    lat_factor * (view(filter_results, lat_index, all()) - truth_interp.lat);
	view(delta_pos_vel, 1, all()) =
	    lon_factor * (view(filter_results, lat_index + 1, all()) - truth_interp.lon);
	view(delta_pos_vel, 2, all()) =
	    -1 * (view(filter_results, lat_index + 2, all()) - truth_interp.alt);
	view(delta_pos_vel, 3, all()) = view(filter_results, vned_index, all()) - truth_interp.vel_n;
	view(delta_pos_vel, 4, all()) =
	    view(filter_results, vned_index + 1, all()) - truth_interp.vel_e;
	view(delta_pos_vel, 5, all()) =
	    view(filter_results, vned_index + 2, all()) - truth_interp.vel_d;

	// Plot
	// index begins with position error 0-2, then velocity error 3-5 from delta_results,
	// then 0-2 in delta_tilts
	vector<int> delta = {0, 0};
	int delta_index   = delta[0];
	// index standard deviation from sigma_results, position index (0-2), velocity index (3-5)
	vector<int> stand_dev = {0, 6};
	int sd_index          = stand_dev[0];
	Matrix delta_data     = delta_pos_vel;
	for (int state = 0; state < 9; state = state + 3) {
		figure();
		suptitle(figure_titles[int(state / 3)]);
		for (int p_idx = state; p_idx < state + 3; p_idx++) {
			subplot(3, 1, 1 + p_idx % 3);

			plot(time_tags_plot, eval(view(delta_data, delta_index, all())), "k");
			plot(time_tags_plot, eval(view(sigma_results, sd_index, all())), "b");
			plot(time_tags_plot, eval(-1 * view(sigma_results, sd_index, all())), "b");
			xlabel("Time (s)");
			ylabel(state_y_labels[p_idx]);
			auto legend_labels = pybind11::list(2);
			legend_labels[0]   = state_names[p_idx];
			legend_labels[1]   = "Filter-Computed 1-Sigma";
			legend(legend_labels);
			grid();
			if (p_idx == 5) {
				// sd_index   = stand_dev[1];
				delta_index = delta[1];
				delta_data  = delta_tilts;
			} else {
				delta_index++;
			}
			sd_index++;
		}
	}
}

void plot_tilt(const Matrix &filter_results,
               const Matrix &true_tilts,
               const Matrix &sigma_results,
               const Vector &time_tags) {
	// Plot the tilt for truth and the tilt computed by the filter
	int sd_index        = 6;
	int tilt_index      = 15;
	int true_tilt_index = 0;
	// Import plotting tools from Python
	auto pyplot         = pybind11::module::import("matplotlib.pyplot");
	auto figure         = pyplot.attr("figure");
	auto plot           = pyplot.attr("plot");
	auto subplot        = pyplot.attr("subplot");
	auto xlabel         = pyplot.attr("xlabel");
	auto ylabel         = pyplot.attr("ylabel");
	auto title          = pyplot.attr("title");
	auto suptitle       = pyplot.attr("suptitle");
	auto legend         = pyplot.attr("legend");
	auto grid           = pyplot.attr("grid");
	auto time_tags_plot = eval(view(time_tags, all()) - time_tags[0]);
	vector<std::string> state_names({"N Tilt estimate", "E Tilt estimate", "D Tilt estimate"});
	vector<std::string> state_y_labels({"radians", "radians", "radians"});
	figure();
	suptitle("Tilt Estimate");
	for (int p_idx = 0; p_idx < 3; p_idx++) {
		subplot(3, 1, p_idx + 1);
		plot(time_tags_plot, eval(view(filter_results, tilt_index, all())), "k");
		plot(time_tags_plot, eval(view(true_tilts, true_tilt_index, all())), "b");
		plot(time_tags_plot, eval(-1 * view(sigma_results, sd_index, all())), "-m");
		plot(time_tags_plot, eval(view(sigma_results, sd_index, all())), "-m");
		xlabel("Time (s)");
		ylabel(state_y_labels[p_idx]);
		auto legend_labels = pybind11::list(3);
		legend_labels[0]   = state_names[p_idx];
		legend_labels[1]   = "True Tilt";
		legend_labels[2]   = "Filter-Computed 1-Sigma";
		legend(legend_labels);
		grid();
		sd_index++;
		tilt_index++;
		true_tilt_index++;
	}
}

void plot_rpy(const TruthResults &truth, const Matrix &filter_results, const Vector &time_tags) {
	// convert truth to plot-able Vector type.
	int truth_size = truth.time.size();
	// Roll index in filter_results
	auto filter_r_index   = 6;
	Matrix plot_truth_rpy = zeros(4, truth_size);
	for (decltype(truth_size) k = 0; k < truth_size; k++) {
		plot_truth_rpy(0, k) = truth.time[k] - truth.time[0];
		plot_truth_rpy(1, k) = truth.att_r[k];
		plot_truth_rpy(2, k) = truth.att_p[k];
		plot_truth_rpy(3, k) = truth.att_y[k];
	}
	// plot the truth and filter roll
	// Import plotting tools from Python
	auto pyplot         = pybind11::module::import("matplotlib.pyplot");
	auto figure         = pyplot.attr("figure");
	auto plot           = pyplot.attr("plot");
	auto subplot        = pyplot.attr("subplot");
	auto xlabel         = pyplot.attr("xlabel");
	auto ylabel         = pyplot.attr("ylabel");
	auto suptitle       = pyplot.attr("suptitle");
	auto legend         = pyplot.attr("legend");
	auto grid           = pyplot.attr("grid");
	auto time_tags_plot = eval(view(time_tags, all()) - time_tags[0]);
	vector<std::string> state_names({"Roll", "Pitch", "Yaw"});
	vector<std::string> state_y_labels({"radians", "radians", "radians"});
	figure();
	suptitle("Attitude Profile");
	for (int p_idx = 0; p_idx < 3; p_idx++) {
		subplot(3, 1, p_idx + 1);
		// plot the truth result
		plot(eval(view(plot_truth_rpy, 0, all())),
		     eval(view(plot_truth_rpy, 1 + p_idx, all())),
		     "b");
		// plot the filter result
		plot(time_tags_plot, eval(view(filter_results, filter_r_index + p_idx, all())), "k");
		xlabel("Time (s)");
		ylabel(state_y_labels[p_idx]);
		auto legend_labels = pybind11::list(2);
		legend_labels[0]   = state_names[p_idx] + " Truth";
		legend_labels[1]   = state_names[p_idx] + " Estimate";
		legend(legend_labels);
		grid();
	}
}

void plot_clock_error(const Matrix &filter_results, const Vector &time_tags) {
	auto pyplot         = pybind11::module::import("matplotlib.pyplot");
	auto figure         = pyplot.attr("figure");
	auto plot           = pyplot.attr("plot");
	auto subplot        = pyplot.attr("subplot");
	auto xlabel         = pyplot.attr("xlabel");
	auto ylabel         = pyplot.attr("ylabel");
	auto suptitle       = pyplot.attr("suptitle");
	auto legend         = pyplot.attr("legend");
	auto grid           = pyplot.attr("grid");
	auto time_tags_plot = eval(view(time_tags, all()) - time_tags[0]);

	figure();
	suptitle("Clock Errors");
	// Clock bias
	subplot(211);
	plot(time_tags, eval(view(filter_results, 24, all()) * 299792458), "k");
	xlabel("Time (s)");
	ylabel("meters error");
	auto legend_label_bias = pybind11::list(1);
	legend_label_bias[0]   = "Clock Bias/Clock Phase Offset";
	legend(legend_label_bias);
	grid();

	// Clock drift
	subplot(212);
	plot(time_tags, eval(view(filter_results, 25, all()) * 299792458), "k");
	xlabel("Time (s)");
	ylabel("m/s error");
	auto legend_label_drift = pybind11::list(1);
	legend_label_drift[0]   = "Clock Drift/Clock Frequency Offset";
	legend(legend_label_drift);
	grid();
}

void plot_pr_bias(const std::map<int, pair<vector<double>, vector<double>>> &pr_bias_results) {
	auto pyplot        = pybind11::module::import("matplotlib.pyplot");
	auto figure        = pyplot.attr("figure");
	auto plot          = pyplot.attr("plot");
	auto xlabel        = pyplot.attr("xlabel");
	auto ylabel        = pyplot.attr("ylabel");
	auto title         = pyplot.attr("title");
	auto legend        = pyplot.attr("legend");
	auto grid          = pyplot.attr("grid");
	size_t num_prns    = pr_bias_results.size();
	auto map_it        = pr_bias_results.begin();
	auto legend_labels = pybind11::list(num_prns);
	figure();
	for (size_t k = 0; k < num_prns; k++) {
		Vector time      = std_to_navtk((map_it->second).first);
		Vector time_tags = eval(view(time, all()) - time[0]);
		Vector pr_bias   = std_to_navtk((map_it->second).second);
		plot(time_tags, pr_bias);
		legend_labels[k] = map_it->first;
		map_it++;
	}
	xlabel("Time (s)");
	ylabel("m error");
	title("Pseudorange Bias");
	legend(legend_labels);
	grid();
}

}  // namespace exampleutils
}  // namespace navtk
