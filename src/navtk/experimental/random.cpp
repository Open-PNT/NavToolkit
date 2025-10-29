#include <navtk/experimental/random.hpp>

#include <navtk/factory.hpp>
#include <navtk/filtering/utils.hpp>
#include <navtk/inspect.hpp>
#include <navtk/linear_algebra.hpp>
#include <navtk/transform.hpp>
#include <xtensor/core/xmath.hpp>
#include <xtensor/generators/xbuilder.hpp>
#include <xtensor/misc/xpad.hpp>
#include <xtensor/views/xindex_view.hpp>
#include <xtensor/views/xstrided_view.hpp>
#include <xtensor/views/xview.hpp>


using navtk::chol;
using navtk::not_null;
using navtk::num_rows;
using navtk::filtering::calc_mean_cov;
using navtk::filtering::EstimateWithCovariance;
using std::make_shared;
using std::shared_ptr;
using xt::all;
using xt::newaxis;
using xt::tile;
using xt::transpose;

namespace {

static not_null<shared_ptr<navtk::experimental::RandomNumberGenerator>> GLOBAL_RNG =
    std::make_shared<navtk::experimental::LocalEngineWrapper>();

}  // namespace


namespace navtk {
namespace experimental {

#define PCG_DEFAULT_INIT_STATE_HIGH_BITS 0x853c49e6748fea9ULL
#define PCG_DEFAULT_INIT_STATE_LOW_BITS 0x281830dbdfe90a3ULL

static uint64_t PCG_STATE_HIGH_BITS = PCG_DEFAULT_INIT_STATE_HIGH_BITS;
static uint64_t PCG_STATE_LOW_BITS  = PCG_DEFAULT_INIT_STATE_LOW_BITS;
static uint64_t PCG_MULTIPLIER      = 6364136223846793005ULL;
static uint64_t PCG_INCREMENT       = 1442695040888963407ULL;
static double DOUBLE_SCALE_TINY     = pow(2, -64);

/**
 * PCG Random Number Generation for C.
 *
 * Copyright 2014 Melissa O'Neill <oneill@pcg-random.org>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * For additional information about the PCG random number generation scheme,
 * including its license and other licensing options, visit
 *
 *     http://www.pcg-random.org
 */

double rand_local() { return as_double(pcg_random_r()); }

/**
 * Transform the random state value to a new 64-bit unsigned integer.
 *
 * @param state State value for transformation.
 *
 * @return A transformed 64-bit unsigned int
 */
inline uint64_t pcg_state_transform(uint64_t state) {
	uint64_t xorshifted = ((state >> 18u) ^ state) >> 27u;
	uint64_t rot        = state >> 59u;
	uint64_t val        = (xorshifted >> rot) | (xorshifted << ((-rot) & 31));
	return val;
}

/**
 * Transform the random state value to a new random 64-bit unsigned integer.
 *
 * @param state_high High state to produce upper 32 bits for computing random number.
 * @param state_low Low state to produce lower 32 bits for computing random number.
 *
 * @return A uniformly distributed random 64-bit unsigned int
 */
inline uint64_t pcg_output_r(uint64_t const state_high, uint64_t const state_low) {
	uint64_t high_bits = pcg_state_transform(state_high);
	uint64_t low_bits  = pcg_state_transform(state_low);

	return (high_bits << 32) + low_bits;
}

/**
 * Transform the random PCG state value to the next state.
 *
 * @param Prior state value.
 *
 * @return updated state value
 */
inline uint64_t pcg_next_state(uint64_t prior_state) {
	uint64_t state = prior_state * PCG_MULTIPLIER + (PCG_INCREMENT | 1);
	return state;
}

/**
 * Transform the random state value to a new random 64-bit unsigned integer.
 *
 * @return A uniformly distributed random 64-bit unsigned int
 */
inline uint64_t pcg_random_r() {
	PCG_STATE_HIGH_BITS = pcg_next_state(PCG_STATE_HIGH_BITS);
	PCG_STATE_LOW_BITS  = pcg_next_state(PCG_STATE_LOW_BITS);
	return pcg_output_r(PCG_STATE_HIGH_BITS, PCG_STATE_LOW_BITS);
}

/**
 * Set the seed of the random number generator
 *
 * @param seed initial state
 *
 */
void s_rand_local(uint64_t const seed) {
	PCG_STATE_HIGH_BITS = seed;
	PCG_STATE_LOW_BITS  = (seed >> 4) | 1;
}

/**
 * Convert uint64 to double
 * @param v The 64-bit value to be converted to a double-precision floating-point value.
 *
 * @return A double value in floating-point form.
 */
double as_double(uint64_t v) { return (v << 11) * DOUBLE_SCALE_TINY; }

/**
 * Use Marsaglia's Box-Muller polar method for generating a sample from a "normal" distribution from
 * uniformly distributed samples.
 *
 * @return A pseudo random sample from a "normal" distribution.
 */
double RandomNumberGenerator::rand_n() {
	// Box-Mueller method
	if (has_spare) {
		has_spare = false;
		return spare;
	} else {
		double u, v, s;
		do {
			u = this->rand() * 2 - 1;
			v = this->rand() * 2 - 1;
			s = u * u + v * v;
		} while (s >= 1 || s == 0);
		s         = std::sqrt(-2.0 * std::log(s) / s);
		spare     = v * s;
		has_spare = true;
		return u * s;
	}
}

Vector RandomNumberGenerator::rand(int num) {
	auto out = zeros(num);
	for (auto& scalar : out) scalar = this->rand();
	return out;
}

Matrix RandomNumberGenerator::rand(int num_rows, int num_cols) {
	auto out = zeros(num_rows, num_cols);
	for (auto& scalar : out) scalar = this->rand();
	return out;
}

Vector RandomNumberGenerator::rand_n(int num) {
	auto out = zeros(num);
	for (auto& scalar : out) scalar = this->rand_n();
	return out;
}


Matrix RandomNumberGenerator::rand_n(int num_rows, int num_cols) {
	auto out = zeros(num_rows, num_cols);
	for (auto& scalar : out) scalar = this->rand_n();
	return out;
}

not_null<shared_ptr<RandomNumberGenerator>> get_global_rng() { return GLOBAL_RNG; }


void set_global_rng(not_null<shared_ptr<RandomNumberGenerator>> randomness) {
	GLOBAL_RNG = std::move(randomness);
}


void s_rand(uint64_t seed) { GLOBAL_RNG->seed(seed); }

double rand() { return GLOBAL_RNG->rand(); }


Vector rand(int num) { return GLOBAL_RNG->rand(num); }


Matrix rand(int num_rows, int num_cols) { return GLOBAL_RNG->rand(num_rows, num_cols); }


double rand_n() { return GLOBAL_RNG->rand_n(); }


Vector rand_n(int num) { return GLOBAL_RNG->rand_n(num); }


Matrix rand_n(int num_rows, int num_cols) { return GLOBAL_RNG->rand_n(num_rows, num_cols); }

}  // namespace experimental
}  // namespace navtk
