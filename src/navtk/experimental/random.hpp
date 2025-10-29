#pragma once

#include <memory>
#include <random>
#include <xtensor/generators/xrandom.hpp>

#include <navtk/not_null.hpp>
#include <navtk/tensors.hpp>

namespace navtk {
// TODO #613: Remove random number generation support from experimental namespace once rng is
// verified.

/**
 * Namespace for navtk features that may come with one or more caveats.
 */
namespace experimental {

/**
 * Set the seed of the current global random number generator.
 *
 * This is equivalent to calling `get_global_rng()->seed(seed)`.
 *
 * @param seed New seed value to be passed to the underlying random number generator.
 */
void s_rand(uint64_t seed);

/**
 * Set the seed of the normalized random number generator.
 *
 * @param seed New seed value to be passed to the underlying random number generator.
 */
void s_rand_local(uint64_t const seed);

/**
 * Return a sample from the uniform distribution between 0 and 1.
 *
 * @return A draw from the uniform distribution between 0 and 1.
 */
double rand_local();

/**
 * Convert `uint64_t` to `double`
 * @param v The 64-bit value to be converted to a double-precision floating-point value.
 *
 * @return A `double` value in floating-point form.
 */
double as_double(uint64_t v);

/**
 * Transform the random state value to a new random 64-bit unsigned integer.
 *
 * @return A uniformly distributed random 64-bit unsigned integer
 */
inline uint64_t pcg_random_r();

/**
 * Abstraction layer for random number generation. Subclasses of this type must define a seed setter
 * and a function returning random double-precision numbers uniformly distributed between 0 and 1.
 * Implementations should strive for consistent high performance over cryptographic
 * unpredictability. Avoid depending on finite pools of entropy (such as `/dev/random`) as these
 * will be quickly exhausted and cause filtering performance issues.
 *
 * Namespace-level random functions, including s_rand(), navtk::experimental::rand() and
 * navtk::experimental::rand_n(), are all implemented by calling methods on the current value of
 * get_global_rng().
 *
 * If you have an existing random number generator that conforms to the C++ RandomNumberEngine named
 * requirements, you can use the RandomNumberEngineWrapper template class to convert it to
 * RandomNumberGenerator.
 */
class RandomNumberGenerator {
public:
	RandomNumberGenerator() = default;
	/** Copy and move are deleted to prevent object slicing. Use `std::shared_ptr`. */
	RandomNumberGenerator(const RandomNumberGenerator&) = delete;
	/** Copy and move are deleted to prevent object slicing. Use `std::shared_ptr`. */
	RandomNumberGenerator(RandomNumberGenerator&&) = delete;
	/** Copy and move are deleted to prevent object slicing. Use `std::shared_ptr`. */
	RandomNumberGenerator& operator=(const RandomNumberGenerator&) = delete;
	/** Copy and move are deleted to prevent object slicing. Use `std::shared_ptr`. */
	RandomNumberGenerator& operator=(RandomNumberGenerator&&) = delete;

	virtual ~RandomNumberGenerator() = default;

	/**
	 * Reset the state of the underlying random number generation algorithm.
	 *
	 * @param seed New seed value.
	 */
	virtual void seed(uint64_t seed) = 0;

	/**
	 * @return A single random number from a uniform distribution between 0 and 1.
	 */
	virtual double rand() = 0;

	/**
	 * @return A Vector of the requested size populated with samples from a uniform distribution
	 * between 0 and 1, using calls to this->rand() to generate those values.
	 *
	 * @param num Size of desired output Vector.
	 */
	Vector rand(int num);

	/**
	 * @return A Matrix of the requested shape populated with samples from a uniform distribution
	 * between 0 and 1, using calls to this->rand() to generate those values.
	 *
	 * @param num_rows Row count of the output Matrix.
	 * @param num_cols Column count of the output Matrix.
	 */
	Matrix rand(int num_rows, int num_cols);

	/**
	 * Return a single random number from a normal (Gaussian) distribution with mean=0, sigma=1.
	 *
	 * The default implementation uses this->rand() as a source of uniform randomness and transforms
	 * this to gaussian using the Marsaglia Polar Method. Subclasses may override this function if
	 * a different sampling algorithm is desired.
	 *
	 * @return A single random number from a normal (Gaussian) distribution with mean=0, sigma=1.
	 */
	virtual double rand_n();

	/**
	 * @return A Vector of the requested size populated with samples from a normal (Gaussian)
	 * distribution with mean=0, sigma=1, using calls to this->rand_n() to generate those values.
	 *
	 * @param num Size of the output Vector
	 */
	Vector rand_n(int num);

	/**
	 * @return A Matrix of the requested shape populated with samples from a normal (Gaussian)
	 * distribution with mean=0, sigma=1, using calls to this->rand_n() to generate those values.
	 *
	 * @param num_rows Row count of the output Matrix.
	 * @param num_cols Column count of the output Matrix.
	 */
	Matrix rand_n(int num_rows, int num_cols);

private:
	double spare   = 0.0;
	bool has_spare = false;
};

/**
 * Wrapper implementing RandomNumberGenerator based on the C++ `RandomNumberEngine` named
 * requirements. This provides the boilerplate to allow navtk to use any conforming
 * implementation of `RandomNumberEngine` as its source of randomness.
 *
 * This class uses `std::uniform_real_distribution` to convert the values emitted by TEngine<> to
 * `double`.
 */
template <typename TEngine>
class RandomNumberEngineWrapper : public RandomNumberGenerator {
public:
	/** Underlying C++-style `RandomNumberEngine` instance */
	TEngine engine;

	/** Conversion class to convert the engine's integer output to doubles */
	std::uniform_real_distribution<double> converter;

	/** Default constructor */
	RandomNumberEngineWrapper() : RandomNumberGenerator(), engine(), converter(0.0, 1.0) {}

	double rand() override { return converter(engine); }

	void seed(uint64_t seed) override { engine.seed(seed); }
};

/**
 * Class that implements our local NavToolkit random number generator.
 *
 */
class LocalEngineWrapper : public RandomNumberGenerator {
public:
	double rand() override { return rand_local(); }

	void seed(uint64_t seed) override { s_rand_local(seed); }
};

/**
 * Return navtk's current global random number generator.
 *
 * By default, this will return a RandomNumberGenerator backed by `pcg64_oneseq`. It can be
 * changed at runtime by calling set_global_rng().
 *
 * @return The currently selected global RandomNumberGenerator.
 */
not_null<std::shared_ptr<RandomNumberGenerator>> get_global_rng();


/**
 * Replace the current global random number generator.
 *
 * This method allows you to choose a different algorithm for navtk's default random number
 * generator, used by the global rand() and rand_n()
 * functions when their `randomness` parameter is not supplied.
 *
 * To change the seed, rather than the algorithm, of the current global random number generator, use
 * s_rand() instead.
 *
 * @param randomness Your desired RandomNumberGenerator instance.
 */
void set_global_rng(not_null<std::shared_ptr<RandomNumberGenerator>> randomness);


/**
 * Replace the current global random number generator with a class conforming to the
 * `RandomNumberEngine` named requirements.
 */
template <typename TEngine>
void set_global_rng() {
	set_global_rng(std::make_shared<RandomNumberEngineWrapper<TEngine>>());
}


/**
 * Return a single random number from a uniform distribution between 0 and 1.
 * @return A random number from a uniform distribution between 0 and 1.
 */
double rand();


/**
 * Return a Vector of random numbers from a uniform distribution between 0 and 1.
 * @param num The number of desired random numbers.
 *
 * @return A vector of random numbers from a uniform distribution between 0 and 1.
 */
Vector rand(int num);


/**
 * Return a Matrix of random numbers from a uniform distribution between 0 and 1.
 * @param num_rows The number of rows in the desired array.
 * @param num_cols The number of columns in the desired array.
 *
 * @return Matrix of uniform distribution random numbers between 0 and 1.
 */
Matrix rand(int num_rows, int num_cols);


/**
 * Return a single random number from a normal distribution with mean=0, sigma=1.
 * @return A random number from a normal distribution.
 */
double rand_n();


/**
 * Return a Vector of random numbers from a normal distribution with mean=0, sigma=1.
 * @param num The number of desired random numbers.
 *
 * @return A Vector of random numbers from a normal distribution.
 *
 */
Vector rand_n(int num);


/**
 * Return a Matrix of random numbers from a normal distribution with mean=0, sigma=1.
 * @param num_rows The number of rows in the desired array.
 * @param num_cols The number of columns in the desired array.
 *
 * @return Matrix of normal distribution random numbers.
 */
Matrix rand_n(int num_rows, int num_cols);

}  // namespace experimental
}  // namespace navtk
