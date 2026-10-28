#ifndef FILTER_LPF_1
#define FILTER_LPF_1

namespace filter
{
	/**
	 * @namespace filter
	 * @brief Contains digital signal filtering algorithms.
	 */

	namespace low_pass
	{
		/**
		 * @namespace low_pass
		 * @brief Contains low-pass filter implementations.
		 */

		/**
		 * @class LPF_1
		 * @brief First-order low-pass filter.
		 *
		 * @details
		 * The LPF_1 class implements a discrete-time first-order
		 * low-pass filter.
		 *
		 * The filter attenuates high-frequency components while allowing
		 * low-frequency components of the input signal to pass.
		 *
		 * The filter can be configured using the cutoff frequency and
		 * sampling time. Alternatively, individual filter parameters such
		 * as the cutoff frequency, time constant, sampling time, and
		 * filter coefficient can be configured directly.
		 *
		 * @tparam T Numeric data type used for the filter.
		 *
		 * @par Example
		 * @code{.cpp}
		 * filter::low_pass::LPF_1<double> filter;
		 *
		 * filter.config(10.0, 0.001);
		 *
		 * double output = filter.update(input);
		 * @endcode
		 */
		template <typename T>
		class LPF_1
		{
		public:
			/**
			 * @brief Constructs a first-order low-pass filter.
			 *
			 * @details
			 * Constructs the filter with its parameters initialized to
			 * their default values.
			 */
			LPF_1();

			/**
			 * @brief Configures the low-pass filter.
			 *
			 * @param[in] fc_ Cutoff frequency of the filter in Hz.
			 * @param[in] dt_ Sampling time of the filter in seconds.
			 *
			 * @details
			 * Configures the filter using the cutoff frequency and
			 * sampling time.
			 */
			void config(T fc_, T dt_);

			/**
			 * @brief Sets the filter parameters.
			 *
			 * @param[in] fc_ Cutoff frequency of the filter in Hz.
			 * @param[in] dt_ Sampling time of the filter in seconds.
			 *
			 * @details
			 * Updates the cutoff frequency and sampling time used by
			 * the filter.
			 */
			void set_param(T fc_, T dt_);

			/**
			 * @brief Processes a new input sample.
			 *
			 * @param[in] x_k Current input sample.
			 *
			 * @return Filtered output sample.
			 *
			 * @details
			 * Updates the filter state using the current input sample
			 * and returns the filtered output.
			 */
			T update(T x_k);

			/**
			 * @brief Resets the filter state.
			 *
			 * @details
			 * Resets the previous output and startup state of the filter.
			 */
			void reset();

			/**
			 * @brief Sets the cutoff frequency.
			 *
			 * @param[in] fc_ Cutoff frequency in Hz.
			 */
			void set_fc(T fc_);

			/**
			 * @brief Sets the filter time constant.
			 *
			 * @param[in] tau_ Filter time constant in seconds.
			 */
			void set_tau(T tau_);

			/**
			 * @brief Sets the sampling time.
			 *
			 * @param[in] dt_ Sampling time in seconds.
			 */
			void set_dt(T dt_);

			/**
			 * @brief Sets the filter coefficient.
			 *
			 * @param[in] alpha_ Filter coefficient used by the
			 * discrete-time filter.
			 */
			void set_alpha(T alpha_);

			/**
			 * @brief Gets the cutoff frequency.
			 *
			 * @return Cutoff frequency in Hz.
			 */
			T get_fc();

			/**
			 * @brief Gets the filter time constant.
			 *
			 * @return Filter time constant in seconds.
			 */
			T get_tau();

			/**
			 * @brief Gets the sampling time.
			 *
			 * @return Sampling time in seconds.
			 */
			T get_dt();

			/**
			 * @brief Gets the filter coefficient.
			 *
			 * @return Filter coefficient.
			 */
			T get_alpha();

			/**
			 * @brief Gets the sampling frequency.
			 *
			 * @return Sampling frequency in Hz.
			 *
			 * @details
			 * The sampling frequency is the reciprocal of the sampling
			 * time:
			 *
			 * @f[
			 * f_s = \frac{1}{dt}
			 * @f]
			 */
			T get_fs();

		private:
			/**
			 * @brief Cutoff frequency of the filter.
			 *
			 * @details
			 * Cutoff frequency in Hz.
			 */
			T fc = 0.0;

			/**
			 * @brief Filter time constant.
			 *
			 * @details
			 * Time constant in seconds.
			 */
			T tau = 0.0;

			/**
			 * @brief Sampling time of the filter.
			 *
			 * @details
			 * Sampling period in seconds.
			 */
			T dt = 0.0;

			/**
			 * @brief Discrete-time filter coefficient.
			 */
			T alpha = 0.0;

			/**
			 * @brief Previous filter output.
			 *
			 * @details
			 * Stores the output sample from the previous filter iteration.
			 */
			T y_k_1 = 0.0;

			/**
			 * @brief Indicates whether the filter has received its first sample.
			 *
			 * @details
			 * This flag is used to handle initialization of the filter
			 * state when processing the first input sample.
			 */
			bool start = false;
		};

#include "LPF_1.tpp"
	} // namespace low_pass

} // namespace filter

#endif // FILTER_LPF_1