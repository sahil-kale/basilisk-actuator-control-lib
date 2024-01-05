#ifndef MATH_CLARKE_PARKE_HPP
#define MATH_CLARKE_PARKE_HPP

#include <stdint.h>

#include "math_util.hpp"

namespace math {

/**
 * @brief Storage class for alpha/beta values
 */
class alpha_beta_t {
   public:
    alpha_beta_t() = default;
    /**
     * @brief Construct a new alpha_beta_t object
     * @param alpha The alpha value of the transform
     * @param beta The beta value of the transform
     */
    alpha_beta_t(float alpha, float beta) : alpha(alpha), beta(beta) {}

    /** Alpha value of the transform */
    float alpha = 0.0f;
    /** Beta value of the transform */
    float beta = 0.0f;

    /**
     * @brief Equality operator for alpha_beta_t
     * @param rhs The right hand side of the equality
     * @return bool True if the values are equal, false otherwise
     */
    bool operator==(const alpha_beta_t& rhs) const {
        const bool alpha_equal = math::float_equals(alpha, rhs.alpha);
        const bool beta_equal = math::float_equals(beta, rhs.beta);

        return alpha_equal && beta_equal;
    }
};

/**
 *  @brief Storage class for direct/quadrature values
 */
class direct_quad_t {
   public:
    direct_quad_t() = default;
    /**
     * @brief Construct a new direct_quad_t object
     * @param direct The direct value of the transform
     * @param quadrature The quadrature value of the transform
     */
    direct_quad_t(float direct, float quadrature) : direct(direct), quadrature(quadrature) {}
    /** Direct transform */
    float direct = 0.0f;
    /** Quadrature transform */
    float quadrature = 0.0f;

    /**
     * @brief Equality operator for direct_quad_t
     * @param rhs The right hand side of the equality
     * @return bool True if the values are equal, false otherwise
     */
    bool operator==(const direct_quad_t& rhs) const {
        const bool direct_equal = math::float_equals(direct, rhs.direct);
        const bool quadrature_equal = math::float_equals(quadrature, rhs.quadrature);

        return direct_equal && quadrature_equal;
    }
};

/**
 * @brief Storage class for abc values
 */
class abc_t {
   public:
    /** A phase value */
    float a = 0.0f;
    /** B phase value */
    float b = 0.0f;
    /** C phase value */
    float c = 0.0f;

    /**
     * @brief Equality operator for abc_t
     * @param rhs The right hand side of the equality
     * @return bool True if the values are equal, false otherwise
     */
    bool operator==(const abc_t& rhs) const {
        const bool a_equal = math::float_equals(a, rhs.a);
        const bool b_equal = math::float_equals(b, rhs.b);
        const bool c_equal = math::float_equals(c, rhs.c);

        return a_equal && b_equal && c_equal;
    }
};

/**
 * @brief Perform a clarke transform on the given 3-phase variable values.
 * @param a phase A value to transform to alpha-beta frame
 * @param b phase B value to transform to alpha-beta frame
 * @param c phase C value to transform to alpha-beta frame
 * @return alpha_beta_t The result of the clarke transform
 */
alpha_beta_t clarke_transform(float a, float b, float c);

/**
 * @brief Perform a park transform on the given alpha-beta values to get d/q frame values.
 * @param alpha Current in alpha
 * @param beta Current in beta
 * @param theta The angle (degrees) of the park transform
 * @return direct_quad_t The result of the park transform
 */
direct_quad_t park_transform(float alpha, float beta, float theta);

/**
 * @brief Perform an inverse park transform on the given Vq/Vd voltage values.
 * @param d parameter in the d direction
 * @param q parameter in the q direction
 * @param theta The angle (degrees) of the inverse park transform
 * @return alpha_beta_t The result of the inverse park transform
 */
alpha_beta_t inverse_park_transform(float d, float q, float theta);

/**
 * @brief Perform an inverse clarke transform on the given alpha/beta voltage values.
 * @param alpha Voltage in the alpha frame
 * @param beta Voltage in the beta frame
 * @return abc_t The result of the inverse clarke transform
 */
abc_t inverse_clarke_transform(float alpha, float beta);

}  // namespace math

#endif  // MATH_CLARKE_PARKE_HPP
