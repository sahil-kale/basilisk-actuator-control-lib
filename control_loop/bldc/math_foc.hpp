#ifndef MATH_CLARKE_PARKE_HPP
#define MATH_CLARKE_PARKE_HPP

#include <stdint.h>

namespace math {

/**
 * @brief Storage class for alpha/beta values
 */
class alpha_beta_t {
   public:
    /** Alpha value of the transform */
    float alpha = 0.0f;
    /** Beta value of the transform */
    float beta = 0.0f;
};

/**
 *  @brief Storage class for direct/quadrature values
 */
class direct_quad_t {
   public:
    /** Direct transform */
    float direct = 0.0f;
    /** Quadrature transform */
    float quadrature = 0.0f;
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
