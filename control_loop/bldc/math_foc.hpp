#ifndef MATH_CLARKE_PARKE_HPP
#define MATH_CLARKE_PARKE_HPP

#include <stdint.h>

namespace math {

typedef struct {
    float alpha;
    float beta;
} clarke_transform_result_t;

typedef struct {
    float d;
    float q;
} park_transform_result_t;

typedef struct {
    float alpha;
    float beta;
} inverse_park_transform_result_t;

typedef struct {
    float a;
    float b;
    float c;
} inverse_clarke_transform_result_t;

typedef struct {
    float dutyCycleU;
    float dutyCycleV;
    float dutyCycleW;
} svpwm_duty_cycle_t;

/**
 * @brief Perform a clarke transform on the given 3-phase variable values.
 * @param a phase A value to transform to alpha-beta frame
 * @param b phase B value to transform to alpha-beta frame
 * @param c phase C value to transform to alpha-beta frame
 * @return clarke_transform_result_t The result of the clarke transform
 */
clarke_transform_result_t clarke_transform(float a, float b, float c);

/**
 * @brief Perform a park transform on the given alpha-beta values to get d/q frame values.
 * @param alpha Current in alpha
 * @param beta Current in beta
 * @param theta The angle (degrees) of the park transform
 * @return park_transform_result_t The result of the park transform
 */
park_transform_result_t park_transform(float alpha, float beta, float theta);

/**
 * @brief Perform an inverse park transform on the given Vq/Vd voltage values.
 * @param d parameter in the d direction
 * @param q parameter in the q direction
 * @param theta The angle (degrees) of the inverse park transform
 * @return inverse_park_transform_result_t The result of the inverse park transform
 */
inverse_park_transform_result_t inverse_park_transform(float d, float q, float theta);

/**
 * @brief Perform an inverse clarke transform on the given alpha/beta voltage values.
 * @param alpha Voltage in the alpha frame
 * @param beta Voltage in the beta frame
 * @return inverse_clarke_transform_result_t The result of the inverse clarke transform
 */
inverse_clarke_transform_result_t inverse_clarke_transform(float alpha, float beta);

/**
 * @brief Perform a space vector pulse width modulation on the given alpha/beta voltage values.
 * @param Vd Voltage in the d frame
 * @param Vq Voltage in the q frame
 * @param theta_el The electrical angle of the rotor
 * @param Vbus The bus voltage
 * @return svpwm_duty_cycle_t The result of the svpwm
 */
svpwm_duty_cycle_t svpwm(float Vd, float Vq, float theta_el, float Vbus);

}  // namespace math

#endif  // MATH_CLARKE_PARKE_HPP
