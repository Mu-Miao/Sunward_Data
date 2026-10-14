#ifndef CAR_PI_H
#define CAR_PI_H
/* No HAL dependency. Called after wheel feedback/protection checks.
 * target/measured are positive mm/s magnitudes; seconds is the actual dt.
 * integral/output use raw motor-command units and remain in [0, limit].
 * Valid configuration and non-null integral are required by the wheel layer.
 */
float CarPI_Update(float target, float measured, float kp, float ki,
                   float seconds, int limit, float *integral);
#endif
