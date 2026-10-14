#ifndef CAR_PI_H
#define CAR_PI_H
/* PI 计算不依赖 HAL，在轮速测量和保护检查后调用。
 * target/measured 是轮速幅值，单位毫米/秒；seconds 是实际采样间隔。
 * 积分与输出采用电机命令的原始单位，限制在 0 到 limit 之间。
 * 轮速模块负责保证参数有效以及积分指针非空。 */
float CarPI_Update(float target, float measured, float kp, float ki,
                   float seconds, int limit, float *integral);
#endif
