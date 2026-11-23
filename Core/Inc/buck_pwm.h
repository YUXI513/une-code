#ifndef BUCK_PWM_H
#define BUCK_PWM_H

/* External 8 MHz HSE crystal, PLL M=2/N=84/R=2: TIM1CLK=168 MHz.
 * Duty before dead-time insertion is fixed at 0.5.
 * DeadTime=200 is the raw CubeMX/TIM1 BDTR.DTG value (0xC8), NOT ns.
 * DTG=200 means 320 timer clocks: 1.904762 us with CKD=DIV1.
 * At 20 kHz, each pin is high for 23.095238 us per 50 us period (46.19%).
 */
#define BUCK_PWM_FREQUENCY_HZ  20000U
#define BUCK_PWM_DEADTIME_CODE 200U

/* Independent GPIO slew-rate settings for edge/overshoot comparison. */
#define BUCK_PWM_PA10_SPEED   GPIO_SPEED_FREQ_MEDIUM
#define BUCK_PWM_PB13_SPEED   GPIO_SPEED_FREQ_LOW

/* Active-high outputs: PA10 = TIM1_CH3, PB13 = TIM1_CH1N (AF6).
 * These are different channels. CCR1 and CCR3 must ALWAYS remain equal.
 * Init holds both pins low; Start enables switching; Stop holds both low.
 * This module owns TIM1 and does not configure a hardware fault/break input.
 */
void BuckPWM_Init(void);
void BuckPWM_Start(void);
void BuckPWM_Stop(void);

#endif /* BUCK_PWM_H */
