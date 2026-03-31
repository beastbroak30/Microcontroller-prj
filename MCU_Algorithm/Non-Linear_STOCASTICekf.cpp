/* 
 * ========================================================== 
 * PROJECT: NONLINEAR STOCHASTIC MPC + EKF (RP2350) 
 * ========================================================== 
 * Author: Based on Rasit Evduzen (MATLAB) 
 * Port: C++ for Raspberry Pi Pico 2 (RP2350) 
 * Date: March 30, 2026 
 * 
 * DESCRIPTION: 
 * This program implements a predictive control strategy 
 * for a nonlinear, noisy system. 
 * 
 * COMPONENTS: 
 * 1. EXTENDED KALMAN FILTER (EKF): 
 * Estimates internal states (x1, x2) from a noisy 
 * output signal (y). 
 * 
 * 2. NONLINEAR MPC (NMPC): 
 * Uses the model to find the optimal control input (u) 
 * over a prediction horizon (Ky). 
 * 
 * 3. GAUSS-NEWTON OPTIMIZATION: 
 * Solves the nonlinear problem via local linearization 
 * using an analytic Jacobian matrix. 
 * 
 * SYSTEM DYNAMICS: 
 * x1(k+1) = 0.1 - x1^2 + x1*x2 
 * x2(k+1) = -x1 + exp(-x2) + u 
 * y(k) = x1 + x2 (Measured Output) 
 * 
 * HARDWARE NOTE: 
 * The RP2350 utilizes its FPU (Floating Point Unit) to 
 * solve matrix inversions in real-time (< 1ms). 
 * ========================================================== 
 */
/* 
 * ---------------------------------------------------------- 
 * MATHEMATICAL FOUNDATIONS OF THE IMPLEMENTATION 
 * ---------------------------------------------------------- 
 * 
 * THE EKF CYCLE: 
 * A) PREDICT: x_hat(-) = f(x_hat, u) 
 * Calculates expectation based on system physics. 
 * B) UPDATE: x_hat(+) = x_hat(-) + K * (y_meas - y_hat) 
 * Corrects the estimate using real measurements. 
 * 
 * THE NMPC CYCLE: 
 * 1. PREDICTION: Simulates Ky steps into the future. 
 * 2. JACOBIAN: Calculates sensitivity of outputs to 
 * input changes (dy/du). 
 * 3. OPTIMIZATION: Minimizes the cost function: 
 * J = sum(Error^2) + lambda * sum(delta_u^2) 
 * ---------------------------------------------------------- 
 */
/* 
 * ---------------------------------------------------------- 
 * VARIABLE GUIDE: 
 * Ts : Sampling rate (Standard 10ms). 
 * Ky : Prediction Horizon (Look-ahead steps). 
 * Ku : Control Horizon (Degrees of freedom). 
 * lamda : Penalty weight (Input smoothing). 
 * P_obs : EKF Uncertainty (Covariance matrix). 
 * L : Tridiagonal matrix for delta-u penalty. 
 * ---------------------------------------------------------- 
 */
/* 
 * ========================================================== 
 * APPLICATIONS AND FIELDS OF USE 
 * ========================================================== 
 * This SNMPC (Stochastic Nonlinear MPC) is designed for 
 * systems requiring high precision and proactive action. 
 * It is ideal for the following scenarios: 
 * 
 * 1. HIGH-DYNAMIC ROBOTICS & DRONES: 
 * - Flight Control: Wind gust compensation via EKF 
 * and predictive path planning. 
 * - Inverted Pendulum: Stabilizing unstable systems 
 * with strong nonlinear dynamics. 
 * 
 * 2. AUTONOMOUS DRIVING (TRAJECTORY TRACKING): 
 * - Cornering: The controller "sees" the road ahead 
 * in the horizon (Ky) and initiates steering before 
 * - reaching the curve, rather than just reacting. 
 * - Obstacle Avoidance: Direct consideration of 
 * physical limits (u_min/u_max) in the core solver. 
 * 
 * 3. PROCESS ENGINEERING: 
 * - Chemical Reactors: Monitoring exothermic processes 
 * that become unstable if temperature limits break. 
 * - Distillation: Controlling coupled variables like 
 * pressure and temperature in complex columns. 
 * 
 * 4. ENERGY MANAGEMENT & SMART GRIDS: 
 * - Battery Tech: Optimizing charge cycles based on 
 * future load forecasts. 
 * - Wind Power: Predictive pitch angle adjustment 
 * during gusts to reduce mechanical wear. 
 * 
 * PREREQUISITES FOR USE: 
 * - A mathematical model (f_sys) must be defined. 
 * - Hardware requires an FPU (like RP2350) for 
 * timely matrix inversions. 
 * - Sensors may be noisy, as the EKF provides filtering. 
 * 
 * WHEN IS IT UNSUITABLE? 
 * - Simple SISO systems (e.g., room temperature control), 
 * where a basic PID controller is more efficient. 
 * - Extremely fast systems without sufficient CPU power. 
 * ========================================================== 
 */
/* 
 * ---------------------------------------------------------- 
 * APPLICATION NOTE: 
 * For visualization in the Arduino IDE, open the 
 * "Serial Plotter" (Ctrl+Shift+L). 
 * Output is compatible with the plotter format: 
 * Ref, Actual, Estimate, Confidence Interval (+/- 2 Sigma). 
 * ---------------------------------------------------------- 
 */
#include <BasicLinearAlgebra.h> 
#include <math.h> 
using namespace BLA;

// --- Configuration --- 
const float Ts = 0.01; // 10ms sampling rate 
const int Ku = 5; // Control Horizon 
const int Ky = 10; // Prediction Horizon 
const float lamda = 1.0; // Delta-u penalty weight 
const float deltaumax = 0.2; // Max input change per step 
const float umin = -1.0, umax = 1.0;

// --- Global Variables & Matrices --- 
Matrix<2, 1> x_hat = {0, 0}; // EKF estimated state 
Matrix<2, 2> P_obs = {1, 0, 0, 1}; // Estimate error covariance 
Matrix<2, 2> Q_obs = {1e-4, 0, 0, 1e-4}; // Process noise 
float R_obs = 5e-3; // Measurement noise 
Matrix<2, 1> c = {1, 1}; // Output matrix (y = c'*x) 
Matrix<Ku + 1, 1> U_k; // Current input horizon 
Matrix<Ku + 1, Ku + 1> L; // Penalty matrix for Delta-U 

// --- System Dynamics (Nonlinear) --- 
Matrix<2, 1> f_sys(Matrix<2, 1> x, float u) {
 Matrix<2, 1> x_next;
 x_next(0) = 0.1 - pow(x(0), 2) + x(0) * x(1);
 x_next(1) = -x(0) + exp(-x(1)) + u;
 return x_next;
}

// Jacobian df/dx 
Matrix<2, 2> Jf_x(Matrix<2, 1> x) {
 Matrix<2, 2> F;
 F(0, 0) = -2.0 * x(0) + x(1);
 F(0, 1) = x(0);
 F(1, 0) = -1.0;
 F(1, 1) = -exp(-x(1));
 return F;
}

// --- EKF: Extended Kalman Filter --- 
void update_EKF(float y_measured, float u_applied) {
 // 1. Predict 
 Matrix<2, 1> x_pred = f_sys(x_hat, u_applied);
 Matrix<2, 2> F = Jf_x(x_hat);
 Matrix<2, 2> P_pred = F * P_obs * ~F + Q_obs;
 
 // 2. Update 
 float innovation = y_measured - (c(0) * x_pred(0) + c(1) * x_pred(1));
 float S = (c(0) * (P_pred(0,0)*c(0) + P_pred(0,1)*c(1)) + c(1) * (P_pred(1,0)*c(0) +
P_pred(1,1)*c(1))) + R_obs;
 Matrix<2, 1> K = P_pred * c * (1.0 / S);
 x_hat = x_pred + K * innovation;
 P_obs = (Identity<2, 2>() - K * ~c) * P_pred;
}

// --- NMPC: Nonlinear MPC Step --- 
void solve_NMPC(float y_ref) {
 Matrix<2, Ky + 1> x_traj;
 Matrix<Ky, 1> y_hat;
 x_traj.column(0) = x_hat;
 
 // 1. Forecast over the horizon 
 for (int k = 0; k < Ky; k++) {
 float uk = U_k(min(k, Ku));
 x_traj.column(k + 1) = f_sys(x_traj.column(k), uk);
 y_hat(k) = c(0) * x_traj(0, k + 1) + c(1) * x_traj(1, k + 1);
 }
 
 // 2. Calculate Analytic Jacobian Matrix 
 Matrix<Ky, Ku + 1> Jac;
 Jac.Fill(0);
 Matrix<2, 1> dfdu = {0, 1}; 
 
 for (int j = 0; j <= Ku; j++) {
 Matrix<2, 1> dxdu = dfdu;
 Jac(j, j) = (c(0) * dxdu(0) + c(1) * dxdu(1));
 for (int k = j + 1; k < Ky; k++) {
 Matrix<2, 2> Fk = Jf_x(x_traj.column(k));
 dxdu = Fk * dxdu; 
 Jac(k, j) = (c(0) * dxdu(0) + c(1) * dxdu(1));
 }
 }
 
 // 3. Gauss-Newton Optimization 
 Matrix<Ky, 1> e; // Error vector 
 for (int k = 0; k < Ky; k++) e(k) = y_ref - y_hat(k);
 Matrix<Ku + 1, Ku + 1> Hes = (~Jac * Jac) + (L * lamda);
 
 // Gradient: -J'*e + lambda*L*U 
 Matrix<Ku + 1, 1> g = ((~Jac * e) * -1.0) + (L * U_k * lamda);
 
 // Newton step: H * delta_u = -g 
 Matrix<Ku + 1, 1> delta_u = Invert(Hes) * (g * -1.0);
 
 // 4. Constraints & Update 
 float max_val = 0.01; // Avoid division by zero 
 for(int i=0; i<=Ku; i++) if(abs(delta_u(i)) > max_val) max_val = abs(delta_u(i));
 float mu = min(1.0f, deltaumax / max_val);
 U_k += delta_u * mu;
 
 // Saturate current input 
 U_k(0) = max(umin, min(umax, U_k(0)));
}

void setup() {
 Serial.begin(115200);
 
 // Initialize Penalty Matrix L 
 L.Fill(0);
 for (int i = 0; i <= Ku; i++) {
 L(i, i) = 2.0;
 if (i < Ku) { L(i + 1, i) = -1.0; L(i, i + 1) = -1.0; }
 }
 L(Ku, Ku) = 1.0;
 U_k.Fill(0);
}

void loop() {
 static float time = 0;
 
 // --- Generate Reference Signal (as in MATLAB) --- 
 float y_ref = (time < 10.0) ? 0.8 : (0.3 * sin(0.5 * PI * (time - 10.0)) + 0.8);
 
 // --- Simulation / Measurement --- 
 // In reality, y_meas would come from ADC. Simulating the system here: 
 static Matrix<2, 1> x_true = {0, 0};
 x_true = f_sys(x_true, U_k(0)); // True system state 
 float y_meas = (c(0) * x_true(0) + c(1) * x_true(1)) + (sqrt(R_obs) * ((float)rand()/RAND_MAX -
0.5));
 
 // --- Control Cycle --- 
 uint32_t start = micros();
 update_EKF(y_meas, U_k(0)); // Step 1: Where are we? 
 solve_NMPC(y_ref); // Step 2: What do we do? 
 uint32_t duration = micros() - start;
 
 // --- Output for Serial Plotter --- 
 float y_hat_val = c(0) * x_hat(0) + c(1) * x_hat(1);
 float sigma = sqrt(c(0)*(P_obs(0,0)*c(0)+P_obs(0,1)*c(1)) +
c(1)*(P_obs(1,0)*c(0)+P_obs(1,1)*c(1)));
 
 Serial.print("Ref:"); Serial.print(y_ref); Serial.print(",");
 Serial.print("Act:"); Serial.print(y_meas); Serial.print(",");
 Serial.print("Est:"); Serial.print(y_hat_val); Serial.print(",");
 Serial.print("Upper:"); Serial.print(y_hat_val + 2*sigma); Serial.print(",");
 Serial.print("Lower:"); Serial.print(y_hat_val - 2*sigma); Serial.print(",");
 Serial.print("U:"); Serial.println(U_k(0));
 
 time += Ts;
 delay(Ts * 1000); // Real-time simulation delay 
}
