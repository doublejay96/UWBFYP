#include <iostream>
#include <Eigen/Dense>
#include <Eigen/StdVector>

//Propagate to next state based on state space model
Eigen::Vector3f next_state(Eigen::Vector3f state, Eigen::Vector3f control, Eigen::Matrix3f A, Eigen::Matrix3f B) {
	return (A * state) + (B * control);
}

//The actual LQR function
Eigen::Vector3f LQR(Eigen::Vector3f state_error, Eigen::Matrix3f Q, Eigen::Matrix3f R, Eigen::Matrix3f A, Eigen::Matrix3f B) {
	int N = 50;
	std::vector<Eigen::Matrix3f> P_vec (N+1);
	P_vec[N] = Q;
	//Pre-compute transposes of A, B to save function calls
	Eigen::Matrix3f A_t = A.transpose();
	Eigen::Matrix3f B_t = B.transpose();
	for (int i = N; i > 0; i--) {
		//Discrete-time Algebraic Riccati Equation
		P_vec[i-1] = (A_t * P_vec[i] * A) - (A_t * P_vec[i] * B) * (R + (B_t * P_vec[i] * B)).inverse() * (B_t * P_vec[i] * A) + Q;
	}
	std::vector<Eigen::Matrix3f> K_vec (N);
	std::vector<Eigen::Vector3f> inputs_vec (N);
	for (int i = 0; i < N; i++) {
		K_vec[i] = -(R + (B_t * P_vec[i+1] * B)).inverse() * B_t * P_vec[i+1] * A;
		inputs_vec[i] = K_vec[i] * state_error;
	}
	Eigen::Vector3f optimal_input = inputs_vec[N-1];
	return optimal_input;
}

int main() {
	Eigen::MatrixXd m(2,2);
	m(0,0) = 3;
	m(1,0) = 2.5;
	m(0,1) = -1;
	m(1,1) = m(1,0) + m(0,1);
	std::cout << m << std::endl;
}
