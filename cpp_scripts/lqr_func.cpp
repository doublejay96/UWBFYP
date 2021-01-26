#include <iostream>
#include <vector>
#include <fstream>
#include <Eigen/Dense>

//Propagate to next state based on state space model
Eigen::Vector3f next_state(Eigen::Vector3f state, Eigen::Vector3f control, Eigen::Matrix3f A, Eigen::Matrix3f B) {
	return (A * state) + (B * control);
}

//The actual LQR function
Eigen::Vector3f LQR(Eigen::Vector3f state_error, Eigen::Matrix3f Q, Eigen::Matrix3f R, Eigen::Matrix3f A, Eigen::Matrix3f B) {
	int N = 50;//time-horizon
	std::vector<Eigen::Matrix3f> P_vec (N+1);//Positive,definite,symmetric solution to DARE
	P_vec[N] = Q;//terminal condition
	//Pre-compute transposes of A, B to save function calls, A,B do not vary
	Eigen::Matrix3f A_t = A.transpose();
	Eigen::Matrix3f B_t = B.transpose();
	for (int i = N; i > 0; i--) {//Iterate backwards in time
		//Discrete-time Algebraic Riccati Equation
		P_vec[i-1] = (A_t * P_vec[i] * A) - (A_t * P_vec[i] * B) * (R + (B_t * P_vec[i] * B)).inverse() * (B_t * P_vec[i] * A) + Q;
	}
	std::vector<Eigen::Matrix3f> K_vec (N);//feedback gain matrix K
	std::vector<Eigen::Vector3f> inputs_vec (N);//control inputs
	for (int i = 0; i < N; i++) {//Iterate forwards in time
		K_vec[i] = -(R + (B_t * P_vec[i+1] * B)).inverse() * B_t * P_vec[i+1] * A;
		inputs_vec[i] = K_vec[i] * state_error;
	}
	Eigen::Vector3f optimal_input = inputs_vec[N-1];
	return optimal_input;
}

int main() {
	Eigen::Matrix3f A, B, Q, R;
	Eigen::Vector3f state, control;
	std::string SSmodel_file_path = "state_space_model.csv";
	std::ifstream SSmodel_file;
	SSmodel_file.open(SSmodel_file_path.c_str());
	std::vector<float> SSmodel_A, SSmodel_B;//temp vector storage for the floats
	char line[256];
	char* p;
	for (int l = 0; l < 6; l++) {
		SSmodel_file.getline(line, 256);
		p = line;
		for (int i = 0; i < 3; i++) {
			if (l < 3) {
				SSmodel_A.push_back(atof(p));
			} else {
				SSmodel_B.push_back(atof(p));
			}
			p = strstr(p, ",") + 1;
		}
	}
	for (int i = 0; i < SSmodel_A.size(); i++) std::cout << SSmodel_A[i] << std::endl;
	for (int i = 0; i < SSmodel_B.size(); i++) std::cout << SSmodel_B[i] << std::endl;
	A = Eigen::Map<Eigen::Matrix3f>(SSmodel_A.data());
	std::cout << A << std::endl;
	B = Eigen::Map<Eigen::Matrix3f>(SSmodel_B.data());
	std::cout << B << std::endl;
	Q.setIdentity();
	std::cout << Q << std::endl;
	R.setIdentity();
	R = R * 0.01;
	std::cout << R << std::endl;
	state << 1.0, -0.5, 0.2;
	std::cout << R << std::endl;
	double error_mag = 0;
	for (int t = 0; t < 100; t++) {
		printf("Current time: %.2f seconds\n", t*0.1);
		error_mag = state.norm();
		std::cout << "Current state " << state;
		printf(", state error magnitude %.4f\n", error_mag);
		control = LQR(state, Q, R, A, B);
		std::cout << "Control inputs: " << control << std::endl;
		state = next_state(state, control, A, B);
		if (error_mag < 0.01) {
			printf("Setpoint reached\n");
			break;
		}
	}
	return 0;
}
