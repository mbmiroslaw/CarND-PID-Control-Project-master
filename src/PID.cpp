#include "PID.h"
#include <cmath>
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	
    PID::Kp = Kp;
    PID::Ki = Ki;
    PID::Kd = Kd;
	
	p_error = 0;
	i_error = 0;
	d_error = 0;
	best_error = 1000000;
	flag = 0; //twiddle on = 0; off = 1
	num_twiddle = 100;
	twiddle_i = 0;
	total_error = 100000;
	twiddle_type = 1;
	
	dp = {0.05*Kp, 0.05*Ki, 0.05*Kd};
}

void PID::UpdateError(double cte) {
	i_error += cte;
	d_error = cte - p_error;
	p_error = cte;

	if((twiddle_i < num_twiddle) && (flag < 1)){
		twiddle_i ++;
		//pid.total_error += (pow(cte,2))/pid.twiddle_i;
		
		double p[3];
		p[0] = Kp;
		p[1] = Ki;
		p[2] = Kd;
		
		total_error = cte*cte;
		if (twiddle_type == 1){
			twiddle1(0.0001, cte);
			twiddle_type = 2;
			for (int i = 0; i < 3; i++){
			p[i] += dp[i];
		}
		}
		else{
			twiddle2(0.0001, cte);
			twiddle_type = 1;
		}
	}
}

double PID::TotalError() {
	return -Kp*p_error - Kd * d_error - Ki * i_error; 
}

void PID::twiddle1(double tol, double cte){
	double p[3];
	p[0] = Kp;
	p[1] = Ki;
	p[2] = Kd;
	cout << "total_error = " << total_error << " best_error = " << best_error << endl;
	
	for (int i = 0; i < 3; i++){
//		p[i] += dp[i];
		if (total_error < best_error){
			best_error = total_error;
			dp[i] *=1.1;
		}
		else{
			p[i] -=2*dp[i];
		}
	}
	
	Kp = p[0];
	Ki = p[1];
	Kd = p[2];
	
	if ((dp[0]+dp[1]+dp[2]) < tol){
		flag = 1;
	}
	
	cout << "Kp = " << Kp << " Ki = " << Ki << " Kd = " << Kd << endl;

}

void PID::twiddle2(double tol, double cte){
	double p[3];
	p[0] = Kp;
	p[1] = Ki;
	p[2] = Kd;
	
	cout << "total_error = " << total_error << " best_error = " << best_error << endl;
	for (int i = 0; i < 3; i++){
//		p[i] += dp[i];
		if (total_error < best_error){
			best_error = total_error;
			dp[i] *=1.1;
		}
		else{
			p[i] += dp[i];
			dp[i] *= 0.9;
		}
	}
	Kp = p[0];
	Ki = p[1];
	Kd = p[2];
	
	if ((dp[0]+dp[1]+dp[2]) < tol){
		flag = 1;
	}
	
	cout << "Kp = " << Kp << " Ki = " << Ki << " Kd = " << Kd << endl;
}


