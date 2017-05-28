//
// inv_pendulum.c
//
// Simulation of an inverted pendulum system using both linear and nonlinear controllers
//
// Suvadeep Banerjee 03/22/2017
//
#include <stdio.h>
#include <math.h>

int main() {

	FILE *fp;
	fp = fopen("data.dat", "w");

// Parameters of an inverted pendulum system
//
	double M = 2;		// Mass of the cart in kg
	double m = 0.2;		// Mass of the pendulum bob in kg
	double l = 0.31;	// Length of the pendulum in m
	double g = 9.81;	// Acceleration due to gravity

// Simulation Parameters
//
	double ts = 0.00125;	// Sampling Time
	double duration = 30;	// Simulation Duration
	double timepts = duration/ts;
	timepts = timepts+1;

// Initialization of Parameters
//
	double t[(int)timepts];		// Time Vector
	t[0] = 0;
	int i = 1;
	for(i=1;i<timepts;i++){
		t[i] = t[i-1] + ts;
	}

	double x[4][(int)timepts];	// State Vectors
	double pi = 3.1416;
	double theta_0 = 180*pi/180;	// Initial Angle
	x[0][0] = theta_0;
	double xd[(int)timepts];

// Linear Controller Parameters
//
	double K[4] = {-234.1346, -27.8820, -208.4806, -57.9189};	// State Feedback Control Law Gain

// Nonlinear Energy-based Controller Parameters
//
	double wn = sqrt(g/l);
	double a0 = 1.3;
	double b0 = 0.833;
	double gw = 0.4612;
	double phi = -1.4831;
	double f1 = 39.0681;
	double f2 = 15.0011;
	double Vd = 2*m*g*l;
	int linear = 0;
	int nonlinear = 1;

// Control Strategy Switching Conditions
//
	double switch_angle = 30;
	if(fabs(x[0][0]) <= (switch_angle*pi/180)){
		int nonlinear = 0;
		int linear = 1;
	}else{
		int nonlinear = 1;
		int linear = 0;
	}



// Plant Simulation Initialization Parameters
//
	double theta = 0;
	double thetadot = 0;
	double disp = 0;
	double vel = 0;
	double u = 0;
	double psi = 0;
	double V = 0;
	double a = 0;
	double f1x = 0;
	double f2x = 0;
	double delta_x[4];


// Plant Simulation
//
	for(i=1; i<timepts; i++){
		theta = x[0][i-1];
		thetadot = x[1][i-1];
		disp = x[2][i-1];
		vel = x[3][i-1];

// Switching to Linear Controller Rule
//
		if((fabs(theta) < (switch_angle*pi/180)) || (linear == 1)){
			u = (-1)*(K[0]*theta+K[1]*thetadot+K[2]*disp+K[3]*vel);
			linear = 1;
			nonlinear = 0;

// Nonlinear Controller Rule Switching
//
		}else{
			psi = (-1)*atan((thetadot/wn)/theta - pi);
			V = 0.5*m*pow(l,2)*pow(thetadot,2) + m*g*l*(1 - cos(theta-pi));


			if(fabs(V-Vd) >= b0){
				if(V>Vd){
					a = a0;
				}else if(V==Vd){
					a = 0;
				}else{
					a = -a0;
				}
			}else if(fabs(V-Vd) < b0){
				a = a0*(V-Vd)/b0;
			}

			xd[i] = (a/gw)*sin(psi-pi+phi);
			u = (M+m*pow(sin(theta-pi),2))*(f1*(xd[i]-disp)-f2*vel)+m*g*cos(theta-pi)*sin(theta-pi)+m*l*pow(thetadot,2)*sin(theta-pi);
		}

// Plant State Update according to the control strategy chosen
//
		
		f1x = (u*cos(theta)-(M+m)*g*sin(theta)+m*l*cos(theta)*sin(theta)*pow(thetadot,2))/(m*l*(pow(cos(theta),2))-(M+m)*l);
		f2x = (u+m*l*sin(theta)*pow(thetadot,2)-m*g*cos(theta)*sin(theta))/(M+m-m*pow(cos(theta),2));

		delta_x[0] = thetadot;
		delta_x[1] = f1x;
		delta_x[2] = vel;
		delta_x[3] = f2x;

// State update
//
		x[0][i] = x[0][i-1]+delta_x[0]*ts;
		x[1][i] = x[1][i-1]+delta_x[1]*ts;
		x[2][i] = x[2][i-1]+delta_x[2]*ts;
		x[3][i] = x[3][i-1]+delta_x[3]*ts;
	}

// Write Data to output file
//
	for(i=0;i<timepts;i++){
		fprintf(fp, "%f %f %f %f %f\n", t[i], x[0][i], x[1][i], x[2][i], x[3][i]);
	}
	fclose(fp);
	return 0;
} 


