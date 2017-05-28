//
//  main.c
//  invertedUnlinear
//
//  Created by Ce Wei on 10/22/15.
//  Copyright © 2015 Ce Wei. All rights reserved.
//

#include <stdio.h>
#include <math.h>
//double log(double x)
//double exp(double x)
/*
struct comNumber{
    float real;
    float complex;
};
*/
int main() {
    // insert code here...
    FILE *Fp;
    Fp = fopen("data.dat", "w");
    
    //parameters of an inverted pendulum system
    float M = 2;
    float m = 0.2;
    float l = 0.31;
    float g = 9.81;
    float cart_len = 0.05;
    //other parameters
    float ts = 0.00125;
    float duration = 30;
    
    int i = 0;
    //xline and yline are the vectors from 0 to 0.31 with grid 0.001
    float xline[311], yline[311];
    xline[0] = 0;
    yline[0] = 0;
    for(i=1;i<311;i++){
        //from 0 to 310
        xline[i] = xline[i-1]+0.001;
        yline[i] = yline[i-1]+0.001;
    }
    //xcar1 is a vector from -cart_len(-0.05) to cart_len(0.05) with grid of 0.01
    float xcart1[11];
    xcart1[5] = 0;
    for(i=6;i<11;i++){
        xcart1[i] = xcart1[i-1]+0.01;
    }
    for(i=4;i>=0;i--){
        xcart1[i] = xcart1[i+1]-0.01;
    }
    
    //the duration this time becomes to 30 seconds
    float t[24001];
    t[0] = 0;
    for(i=1;i<24001;i++){
        t[i] = t[i-1] + 0.00125;
    }
    //points count for the t
    float timepts = 24001;
    //error
    float error[24001];
    //the error for each--angle, angular velocity...
    //float error_m[4][24001];
    //the state vectors
    float x[4][24001];
    //initial angle
    float pi = 3.1415;
    float theta_0 = pi;
    x[0][0] = 3.1415;
    float xd[24001];
    float control[24001];
    float K[4] = { -280.9615  -33.4584 -250.1768  -69.5026};
    //nonlinear controller parameters
    float wn = 5.6254;
    float gw = 0.4612;
    float phi = -1.4831;
    float f1 = 39.0681;
    float f2 = 15.0011;
    float Vd = 1.2164;
    float a0 = 1.3;
    float b0 = 0.833;
    int nonlinear = 1;
    int linear = 0;
    //timepts is 24001, simulate from 0 to 24001, time is 30 seconds
    float theta=0;
    float thetadot=0;
    float disp=0;
    float vel=0;
    float u=0;
    float psi=0;
    float V=0;
    float a=0;
    float f1x=0;
    float f2x=0;
    float delta_x[4];
	int flag = 1;
	x[0][0] = 3.1415;
	    
    for(i=1;i<timepts;i++){
        theta = x[0][i-1];
        thetadot = x[1][i-1];
        disp = x[2][i-1];
        vel = x[3][i-1];
        //set the u
        if ((fabs(theta)<(40*3.1415/180))||(linear==1)){
            u = -(K[0]*theta+K[1]*thetadot+K[2]*disp+K[3]*vel);
	    	//if(i<10000){
		//printf("%d,%f\n", i, theta);
		//printf("%f\n", x[0][i-1]);
		//}
	    //linear = 1;
	    //nonlinear = 0;
        }else if(nonlinear==1){
            psi = -1*atan((thetadot/wn)/theta-pi);
            V = 0.5*m*l*l*thetadot*thetadot+m*g*l*(1-cos(theta-pi));
            if (fabs(V-Vd) >= b0){
                if(V<Vd){
                    a = -a0;
                }else if(V==Vd){
                    a = 0;
                }else{
                    a = a0;
                }
            }
            else if (fabs(V-Vd) < b0){
                a = a0*(V-Vd)/b0;
            }
            xd[i] = (a/gw)*sin(psi-pi+phi);
            u = (M+m*(sin(theta-pi)*sin(theta-pi)))*(f1*(xd[i]-disp)-f2*vel)+m*g*cos(theta-pi)*sin(theta-pi)+m*l*thetadot*thetadot*sin(theta-pi);
        }
        control[i] = u;
        //compute functions of the original nonlinear state equation
        f1x = (u*cos(theta)-(M+m)*g*sin(theta)+m*l*cos(theta)*sin(theta)*thetadot*thetadot)/(m*l*(cos(theta)*cos(theta))-(M+m)*l);
        f2x = (u+m*l*sin(theta)*thetadot*thetadot-m*g*cos(theta)*sin(theta))/(M+m-m*(cos(theta)*cos(theta)));
        
        //Find out the update in the states and update accordingly
        delta_x[0] = thetadot;
        delta_x[1] = f1x;
        delta_x[2] = vel;
        delta_x[3] = f2x;
        x[0][i] = x[0][i-1] + delta_x[0]*ts;
        x[1][i] = x[1][i-1] + delta_x[1]*ts;
        x[2][i] = x[2][i-1] + delta_x[2]*ts;
        x[3][i] = x[3][i-1] + delta_x[3]*ts;
    }//end of for loop, simulation done
    for(i=0;i<24001;i++){
        fprintf(Fp, "%f %f %f %f %f\n", t[i], x[0][i], x[1][i], x[2][i], x[3][i]);
    }
    fclose(Fp);
    return 0;
}


