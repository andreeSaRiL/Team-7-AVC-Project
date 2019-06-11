#include <iostream>
#include <math.h>
#include <sys/time.h>
#include "E101.h"

using namespace std;

/*-----300506397-----*/

class Avc{ //declarations
	private:
		const int cam_width = 320;  //screen width
		const int cam_height = 240; //screen height
		const int v_left_go = 4;   //default left motor speed
		const int v_right_go = 4;  //default right motor speed
		int v_left, v_right, cam_tilt; //left/right motor speed, camera angle
		int quadrant;
		int dv;
		int last_error = 0; //for quadrants 2 and 3
		int whiteness[320];
		
	public:
	    Avc(){}; //default constructor
	    /*-----Functions-----*/
	    void InitHardware();  //initialise hardware
	    void OpenGate();      //open the gate
	    void ReadSetMotors(); //get the motor speed
	    void SetMotors();     //set the motor speed
	    bool ReadLine();
	    bool CheckLeft();
	    bool CheckRight();
	    bool CheckStraight();
	    void TurnLeft();
	    void TurnRight();
	    void DoLineAction();    //create int array of line position on screen
	    void FollowLine(int err);     //make the robot follow the line
};

void Avc::InitHardware(){
	int err;
	err = init(0);
}

void Avc::OpenGate(){
	//Put quadrant 1 code into here
}

void Avc::ReadSetMotors(){
	dv = v_left - v_right;
	cout<<"The robot's speed is "<<dv<<endl;
}

void Avc::SetMotors(){
	set_motors(1, 47 + v_left);
	set_motors(5, 47 - v_right);
	cout<<"Left motor speed: "<<v_left<<endl;
	cout<<"Right motor speed: "<<v_left<<endl;
	hardware_exchange();
}

bool Avc::ReadLine(){
	int avg = 0;
	
	for(int col = 0; col < cam_width; col++){
		whiteness[col] = get_pixel((cam_height/2), col, 3); //rewrites whiteness[] to the current centre line
		avg += whiteness[col];//get_pixel((cam_height/2), col, 3);
	}
	
	avg /= cam_width;  //average whiteness across the centre line
	
	int darkCount = 0;
	
	for(int i = 0; i < cam_width; i++){
		if(whiteness[i] < (avg * 0.4)){
			whiteness[i] = 1; //this is a dark pixel, set value to 1
			darkCount++;
		}
		else{
			whiteness[i] = 0; //this is a light pixel, set value to 0
		}
	}
	if(darkCount > (cam_width * 0.05)){
		cout<<"Dark pixels: "<<darkCount<<"/"<<cam_width<<endl;
		return true;  //line is present
	}
	else{
		return false; //line is not present
	}
}

void Avc::FollowLine(int err){
	int errorCheck[cam_width];
	int err = 0;
	for(int i = 0; i < cam_width; i++){
		errorCheck[i] = i;              //initialise
		errorCheck[i] -= (cam_width/2); //create values
		errorCheck[i] *= whiteness[i];  //value either exists or is 0
				
		err += errorCheck[i]; //total error
		cout<<"Error value: "<<err<<endl;
		
		double kp = 0.00;//1; //Kp (sensitivity)
		double kd = 0.0;//1; //Kd for further adjustment
		double dif = (err - last_error) / 0.3; //de/dt, need to implement time function to get dt
		double adj = (kp * err) + (kd * dif);
		cout<<"Adjusted: "<<adj<<endl;
		//adjustment = (Kp * error) + (Kd * de/dt)
		
		if(err < 0){
			v_left = v_left_go + adj;   //speed left motor
			v_right = v_right_go - adj; //slow right motor
		}
		
		else if(err > 0){
			v_left = v_left_go - adj;   //slow left motor
			v_right = v_right_go + adj; //speed right motor
		}
		
		dv = v_left - v_right;
		last_error = err; //set error for next time
		
		SetMotors();
	}
}

bool Avc::CheckLeft(){
	int leftAvg = 0;
	
	for(int i = 0; i < cam_width/3; i++){
		leftAvg += whiteness[i];
	}
	
	leftAvg /= cam_width/3;
	
	if(leftAvg <= 50){
		return true;
	}
	
	return false;
}

bool Avc::CheckRight(){
	int rightAvg = 0;
	
	for(int i = cam_width * (2/3); i < cam_width; i++){
		rightAvg += whiteness[i];
	}
	
	rightAvg /= cam_width/3;
	
	if(rightAvg <= 50){
		return true;
	}
	
	return false;
}

bool Avc::CheckStraight(){
	int straightAvg = 0;
	int vertWhiteness[cam_height/2];
	
	for(int row = cam_height/2; row < cam_height; row++){
		vertWhiteness[i] = get_pixel(row, cam_width / 2, 3);
		straightAvg += vertWhiteness[i];
	}
	
	straightAvg /= cam_height/2;
	
	if(straightAvg <= 50){
		return true;
	}
	
	return false;
}

void Avc::TurnLeft(){
	while(!(ReadLine())){ //while line is not present, turn left
		v_left = v_left_go + 2;
		v_right = v_right_go - 2;
		SetMotors();
	}
	//Reset motor speeds
	v_left = v_left_go;
	v_right = v_right_go;
}

void Avc::TurnRight(){
	while(!(ReadLine())){ //while line is not present, turn right
		v_left = v_left_go - 2;
		v_right = v_right_go + 2;
		SetMotors();
	}
	//Reset motor speeds
	v_left = v_left_go;
	v_right = v_right_go;
}

void Avc::DoLineAction(){
	if(quadrant == 2){
		bool line_present = ReadLine();
		if(line_present){
			FollowLine();
		}
		else{
			if(quadrant == 2){
				quadrant = 3;
			}
		}
	}
	else if(quadrant == 3){
		bool lineLeft = CheckLeft();
		bool lineRight = CheckRight();
		bool lineStraight = CheckStraight();
		
		int dir = 0; //last direction turned. straight = 0, left = 1, right = 2
		bool deadEnd = false;
		
		if(!(lineLeft) && !(lineRight) && !(lineStraight)){   //line not present
			TurnLeft();
			deadEnd = true;
		}
		
		else if(!(lineLeft) && !(lineRight) && lineStraight){ //line straight only, do quadrant 2 code
			ReadLine();
			FollowLine();
			dir = 0;
		}
		
		else if(lineLeft && !(lineStraight) && !(lineRight)){ //line turns left
			TurnLeft();
			dir = 1;
		}
		
		else if(!(lineLeft) && !(lineStraight) && lineRight){ //line turns right
			TurnRight();
			dir = 2;
		}
		
		else if(lineLeft && lineRight && !(lineStraight)){ //intersection
			if(dir = 1 && deadEnd){ //If there was a dead end
				TurnLeft(); //Turn left so as to not backtrack
				deadEnd = false;
			}
			if(dir = 2 && deadEnd){ //If there was a dead end
				TurnRight(); //Turn right so as to not backtrack
				deadEnd = false;
			}
		}
		
		
		/*Stops the robot
		cout<<"No line present"<<endl;
		v_left -= 2;
		v_right += 2;
		SetMotors();*/
	}
}

int main(){   
    Avc Seven;
    Seven.InitHardware();
    
    open_screen_stream();
    /*-----Main body of code-----*/
    Seven.quadrant = 1;
    Seven.OpenGate(); //Quadrant 1
    Seven.quadrant = 2;
    int duration = 0;
    while(duration < 10000){
		take_picture();
		update_screen();
		Seven.DoLineAction(); //Measure the line   
		cout<<duration<<endl;
		duration++;
	}
    /*-----Once the program is finished:-----*/
    close_screen_stream();
    stoph();
	
	return 0;
}
