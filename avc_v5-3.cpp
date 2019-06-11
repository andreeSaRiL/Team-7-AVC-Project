#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <sys/time.h>
#include "E101.h"

using namespace std;

/*-----300506397-----*/

class Avc{ //declarations
	private:
		const int cam_width = 320;  //screen width
		const int cam_height = 240; //screen height
		const int v_left_go = 6;   //default left motor speed
		const int v_right_go = 6;  //default right motor speed
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
	    void TurnStraight(bool lineLeft, bool lineRight);
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
	set_motors(1, 48 + v_left);
	set_motors(5, 48 - v_right);
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
	while(CheckStraight()){ //turn left until there is no line present
		v_left = v_left_go;
		v_right = 0 - v_right_go;
		SetMotors;
	}
	while(!(CheckStraight())){ //while line is not present, turn left until there is one present
		v_left = v_left_go;
		v_right = 0 - v_right_go;
		SetMotors();
	}
	//Reset motor speeds
	v_left = v_left_go;
	v_right = v_right_go;
}

void Avc::TurnRight(){
	while(CheckStraight()){ //turn right until there is no line present
		v_left = 0 - v_left_go;
		v_right = v_right_go;
		SetMotors;
	}
	while(!(CheckStraight())){ //while line is not present, turn right until there is one present
		v_left = 0 - v_left_go;
		v_right = v_right_go;
		SetMotors();
	}
	//Reset motor speeds
	v_left = v_left_go;
	v_right = v_right_go;
}

void Avc::TurnStraight(bool lineLeft, bool lineRight){
	while(lineLeft || lineRight){
		v_left = v_left_go;
		v_right = v_right_go;
		lineLeft = checkLeft();
		lineRight = checkRight();
	}
}

void Avc::DoLineAction(){
	if(quadrant == 2){
		if(ReadLine()){
			FollowLine();
		}
		else{
			if(quadrant == 2){
				quadrant = 3;
			}
		}
	}
	else if(quadrant == 3){
		bool lineLeft = CheckLeft();         //Is there a line to the left?
		bool lineRight = CheckRight();       //Is there a line to the right?
		bool lineStraight = CheckStraight(); //Is there a line in front?
		
		int dir = 0;              //The value of the last turn direction. straight = 0, left = 1, right = 2
		bool deadEnd = false;     //Was there a dead end after the last turn?
		
		int lastDir = -1;         //The value of the turn direction before last, used in 4 way intersections.
		bool lastDeadEnd = false; //Second dead end check for 4 way intersections
		
		if(!(lineLeft) && !(lineRight) && !(lineStraight)){   //If line is not present:
			deadEnd = true;
			TurnLeft(); //turn around
		}
		
		else if(!(lineLeft) && !(lineRight) && lineStraight){ //line straight only, do quadrant 2 code
			dir = 0;
			ReadLine();
			FollowLine();
		}
		
		else if(lineLeft && !(lineRight) && !(lineStraight)){ //line left only, left turn
			dir = 1;
			TurnLeft();
		}
		
		else if(!(lineLeft) && lineRight && !(lineStraight)){ //line right only, right turn
			dir = 2;
			TurnRight();
		}
		
		else if(lineLeft && lineRight && !(lineStraight)){ //left/right intersection
			if(deadEnd){ //If there was a dead end:
				if(dir == 1){ //If last turn was left:
					TurnLeft(); //Turn left so as to not backtrack
					deadEnd = false;
				}
				else if(dir == 2 && deadEnd){ //If last turn was right:
					TurnRight(); //Turn right so as to not backtrack
					deadEnd = false;
				}
			}
			else{ //If there wasn't a dead end:
				dir = (rand()%2) + 1; //random integer between 1 and 2 (left/right)
				if(dir == 1){ //If last turn was left:
					TurnLeft();
				}
				else if(dir == 2){
					TurnRight();
				}
			}
		}
		
		else if(lineLeft && !(lineRight) && lineStraight){ //left/straight intersection
			if(deadEnd){ //If there was a dead end:
				if(dir == 0 || dir == 1){ //If last turn was straight or left:
					dir = 1;
					TurnLeft(); //Turn left so as to not backtrack
					deadEnd = false;
				}
				else if(dir == 2){ //If last turn was right:
					dir = 0; //Go straight so as to not backtrack
					TurnStraight(lineLeft, lineRight);
					deadEnd = false;
				}
			}
			else{
				dir = (rand()%2); //random integer between 0 and 1 (straight/left)
				if(dir == 0){
					TurnStraight(lineLeft, lineRight);
				}
				else if(dir = 1){
					TurnLeft();
				}
			}
		}
		
		else if(!(lineLeft) && lineRight && lineStraight){ //right/straight intersection
			if(deadEnd){
				if(dir == 0 || dir == 2){ //If last turn was straight or right:
					dir = 2;
					TurnRight(); //Turn right so as to not backtrack
					deadEnd = false;
				}
				else if(dir == 1){ //If last turn was left:
					dir = 0; //Go straight so as to not backtrack
					TurnStraight(lineLeft, lineRight);
					deadEnd = false;
				}
			}
			else{
				dir = (rand()%2); //random integer between 0 and 1
				if(dir == 0){ //either go straight
					TurnStraight(lineLeft, lineRight);
				}
				else if(dir == 1){
					dir = 2; //or turn right
					TurnRight();
				}
			}
		}
		
		else if(lineLeft && lineRight && lineStraight){//4 way intersection aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
			if(deadEnd && lastDeadEnd){ //If the robot has hit 2 dead ends:
				if(lastDir == 0 && dir == 1){ //If the robot went straight, hit a dead end, then went left and hit another:
					dir = 0;      
					TurnStraight(); //go straight
				}
				else if(lastDir == 0 && dir == 2){ //If the robot went straight, hit a dead end, then went right and hit another:
					dir = 0;      
					TurnStraight(); //go straight
				}
				else if(lastDir == 1 && dir == 0){ //If the robot went left, hit a dead end, then went straight and hit another:
					dir = 2;
					TurnRight(); //turn right
				}
				else if(lastDir == 1 && dir == 1){ //If the robot went left, hit a dead end, then went left and hit another:
					dir = 1;
					TurnLeft(); //turn left
				}
				else if(lastDir == 2 && dir == 0){
					dir = 1;
					TurnLeft(); //turn left
				}
				else if(lastDir == 2 && dir == 2){
					dir = 2;
					TurnRight();
				}
				//Reset necessary values, no longer at a 4 way intersection
				lastDir = -1;
				lastDeadEnd = false;
				deadEnd = false;
			}
			else if(deadEnd && !(lastDeadEnd){ //If the robot has hit 1 dead end:
				lastDir = dir;      // Save last turn direction
				lastDeadEnd = true; // The robot has hit one dead end
				dir = (rand()%2);   // reroll turn direction, random integer between 0 and 1
				//If the robot last went straight and hit a dead end, turn either left or right
				if(lastDir == 0 && dir == 0){      //left
					dir = 1;
					TurnLeft();
				}
				else if(lastDir == 0 && dir == 1){ //right
					dir = 2;
					TurnRight();
				}
				//If the robot last turned left and hit a dead end, turn either straight or left
				else if(lastDir == 1 && dir == 0){ //straight
					dir = 0;
					TurnStraight();
				}
				else if(lastDir == 1 && dir == 1){ //left
					dir = 1;
					TurnLeft();
				}
				//If the robot last turned right and hit a dead end, turn either straight or right
				else if(lastDir == 2 && dir == 0){ //straight
					dir = 0;
					TurnStraight;
				}
				else if(lastDir == 2 && dir == 1){ //right
					dir = 2;
					TurnRight;
				}
			}
			else{ //If the robot has not hit any dead ends:
				dir = (rand()%3); //random integer between 0 and 2
				//Turn the robot in a random direction, straight, left or right and save this direction
				if(dir == 0){
					TurnStraight(lineLeft, lineRight);
				}
				else if(dir == 1){
					TurnLeft();
				}
				else if(dir == 2){
					TurnRight();
				}
			}
		}
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
		Seven.DoLineAction(); //Do stuff
		cout<<duration<<endl;
		duration++;
	}
    /*-----Once the program is finished:-----*/
    close_screen_stream();
    stoph();
	
	return 0;
}
