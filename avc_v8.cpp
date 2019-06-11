#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <sys/time.h>
#include "E101.h"

using namespace std;

int quadrant = 0;

class Avc{ //declarations
	private:
		const int cam_width = 320;  //screen width
		const int cam_height = 240; //screen height
		const int v_left_go = 8;    //default left motor speed
		const int v_right_go = 6;   //default right motor speed
		int v_left, v_right, cam_tilt; //left/right motor speed, camera angle
		int prevErr;
		int whiteness[320];
		bool debug = false;
		
	public:
	    Avc(){}; //default constructor
	    /*-----Functions-----*/
	    void InitHardware();  //initialise hardware
	    void OpenGate();      //open the gate
	    void SetMotors();     //set the motor speed
	    bool ReadLine();
	    void FollowLine();
	    void MoveBack();
	    bool CheckLeft();
	    bool CheckRight();
	    bool CheckStraight();
	    void TurnLeft();
	    void TurnRight();
	    void TurnStraight();
	    void DoLineAction();
	    void DoMazeAction();
	    void FollowLine(int err);     //make the robot follow the line
};

void Avc::InitHardware(){
	int err;
	err = init(0);
}

/*----------Quadrant 1----------*/
void Avc::OpenGate(){
	char server_addr[15]="130.195.6.196";
	int port = 1024;
	char message [24] = "Please";
	char messageR[24];

	if (connect_to_server(server_addr,port) == 0){
		send_to_server(message);
		receive_from_server(messageR);
		send_to_server(messageR);
	}
}

/*----------Adjust motor values----------*/
void Avc::SetMotors(){
	set_motors(1, 48 + v_left);
	set_motors(5, 47 - v_right);
	//set_motors(1, 52);
	//set_motors(5, 43);
	//cout<<"Left motor speed: "<<v_left<<endl;
	//cout<<"Right motor speed: "<<v_left<<endl;
	hardware_exchange();
}

/*----------Read position of line based on centre line of the camera----------*/
bool Avc::ReadLine(){
	int avg = 0;
	
	for(int col = 0; col < cam_width; col++){
		for(int row = 0; row < cam_height; row++){
			avg += get_pixel(row, col, 3);
		} 
	}
	
	avg /= (cam_width * cam_height);  //average whiteness across the whole image
	
	for(int col = 0; col < cam_width; col++){
		//whiteness[col] = get_pixel((cam_height * (2/3)), col, 3); //rewrites whiteness[] to the current centre line
		whiteness[col] = get_pixel((cam_height * (1 / 2)), col, 3);
		//avg += whiteness[col];//get_pixel((cam_height/2), col, 3);
	}
	
	int darkCount = 0;
	
	for(int i = 0; i < cam_width; i++){
		if(whiteness[i] < (avg * 0.6)){
			whiteness[i] = 1; //this is a dark pixel, set value to 1
			darkCount++;
		}
		else{
			whiteness[i] = 0; //this is a light pixel, set value to 0
		}
	}
	//cout<<"Dark pixels: "<<darkCount<<"/"<<cam_width * 0.07<<endl;
	if(darkCount > (cam_width * 0.05)){
		if(debug){
			cout<<"Above threshold"<<endl;
			
		}
		return true;  //line is present
	}
	else{
		return false; //line is not present
	}
}

/*----------Make adjustments to the motors based on line position----------*/
void Avc::FollowLine(){
	int errorCheck[cam_width];
	double err = 0;
	for(int i = 0; i < cam_width; i++){
		errorCheck[i] = (i - cam_width/2);
		errorCheck[i] *= whiteness[i];  //value either exists or is 0
				
		err += errorCheck[i]; //total error
	}
	
	double Kp = 2000;
	int correction = err / Kp; // /= Kp // Initial correction to motors based on error
	// change of about 3 to motors at 10,000 max error (-3 at -10,000 error)
	
	/*if(((err / Kp) % 1 > 0.5) && err > 0){ // Rounding if necessary
		correction += 1;
	}
	else if(((err / (0 - Kp)) % 1 > 0.5) && err < 0){
		correction -= 1;
	}*/
	
	double Kd = 1000;
	double errDiff = err - prevErr;
	int align = errDiff / Kd;  // /= Kd // Additional adjustment proportional to change in error
	//change of about 1 to motors at 2,000 change in error
	
	/*if(((errDiff / Kd) % 1 > 0.5) && errDiff > 0){ //Rounding if necessary
		align += 1;
	}
	else if(((errDiff / (0 - Kd)) % 1 > 0.5) && errDiff < 0){
		align -= 1;
	}*/
	
	int adjustment = correction + align;  // total adjustment
	
	v_left = v_left_go + adjustment;
	v_right = v_right_go - adjustment;
	prevErr = err;
	
	if(debug){
		cout<<"Total error: "<<err<<endl;   //typically ranges from -10,000 to +10,000
		cout<<"Adjustment: "<<correction<<" initial, plus an additional "<<align<<endl;
		if(v_left > v_right){
		cout<<"The robot is turning left"<<endl;
		}
		else if(v_left < v_right){
			cout<<"The robot is turning right"<<endl;
		}
		else if(v_left == v_right){
			cout<<"The robot is travelling straight"<<endl;
		}
	}
	
	SetMotors();
	//reset values
	//v_left = v_left_go;
	//v_right = v_right_go;
}

void Avc::MoveBack(){
	//while(!ReadLine()){
	v_left = 0;
	v_right = 0;
	SetMotors();
	if(prevErr > 0){
	    v_left = 4;
	    v_right = -4;
	}
	else if(prevErr <= 0){
	    v_left = -4;
	    v_right = 4;
	}
	SetMotors();
	//}
	//v_left = 0;
	//v_right = 0;
	//SetMotors();
}

/*----------Quadrant 2----------*/
void Avc::DoLineAction(){
	//if(CheckRight() && !(CheckLeft()) && !(CheckStraight())){
	//	quadrant = 3;
	//	return;
	//}
	if(ReadLine()){
		FollowLine();
	}
	else{
		MoveBack();
	}
}

/*----------Check for straight lines to the left----------*/
bool Avc::CheckLeft(){
	int leftAvg = 0;
	
	for(int i = 0; i < cam_width/3; i++){
		leftAvg += whiteness[i];
	}
	
	leftAvg /= cam_width/3; //average brightness of the row of pixels 2/3 of the way down the image between 0 and 1/3 of the way from the left
	
	int avg = 0;
	
	for(int col = 0; col < cam_width; col++){
		for(int row = 0; row < cam_height; row++){
			avg += get_pixel(row, col, 3);
		} 
	}
	
	avg /= (cam_width * cam_height);  //average whiteness across the whole image
	
	if(leftAvg <= avg * 0.4){
		if(debug){
			cout<<"Line detected to the left"<<endl;
		}
		return true;
	}
	if(debug){
		cout<<"No line detected to the left"<<endl;
	}
	return false;
}

/*----------Check for straight lines to the right----------*/
bool Avc::CheckRight(){
	int rightAvg = 0;
	
	for(int i = cam_width * (2/3); i < cam_width; i++){
		rightAvg += whiteness[i];
	}
	
	rightAvg /= cam_width/3; //average brightness of the row of pixels 2/3 of the way down the image between 2/3 and 1 of the way from the left
	
	int avg = 0;
	
	for(int col = 0; col < cam_width; col++){
		for(int row = 0; row < cam_height; row++){
			avg += get_pixel(row, col, 3);
		} 
	}
	
	avg /= (cam_width * cam_height);  //average whiteness across the whole image
	
	if(rightAvg <= avg * 0.4){
		if(debug){
			cout<<"Line detected to the right"<<endl;
		}
		return true;
	}
	if(debug){
		cout<<"No line detected to the right"<<endl;
	}
	return false;
}

/*----------Check for straight lines ahead----------*/
bool Avc::CheckStraight(){
	int straightAvg = 0;
	int vertWhiteness[cam_height * (2/3)];
	
	for(int row = cam_height * (2/3); row < cam_height; row++){
		vertWhiteness[row] = get_pixel(row, cam_width / 2, 3); //get whiteness for pixels across the vertical centre line
		straightAvg += vertWhiteness[row];
	}
	
	straightAvg /= cam_height/2; //average brightness of the column of pixels 2/3 to 1 down the image along the vertical centre line
	
	int avg = 0;
	
	for(int col = 0; col < cam_width; col++){
		for(int row = 0; row < cam_height; row++){
			avg += get_pixel(row, col, 3);
		} 
	}
	
	avg /= (cam_width * cam_height);  //average whiteness across the whole image
	
	if(straightAvg <= avg * 0.4){
		if(debug){
			cout<<"Line detected ahead"<<endl;
		}
		return true;
	}
	if(debug){
		cout<<"No line detected ahead"<<endl;
	}
	return false;
}

/*----------Turn left until there is a line detected in front----------*/
void Avc::TurnLeft(){
	while(CheckStraight()){ //turn left until there is no line present
		v_left = v_left_go;
		v_right = 0 - (v_right_go);
		SetMotors();
	}
	while(!(CheckStraight())){ //while line is not present, turn left until there is one present
		v_left = v_left_go;
		v_right = 0 - (v_right_go);
		SetMotors();
	}
	//Reset motor speeds
	v_left = v_left_go;
	v_right = v_right_go;
}

/*----------Turn right until there is a line detected in front----------*/
void Avc::TurnRight(){
	while(CheckStraight()){ //turn right until there is no line present
		v_left = 0 - (v_left_go);
		v_right = v_right_go;
		SetMotors();
	}
	while(!(CheckStraight())){ //while line is not present, turn right until there is one present
		v_left = 0 - (v_left_go);
		v_right = v_right_go;
		SetMotors();
	}
	//Reset motor speeds
	v_left = v_left_go;
	v_right = v_right_go;
}

/*----------Move straight until there are no lines to the left or right----------*/
void Avc::TurnStraight(){
	bool l = CheckLeft();
	bool r = CheckRight();
	while(l || r){
		v_left = v_left_go;
		v_right = v_right_go;
		l = CheckLeft();
		r = CheckRight();
	}
}


/*----------Quadrant 3----------*/
void Avc::DoMazeAction(){
	
	//v_left_go /= 2;
	//v_right_go /= 2;
	
	bool lineLeft = CheckLeft();         //Is there a line to the left?
	bool lineRight = CheckRight();       //Is there a line to the right?
	bool lineStraight = CheckStraight(); //Is there a line in front?
	
	int dir = 0;              //The value of the last turn direction. straight = 0, left = 1, right = 2
	bool deadEnd = false;     //Was there a dead end after the last turn?
	
	int lastDir = -1;         //The value of the turn direction before last, used in 4 way intersections.
	bool lastDeadEnd = false; //Second dead end check for 4 way intersections
	
	//maybe i should have used switch statements instead of a million else/if's but whatever :)
	
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
				TurnStraight();
				deadEnd = false;
			}
		}
		else{
			dir = (rand()%2); //random integer between 0 and 1 (straight/left)
			if(dir == 0){
				TurnStraight();
			}
			else if(dir == 1){
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
				TurnStraight();
				deadEnd = false;
			}
		}
		else{
			dir = (rand()%2); //random integer between 0 and 1
			if(dir == 0){ //either go straight
				TurnStraight();
			}
			else if(dir == 1){
				dir = 2; //or turn right
				TurnRight();
			}
		}
	}
	
	else if(lineLeft && lineRight && lineStraight){//4 way intersection aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
		if(deadEnd && lastDeadEnd){ //If the robot has hit 2 dead ends:
			if(lastDir == 0 && dir == 1){      //If the robot went straight, hit a dead end, then went left and hit another:
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
		else if(deadEnd && !(lastDeadEnd)){ //If the robot has hit 1 dead end:
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
				TurnStraight();
			}
			else if(lastDir == 2 && dir == 1){ //right
				dir = 2;
				TurnRight();
			}
		}
		else{ //If the robot has not hit any dead ends:
			dir = (rand()%3); //random integer between 0 and 2
			//Turn the robot in a random direction, straight, left or right and save this direction
			if(dir == 0){
				TurnStraight();
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

int main(){   
    Avc Seven;
    Seven.InitHardware();
    open_screen_stream();
    /*-----Main body of code-----*/
    quadrant = 1;
    Seven.OpenGate();
    quadrant = 2;
    int duration = 0;
    while(duration < 10000){
		take_picture();
		update_screen();
		//if(quadrant == 1){
		//	Seven.OpenGate();
		//	quadrant++;
		//}
		//else if(quadrant == 2){
			Seven.DoLineAction();
		//}
		//else if(quadrant == 3){
		//	Seven.DoMazeAction();
		//}
		//cout<<"Duration: "<<duration<<endl;
		duration++;
	}
    /*-----Once the program is finished:-----*/
    close_screen_stream();
    stoph();
	
	return 0;
}
