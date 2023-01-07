#include <iostream>
#include <uWS/uWS.h>
#include <math.h>
#include <vector>
#include "json.hpp"

using namespace std;
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

string hasData(string s) {
	auto found_null = s.find("null");
  	auto b1 = s.find_first_of("{");
  	auto b2 = s.find_first_of("}");
  	if (found_null != string::npos) {
    	return "";
  	} 
  	else if (b1 != string::npos && b2 != string::npos) {
    	return s.substr(b1, b2 - b1 + 1);
  	}
  	return "";
}


double angle_between_points(double x1, double y1, double x2, double y2){
	return atan2(y2-y1, x2-x1);
}

void path_planning(vector<double>& x_points, vector<double>& y_points, double yaw, double gap, double radius, int max_points = 200){

	while( x_points.size() < max_points){
		double point_x = x_points[x_points.size()-1];
		double point_y = y_points[x_points.size()-1];

		if(radius != 0){
			if(x_points.size() > 1){
				yaw = angle_between_points(x_points[x_points.size()-2], y_points[y_points.size()-2], x_points[x_points.size()-1], y_points[y_points.size()-1]);
			}
			double dt = gap / abs(radius);
			double x = abs(radius) * cos(-pi()/2 + dt);
			double y = radius * sin(-pi()/2 + dt) + radius;
			point_x += x * cos(yaw) - y * sin(yaw);
			point_y += y * cos(yaw) + x * sin(yaw);
		}
		else{
			point_x += gap;
		}
		//print('x: ',point_x)
		//print('y: ',point_y)
		x_points.push_back(point_x);
		y_points.push_back(point_y);
	}
}

int main ()
{
	cout << "starting server" << endl;
	uWS::Hub h;
	
		//sio.emit('path', {'trajectory': {'x': x_points, 'y': y_points}})

	h.onMessage([](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
	{
		
      	auto s = hasData(data);
      	//cout << "string " << s << endl;

      	if (s != "") {
        	auto data = json::parse(s);

        	vector<double> x_points = data["way_points_x"];
        	vector<double> y_points = data["way_points_y"];
        	double yaw = data["yaw"];
        	double sim_time = data["time"];

        	if( (int)(sim_time/2)%2 == 0){
        		path_planning(x_points, y_points, yaw, 0.15, -200);
        	}
        	else{
        		path_planning(x_points, y_points, yaw, 0.15, 70);
        	}

			json msgJson;
      		msgJson["trajectory_x"] = x_points;
        	msgJson["trajectory_y"] = y_points;

        	auto msg = msgJson.dump();
	
			ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);	
			
		}

	});
	
	
	h.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) 
	{
    	cout << "Connected!!!" << endl;
  	});

	
  	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) 
  	{
    	ws.close();
    	cout << "Disconnected" << endl;
  	});

  	int port = 4567;
  	if (h.listen("0.0.0.0", port))
  	{
    	cout << "Listening to port " << port << endl;
    	h.run();
  	} 
  	else 
  	{
    	cerr << "Failed to listen to port" << endl;
    	return -1;
  	}
  	

}