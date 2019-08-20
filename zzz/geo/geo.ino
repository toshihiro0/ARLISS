#include <math.h>
#include <EEPROM.h>

#define goal_latitude 35.670357
#define goal_longtitude 139.775163
#define goal_altitude 5
#define difference_lat 111316.2056
static const float difference_lon = cos(goal_latitude/180*M_PI)*M_PI*6378.137/180*1000;

void setup()
{
    Serial.begin(57600);
}
void loop()
{
    unsigned long time1;
    unsigned long time2;
    float lat = 35.672972;
    float lon = 139.776284;
    float alt = 5.0;
    time1 = micros();
    float distance = detect_waypoint(lat,lon,alt);
    time2 = micros();
    Serial.println(time2-time1);
    time1 = micros();
    EEPROM.put(0,distance);
    time2 = micros();
    Serial.println(time2-time1);
    while(true){}
}

float detect_waypoint(float latitude,float longtitude,float altitude)
{
    float distance = sqrt((latitude-goal_latitude)*(latitude-goal_latitude)*difference_lat*difference_lat+(longtitude-goal_longtitude)*(longtitude-goal_longtitude)*difference_lon*difference_lon);
    Serial.println(distance);
    if((altitude-goal_altitude) < 20.0 && distance < 20.0){
        return distance;
    }else{
        return distance;
    }
}
