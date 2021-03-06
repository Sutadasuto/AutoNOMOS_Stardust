#ifndef TECLADO_H 
#define TECLADO_H

#include <ros/ros.h>
#include <ros/package.h>

#include "std_msgs/Float32.h"

#include <QWidget>
#include <QtGui>

#include <QKeyEvent>

using namespace std;

class KeyPress: public QWidget 
{

  public:
    
    std_msgs::Float32 velocity;
    std_msgs::Float32 steering;

  public:
    KeyPress(QWidget *parent = 0);//constructor
    ~KeyPress();//destructor
	
  protected:
    void keyPressEvent(QKeyEvent * e);
  
  private:
  
  	ros::NodeHandle nh_;
  	ros::Publisher velocity_;
	ros::Publisher steering_;	
};

#endif
