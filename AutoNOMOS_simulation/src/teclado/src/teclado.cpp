//teclado.cpp
#include <QKeyEvent>
#include "teclado.h"

KeyPress::KeyPress(QWidget *parent): 
		nh_("~"), QWidget(parent)
{
	ROS_INFO("Init node");
	velocity_ = nh_.advertise<std_msgs::Float32>("/AutoNOMOS_mini/manual_control/velocity", 3);
	steering_ = nh_.advertise<std_msgs::Float32>("/AutoNOMOS_mini/manual_control/steering", 3);	
	
        velocity.data = 0.0;
        steering.data = 0.0;
}

KeyPress::~KeyPress()
{

}

void KeyPress::keyPressEvent(QKeyEvent *event) 
{

	if(event->key() == Qt::Key_Up || event->key() == Qt::Key_W)
	{	
		velocity.data += 5.0;
	}
	else if(event->key() == Qt::Key_Down || event->key() == Qt::Key_S)
	{	
		velocity.data -= 5.0;
		
	}
	else if(event->key() == Qt::Key_Left || event->key() == Qt::Key_A)
	{	
		steering.data += 10.0;
	}
	else if(event->key() == Qt::Key_Right || event->key() == Qt::Key_D)
	{	
		steering.data -= 10.0;
	}	
	
        if(event->key() == Qt::Key_Space) 
	{
		velocity.data = 0.0;
                steering.data = 0.0;
	}
        
	if(event->key() == Qt::Key_Escape) 
	{
		qApp->quit();
	}
	
	velocity_.publish(velocity);
        steering_.publish(steering);
	
}

