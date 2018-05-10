#include <QKeyEvent>
#include "teclado.h"

KeyPress::KeyPress(QWidget *parent): 
		nh_("~"), QWidget(parent)
{
	ROS_INFO("Init node");
	key_ = nh_.advertise<std_msgs::Int8>("/cylinder/keyboard", 3);	        	
}

KeyPress::~KeyPress()
{

}

void KeyPress::keyPressEvent(QKeyEvent *event) 
{

        //ROS_INFO_STREAM("Tecla recibida");
        
	if(event->key() == Qt::Key_Up || event->key() == Qt::Key_W)
	{	
		key.data = 1;
                key_.publish(key);
	}
        else if(event->key() == Qt::Key_Right || event->key() == Qt::Key_D)
	{	
		key.data = 2;
                key_.publish(key);
	}	
	else if(event->key() == Qt::Key_Down || event->key() == Qt::Key_S)
	{	
		key.data = 3;
                key_.publish(key);
		
	}
	else if(event->key() == Qt::Key_Left || event->key() == Qt::Key_A)
	{	
		key.data = 4;
                key_.publish(key);
	}
	
        
	if(event->key() == Qt::Key_Escape) 
	{
		qApp->quit();
	}
	
}

