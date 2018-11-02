#include "keyboard.h"

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "move_cylinder");
  QApplication app(argc, argv);  
    
  KeyPress window;
  
  window.resize(80, 80);
  window.setWindowTitle("Move_cylinder");
  window.show();
  
  return app.exec();
}
