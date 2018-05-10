#include "teclado.h"

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "manual_control");
  QApplication app(argc, argv);  
    
  KeyPress window;
  
  window.resize(80, 80);
  window.setWindowTitle("Manual_control");
  window.show();
  
  return app.exec();
}
