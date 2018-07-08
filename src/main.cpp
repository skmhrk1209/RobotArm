#include "ofMain.h"
#include "robot_arm_app.hpp"

int main() {
    ofSetupOpenGL(1024, 768, OF_WINDOW);
    ofRunApp(new RobotArmApp());
}
