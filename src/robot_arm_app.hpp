#pragma once

#include <eigen3/Eigen/Dense>
#include "ofMain.h"
#include "ofxGui.h"

template <int rows>
using Vector = Eigen::Matrix<float, rows, 1>;
template <int rows, int cols>
using Matrix = Eigen::Matrix<float, rows, cols>;

class RobotArm {
   public:
    RobotArm(const std::array<float, 3>& lengths) {
        for (auto i = 0; i < 3; ++i) {
            joints[i].length = lengths[i];
        }
    }

    void update() {
        for (auto& joint : joints) {
            joint.update();
        }
    }

    void draw() {
        for (const auto& joint : joints) {
            ofRotateRad(joint.angle);
            ofTranslate(joint.length / 2, 0, 0);
            ofSetColor(0, 0, 0, 63);
            ofFill();
            ofDrawBox(joint.length, 20, 20);
            ofSetColor(0, 0, 0, 255);
            ofNoFill();
            ofDrawBox(joint.length, 20, 20);
            ofTranslate(joint.length / 2, 0, 0);
        }
    }

    Vector<3> function(const Vector<3>& q) {
        Vector<3> p;

        p << joints[0].length * cos(q(0)) +
                 joints[1].length * cos(q(0) + q(1)) +
                 joints[2].length * cos(q(0) + q(1) + q(2)),
            joints[0].length * sin(q(0)) + joints[1].length * sin(q(0) + q(1)) +
                joints[2].length * sin(q(0) + q(1) + q(2)),
            q(0) + q(1) + q(2);

        return p;
    }

    Matrix<3, 3> Jacobian(const Vector<3>& q) {
        Matrix<3, 3> J;

        J << joints[0].length * -sin(q(0)) +
                 joints[1].length * -sin(q(0) + q(1)) +
                 joints[2].length * -sin(q(0) + q(1) + q(2)),
            joints[1].length * -sin(q(0) + q(1)) +
                joints[2].length * -sin(q(0) + q(1) + q(2)),
            joints[2].length * -sin(q(0) + q(1) + q(2)),
            joints[0].length * +cos(q(0)) +
                joints[1].length * +cos(q(0) + q(1)) +
                joints[2].length * +cos(q(0) + q(1) + q(2)),
            joints[1].length * +cos(q(0) + q(1)) +
                joints[2].length * +cos(q(0) + q(1) + q(2)),
            joints[2].length * +cos(q(0) + q(1) + q(2)), 1, 1, 1;

        return J;
    }

    void solve(const Vector<3>& p, Vector<3> q) {
        Matrix<3, 3> W;
        W << 1, 0, 0, 0, 1, 0, 0, 0, 1;

        while (q.array().isFinite().all()) {
            Vector<3> e = p - function(q);

            if (e.norm() < 0.1) {
                for (auto i = 0; i < 3; ++i) {
                    joints[i].target = ofWrap(q(i), -PI, PI);
                }

                break;
            }

            Matrix<3, 3> J = Jacobian(q);

            Matrix<3, 3> H = (J.transpose() * W * J).inverse();
            Vector<3> g = J.transpose() * W * e;

            q += H * g;
        }
    }

    struct Joint {
        void update() {
            angle += (target > angle ? 1 : -1) *
                     std::min(speed, abs(target - angle));
        }

        float length;
        float angle = 0;
        float target = 0;
        float speed = 0.05;
    };

    std::array<Joint, 3> joints;
};

class RobotArmApp : public ofBaseApp {
   public:
    RobotArmApp() : robotArm({100, 100, 100}) {}

    void setup() override;
    void update() override;
    void draw() override;
    void fired();
    void orientationChanged(ofVec3f&);

    ofxPanel gui;
    ofxButton button;
    ofParameter<ofVec3f> position;
    ofParameter<ofVec3f> orientation;

    ofEasyCam camera;

    ofTrueTypeFont bigFont;
    ofTrueTypeFont smallFont;

    RobotArm robotArm;
};

void RobotArmApp::setup() {
    ofxGuiSetFont("helvetica", 8);
    gui.setup();
    gui.add(button.setup("solve"));
    gui.add(
        position.set("position", {0, 0, 0}, {-500, -500, -PI}, {500, 500, PI}));
    gui.add(orientation.set("orientation", {0, 0, 0}, {-PI, -PI, -PI},
                            {PI, PI, PI}));
    button.addListener(this, &RobotArmApp::fired);
    orientation.addListener(this, &RobotArmApp::orientationChanged);

    bigFont.load("helvetica", 36);
    smallFont.load("helvetica", 16);
}

void RobotArmApp::update() { robotArm.update(); }

void RobotArmApp::draw() {
    ofSetLineWidth(3);

    camera.begin();

    ofSetColor(0, 0, 0, 63);

    ofPushMatrix();

    ofRotateRad(HALF_PI);

    ofDrawGridPlane(10, 100);

    ofPopMatrix();

    ofSetColor(0, 0, 0, 255);

    ofDrawSphere(position->x, position->y, 10);

    robotArm.draw();

    camera.end();

    ofSetColor(0, 0, 0, 255);

    gui.draw();

    bigFont.drawString("Robot Arm", ofGetWidth() - 300, 80);
    smallFont.drawString("Gauss-Newton Method", ofGetWidth() - 300, 120);
}

void RobotArmApp::fired() {
    robotArm.solve({position->x, position->y, position->z},
                   {orientation->x, orientation->y, orientation->z});
}

void RobotArmApp::orientationChanged(ofVec3f& orientation) {
    robotArm.joints[0].angle = orientation.x;
    robotArm.joints[1].angle = orientation.y;
    robotArm.joints[2].angle = orientation.z;

    robotArm.joints[0].target = orientation.x;
    robotArm.joints[1].target = orientation.y;
    robotArm.joints[2].target = orientation.z;
}
