#include <stdio.h>
#include <GL/glew.h>
#include <GL/glut.h>
#include <GL/freeglut.h>
#include <math.h>
#include <stdbool.h>
#include "wav.h"

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>

float angleY = 0.0;  // Y軸周りの回転角度 今は固定
float heart_color[3] = {0.9, 0.0, 0.0}; //ハートの色 1.0にすると明るさが変わらなくなる

float heart_scale = 1.0;
float scale_direction = 0.05; //ハートの大きさが変化する速さ 変えると音源と合わなくなるかも
bool isScalingUp = true;  // 拡大中かどうか
bool isWaiting = false;   // 待機中かどうか
float waitTime = 0.0;
float pulse_time = 0.8; //鼓動の周期

float vibrationAmplitude = 0.005; // 振動の振幅
float vibrationFrequency = 60.0;  // 振動の頻度

ros::NodeHandle *nh;
std::string package_path = ros::package::getPath("heart");
std::string sound_path = package_path + "/" + "data/beat.wav";

void colorCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    if (msg->data.size() >= 3) {
        heart_color[0] = msg->data[0];
        heart_color[1] = msg->data[1];
        heart_color[2] = msg->data[2];
    }
}

void pulseTimeCallback(const std_msgs::Float32::ConstPtr& msg) {
    pulse_time = msg->data;
}

void updateColor() {
    // 明るさを調整するためのスケール
    float brightness = (heart_scale - 1.0) / 0.2;

    // 明るさを考慮した色を計算
    float red = heart_color[0] * (1.0 + brightness * 0.5);
    float green = heart_color[1] * (1.0 + brightness * 0.5);
    float blue = heart_color[2] * (1.0 + brightness * 0.5);

    // RGBの範囲を[0.0, 1.0]にクリップ
    red = fminf(1.0, fmaxf(0.0, red));
    green = fminf(1.0, fmaxf(0.0, green));
    blue = fminf(1.0, fmaxf(0.0, blue));

    // 色を設定
    glColor3f(red, green, blue); // RGBを設定
}

void drawHeart() {
  glTranslatef(0.0, 1.5, 0.0);
  // copied from https://www.zachmakesgames.com/node/5
  float a1, a2;
  a1 = 2.0;
  a2 = -2.0;
	
  int steps = 10;
  int numSegments = 5;

  float xmod = 1;
  float ymod = 1;
  float xmod1 = 1;
  float ymod1 = 1;
  float shift = 0;
  float shift1 = 0;

  float segshift = 0;
  float segshiftNext = 0.35;

  //This first for loop draws the two halves of the heart
  //Later on I use i to modify the vertices to mirror the
  //half of the heart, skipping i == 0 to do some black magic
  for (int i = -1; i <= 1; ++i) {
    if (i == 0) continue;

    //reset the modifiers for each half of the heart
    xmod = 1;
    ymod = 1;
    xmod1 = 1;
    ymod1 = 1;
    shift = 0;
    shift1 = 0;

    segshift = 0;
    segshiftNext = 0.2;

    //Draw each strip of the heart

    //Any more than six segments with the current
    //code makes some funky effects
    for (int j = 1; j <= numSegments; ++j) {

      //start the triangle strip topology
      glBegin(GL_TRIANGLE_STRIP);

      //Some math to modify how quickly each segment shrinks
      //modify xmod, ymod, and shift
      float modbase = (float)(j) / 10;
      float modbase1 = modbase * modbase;
      float modbase2 = modbase1 * modbase;

      xmod1 -= modbase1;
      ymod1 -= 0.1;
      shift1 +=modbase2;

      //Then walk around the edge of the cardioid
      //Special note here that 90 to 450 is 360 degrees,
      //but this loop goes one step further, this is to fully
      //complete the ring and avoid holes in the mesh
      for (int t = 90; t < 450 + steps; t += steps) {

        //Convert from degress to radians
        float rad = (3.14159 / 180.0)*t;

        //calculate the radius of the cardioid at the given angle in radians
        float r2 = a1 + a2 * sin(rad) + sin(rad)*(sqrt(fabs(cos(rad))) / (sin(rad) + 1.4));

        //now convert from polar coordinates to rectangular
        float x = r2 * cos(rad);
        float y = r2 * sin(rad);
				

        //Draw this current ring
        glVertex3f((float)(x * xmod) * heart_scale, (float)(y*ymod - shift) * heart_scale, (float)(i)*segshift * heart_scale);

        //Draw the next ring out with modified values
        //Don't need no stinkin transformation functions!
        glVertex3f((float)(x * xmod1) * heart_scale, (float)(y*ymod1 - shift1) * heart_scale, (float)(i)*(segshift + segshiftNext) * heart_scale);


      }
      //End this part of the topology
      glEnd();
			
      //Change the modifiers for the next ring
      segshift = segshift + segshiftNext;
      xmod = xmod1;
      ymod = ymod1;
      shift = shift1;

    }
    //draw end caps
    //This is the same code for the very last ring
    //except using a polygon instead of a triangle strip
    for (int j = 1; j <= numSegments; ++j) {
      glBegin(GL_POLYGON);

      for (int t = 90; t < 450 + steps; t += steps) {
        float rad = (3.14159 / 180.0)*t;

        float r2 = a1 + a2 * sin(rad) + sin(rad)*(sqrt(fabs(cos(rad))) / (sin(rad) + 1.4));
        
        float x = r2 * cos(rad);
        float y = r2 * sin(rad);

        glVertex3f((float)(x * xmod) * heart_scale, (float)(y*ymod - shift) * heart_scale, (float)(i)*segshift * heart_scale);
      }
      glEnd();
    }
  }
}

void display(void) {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // 画面のクリア
  glPushMatrix();
  
  glRotatef(angleY, 0.0, 1.0, 0.0);  // Y軸回転
/* 　  glColor3f(heart_color[0], heart_color[1], heart_color[2]);  // ハートの色を赤に設定 (R, G, B) */
  float vibration = sin(glutGet(GLUT_ELAPSED_TIME) * 0.001 * vibrationFrequency) * vibrationAmplitude;
  glTranslatef(0.0, vibration, 0.0);  // Y方向に振動を適用
  updateColor();
  drawHeart();
  glPopMatrix();
  glutSwapBuffers();  // バッファを交換して画面に表示
}

void reshape(int w, int h) {
    glViewport(0, 0, (GLsizei)w, (GLsizei)h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, (GLfloat)w/(GLfloat)h, 1.0, 200.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);  // 視点を設定
}

void idle() {
    if (!isWaiting) {
        // 拡大または縮小を行う
        if (isScalingUp) {
            heart_scale += scale_direction;  // スケールを増加
            if (heart_scale >= 1.2) {  // 上限に達したら
                heart_scale = 1.2;
                isScalingUp = false;  // 縮小方向へ
            }
        } else {
            heart_scale -= scale_direction;  // スケールを減少
            if (heart_scale <= 1.0) {  // 下限に達したら
                heart_scale = 1.0;
                isScalingUp = true;  // 拡大方向へ
                isWaiting = true;  // 待機を開始
                waitTime = 0.0;  // 待機時間をリセット
            }
        }
    } else {
        // 待機時間をカウント
        waitTime += 0.01;  // 一定の速度でカウント（適宜調整）
        if (waitTime >= pulse_time) {  // 1秒間待ったら
            isWaiting = false;  // 待機終了
            startPlayWAV(sound_path.c_str()); // WAVファイルの再生
        }
    }
    
    glutPostRedisplay();  // 画面の再描画を要求
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "heart_node");  // ROSノードを初期化
  nh = new ros::NodeHandle();
  ros::Subscriber color_sub = nh->subscribe("heart_color", 10, colorCallback);
  ros::Subscriber pulse_time_sub = nh->subscribe("pulse_time", 10, pulseTimeCallback);

  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
  glutInitWindowSize(640, 480);
  glutCreateWindow("3D Heart");
  glutFullScreen();

  glEnable(GL_DEPTH_TEST);  // 深度テストを有効化
  glutDisplayFunc(display);
  glutReshapeFunc(reshape);

  while (ros::ok()) {
    glutMainLoopEvent();  // GLUTイベントを処理
    idle();
    ros::spinOnce();      // ROSのコールバックを処理
  }
  
  delete nh;  // ノードハンドルを解放

  return 0;
}
