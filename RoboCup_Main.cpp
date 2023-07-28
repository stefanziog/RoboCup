/*

 Copyright © 2022 DTU, Christian Andersen jcan@dtu.dk

 The MIT License (MIT)  https://mit-license.org/

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 and associated documentation files (the “Software”), to deal in the Software without restriction,
 including without limitation the rights to use, copy, modify, merge, publish, distribute,
 sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
 is furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all copies
 or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
 FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE. */

#include <iostream>
#include "src/ubridge.h"
#include "src/uvision.h"
#include "src/upose.h"
#include "src/ucomment.h"
#include "src/ustate.h"
#include "src/uplay.h"
#include "src/uevent.h"
#include "src/utime.h"

 // to avoid writing std::
using namespace cv;
using namespace std;

void setup(int argc, char** argv)
{ // check for command line parameters
    for (int i = 1; i < argc; i++)
    { // check for command line parameters
      // for process debug
        if (strcmp(argv[i], "help") == 0)
        {
            printf("-----\n# User mission command line help\n");
            printf("# usage:\n#   ./user_mission [help] [ball] [show] [aruco] [videoX]\n-----\n");
            return;
        }
    }
    // connect to robot hardware using bridge
    bridge.setup("127.0.0.1", "24001", argc, argv);
    if (true or bridge.connected)
    { /// call setup for data structures
        pose.setup();
        comment.setup();
        state.setup();
        event.setup();
        printf("# Setup finished OK\n");
    }
    else
        printf("# Setup failed\n");
}

// Function written by George
void followLineUntilRamp()
{
    bridge.tx("regbot mclear\n");
    // clear events received from last mission
    event.clearEvents();
    bridge.tx("regbot madd vel=0.4: dist=0.3 \n");
    bridge.tx("regbot madd vel=0.4, log=20: lv=20,dist=1 \n");
    bridge.tx("regbot madd vel=0.5,edger=0.0,white=1: lv<4 , time=14 \n");
    bridge.tx("regbot madd vel=0.4, log=20,edgel=0.0,white=1: lv<6 , xl>15 \n");
    bridge.tx("regbot madd tr=-0.03, log=20: turn= 82 \n");
    bridge.tx("regbot madd vel=0.0, log=20,white=1: lv=20, time=1 \n");
    bridge.tx("regbot madd vel=0.25, log=20,edgel=-0.5,white=1: dist=1.21 \n");
    bridge.tx("regbot madd vel=0.0, log=20: time=2 \n");
    bridge.tx("regbot madd servo=2, log=20, pservo=0:time=2 \n");
    bridge.tx("regbot madd servo=2, log=20, pservo=850, vservo=120:time=6 \n");
    bridge.tx("regbot madd vel=0.0, log=20: time=1 \n");
    bridge.tx("regbot madd vel=0.2, log=20: lv=20, dist=0.1 \n");
    bridge.tx("regbot madd vel=0.1, log=20,edgel=-0.5,white=1: lv<4 \n");
    bridge.tx("regbot madd vel=0.0, log=20: time=1 \n");
    bridge.tx("regbot madd tr=-0.01, log=20: turn= 5\n");
    bridge.tx("regbot madd vel=0.3, log=20 ,white=1: xl>10 , dist=3 \n");
    bridge.tx("regbot madd vel=0.3, log=20: dist=0.1 \n");
    bridge.tx("regbot madd vel=0.15, log=20 ,white=1: xl>10 , dist=1.1 \n");
    bridge.tx("regbot madd tr=-0.01, log=20: turn= -60\n");
    bridge.tx("regbot madd vel=0.15, log=20: dist=0.03 \n");
    bridge.tx("regbot madd vel=0.2, log=20,white=1: lv=20 ,dist=0.5 \n");
    bridge.tx("regbot madd vel=0.2, log=20,edgel=-0.25,white=1: lv<10 , dist=2 \n");
    bridge.tx("regbot madd vel=0.2, log=20,white=1: lv=20 ,dist=0.25 \n");
    bridge.tx("regbot madd vel=0.3, log=20, edgel=-0.5 ,white=1: lv<10, dist=2.3 \n");
    bridge.tx("regbot madd tr=-0.01, log=20: turn= -190 \n");
    bridge.tx("regbot madd vel=0.0, log=20,white=1: time=1 \n");
    bridge.tx("regbot madd vel=0.2, log=20: lv=20, dist=1 \n");
    bridge.tx("regbot madd vel=0.3, log=20 ,edgel=0.0,white=1: lv<10 , time=2 \n");
    bridge.tx("regbot madd vel=0.55, log=20,edgel=0.0,white=1: lv<10 , dist=3 \n");
    bridge.tx("regbot madd servo=2, log=20, pservo=900:time=0.1 \n");
    bridge.tx("regbot madd vel=0.2, log=20: lv=20, dist=1 \n");
    bridge.tx("regbot madd vel=0.3, log=20,edgel=-0.25,white=1: lv<4, dist=0.81 \n");
    bridge.tx("regbot madd servo=2, log=20, pservo=820:time=0.1 \n");
    bridge.tx("regbot madd tr=-0.01, log=20: turn= 100\n");
    bridge.tx("regbot madd servo=2, log=20, pservo=500:time=0.1 \n");


    // start this mission
    bridge.tx("regbot start\n");
    event.waitForEvent(0);
}


void cycleMission()
{
    bridge.tx("regbot mclear\n");
    // clear events received from last mission
    event.clearEvents();
    bridge.tx("regbot madd vel=0.4,log=20,white=1: lv=20,dist=1 \n");
    bridge.tx("regbot madd vel=0.4,log=20,edgel=-0.5,white=1: lv<4, xl>17 \n");
    bridge.tx("regbot madd tr=-0.01,log=20: turn= 70 \n");
    bridge.tx("regbot madd vel=0.0,log=20: time = 2\n");
    bridge.tx("regbot madd vel=0.0,log=20: ir2>0.2 \n");
    bridge.tx("regbot madd vel=0.0,log=20: ir2<0.15 \n");
    bridge.tx("regbot madd vel=0.0,log=20: time=1 \n");
    bridge.tx("regbot madd vel=0.4,log=20: dist=0.6 \n");
    bridge.tx("regbot madd vel=0.45,white=1,log=20: lv=20, dist=0.7 \n");
    // bridge.tx("regbot madd vel=0.45,edgel=-0.5,white=1: lv<4 ,time=17.5\n");
    bridge.tx("regbot madd vel=0.3,log=20,edgel=-0.5,white=1: lv<4 ,xl>15\n");
    bridge.tx("regbot madd tr=-0.1,log=20: turn= 50 \n");
    bridge.tx("regbot madd vel=-0.4,log=20 : dist=-0.3 \n");
    bridge.tx("regbot madd vel=-1,log=20,acc=2: dist=-0.26 \n");
    bridge.tx("regbot madd tr=-0.01,log=20: turn= -50\n");
    bridge.tx("regbot madd vel=0.3,log=20: dist=0.1 \n");
    bridge.tx("regbot madd tr=-0.46,log=20: turn= -120 \n");
    bridge.tx("regbot madd vel=0.3,log=20: dist=0.1 \n");
    bridge.tx("regbot madd tr=-0.41,log=20: turn= -250 \n");
    bridge.tx("regbot madd tr=-0.01,log=20: turn= 95 \n");
    bridge.tx("regbot madd vel=0.0,log=20: ir2>0.3 \n");
    bridge.tx("regbot madd vel=0.0,log=20: ir2<0.15 \n");
    bridge.tx("regbot madd vel=0.4,log=20,white=1: dist=0.3 \n");
    bridge.tx("regbot madd vel=0.4,log=20,white=1: dist=4, xl>10 \n");
    bridge.tx("regbot madd tr=-0.01,log=20: turn= -95\n");
    bridge.tx("regbot madd vel=0.3,log=20,white=1: lv=20, dist=0.1 \n");
    bridge.tx("regbot madd vel=0.4,log=20,edgel=-0.5,white=1: lv<4 , time=15 \n");
    bridge.tx("regbot madd tr=-0.01,log=20: turn= 90 \n");
    bridge.tx("regbot madd vel=0.3,log=20,white=1: xl>15, dist=1 \n");
    bridge.tx("regbot madd tr=-0.01,log=20: turn= -90 \n");
    bridge.tx("regbot madd vel=0.4,log=20,edgel=-0.5,white=1: lv<4 , time=15 \n");

    bridge.tx("regbot start\n");
    event.waitForEvent(0);
}

// void followOrangeBall() //Kinda working GETTING TOO CLOSE TO BALL, AND SERVO NOT SYNCH
// {
//   bridge.tx("regbot mclear\n");
//   // clear events received from last mission
//   event.clearEvents();

//   string tmp= "regbot madd vel=0.2: dist=" + to_string(vision.ballDistance - 0.25) + "\n";
//   cout << vision.angle << endl;
//   cout << vision.ballDistance << endl;
//   string turn = "regbot madd tr=0.09: turn=" + to_string(vision.angle) +  "\n";

//   bridge.tx(turn.c_str());
//   bridge.tx(tmp.c_str());
//   bridge.tx("regbot madd vel=0.0: time=2 \n");
//   bridge.tx("regbot madd servo=2, pservo=0:time=2 \n");
//   bridge.tx("regbot madd servo=2, pservo=800, vservo=120:time=6 \n");
//   bridge.tx("regbot start\n");
//   event.waitForEvent(0);
// }

void openDoor() {
    bridge.tx("regbot mclear\n");
    // clear events received from last mission
    event.clearEvents();
    bridge.tx("regbot madd vel=0.5,log=20,white=1: lv=20 , dist=2 \n");
    bridge.tx("regbot madd vel=0.5,log=20,edger=0.5,white=1: lv<4 , time=2 \n");
    bridge.tx("regbot madd vel=1,log=20,edger=0.5,white=1: lv<4 , dist= 0.2 \n");
    bridge.tx("regbot madd vel=1,log=20,edger=0.5,white=1: lv<4, time=6.3 \n");
    bridge.tx("regbot madd vel=0.5,log=20,edger=0.5,white=1: lv<4  \n");
    bridge.tx("regbot madd vel=0.4,log=20: dist=0.55 \n");
    bridge.tx("regbot madd tr=-0.01,log=20: turn=-95 \n");
    bridge.tx("regbot madd vel=0.4,log=20, white =1: dist=2 , xl>16 \n");
    bridge.tx("regbot madd tr=-0.01,log=20: turn=-85 \n");
    bridge.tx("regbot madd vel=0.2,log=20, edgel=-0.5, white=1: lv<4, dist=0.15 \n");
    bridge.tx("regbot madd tr=-0.01,log=20: turn=95 \n");
    bridge.tx("regbot madd vel=0.0,log=20: time=1 \n");
    bridge.tx("regbot madd vel=0.3,log=20: dist=1.3, time=12 \n");
    bridge.tx("regbot madd tr=-0.01,log=20: turn=-85 \n");
    bridge.tx("regbot madd vel=0.3,log=20: dist = 0.25 \n");
    bridge.tx("regbot madd tr=-0.5,log=20: turn=91 \n");
    bridge.tx("regbot madd vel=-0.3,log=20: dist = -0.05 \n");
    bridge.tx("regbot madd tr=-0.02,log=20: turn= 93 \n");
    bridge.tx("regbot madd vel=0.3,log=20: dist = 1.6 \n");
    bridge.tx("regbot madd tr=-0.02,log=20: turn=-88 \n");
    bridge.tx("regbot madd vel=0.3,log=20: dist = 0.35 \n");
    bridge.tx("regbot madd tr=-0.02,log=20: turn=-85 \n");
    bridge.tx("regbot madd vel=0.3,log=20: dist = 0.95 \n");
    bridge.tx("regbot madd tr=-0.4,log=20: turn=-155, time=6 \n");
    bridge.tx("regbot madd vel=-0.3,log=20: dist = -0.2 \n");
    bridge.tx("regbot madd tr=-0.03,log=20: turn=45 \n");
    bridge.tx("regbot madd vel=0.3,log=20: dist = 0.7 \n");
    bridge.tx("regbot madd tr=-0.03,log=20: turn=-85 \n");
    bridge.tx("regbot madd vel=0.3,log=20: dist = 0.9 \n");
    bridge.tx("regbot madd tr=-0.5,log=20: turn=-155, time=6 \n");
    bridge.tx("regbot madd vel=-0.3,log=20: dist = -0.4 \n");
    bridge.tx("regbot madd tr=-0.03,log=20: turn=-140 \n");
    bridge.tx("regbot start\n");
    event.waitForEvent(0);
}

void AxeGate()
{
    bridge.tx("regbot mclear\n"); // clear any previously added mission commands
    event.clearEvents(); // clear any previously received events

    // DO THE PATH FROM STAIRS UNTIL AXEGATE

    // bridge.tx("regbot madd vel=0.3,edgel=-0.25,white=1: lv=20,dist=0.1 \n");
    // bridge.tx("regbot madd vel=0.3,edgel=-0.25,white=1: lv<4 \n");
    // bridge.tx("regbot madd vel=0.3, white=1: lv=20, dist=0.25 \n");
    // bridge.tx("regbot madd vel=0.25,edgel=-0.25,white=1: lv<10 \n");
    // bridge.tx("regbot madd vel=0.3, white=1: dist=0.05 \n");
    // bridge.tx("regbot madd tr=-0.01: turn= -70 \n");
    // bridge.tx("regbot madd vel=0.3,white=1: lv=20,dist=0.4 \n");
    // bridge.tx("regbot madd vel=0.3,edgel=0.25,white=1: lv<4, dist=0.6 \n");
    // bridge.tx("regbot madd vel=0.0: time = 2\n");

    // WAIT FOR AXEGATE TO TURN A COUPLE OF TIMES

    bridge.tx("regbot madd vel=0.0, log=20: ir2>0.25\n");
    bridge.tx("regbot madd vel=0.0, log=20: ir2<0.25\n");
    bridge.tx("regbot madd vel=0.0, log=20: ir2>0.25\n");
    bridge.tx("regbot madd vel=0.0, log=20: ir2<0.25\n");

    //START!!!
    bridge.tx("regbot madd vel=0.0, log=20, white=1:lv=20, time=0.1 \n");
    bridge.tx("regbot madd vel=0.6,log=20, edgel=-0.25, white=1: lv<8 , time=2 \n");

    //FIRST CROSSING LINE

    bridge.tx("regbot madd vel=0.3, log=20: xl>5, dist=1 \n");
    bridge.tx("regbot madd vel=0.3, log=20: dist=0.05 \n");

    // SECOND CROSSING LINE TURN AND GO TO THE GATES
    bridge.tx("regbot madd vel=0.2, log=20, white=1: xl>10 , dist=1 \n");
    bridge.tx("regbot madd tr=0.01, log=20: turn= 100 \n");
    bridge.tx("regbot madd vel=0.1, log=20, edger=0.25,white=1: lv<4, time = 0.3 \n");
    bridge.tx("regbot madd vel=0.3,log=20, edger=0.25,white=1: lv<4, time = 3.2 \n");

    //TURN AGAIN TOWARDS ROBOTIZE
    bridge.tx("regbot madd vel=-0.3, log=20: time=1 \n");
    bridge.tx("regbot madd tr=-0.01, log=20: turn= -190 \n");
    bridge.tx("regbot madd vel=0.2,log=20, white=1: lv=20,dist=0.1 \n");
    bridge.tx("regbot madd vel=0.3, log=20,edger=0.0,white=1: lv<4 , time=3 \n");
    bridge.tx("regbot madd vel=0.5,log=20, edger=0.0,white=1: lv<4 , time=2 \n");
    bridge.tx("regbot madd vel=1,log=20, edger=0.0,white=1: lv<4 , dist= 0.2 \n");
    bridge.tx("regbot madd vel=1,log=20, edger=0.0,white=1: lv<4  \n");


    // TURN AND GO TOWARS BELL

    bridge.tx("regbot start\n");
    event.waitForEvent(0);
}

void Hall()
{
    bridge.tx("regbot mclear\n"); // clear any previously added mission commands
    event.clearEvents(); // clear any previously received events
    bridge.tx("regbot madd servo=2, pservo=850 :time=2 \n");
    bridge.tx("regbot madd vel=0.3, white =1 : lv=20,dist=0.15 \n");

    bridge.tx("regbot madd vel=0.6,edgel=0.0,white=1: lv<4 , timer=5 \n");
    bridge.tx("regbot madd vel=0.6,edgel=0.0,white=1: ir1>1.1 \n");
    bridge.tx("regbot madd vel=0.5,edgel=0.0,white=1: lv<4, xl>16 \n");
    bridge.tx("regbot madd tr=-0.01: turn= -180 \n");
    bridge.tx("regbot madd vel=0.4,white=1: lv=20, dist= 0.4 \n");
    bridge.tx("regbot madd vel=0.4,edgel=-0.5,white=1: lv<4, dist= 0.4 \n");
    bridge.tx("regbot madd vel=0.3: dist=0.4 \n");
    bridge.tx("regbot madd tr=-0.01: turn= -45 \n");
    bridge.tx("regbot madd tr=-0.01: turn= -25 \n");
    bridge.tx("regbot madd vel=-0.25: dist=-0.05 \n");
    bridge.tx("regbot madd tr=-0.01: turn= 30 \n");
    bridge.tx("regbot madd tr=-0.01: turn= -25 \n");
    bridge.tx("regbot madd vel=-0.25: dist=-0.05 \n");
    bridge.tx("regbot madd tr=-0.01: turn= 30 \n");
    bridge.tx("regbot madd tr=-0.01: turn= -25 \n");
    bridge.tx("regbot madd vel=-0.25: dist=-0.05 \n");
    bridge.tx("regbot madd tr=-0.01: turn= 30 \n");
    bridge.tx("regbot madd tr=-0.01: turn= -25 \n");
    bridge.tx("regbot madd vel=-0.25: dist=-0.05 \n");
    bridge.tx("regbot madd tr=-0.01: turn= 30 \n");
    bridge.tx("regbot madd tr=-0.01: turn= -25 \n");

    // bridge.tx("regbot madd vel=0.3,edger=0,white=1: dist=0.6 \n");
    // bridge.tx("regbot madd vel=0.3,edger=0,white=1: lv<4 , xl>15 \n");
    // bridge.tx("regbot madd tr=-0.02: turn= -68 \n");
    // bridge.tx("regbot madd vel=0.3: dist=0.1 \n");
    // bridge.tx("regbot madd vel=0.0: time=1 \n");

    //bridge.tx("regbot madd vel=-0.3: dist=0.1 \n");

    // bridge.tx("regbot madd vel=0.0: dist=0.0 \n");


    bridge.tx("regbot start\n");
    event.waitForEvent(0);
}



void stairs()
{
    bridge.tx("regbot mclear\n"); // clear any previously added mission commands
    event.clearEvents(); // clear any previously received events

    //FOLLOW LINE UNTIL STAIRS
    bridge.tx("regbot madd vel=0.4: lv=20,dist=1 \n");
    bridge.tx("regbot madd vel=0.5,edger=0.0,white=1: lv<10 , time=14 \n");

    bridge.tx("regbot madd vel=0.4,edgel=0.0,white=1: lv<6 , xl>15 \n");
    bridge.tx("regbot madd vel=0.4,edgel=-0.25,white=1: dist=1.3 \n");

    //bridge.tx("regbot madd servo=2, pservo=0 :time=1 \n");

    bridge.tx("regbot madd vel=0.0 : time =1 \n");

    bridge.tx("regbot madd servo=2, pservo=0, vservo=140 :time=2 \n");

    bridge.tx("regbot madd servo=2, pservo=750, vservo=140 :time=6 \n");


    //STAIR ENA
    bridge.tx("regbot madd vel=0.2, white =1 : lv=20, dist=0.8 \n");
    bridge.tx("regbot madd vel=0.2, edgel=-0.2, white =1 : lv<4,dist=0.4 \n");

    bridge.tx("regbot madd vel=0.2, white =1 : dist=0.15 \n");
    bridge.tx("regbot madd vel=-0.2, white =1 : time = 2 \n");

    // STAIR DUO
    bridge.tx("regbot madd vel=0.1, white =1 : lv=20, dist=0.3 \n");
    bridge.tx("regbot madd vel=0.2, edgel=-0.2, white =1 : lv<4,dist=0.4 \n");

    bridge.tx("regbot madd vel=0.2, white =1 : dist=0.15 \n");
    bridge.tx("regbot madd vel=-0.2, white =1 : time = 2 \n");



    // STAIR TRIA

    bridge.tx("regbot madd vel=0.1, white =1 : lv=20, dist=0.3 \n");
    bridge.tx("regbot madd vel=0.2, edgel=-0.2, white =1 : lv<4,dist=0.4 \n");

    bridge.tx("regbot madd vel=0.2, white =1 : dist=0.15 \n");
    bridge.tx("regbot madd vel=-0.2, white =1 : time = 2 \n");

    //STAIR TESSERA

    bridge.tx("regbot madd vel=0.1, white =1 : lv=20, dist=0.3 \n");
    bridge.tx("regbot madd vel=0.2, edgel=-0.2, white =1 : lv<4,dist=0.4 \n");

    bridge.tx("regbot madd vel=0.2, white =1 : dist=0.15 \n");
    bridge.tx("regbot madd vel=-0.2, white =1 : time = 2 \n");



    // STAIR PENTE
    bridge.tx("regbot madd vel=0.1, white =1 : lv=20, dist=0.3 \n");
    bridge.tx("regbot madd vel=0.2, edgel=-0.2, white =1 : lv<4,dist=0.4 \n");

    bridge.tx("regbot madd vel=0.2, white =1 : dist=0.15 \n");
    bridge.tx("regbot madd vel=-0.2, white =1 : time = 2 \n");


    // GET IN THE WHITE LINE

    bridge.tx("regbot madd vel=0.1, white =1 : lv=20, dist=0.3 \n");
    bridge.tx("regbot madd vel=0.1, edgel=-0.2, white =1 : lv<4,dist=0.4 \n");

    // GET SERVO TO ORIGINAL POSITION / PREPARE FOR AXEGATE
    bridge.tx("regbot madd servo=2, pservo=-900 : time=3 \n");

    // DO THE PATH FROM STAIRS UNTIL AXEGATE



    bridge.tx("regbot start\n");
    event.waitForEvent(0);


}

void secondBall(int argc, char** argv)
{

    bridge.tx("regbot mclear\n"); // clear any previously added mission commands
    event.clearEvents(); // clear any previously received events



    // GO UNTIL CROSS LINE THEN TURN

    bridge.tx("regbot madd vel=0.3,edgel=0.0,white=1: dist=0.6 \n");
    //bridge.tx("regbot madd vel=0.3,edger=-0.15,white=1: lv<10 , xl>14 \n");
    bridge.tx("regbot madd vel=0.35,edger=0.15,white=1: lv<10 , dist = 1 \n");
    bridge.tx("regbot madd vel=0.2,edger=0.15,white=1: lv<10 , xl>14 \n");


    bridge.tx("regbot madd tr=-0.01: turn= -185 \n");
    bridge.tx("regbot madd vel=0.3: dist=0.2 \n");


    //TRY TO CATCH ORANGE BALL
    bridge.tx("regbot madd vel=0.3,edgel=-0.25,white=1: lv<4, dist=1.15 \n");
    bridge.tx("regbot madd tr=0.02: turn= 106 \n");
    bridge.tx("regbot madd vel=0.0: time =1 \n");

    bridge.tx("regbot madd servo=2, pservo=0 :time=1 \n");

    bridge.tx("regbot madd servo=2, pservo=790, vservo=140 :time=2 \n");


    bridge.tx("regbot madd vel=-0.3: dist = 0.1 \n");

    bridge.tx("regbot madd tr=0.01: turn= 190 \n");
    bridge.tx("regbot madd vel=0.3: dist=0.39 \n");
    bridge.tx("regbot madd vel=0.15: dist=0.02 \n");

    bridge.tx("regbot madd tr=0.01: turn = 20 \n");
    bridge.tx("regbot madd vel=-0.2: dist=-0.02 \n");
    bridge.tx("regbot madd tr=-0.01: turn = -40 \n");
    bridge.tx("regbot madd tr=0.01: turn = 40 \n");

    //bridge.tx("regbot madd vel=0.35,edgel=0.0,white=1: lv<10 , dist = 0.2 \n");




    //bridge.tx("regbot madd vel=0.3: dist=0.001 \n");
    //vision.setup(argc, argv);

    // vision.processImage(8);
    // string turn = "regbot madd tr=0.01: turn=" + to_string(vision.angle) +  "\n";

    // bridge.tx(turn.c_str());

    //bridge.tx("regbot madd servo=2, pservo=850:time=2 \n");






    bridge.tx("regbot start\n");
    event.waitForEvent(0);
}


void bell()
{

    bridge.tx("regbot mclear\n"); // clear any previously added mission commands
    event.clearEvents(); // clear any previously received events

    bridge.tx("regbot madd vel=0.4,log=20: dist=0.55 \n");
    bridge.tx("regbot madd tr=-0.01,log=20: turn=-100 \n");
    bridge.tx("regbot madd vel=0.4,log=20, white =1: dist=2 , xl>16 \n");
    bridge.tx("regbot madd tr=-0.01,log=20: turn=-90 \n");
    bridge.tx("regbot madd vel=0.2,log=20, edgel=-0.5, white=1: lv<4, time = 5 \n");
    bridge.tx("regbot madd servo=2, pservo=0, vservo=140 :time=2 \n");




    bridge.tx("regbot start\n");
    event.waitForEvent(0);


}



int main(int argc, char** argv)
{
    std::cout << "# Hello, Robobot user mission starting ...\n";
    setup(argc, argv);
    //stairs();
    //AxeGate();
    openDoor();
    //followLineUntilRamp();
    //stairs();
    //AxeGate();
    //test();
    //bell();

    //secondBall(argc, argv);
    //cycleMission();
    //followLineUntilRamp()

    return 0;
}
