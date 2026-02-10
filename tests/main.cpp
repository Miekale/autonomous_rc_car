#include "pure_pursuit/TestStraightLine.h"
#include "pure_pursuit/TestLoop.h"
#include "serial/TestSerial.h"
int main() {
    test_straight_line();
    test_loop();
    test_serial();
}