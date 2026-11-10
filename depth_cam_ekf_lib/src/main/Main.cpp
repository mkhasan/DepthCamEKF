//
// Created by Hasan on 11/9/26.
//


#include <iostream>

#include "Log.h"


using namespace std;
using namespace Log;

int main(int argc, const char * argv[]) {

    set_level(Level::DEBUG);

    log(Level::INFO, "Hello World!");

    return 0;

}