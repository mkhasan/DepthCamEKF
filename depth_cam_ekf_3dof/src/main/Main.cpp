//
// Created by Hasan on 11/23/26.
//

#include "Log.h"
#include <iostream>

using namespace std;
using namespace Log;

int main() {

    set_level(Level::DEBUG);
    log(Level::DEBUG, "Hello");
    return 0;
}

