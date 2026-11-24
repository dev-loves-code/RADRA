/**
 * @file bbRDRM.cpp
 * Entry point. author: Nawar ELZEIN
 **/
#include <iostream>
#include "bbRDRM5Code.hpp"

using namespace std;
using namespace BlinkyBlocks;

int main(int argc, char **argv) {
    try {
        createSimulator(argc, argv, BbRDRM5Code::buildNewBlockCode);
        getSimulator()->printInfo();
        BaseSimulator::getWorld()->printInfo();
        deleteSimulator();
    } catch (std::exception const &e) {
        cerr << "Uncaught exception: " << e.what();
    }
    return 0;
}
