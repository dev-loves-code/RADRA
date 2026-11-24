/**
 * @file bbFlooding.cpp
 * Entry point. author: Nawar ELZEIN
 **/
#include <iostream>
#include "bbFloodingCode.hpp"

using namespace std;
using namespace BlinkyBlocks;

int main(int argc, char **argv) {
    try {
        createSimulator(argc, argv, BbFloodingCode::buildNewBlockCode);
        getSimulator()->printInfo();
        BaseSimulator::getWorld()->printInfo();
        deleteSimulator();
    } catch (std::exception const &e) {
        cerr << "Uncaught exception: " << e.what();
    }
    return 0;
}
