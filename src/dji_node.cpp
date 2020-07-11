#include "dji_bridge.h"

int main(int argc, char* *argv) {
    dji_bridge bridge(argc, argv);
    return bridge.run();
}