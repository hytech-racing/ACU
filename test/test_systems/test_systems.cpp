#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "test_acu_controller.h"
#include "test_acu_state_machine.h"

int main(int argc, char **argv) {
    testing::InitGoogleMock(&argc, argv);

    if (RUN_ALL_TESTS()) {
        // nothing, let PlatformIO parse result
    }
    return 0;
}

