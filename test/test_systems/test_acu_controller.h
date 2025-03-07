#include "gtest/gtest.h"
#include "ACUController.h"

constexpr size_t num_chips = 1; 
ACUController controller = ACUControllerInstance<num_chips>::instance();

TEST (ACUControllerTesting, initial_state) {
    
}