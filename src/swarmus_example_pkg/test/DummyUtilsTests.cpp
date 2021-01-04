#include "swarmus_example_pkg/DummyUtils.h"
#include <gtest/gtest.h>

TEST(TestSuite, dummyTestCase) { ASSERT_TRUE(true); }

TEST(TestSuite, testAddTwoNumbers) {
    int sum = DummyUtils::addTwoNumbers(2, 2);

    ASSERT_EQ(sum, 4);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}