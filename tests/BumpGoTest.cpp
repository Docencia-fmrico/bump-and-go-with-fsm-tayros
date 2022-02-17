#include <gtest/gtest.h>
#include "ros/ros.h"
#include "fsm_bump_go/SensorGo.h"

TEST(BumpGoTest, initial_tests)
{
    fsm_bump_go::SensorGo bump_go_test;
    EXPECT_EQ(bump_go_test.get_state(), 0);
    EXPECT_EQ(bump_go_test.pressed_, false);
    EXPECT_EQ(bump_go_test.turn_direction_, bump_go_test.get_turn_direction());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "final_bump_go");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}