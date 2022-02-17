#include <gtest/gtest.h>
#include "ros/ros.h"
#include "fsm_bump_go/SensorGo.h"
#include "fsm_bump_go/FinalBumpGo.h"

TEST(BumpGoTest, initial_tests)
{
    fsm_bump_go::SensorGo initial_test;
    EXPECT_EQ(initial_test.get_state(), 0);
    EXPECT_EQ(initial_test.pressed_, false);
    EXPECT_EQ(initial_test.turn_direction_, initial_test.get_turn_direction());
}

TEST(BumpGoTest, bump_tests)
{
    fsm_bump_go::FinalBumpGo bump_test;

    bump_test.set_bumper(0);
    EXPECT_EQ(bump_test.get_bumper(), 0);

    bump_test.set_bumper(1);
    EXPECT_EQ(bump_test.get_bumper(), 1);

    bump_test.set_bumper(2);
    EXPECT_EQ(bump_test.get_bumper(), 2);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "final_bump_go");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}