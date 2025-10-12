#include <gtest/gtest.h>

#define TESTING_EXCLUDE_MAIN
#include "../src/FinalGradeDeterminator.cpp"

class FinalGradeDeterminator;

class TestFinalGradeDeterminator : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<FinalGradeDeterminator>();
  }
  void TearDown() override
  {
    node_.reset();
    rclcpp::shutdown();
  }
  std::shared_ptr<FinalGradeDeterminator> node_;
};


TEST_F(TestFinalGradeDeterminator, TestNodeCreation)
{
    // Node should exist
    ASSERT_NE(node_, nullptr);


      // Check if the node name matches what we expect
  EXPECT_EQ(std::string(node_->get_name()), std::string("FinalGradeDeterminator_node"));

    // Check parameters directly
    EXPECT_TRUE(node_->has_parameter("DB_CHECK_INTERVAL"));
    EXPECT_TRUE(node_->has_parameter("CALC_GRADE_TIMEOUT"));
    EXPECT_EQ(node_->get_parameter("DB_CHECK_INTERVAL").as_int(), 10);
    EXPECT_EQ(node_->get_parameter("CALC_GRADE_TIMEOUT").as_int(), 5);

    // Check publishers exist
    EXPECT_GT(node_->count_publishers("add_students"), 0u);
    EXPECT_GT(node_->count_publishers("remove_students"), 0u);

    // Check subscriber exists
    EXPECT_GT(node_->count_subscribers("exam_results"), 0u);

    // Check service client exists
    EXPECT_GT(node_->count_clients("GradeCalculator"), 0u);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
