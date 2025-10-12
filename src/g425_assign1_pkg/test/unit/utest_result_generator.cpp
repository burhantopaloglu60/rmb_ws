#include <gtest/gtest.h>

#define TESTING_EXCLUDE_MAIN
#include "../src/ResultGenerator.cpp"

class ResultGenerator;

class TestResultGenerator : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<ResultGenerator>();
  }
  void TearDown() override
  {
    node_.reset();
    rclcpp::shutdown();
  }
  std::shared_ptr<ResultGenerator> node_;
};

TEST_F(TestResultGenerator, TestNodeCreation)
{
  // Node should exist
  ASSERT_NE(node_, nullptr);

  // Check if the node name matches what we expect
  EXPECT_EQ(std::string(node_->get_name()), std::string("ResultGenerator_node"));

  // Parameters exist and have correct default values
  EXPECT_TRUE(node_->has_parameter("EXAM_PUBLISH_INTERVAL"));
  EXPECT_TRUE(node_->has_parameter("MIN_MARK"));
  EXPECT_TRUE(node_->has_parameter("MAX_MARK"));

  EXPECT_EQ(node_->get_parameter("EXAM_PUBLISH_INTERVAL").as_int(), 2);
  EXPECT_EQ(node_->get_parameter("MIN_MARK").as_int(), 10);
  EXPECT_EQ(node_->get_parameter("MAX_MARK").as_int(), 100);

  // Publisher exists
  EXPECT_GT(node_->count_publishers("exam_results"), 0u);

  // Subscribers exist
  EXPECT_GT(node_->count_subscribers("add_students"), 0u);
  EXPECT_GT(node_->count_subscribers("remove_students"), 0u);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
