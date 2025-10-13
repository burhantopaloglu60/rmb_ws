#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include <queue>
#include <set>
#include <mutex>
#include <mysql/mysql.h>
#include <stdexcept>

#include "../src/RetakeScheduler.cpp"
#include "g425_assign1_pkg/GGDatabase.hpp"

class TestableRetakeScheduler : public RetakeScheduler {
public:
    using RetakeScheduler::RetakeScheduler;

    void startNextRetake() override {
        // Do nothing so the queues stay the same
    }
};

void clearFinalGradesTable() {
    MYSQL *conn = mysql_init(nullptr);
    if (!mysql_real_connect(conn, "localhost", "john_gradegenerator", "1234", "grade_generator", 0, nullptr, 0)) {
        throw std::runtime_error(mysql_error(conn));
    }

    // delete all rows from the finalgrade table
    if (mysql_query(conn, "DELETE FROM final_grades")) {
        throw std::runtime_error(mysql_error(conn));
    }

    // reset auto_increment
    if (mysql_query(conn, "ALTER TABLE final_grades AUTO_INCREMENT = 1")) {
        throw std::runtime_error(mysql_error(conn));
    }

    mysql_close(conn);
}

class RetakeSchedulerTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        node = std::make_shared<TestableRetakeScheduler>(false, true);
        clearFinalGradesTable();
    }

    void TearDown() override {
        rclcpp::shutdown();
    }

    std::shared_ptr<TestableRetakeScheduler> node;
};

TEST_F(RetakeSchedulerTest, QueueStartsEmpty)
{
    std::lock_guard<std::mutex> lock(node->queueMutex);
    EXPECT_TRUE(node->retakeQueue.empty());
}

TEST_F(RetakeSchedulerTest, ScheduledRetakesStartsEmpty)
{


    EXPECT_TRUE(node->scheduledRetakes.empty());
}

TEST_F(RetakeSchedulerTest, PassedStudentNotAddedToQueue) {
    // Student with passed final grade
    DBT_FinalGrade passingStudent;
    passingStudent.student_id = 2;
    passingStudent.course_id = 1;
    passingStudent.number_of_exams = 3;
    passingStudent.final_grade = 75.00; 

    // Add the student to database
    node->db.addFinalGrade(passingStudent); 

    // Trigger the scheduling
    node->check_and_schedule();  //queue is being filled, startNextRetake() function does nothing in TestableRetakeScheduler

    // Check that the queue remains empty
    std::lock_guard<std::mutex> lock(node->queueMutex);
    EXPECT_TRUE(node->retakeQueue.empty()) << "Queue must remain empty for passed student";

    // Check that scheduled retakes remains empty
    EXPECT_TRUE(node->scheduledRetakes.empty()) << "scheduledRetakes must be left emtpy for passed students";
}

TEST_F(RetakeSchedulerTest, SingleFailingStudentAddedToQueue) {
    DBT_FinalGrade failingStudent;
    failingStudent.student_id = 1;
    failingStudent.course_id = 3;
    failingStudent.number_of_exams = 3;
    failingStudent.final_grade = 40.00;

    node->db.addFinalGrade(failingStudent); 

    node->check_and_schedule();

    std::lock_guard<std::mutex> lock(node->queueMutex);
    EXPECT_EQ(node->retakeQueue.size(), 1u); // retakeQueue must be size 1
    EXPECT_FALSE(node->scheduledRetakes.empty());
    EXPECT_FALSE(node->retakeQueue.empty());
}


TEST_F(RetakeSchedulerTest, TwoFailingAndOnePassingStudents) {
    // Three students created — 2 fail, 1 pass
    DBT_FinalGrade studentFail1;
    studentFail1.student_id = 5;
    studentFail1.course_id = 1;
    studentFail1.number_of_exams = 3;
    studentFail1.final_grade = 45.0; 

    DBT_FinalGrade studentFail2;
    studentFail2.student_id = 6;
    studentFail2.course_id = 2;
    studentFail2.number_of_exams = 3;
    studentFail2.final_grade = 30.0;  

    DBT_FinalGrade studentPass;
    studentPass.student_id = 3;
    studentPass.course_id = 3;
    studentPass.number_of_exams = 4;
    studentPass.final_grade = 80.0;  

    // Add them to the database
    node->db.addFinalGrade(studentFail1);
    node->db.addFinalGrade(studentFail2);
    node->db.addFinalGrade(studentPass);

    // Call the scheduler
    node->check_and_schedule();

    std::lock_guard<std::mutex> lock(node->queueMutex);

    // Expected: only the 2 fails in the queue
    EXPECT_EQ(node->retakeQueue.size(), 2u) << "Queue must contain 2 fails";

    // Check that scheduledRetakes also contains 2 items
    EXPECT_EQ(node->scheduledRetakes.size(), 2u) << "scheduledRetakes must contain 2 combinations";

    // Check that the correct students are in it
    std::set<std::pair<int, int>> expectedKeys = {
        {5, 1},
        {6, 2}
    };
    EXPECT_EQ(node->scheduledRetakes, expectedKeys) << "Only the failed grades may be included in scheduledRetakes";

    // Make sure no one has been added with a passed grade
    for (const auto &key : node->scheduledRetakes) {
        EXPECT_NE(key.first, 3) << "Student with a passing grade may not be included in scheduledRetakes";
    }
}