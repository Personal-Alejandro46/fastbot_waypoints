// Copyright 2024 Fastbot — Licensed under Apache 2.0

#include "fastbot_msgs/action/waypoint_action.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <fstream>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;
using WaypointAction = fastbot_msgs::action::WaypointAction;

constexpr double TARGET_X = 0.05;
constexpr double TARGET_Y = 0.05;
constexpr double MAP_RADIUS = 5.0;
constexpr double POSITION_TOLERANCE = 0.10;
constexpr double YAW_TOLERANCE = 0.50;

static double normalize_angle(double a) {
  while (a > M_PI)
    a -= 2.0 * M_PI;
  while (a < -M_PI)
    a += 2.0 * M_PI;
  return a;
}

// =============================================================================
//  ResultPatchListener
//
//  Patches the GTest XML after the run to achieve correct colcon reporting:
//
//    Problem 1 — GTest 1.10 never emits <skipped/> for GTEST_SKIP() tests.
//                colcon reads <skipped/> to count skips → always "0 skipped".
//
//    Problem 2 — GTest always emits <failure> even for unhandled exceptions.
//                colcon distinguishes <failure> ("failures") from <error>
//                ("errors"). We want infrastructure / out-of-map errors to
//                appear as "errors", not "failures".
//
//  After RUN_ALL_TESTS() flushes the XML file, this listener:
//    • Converts <failure …/> → <error …/> for tests that threw exceptions.
//    • Adds <skipped/> for tests that called GTEST_SKIP().
//    • Updates the <testsuite errors="…" failures="…" skipped="…"> counts.
// =============================================================================
class ResultPatchListener : public ::testing::EmptyTestEventListener {
public:
  explicit ResultPatchListener(std::string path) : path_(std::move(path)) {}

  void OnTestStart(const ::testing::TestInfo &info) override {
    current_ = info.name();
    threw_ = false;
  }

  void OnTestPartResult(const ::testing::TestPartResult &r) override {
    using R = ::testing::TestPartResult;
    if (r.type() == R::kSkip) {
      skipped_.push_back({current_, r.message() ? r.message() : ""});
    }
    // GTest reports caught exceptions as kFatalFailure with a message that
    // starts with "C++ exception with description". Track those separately
    // so we can convert them from <failure> to <error> in the XML.
    if ((r.type() == R::kFatalFailure || r.type() == R::kNonFatalFailure) &&
        r.message() &&
        std::string(r.message()).find("C++ exception") != std::string::npos) {
      error_tests_.insert(current_);
      threw_ = true;
    }
  }

  void OnTestProgramEnd(const ::testing::UnitTest &) override {
    if (path_.empty())
      return;

    std::ifstream in(path_);
    if (!in)
      return;
    std::string xml((std::istreambuf_iterator<char>(in)), {});
    in.close();

    // ── 1. Convert <failure> → <error> for exception-throwing tests ──────
    for (const auto &name : error_tests_) {
      replace_element_in_testcase(xml, name, "failure", "error");
    }

    // ── 2. Add <skipped/> for GTEST_SKIP() tests ─────────────────────────
    for (const auto &s : skipped_) {
      // GTest 1.10 emits:  <testcase name="X" status="notrun" … />
      // We replace the /> with:  ><skipped message="…"/></testcase>
      const std::string needle = "name=\"" + s.name + "\" status=\"notrun\"";
      auto pos = xml.find(needle);
      if (pos == std::string::npos)
        continue;
      auto end = xml.find("/>", pos);
      if (end == std::string::npos)
        continue;
      xml.replace(end, 2,
                  "><skipped message=\"" + esc(s.message) + "\"/></testcase");
    }

    // ── 3. Update <testsuite errors="…" failures="…" skipped="…"> ────────
    set_attr(xml, "errors", error_tests_.size());
    // failures = total failures minus those we reclassified as errors
    size_t orig_failures = count_elements(xml, "<failure");
    set_attr(xml, "failures", orig_failures); // already patched above
    set_attr(xml, "skipped", skipped_.size());

    std::ofstream out(path_);
    out << xml;
  }

private:
  struct Skip {
    std::string name, message;
  };

  std::string path_, current_;
  bool threw_ = false;
  std::vector<Skip> skipped_;
  std::set<std::string> error_tests_;

  // Replace the first <failure …> block inside the <testcase name="N"> with
  // <error …>
  static void replace_element_in_testcase(std::string &xml,
                                          const std::string &test_name,
                                          const std::string &from,
                                          const std::string &to) {
    const std::string tc_marker = "name=\"" + test_name + "\"";
    auto tc = xml.find(tc_marker);
    if (tc == std::string::npos)
      return;

    // opening tag
    auto open = xml.find("<" + from, tc);
    if (open == std::string::npos)
      return;
    xml.replace(open, 1 + from.size(), "<" + to);

    // closing tag (self-closing or explicit)
    // After replacement the opening tag size changed by (to.size()-from.size())
    size_t delta = to.size() > from.size() ? to.size() - from.size() : 0;
    auto close = xml.find("</" + from, open + delta);
    if (close != std::string::npos)
      xml.replace(close, 2 + from.size(), "</" + to);
  }

  static size_t count_elements(const std::string &xml, const std::string &tag) {
    size_t count = 0, pos = 0;
    while ((pos = xml.find(tag, pos)) != std::string::npos) {
      ++count;
      ++pos;
    }
    return count;
  }

  static void set_attr(std::string &xml, const std::string &attr,
                       size_t value) {
    auto pos = xml.find(attr + "=\"");
    if (pos == std::string::npos)
      return;
    auto s = pos + attr.size() + 2;
    auto e = xml.find('"', s);
    if (e != std::string::npos)
      xml.replace(s, e - s, std::to_string(value));
  }

  static std::string esc(const std::string &s) {
    std::string r;
    for (char c : s)
      switch (c) {
      case '&':
        r += "&amp;";
        break;
      case '<':
        r += "&lt;";
        break;
      case '>':
        r += "&gt;";
        break;
      case '"':
        r += "&quot;";
        break;
      default:
        r += c;
      }
    return r;
  }
};

// =============================================================================
//  Test fixture
// =============================================================================
class WaypointTest : public ::testing::Test {
public:
  static void SetUpTestSuite() {
    node_ = rclcpp::Node::make_shared("waypoint_test_node");
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([]() { executor_->spin(); });

    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/fastbot/odom", 10, &WaypointTest::odomCallback);

    auto reset = node_->create_client<std_srvs::srv::Empty>("/reset_world");
    if (!reset->wait_for_service(3s)) {
      RCLCPP_WARN(node_->get_logger(), "reset_world not available");
    } else {
      reset->async_send_request(
          std::make_shared<std_srvs::srv::Empty::Request>());
      std::this_thread::sleep_for(500ms);
      RCLCPP_INFO(node_->get_logger(), "World reset");
    }
    std::this_thread::sleep_for(500ms);

    {
      std::unique_lock<std::mutex> lk(pose_mutex_);
      odom_cv_.wait_for(lk, 10s, []() { return !first_time_.load(); });
    }
    if (first_time_.load()) {
      setup_error_msg_ = "No /odom after 10 s";
      return;
    }

    goal_x_ = start_pose_x_ + TARGET_X;
    goal_y_ = start_pose_y_ + TARGET_Y;
    coords_valid_ = (std::hypot(goal_x_, goal_y_) <= MAP_RADIUS);

    RCLCPP_INFO(node_->get_logger(),
                "start=(%.4f,%.4f) | goal=(%.4f,%.4f) | dist=%.4f | valid=%s",
                start_pose_x_, start_pose_y_, goal_x_, goal_y_,
                std::hypot(goal_x_, goal_y_), coords_valid_ ? "true" : "false");

    if (!coords_valid_) {
      RCLCPP_WARN(
          node_->get_logger(),
          "Goal (%.4f,%.4f) outside MAP_RADIUS %.1f — navigation skipped",
          goal_x_, goal_y_, MAP_RADIUS);
      return; // no setup_error_msg_ → Position throws (error), Yaw skips
    }

    action_client_ = rclcpp_action::create_client<WaypointAction>(
        node_, "fastbot_waypoint_as");
    if (!action_client_->wait_for_action_server(10s)) {
      setup_error_msg_ = "Action server not available (timeout 10 s)";
      return;
    }

    WaypointAction::Goal gm;
    gm.position.x = goal_x_;
    gm.position.y = goal_y_;
    auto sf = action_client_->async_send_goal(gm);
    if (sf.wait_for(10s) != std::future_status::ready) {
      setup_error_msg_ = "Timeout waiting for goal acceptance";
      return;
    }
    auto gh = sf.get();
    if (!gh) {
      setup_error_msg_ = "Goal rejected";
      return;
    }

    auto rf = action_client_->async_get_result(gh);
    if (rf.wait_for(60s) != std::future_status::ready) {
      setup_error_msg_ = "Timeout waiting for result (60 s)";
      return;
    }
    auto wr = rf.get();
    if (wr.code != rclcpp_action::ResultCode::SUCCEEDED) {
      setup_error_msg_ = "Action ResultCode=" + std::to_string((int)wr.code);
      return;
    }
    if (!wr.result->success) {
      setup_error_msg_ = "success=false";
      return;
    }

    {
      std::lock_guard<std::mutex> lk(pose_mutex_);
      final_x_ = current_x_;
      final_y_ = current_y_;
      final_yaw_ = current_yaw_;
    }

    RCLCPP_INFO(node_->get_logger(), "=== FINAL RESULT ===");
    RCLCPP_INFO(node_->get_logger(), "final=(%.4f, %.4f)  yaw=%.4f rad",
                final_x_, final_y_, final_yaw_);
  }

  static void TearDownTestSuite() {
    executor_->cancel();
    if (spin_thread_.joinable())
      spin_thread_.join();
    odom_sub_.reset();
    action_client_.reset();
    node_.reset();
  }

  static void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    bool notify = false;
    {
      std::lock_guard<std::mutex> lk(pose_mutex_);
      current_x_ = msg->pose.pose.position.x;
      current_y_ = msg->pose.pose.position.y;
      tf2::Quaternion q(
          msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
          msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
      current_yaw_ = tf2::getYaw(q);
      if (first_time_.load()) {
        start_pose_x_ = current_x_;
        start_pose_y_ = current_y_;
        first_time_.store(false);
        RCLCPP_INFO(node_->get_logger(), "Start position X: %.4f | Y: %.4f",
                    start_pose_x_, start_pose_y_);
        notify = true;
      }
    }
    if (notify)
      odom_cv_.notify_all();
  }

  static void checkSetup() {
    if (!setup_error_msg_.empty())
      FAIL() << "SetUpTestSuite failed: " << setup_error_msg_;
  }

  static rclcpp::Node::SharedPtr node_;
  static std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  static std::thread spin_thread_;
  static rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  static rclcpp_action::Client<WaypointAction>::SharedPtr action_client_;
  static std::mutex pose_mutex_;
  static std::condition_variable odom_cv_;
  static std::atomic<bool> first_time_;
  static bool coords_valid_;
  static std::string setup_error_msg_;
  static double current_x_, current_y_, current_yaw_;
  static double start_pose_x_, start_pose_y_;
  static double final_x_, final_y_, final_yaw_;
  static double goal_x_, goal_y_;
};

rclcpp::Node::SharedPtr WaypointTest::node_;
std::shared_ptr<rclcpp::executors::MultiThreadedExecutor>
    WaypointTest::executor_;
std::thread WaypointTest::spin_thread_;
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
    WaypointTest::odom_sub_;
rclcpp_action::Client<WaypointAction>::SharedPtr WaypointTest::action_client_;
std::mutex WaypointTest::pose_mutex_;
std::condition_variable WaypointTest::odom_cv_;
std::atomic<bool> WaypointTest::first_time_(true);
bool WaypointTest::coords_valid_ = false;
std::string WaypointTest::setup_error_msg_;
double WaypointTest::current_x_ = 0, WaypointTest::current_y_ = 0,
       WaypointTest::current_yaw_ = 0;
double WaypointTest::start_pose_x_ = 0, WaypointTest::start_pose_y_ = 0;
double WaypointTest::final_x_ = 0, WaypointTest::final_y_ = 0,
       WaypointTest::final_yaw_ = 0;
double WaypointTest::goal_x_ = 0, WaypointTest::goal_y_ = 0;

// =============================================================================
//  TEST 1 — End position
//  throw → GTest reports <failure type=""> with "C++ exception" message
//         → ResultPatchListener converts it to <error> in the XML
// =============================================================================
TEST_F(WaypointTest, TestEndPosition) {
  checkSetup();
  const double err = std::hypot(goal_x_ - final_x_, goal_y_ - final_y_);
  if (err > POSITION_TOLERANCE) {
    std::ostringstream m;
    m << "Position error " << err << " m > " << POSITION_TOLERANCE << " m"
      << " | final=(" << final_x_ << "," << final_y_ << ")"
      << " | goal=(" << goal_x_ << "," << goal_y_ << ")";
    if (!coords_valid_)
      m << " [goal was outside map — navigation skipped]";
    throw std::runtime_error(m.str());
  }
}

// =============================================================================
//  TEST 2 — End yaw
//  GTEST_SKIP() → ResultPatchListener adds <skipped/> to the XML
// =============================================================================
TEST_F(WaypointTest, TestEndYaw) {
  checkSetup();
  if (!coords_valid_) {
    GTEST_SKIP() << "Goal outside map — yaw test skipped "
                    "(TestEndPosition already reports the error)";
  }
  const double expected =
      std::atan2(goal_y_ - start_pose_y_, goal_x_ - start_pose_x_);
  const double diff = normalize_angle(final_yaw_ - expected);
  if (std::fabs(diff) > YAW_TOLERANCE) {
    std::ostringstream m;
    m << "Yaw error " << std::fabs(diff) * 180 / M_PI << " deg > "
      << YAW_TOLERANCE * 180 / M_PI << " deg"
      << " | final=" << final_yaw_ * 180 / M_PI
      << " | expected=" << expected * 180 / M_PI;
    throw std::runtime_error(m.str());
  }
}

// =============================================================================
//  main
// =============================================================================
int main(int argc, char **argv) {
  // ── Bug Fix: parse XML path BEFORE InitGoogleTest ─────────────────────────
  // InitGoogleTest(&argc, argv) removes --gtest_output=xml:… from argv.
  // If we search for it afterwards the string is already gone and xml_path
  // stays empty — the listener opens nothing and patches nothing.
  std::string xml_path;
  const std::string prefix = "--gtest_output=xml:";
  for (int i = 1; i < argc; ++i) {
    std::string a(argv[i]);
    if (a.rfind(prefix, 0) == 0) {
      xml_path = a.substr(prefix.size());
      break;
    }
  }

  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);

  ::testing::UnitTest::GetInstance()->listeners().Append(
      new ResultPatchListener(xml_path));

  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}