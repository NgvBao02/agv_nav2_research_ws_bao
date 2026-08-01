// Copyright 2026 Adaptive Pivot-G2 Research Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ADAPTIVE_PIVOT_G2_RVIZ__PLANNER_SELECTOR_PANEL_HPP_
#define ADAPTIVE_PIVOT_G2_RVIZ__PLANNER_SELECTOR_PANEL_HPP_

#include <QWidget>

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "std_msgs/msg/string.hpp"

class QComboBox;
class QLabel;
class QPushButton;
class QTableWidget;

namespace adaptive_pivot_g2_rviz
{

class PlannerSelectorPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit PlannerSelectorPanel(QWidget * parent = nullptr);
  ~PlannerSelectorPanel() override;

  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void applySelection();
  void applyExecutionMethod();
  void applyEnvironment();
  void showAllSmoothers();
  void showRawOnly();

private:
  void updateActivePlanner(const std_msgs::msg::String::SharedPtr message);
  void updateActiveExecutionMethod(
    const std_msgs::msg::String::SharedPtr message);
  void updateActiveEnvironment(const std_msgs::msg::String::SharedPtr message);
  void updateEnvironmentStatus(const std_msgs::msg::String::SharedPtr message);
  void updateSmootherVisibility(
    const std_msgs::msg::String::SharedPtr message);
  void updateMetrics(const std_msgs::msg::String::SharedPtr message);
  void setComboPlanner(const QString & planner_id);
  void setComboExecutionMethod(const QString & method_id);
  void setComboEnvironment(const QString & environment_id);
  void setSmootherVisibility(
    const std::vector<std::string> & visible_methods);
  void publishSmootherVisibility();
  std::vector<std::string> selectedSmoothers() const;
  void clearMetricsTable();

  QComboBox * environment_combo_{nullptr};
  QPushButton * environment_apply_button_{nullptr};
  QLabel * environment_status_label_{nullptr};
  QComboBox * planner_combo_{nullptr};
  QPushButton * apply_button_{nullptr};
  QLabel * status_label_{nullptr};
  QComboBox * execution_method_combo_{nullptr};
  QPushButton * execution_method_apply_button_{nullptr};
  QLabel * execution_method_status_label_{nullptr};
  std::vector<QPushButton *> smoother_buttons_;
  QPushButton * show_all_smoothers_button_{nullptr};
  QPushButton * show_raw_only_button_{nullptr};
  QLabel * smoother_status_label_{nullptr};
  QTableWidget * metrics_table_{nullptr};
  bool updating_smoother_buttons_{false};
  std::atomic_bool shutting_down_{false};
  int metrics_generation_{-1};

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr selection_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    execution_method_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    execution_method_status_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    smoother_visibility_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    smoother_visibility_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    metrics_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr environment_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    environment_active_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    environment_status_subscription_;
};

}  // namespace adaptive_pivot_g2_rviz

#endif  // ADAPTIVE_PIVOT_G2_RVIZ__PLANNER_SELECTOR_PANEL_HPP_
