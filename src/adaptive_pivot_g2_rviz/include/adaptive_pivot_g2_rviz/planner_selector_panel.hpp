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

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "std_msgs/msg/string.hpp"

class QComboBox;
class QLabel;
class QPushButton;

namespace adaptive_pivot_g2_rviz
{

class PlannerSelectorPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit PlannerSelectorPanel(QWidget * parent = nullptr);

  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void applySelection();

private:
  void updateActivePlanner(const std_msgs::msg::String::SharedPtr message);
  void setComboPlanner(const QString & planner_id);

  QComboBox * planner_combo_{nullptr};
  QPushButton * apply_button_{nullptr};
  QLabel * status_label_{nullptr};

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr selection_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_subscription_;
};

}  // namespace adaptive_pivot_g2_rviz

#endif  // ADAPTIVE_PIVOT_G2_RVIZ__PLANNER_SELECTOR_PANEL_HPP_
