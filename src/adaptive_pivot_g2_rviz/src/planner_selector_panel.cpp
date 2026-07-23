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

#include "adaptive_pivot_g2_rviz/planner_selector_panel.hpp"

#include <QComboBox>
#include <QFont>
#include <QLabel>
#include <QMetaObject>
#include <QPushButton>
#include <QString>
#include <QVariant>
#include <QVBoxLayout>

#include <array>
#include <string>
#include <utility>

#include "adaptive_pivot_g2_rviz/planner_catalog.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/config.hpp"
#include "rviz_common/display_context.hpp"

namespace adaptive_pivot_g2_rviz
{

namespace
{

struct PlannerDisplay
{
  const char * id;
  const char * label;
};

constexpr std::array<PlannerDisplay, 5> kPlannerDisplays = {{
  {"NavFnAStar", "NavFn A* — lưới 2D"},
  {"NavFnDijkstra", "NavFn Dijkstra — lưới 2D"},
  {"ThetaStar", "Theta* — any-angle"},
  {"Smac2D", "Smac 2D — cost-aware"},
  {"SmacHybrid", "Smac Hybrid — Dubins"},
}};

}  // namespace

PlannerSelectorPanel::PlannerSelectorPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * title = new QLabel("CHỌN GLOBAL PLANNER", this);
  QFont title_font = title->font();
  title_font.setBold(true);
  title->setFont(title_font);

  auto * help = new QLabel(
    "Chọn planner rồi nhấn nút bên dưới. Nếu đã có 2D Goal Pose, "
    "hệ thống sẽ tự tính lại đường tới goal đó.",
    this);
  help->setWordWrap(true);

  planner_combo_ = new QComboBox(this);
  for (const auto & planner : kPlannerDisplays) {
    planner_combo_->addItem(planner.label, planner.id);
  }
  setComboPlanner("ThetaStar");

  apply_button_ = new QPushButton("Áp dụng và lập lại đường", this);
  status_label_ = new QLabel("Đang chờ node so sánh đường...", this);
  status_label_->setWordWrap(true);

  auto * layout = new QVBoxLayout;
  layout->addWidget(title);
  layout->addWidget(help);
  layout->addWidget(planner_combo_);
  layout->addWidget(apply_button_);
  layout->addWidget(status_label_);
  layout->addStretch();
  setLayout(layout);

  connect(
    apply_button_, &QPushButton::clicked,
    this, &PlannerSelectorPanel::applySelection);
  connect(
    planner_combo_,
    QOverload<int>::of(&QComboBox::currentIndexChanged),
    this,
    [this](int) {Q_EMIT configChanged();});
}

void PlannerSelectorPanel::onInitialize()
{
  const auto abstraction =
    getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    status_label_->setText("Lỗi: RViz không cung cấp ROS node.");
    apply_button_->setEnabled(false);
    return;
  }

  node_ = abstraction->get_raw_node();
  const auto selection_qos = rclcpp::QoS(1).transient_local().reliable();
  selection_publisher_ = node_->create_publisher<std_msgs::msg::String>(
    "/planner_selector", selection_qos);
  const auto status_qos = rclcpp::QoS(1).transient_local().reliable();
  status_subscription_ = node_->create_subscription<std_msgs::msg::String>(
    "/research/planner_active",
    status_qos,
    [this](std_msgs::msg::String::SharedPtr message) {
      updateActivePlanner(std::move(message));
    });
}

void PlannerSelectorPanel::applySelection()
{
  if (!selection_publisher_) {
    status_label_->setText("Planner selector chưa kết nối ROS.");
    return;
  }
  const QString planner_id = planner_combo_->currentData().toString();
  if (!is_supported_planner(planner_id.toStdString())) {
    status_label_->setText("Lỗi: planner trong panel không hợp lệ.");
    return;
  }

  std_msgs::msg::String message;
  message.data = planner_id.toStdString();
  selection_publisher_->publish(message);
  status_label_->setText(
    QString("Đã gửi yêu cầu: %1. Đang chờ xác nhận...")
    .arg(planner_id));
}

void PlannerSelectorPanel::updateActivePlanner(
  const std_msgs::msg::String::SharedPtr message)
{
  const QString planner_id = QString::fromStdString(message->data);
  QMetaObject::invokeMethod(
    this,
    [this, planner_id]() {
      setComboPlanner(planner_id);
      status_label_->setText(
        QString("Planner đang hoạt động: %1").arg(planner_id));
    },
    Qt::QueuedConnection);
}

void PlannerSelectorPanel::setComboPlanner(const QString & planner_id)
{
  for (int index = 0; index < planner_combo_->count(); ++index) {
    if (planner_combo_->itemData(index).toString() == planner_id) {
      planner_combo_->setCurrentIndex(index);
      return;
    }
  }
}

void PlannerSelectorPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  QString planner_id;
  if (config.mapGetString("Selected Planner", &planner_id)) {
    setComboPlanner(planner_id);
  }
}

void PlannerSelectorPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue(
    "Selected Planner", planner_combo_->currentData().toString());
}

}  // namespace adaptive_pivot_g2_rviz

PLUGINLIB_EXPORT_CLASS(
  adaptive_pivot_g2_rviz::PlannerSelectorPanel,
  rviz_common::Panel)
