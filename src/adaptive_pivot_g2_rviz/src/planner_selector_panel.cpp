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

#include <QAbstractItemView>
#include <QComboBox>
#include <QFont>
#include <QGridLayout>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QLabel>
#include <QMetaObject>
#include <QPushButton>
#include <QScrollArea>
#include <QString>
#include <QStringList>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QVariant>
#include <QVBoxLayout>

#include <algorithm>
#include <array>
#include <string>
#include <utility>

#include "adaptive_pivot_g2_rviz/environment_catalog.hpp"
#include "adaptive_pivot_g2_rviz/planner_catalog.hpp"
#include "adaptive_pivot_g2_rviz/smoother_catalog.hpp"
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

struct EnvironmentDisplay
{
  const char * id;
  const char * label;
};

constexpr std::array<EnvironmentDisplay, 7> kEnvironmentDisplays = {{
  {"research_warehouse", "Kho nghiên cứu — tổng hợp"},
  {"warehouse_long_aisles", "Kho — các lối đi dài"},
  {"warehouse_cross_aisles", "Kho — các lối đi giao cắt"},
  {"warehouse_dispatch", "Kho — lưu trữ và dispatch"},
  {"narrow_aisles", "Kho — lối đi hẹp zíc-zắc"},
  {"office_maze", "Văn phòng — nhiều phòng và cửa"},
  {"open_arena", "Sân mở — vật cản thưa"},
}};

struct SmootherDisplay
{
  const char * id;
  const char * label;
  const char * color;
};

constexpr std::array<SmootherDisplay, 5> kSmootherDisplays = {{
  {"simple", "Nav2 Simple", "255, 190, 0"},
  {"savitzky_golay", "Nav2 Savitzky–Golay", "0, 220, 255"},
  {"constrained", "Nav2 Constrained", "50, 220, 90"},
  {"pstmo", "PSTMO", "220, 40, 255"},
  {"adaptive_hybrid", "Adaptive Hybrid", "80, 100, 255"},
}};

struct ExecutionMethodDisplay
{
  const char * id;
  const char * label;
};

constexpr std::array<ExecutionMethodDisplay, 6> kExecutionMethodDisplays = {{
  {"raw", "RAW — không smoothing"},
  {"simple", "Nav2 Simple"},
  {"savitzky_golay", "Nav2 Savitzky–Golay"},
  {"constrained", "Nav2 Constrained"},
  {"pstmo", "PSTMO"},
  {"adaptive_hybrid", "Adaptive Hybrid"},
}};

int metricsRow(const QString & method)
{
  if (method == "raw") {
    return 0;
  }
  for (std::size_t index = 0; index < kSmootherDisplays.size(); ++index) {
    if (method == kSmootherDisplays[index].id) {
      return static_cast<int>(index + 1);
    }
  }
  return -1;
}

}  // namespace

PlannerSelectorPanel::PlannerSelectorPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * environment_title = new QLabel("MÔI TRƯỜNG GAZEBO / NAV2", this);
  QFont title_font = environment_title->font();
  title_font.setBold(true);
  environment_title->setFont(title_font);

  auto * environment_help = new QLabel(
    "Chọn map rồi nhấn nút. Gazebo world, Nav2 map và vị trí robot "
    "sẽ được thay đồng bộ; RViz vẫn giữ nguyên.",
    this);
  environment_help->setWordWrap(true);

  environment_combo_ = new QComboBox(this);
  for (const auto & environment : kEnvironmentDisplays) {
    environment_combo_->addItem(environment.label, environment.id);
  }
  setComboEnvironment("research_warehouse");

  environment_apply_button_ = new QPushButton(
    "Đổi map và khởi động lại mô phỏng", this);
  environment_apply_button_->setStyleSheet(
    "QPushButton { background-color: #1565c0; color: white; "
    "font-weight: bold; padding: 6px; }");
  environment_status_label_ = new QLabel(
    "Đang chờ bộ quản lý môi trường...", this);
  environment_status_label_->setWordWrap(true);

  auto * title = new QLabel("CHỌN GLOBAL PLANNER", this);
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

  auto * execution_method_title = new QLabel(
    "CHỌN SMOOTHER ĐỂ XE ĐI THEO", this);
  execution_method_title->setFont(title_font);

  auto * execution_method_help = new QLabel(
    "Chọn RAW hoặc một smoother rồi nhấn áp dụng. Nếu đã có 2D Goal Pose, "
    "hệ thống sẽ lập lại đường và xe sẽ đi theo đúng phương pháp này.",
    this);
  execution_method_help->setWordWrap(true);

  execution_method_combo_ = new QComboBox(this);
  for (const auto & method : kExecutionMethodDisplays) {
    execution_method_combo_->addItem(method.label, method.id);
  }
  setComboExecutionMethod("simple");

  execution_method_apply_button_ = new QPushButton(
    "Áp dụng smoother và chạy lại đường", this);
  execution_method_status_label_ = new QLabel(
    "Đang chờ phương pháp thực thi...", this);
  execution_method_status_label_->setWordWrap(true);

  auto * smoother_title = new QLabel("SO SÁNH TRƯỚC / SAU SMOOTH", this);
  smoother_title->setFont(title_font);

  auto * smoother_help = new QLabel(
    "RAW luôn hiển thị làm chuẩn. Bật/tắt riêng từng phương pháp để "
    "chồng đường và so sánh cùng một đầu vào.",
    this);
  smoother_help->setWordWrap(true);

  auto * smoother_grid = new QGridLayout;
  for (std::size_t index = 0; index < kSmootherDisplays.size(); ++index) {
    const auto & smoother = kSmootherDisplays[index];
    auto * button = new QPushButton(smoother.label, this);
    button->setCheckable(true);
    button->setChecked(true);
    button->setProperty("smoother_id", smoother.id);
    button->setStyleSheet(
      QString(
        "QPushButton { padding: 5px; text-align: left; } "
        "QPushButton:checked { background-color: rgb(%1); color: #111; "
        "font-weight: bold; border: 2px solid white; } "
        "QPushButton:!checked { background-color: #444; color: #bbb; "
        "border: 1px solid #777; }")
      .arg(smoother.color));
    smoother_buttons_.push_back(button);
    smoother_grid->addWidget(
      button,
      static_cast<int>(index / 2),
      static_cast<int>(index % 2));
    connect(
      button, &QPushButton::toggled,
      this,
      [this](bool) {
        if (!updating_smoother_buttons_) {
          publishSmootherVisibility();
          Q_EMIT configChanged();
        }
      });
  }

  show_all_smoothers_button_ = new QPushButton("Hiện tất cả", this);
  show_raw_only_button_ = new QPushButton("Chỉ RAW", this);
  auto * smoother_actions = new QHBoxLayout;
  smoother_actions->addWidget(show_all_smoothers_button_);
  smoother_actions->addWidget(show_raw_only_button_);

  smoother_status_label_ = new QLabel(
    "Đang chờ node so sánh xác nhận...", this);
  smoother_status_label_->setWordWrap(true);

  metrics_table_ = new QTableWidget(
    static_cast<int>(kSmootherDisplays.size() + 1), 5, this);
  metrics_table_->setHorizontalHeaderLabels(
    {"Phương pháp", "κ max", "Eκ", "L (m)", "t (ms)"});
  metrics_table_->verticalHeader()->setVisible(false);
  metrics_table_->horizontalHeader()->setSectionResizeMode(
    QHeaderView::ResizeToContents);
  metrics_table_->horizontalHeader()->setStretchLastSection(true);
  metrics_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  metrics_table_->setSelectionMode(QAbstractItemView::NoSelection);
  metrics_table_->setAlternatingRowColors(true);
  metrics_table_->setMinimumHeight(190);
  metrics_table_->setItem(0, 0, new QTableWidgetItem("RAW"));
  for (std::size_t index = 0; index < kSmootherDisplays.size(); ++index) {
    metrics_table_->setItem(
      static_cast<int>(index + 1), 0,
      new QTableWidgetItem(kSmootherDisplays[index].label));
  }
  clearMetricsTable();

  auto * content_layout = new QVBoxLayout;
  content_layout->addWidget(environment_title);
  content_layout->addWidget(environment_help);
  content_layout->addWidget(environment_combo_);
  content_layout->addWidget(environment_apply_button_);
  content_layout->addWidget(environment_status_label_);
  content_layout->addSpacing(12);
  content_layout->addWidget(title);
  content_layout->addWidget(help);
  content_layout->addWidget(planner_combo_);
  content_layout->addWidget(apply_button_);
  content_layout->addWidget(status_label_);
  content_layout->addSpacing(12);
  content_layout->addWidget(execution_method_title);
  content_layout->addWidget(execution_method_help);
  content_layout->addWidget(execution_method_combo_);
  content_layout->addWidget(execution_method_apply_button_);
  content_layout->addWidget(execution_method_status_label_);
  content_layout->addSpacing(12);
  content_layout->addWidget(smoother_title);
  content_layout->addWidget(smoother_help);
  content_layout->addLayout(smoother_grid);
  content_layout->addLayout(smoother_actions);
  content_layout->addWidget(smoother_status_label_);
  content_layout->addWidget(metrics_table_);
  content_layout->addStretch();

  auto * content = new QWidget(this);
  content->setLayout(content_layout);
  auto * scroll_area = new QScrollArea(this);
  scroll_area->setWidgetResizable(true);
  scroll_area->setFrameShape(QFrame::NoFrame);
  scroll_area->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  scroll_area->setWidget(content);

  auto * panel_layout = new QVBoxLayout;
  panel_layout->setContentsMargins(0, 0, 0, 0);
  panel_layout->addWidget(scroll_area);
  setLayout(panel_layout);

  connect(
    environment_apply_button_, &QPushButton::clicked,
    this, &PlannerSelectorPanel::applyEnvironment);
  connect(
    environment_combo_,
    QOverload<int>::of(&QComboBox::currentIndexChanged),
    this,
    [this](int) {Q_EMIT configChanged();});
  connect(
    apply_button_, &QPushButton::clicked,
    this, &PlannerSelectorPanel::applySelection);
  connect(
    planner_combo_,
    QOverload<int>::of(&QComboBox::currentIndexChanged),
    this,
    [this](int) {Q_EMIT configChanged();});
  connect(
    execution_method_apply_button_, &QPushButton::clicked,
    this, &PlannerSelectorPanel::applyExecutionMethod);
  connect(
    execution_method_combo_,
    QOverload<int>::of(&QComboBox::currentIndexChanged),
    this,
    [this](int) {Q_EMIT configChanged();});
  connect(
    show_all_smoothers_button_, &QPushButton::clicked,
    this, &PlannerSelectorPanel::showAllSmoothers);
  connect(
    show_raw_only_button_, &QPushButton::clicked,
    this, &PlannerSelectorPanel::showRawOnly);
}

PlannerSelectorPanel::~PlannerSelectorPanel()
{
  // RViz spins its ROS node on worker threads.  Stop callbacks from queuing Qt
  // work while QObject children are being destroyed during Ctrl-C shutdown.
  shutting_down_.store(true, std::memory_order_release);
  environment_status_subscription_.reset();
  environment_active_subscription_.reset();
  metrics_subscription_.reset();
  execution_method_status_subscription_.reset();
  smoother_visibility_subscription_.reset();
  status_subscription_.reset();
  environment_publisher_.reset();
  smoother_visibility_publisher_.reset();
  execution_method_publisher_.reset();
  selection_publisher_.reset();
  node_.reset();
}

void PlannerSelectorPanel::onInitialize()
{
  const auto abstraction =
    getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    status_label_->setText("Lỗi: RViz không cung cấp ROS node.");
    environment_status_label_->setText(
      "Lỗi: RViz không cung cấp ROS node.");
    execution_method_status_label_->setText(
      "Lỗi: RViz không cung cấp ROS node.");
    environment_apply_button_->setEnabled(false);
    apply_button_->setEnabled(false);
    execution_method_apply_button_->setEnabled(false);
    execution_method_combo_->setEnabled(false);
    for (auto * button : smoother_buttons_) {
      button->setEnabled(false);
    }
    show_all_smoothers_button_->setEnabled(false);
    show_raw_only_button_->setEnabled(false);
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
  execution_method_publisher_ =
    node_->create_publisher<std_msgs::msg::String>(
    "/research/execute_method", selection_qos);
  execution_method_status_subscription_ =
    node_->create_subscription<std_msgs::msg::String>(
    "/research/execute_method_active",
    status_qos,
    [this](std_msgs::msg::String::SharedPtr message) {
      updateActiveExecutionMethod(std::move(message));
    });
  smoother_visibility_publisher_ =
    node_->create_publisher<std_msgs::msg::String>(
    "/research/smoother_visibility", selection_qos);
  smoother_visibility_subscription_ =
    node_->create_subscription<std_msgs::msg::String>(
    "/research/smoother_visibility_active",
    status_qos,
    [this](std_msgs::msg::String::SharedPtr message) {
      updateSmootherVisibility(std::move(message));
    });
  const auto metrics_qos = rclcpp::QoS(32).transient_local().reliable();
  metrics_subscription_ =
    node_->create_subscription<std_msgs::msg::String>(
    "/research/metrics",
    metrics_qos,
    [this](std_msgs::msg::String::SharedPtr message) {
      updateMetrics(std::move(message));
    });
  environment_publisher_ =
    node_->create_publisher<std_msgs::msg::String>(
    "/research/environment_selector", selection_qos);
  environment_active_subscription_ =
    node_->create_subscription<std_msgs::msg::String>(
    "/research/environment_active",
    status_qos,
    [this](std_msgs::msg::String::SharedPtr message) {
      updateActiveEnvironment(std::move(message));
    });
  environment_status_subscription_ =
    node_->create_subscription<std_msgs::msg::String>(
    "/research/environment_status",
    status_qos,
    [this](std_msgs::msg::String::SharedPtr message) {
      updateEnvironmentStatus(std::move(message));
    });
  publishSmootherVisibility();
}

void PlannerSelectorPanel::applyEnvironment()
{
  if (!environment_publisher_) {
    environment_status_label_->setText(
      "Bộ chọn môi trường chưa kết nối ROS.");
    return;
  }
  if (environment_publisher_->get_subscription_count() == 0u) {
    environment_status_label_->setText(
      "Chưa có environment manager. Hãy chạy "
      "switchable_simulation.launch.py.");
    environment_apply_button_->setEnabled(true);
    return;
  }

  const QString environment_id =
    environment_combo_->currentData().toString();
  if (!is_supported_environment(environment_id.toStdString())) {
    environment_status_label_->setText(
      "Lỗi: môi trường trong panel không hợp lệ.");
    return;
  }

  std_msgs::msg::String message;
  message.data = environment_id.toStdString();
  environment_publisher_->publish(message);
  environment_apply_button_->setEnabled(false);
  environment_status_label_->setText(
    QString("Đã yêu cầu chuyển sang %1. Đang dừng phiên cũ...")
    .arg(environment_id));
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

void PlannerSelectorPanel::applyExecutionMethod()
{
  if (!execution_method_publisher_) {
    execution_method_status_label_->setText(
      "Bộ chọn smoother thực thi chưa kết nối ROS.");
    return;
  }
  const QString method_id =
    execution_method_combo_->currentData().toString();
  if (!is_supported_execution_method(method_id.toStdString())) {
    execution_method_status_label_->setText(
      "Lỗi: smoother thực thi trong panel không hợp lệ.");
    return;
  }

  std_msgs::msg::String message;
  message.data = method_id.toStdString();
  execution_method_publisher_->publish(message);
  execution_method_status_label_->setText(
    QString("Đã yêu cầu xe đi theo: %1. Đang chờ xác nhận...")
    .arg(method_id));
}

std::vector<std::string> PlannerSelectorPanel::selectedSmoothers() const
{
  std::vector<std::string> selected;
  for (const auto * button : smoother_buttons_) {
    if (button->isChecked()) {
      selected.push_back(
        button->property("smoother_id").toString().toStdString());
    }
  }
  return selected;
}

void PlannerSelectorPanel::publishSmootherVisibility()
{
  if (!smoother_visibility_publisher_) {
    return;
  }
  QJsonArray methods;
  for (const auto & method : selectedSmoothers()) {
    methods.append(QString::fromStdString(method));
  }
  QJsonObject object;
  object.insert("methods", methods);
  std_msgs::msg::String message;
  message.data = QJsonDocument(object)
    .toJson(QJsonDocument::Compact).toStdString();
  smoother_visibility_publisher_->publish(message);
  smoother_status_label_->setText(
    QString("Đã chọn %1 phương pháp smooth. Đang chờ xác nhận...")
    .arg(methods.size()));
}

void PlannerSelectorPanel::showAllSmoothers()
{
  std::vector<std::string> all_methods;
  for (const auto & smoother : kSmootherDisplays) {
    all_methods.emplace_back(smoother.id);
  }
  setSmootherVisibility(all_methods);
  publishSmootherVisibility();
  Q_EMIT configChanged();
}

void PlannerSelectorPanel::showRawOnly()
{
  setSmootherVisibility({});
  publishSmootherVisibility();
  Q_EMIT configChanged();
}

void PlannerSelectorPanel::updateActiveEnvironment(
  const std_msgs::msg::String::SharedPtr message)
{
  if (shutting_down_.load(std::memory_order_acquire)) {
    return;
  }
  const QString environment_id = QString::fromStdString(message->data);
  if (!is_supported_environment(environment_id.toStdString())) {
    return;
  }
  QMetaObject::invokeMethod(
    this,
    [this, environment_id]() {
      setComboEnvironment(environment_id);
      environment_apply_button_->setEnabled(true);
      environment_status_label_->setText(
        QString("Môi trường đang hoạt động: %1").arg(environment_id));
    },
    Qt::QueuedConnection);
}

void PlannerSelectorPanel::updateEnvironmentStatus(
  const std_msgs::msg::String::SharedPtr message)
{
  if (shutting_down_.load(std::memory_order_acquire)) {
    return;
  }
  const QString payload = QString::fromStdString(message->data);
  QMetaObject::invokeMethod(
    this,
    [this, payload]() {
      QJsonParseError parse_error;
      const auto document = QJsonDocument::fromJson(
        payload.toUtf8(), &parse_error);
      if (parse_error.error != QJsonParseError::NoError ||
      !document.isObject())
      {
        environment_status_label_->setText(payload);
        environment_apply_button_->setEnabled(true);
        return;
      }

      const auto object = document.object();
      const QString state = object.value("state").toString();
      const QString environment_id =
      object.value("environment").toString();
      const QString text = object.value("message").toString();
      if (is_supported_environment(environment_id.toStdString())) {
        setComboEnvironment(environment_id);
      }
      const bool busy =
      state == "starting" || state == "switching" ||
      state == "stopping";
      environment_apply_button_->setEnabled(!busy);
      if (!text.isEmpty()) {
        environment_status_label_->setText(text);
      }
    },
    Qt::QueuedConnection);
}

void PlannerSelectorPanel::updateActivePlanner(
  const std_msgs::msg::String::SharedPtr message)
{
  if (shutting_down_.load(std::memory_order_acquire)) {
    return;
  }
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

void PlannerSelectorPanel::updateActiveExecutionMethod(
  const std_msgs::msg::String::SharedPtr message)
{
  if (shutting_down_.load(std::memory_order_acquire)) {
    return;
  }
  const QString method_id = QString::fromStdString(message->data);
  if (!is_supported_execution_method(method_id.toStdString())) {
    return;
  }
  QMetaObject::invokeMethod(
    this,
    [this, method_id]() {
      setComboExecutionMethod(method_id);
      execution_method_status_label_->setText(
        QString("Xe sẽ đi theo: %1").arg(method_id));
    },
    Qt::QueuedConnection);
}

void PlannerSelectorPanel::updateSmootherVisibility(
  const std_msgs::msg::String::SharedPtr message)
{
  if (shutting_down_.load(std::memory_order_acquire)) {
    return;
  }
  const QString payload = QString::fromStdString(message->data);
  QMetaObject::invokeMethod(
    this,
    [this, payload]() {
      QJsonParseError parse_error;
      const auto document = QJsonDocument::fromJson(
        payload.toUtf8(), &parse_error);
      if (parse_error.error != QJsonParseError::NoError ||
      !document.isObject())
      {
        smoother_status_label_->setText(
          "Trạng thái smoother không hợp lệ.");
        return;
      }
      std::vector<std::string> visible_methods;
      for (const auto value :
      document.object().value("methods").toArray())
      {
        const auto method = value.toString().toStdString();
        if (is_supported_smoother(method)) {
          visible_methods.push_back(method);
        }
      }
      setSmootherVisibility(visible_methods);
      smoother_status_label_->setText(
        visible_methods.empty() ?
        "Đang chỉ hiển thị RAW màu đỏ." :
        QString("Đang hiển thị RAW + %1 phương pháp smooth.")
        .arg(visible_methods.size()));
    },
    Qt::QueuedConnection);
}

void PlannerSelectorPanel::updateMetrics(
  const std_msgs::msg::String::SharedPtr message)
{
  if (shutting_down_.load(std::memory_order_acquire)) {
    return;
  }
  const QString payload = QString::fromStdString(message->data);
  QMetaObject::invokeMethod(
    this,
    [this, payload]() {
      QJsonParseError parse_error;
      const auto document = QJsonDocument::fromJson(
        payload.toUtf8(), &parse_error);
      if (parse_error.error != QJsonParseError::NoError ||
      !document.isObject())
      {
        return;
      }
      const auto object = document.object();
      if (object.value("event").toString() != "path_ready") {
        return;
      }
      const int generation = object.value("generation").toInt(-1);
      if (generation > metrics_generation_) {
        metrics_generation_ = generation;
        clearMetricsTable();
      } else if (generation < metrics_generation_) {
        return;
      }
      const int row = metricsRow(object.value("method").toString());
      if (row < 0) {
        return;
      }
      const double elapsed_seconds =
      object.contains("smoothing_time_s") ?
      object.value("smoothing_time_s").toDouble() :
      object.value("planning_time_s").toDouble();
      metrics_table_->item(row, 1)->setText(
        QString::number(
          object.value("max_abs_curvature_1pm").toDouble(), 'f', 3));
      metrics_table_->item(row, 2)->setText(
        QString::number(
          object.value("curvature_energy_1pm").toDouble(), 'f', 3));
      metrics_table_->item(row, 3)->setText(
        QString::number(
          object.value("path_length_m").toDouble(), 'f', 3));
      metrics_table_->item(row, 4)->setText(
        QString::number(1000.0 * elapsed_seconds, 'f', 1));
    },
    Qt::QueuedConnection);
}

void PlannerSelectorPanel::setComboEnvironment(
  const QString & environment_id)
{
  for (int index = 0; index < environment_combo_->count(); ++index) {
    if (environment_combo_->itemData(index).toString() == environment_id) {
      environment_combo_->setCurrentIndex(index);
      return;
    }
  }
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

void PlannerSelectorPanel::setComboExecutionMethod(
  const QString & method_id)
{
  for (int index = 0; index < execution_method_combo_->count(); ++index) {
    if (
      execution_method_combo_->itemData(index).toString() == method_id)
    {
      execution_method_combo_->setCurrentIndex(index);
      return;
    }
  }
}

void PlannerSelectorPanel::setSmootherVisibility(
  const std::vector<std::string> & visible_methods)
{
  updating_smoother_buttons_ = true;
  for (auto * button : smoother_buttons_) {
    const auto method =
      button->property("smoother_id").toString().toStdString();
    button->setChecked(
      std::find(
        visible_methods.begin(), visible_methods.end(), method) !=
      visible_methods.end());
  }
  updating_smoother_buttons_ = false;
  Q_EMIT configChanged();
}

void PlannerSelectorPanel::clearMetricsTable()
{
  for (int row = 0; row < metrics_table_->rowCount(); ++row) {
    for (int column = 1; column < metrics_table_->columnCount(); ++column) {
      auto * item = metrics_table_->item(row, column);
      if (!item) {
        item = new QTableWidgetItem;
        item->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
        metrics_table_->setItem(row, column, item);
      }
      item->setText("—");
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
  QString execution_method;
  if (config.mapGetString("Selected Execution Method", &execution_method)) {
    setComboExecutionMethod(execution_method);
  }
  QString environment_id;
  if (config.mapGetString("Selected Environment", &environment_id)) {
    setComboEnvironment(environment_id);
  }
  QString visible_smoothers;
  if (config.mapGetString("Visible Smoothers", &visible_smoothers)) {
    std::vector<std::string> methods;
    for (const auto & method :
      visible_smoothers.split(',', Qt::SkipEmptyParts))
    {
      if (is_supported_smoother(method.toStdString())) {
        methods.push_back(method.toStdString());
      }
    }
    setSmootherVisibility(methods);
  } else {
    bool smoothers_enabled = true;
    if (
      config.mapGetBool("Smoothers Enabled", &smoothers_enabled) &&
      !smoothers_enabled)
    {
      setSmootherVisibility({});
    }
  }
}

void PlannerSelectorPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue(
    "Selected Planner", planner_combo_->currentData().toString());
  config.mapSetValue(
    "Selected Execution Method",
    execution_method_combo_->currentData().toString());
  config.mapSetValue(
    "Selected Environment",
    environment_combo_->currentData().toString());
  QStringList visible_smoothers;
  for (const auto & method : selectedSmoothers()) {
    visible_smoothers.append(QString::fromStdString(method));
  }
  config.mapSetValue(
    "Visible Smoothers", visible_smoothers.join(','));
  config.mapSetValue(
    "Smoothers Enabled", !visible_smoothers.empty());
}

}  // namespace adaptive_pivot_g2_rviz

PLUGINLIB_EXPORT_CLASS(
  adaptive_pivot_g2_rviz::PlannerSelectorPanel,
  rviz_common::Panel)
