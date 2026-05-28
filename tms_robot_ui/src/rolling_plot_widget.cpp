#include "tms_robot_ui/rolling_plot_widget.hpp"

#include <algorithm>
#include <QPainter>
#include <QPen>
#include <QPainterPath>

RollingPlotWidget::RollingPlotWidget(QWidget * parent) : QWidget(parent) {
  timer_.start();
  setMinimumSize(220, 110);
}

void RollingPlotWidget::setTitle(const QString & title) {
  title_ = title;
  update();
}

void RollingPlotWidget::setYAxisLabel(const QString & label) {
  y_axis_label_ = label;
  update();
}

void RollingPlotWidget::setYRange(double min_y, double max_y) {
  min_y_ = min_y;
  max_y_ = max_y;
  update();
}

void RollingPlotWidget::setWindowSec(double window_sec) {
  window_sec_ = std::max(1.0, window_sec);
  update();
}

void RollingPlotWidget::setReferenceLines(const std::vector<double> & values) {
  reference_lines_ = values;
  update();
}

double RollingPlotWidget::nowSec() const {
  return static_cast<double>(timer_.elapsed()) / 1000.0;
}

void RollingPlotWidget::addSample(double value) {
  const double now = nowSec();
  samples_.push_back({now, value});
  while (!samples_.empty() && samples_.front().t_sec < now - window_sec_) {
    samples_.pop_front();
  }
  update();
}

void RollingPlotWidget::clear() {
  samples_.clear();
  timer_.restart();
  update();
}

double RollingPlotWidget::mapX(double t_sec, double now_sec, const QRectF & plot_rect) const {
  const double left_time = now_sec - window_sec_;
  const double ratio = (t_sec - left_time) / window_sec_;
  return plot_rect.left() + std::clamp(ratio, 0.0, 1.0) * plot_rect.width();
}

double RollingPlotWidget::mapY(double value, const QRectF & plot_rect) const {
  if (max_y_ <= min_y_) {
    return plot_rect.center().y();
  }
  const double ratio = (value - min_y_) / (max_y_ - min_y_);
  return plot_rect.bottom() - std::clamp(ratio, 0.0, 1.0) * plot_rect.height();
}

void RollingPlotWidget::paintEvent(QPaintEvent *) {
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.fillRect(rect(), palette().window());
  const QRectF plot_rect = QRectF(38, 22, width() - 48, height() - 36);
  painter.setPen(QPen(Qt::gray, 1));
  painter.drawRect(plot_rect);
  painter.setPen(QPen(Qt::black, 1));
  painter.drawText(QRectF(4, 2, width() - 8, 18), Qt::AlignLeft | Qt::AlignVCenter, title_);
  painter.setPen(QPen(Qt::darkGray, 1));
  painter.drawText(QRectF(2, plot_rect.top(), 34, 16), Qt::AlignRight | Qt::AlignVCenter, QString::number(max_y_, 'f', 0));
  painter.drawText(QRectF(2, plot_rect.bottom() - 16, 34, 16), Qt::AlignRight | Qt::AlignVCenter, QString::number(min_y_, 'f', 0));
  painter.setPen(QPen(Qt::darkGray, 1, Qt::DashLine));
  for (const double ref : reference_lines_) {
    if (ref < min_y_ || ref > max_y_) {
      continue;
    }
    const double y = mapY(ref, plot_rect);
    painter.drawLine(QPointF(plot_rect.left(), y), QPointF(plot_rect.right(), y)); 
  }
  if (samples_.size() < 2) {
    return;
  }
  const double now = nowSec();
  QPainterPath path;
  bool first = true;
  for (const auto & s : samples_) {
    const double x = mapX(s.t_sec, now, plot_rect);
    const double y = mapY(s.value, plot_rect);
    if (first) {
      path.moveTo(x, y);
      first = false;
    } 
    else {
      path.lineTo(x, y);
    }
  }
  painter.setPen(QPen(Qt::blue, 2));
  painter.drawPath(path);
  // const auto latest = samples_.back().value;
  // painter.setPen(QPen(Qt::black, 1));
  // painter.drawText(QRectF(plot_rect.left(), plot_rect.bottom() + 2, plot_rect.width(), 16), 
  //   Qt::AlignRight | Qt::AlignVCenter,
  //   QString("latest %1").arg(latest, 0, 'f', 2));
}