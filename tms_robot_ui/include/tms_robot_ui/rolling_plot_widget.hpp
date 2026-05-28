#pragma once

#include <deque>
#include <string>
#include <vector>
#include <QElapsedTimer>
#include <QWidget>

class RollingPlotWidget : public QWidget {
public:
  explicit RollingPlotWidget(QWidget * parent = nullptr);
  void setTitle(const QString & title);
  void setYAxisLabel(const QString & label);
  void setYRange(double min_y, double max_y);
  void setWindowSec(double window_sec);
  void setReferenceLines(const std::vector<double> & values);
  void addSample(double value);
  void clear();

protected:
  void paintEvent(QPaintEvent * event) override;

private:
  struct Sample { double t_sec; double value; };
  double nowSec() const;
  double mapX(double t_sec, double now_sec, const QRectF & plot_rect) const;
  double mapY(double value, const QRectF & plot_rect) const;
  QString title_{"Plot"};
  QString y_axis_label_;
  double min_y_{0.0};
  double max_y_{1.0};
  double window_sec_{10.0};
  std::vector<double> reference_lines_;
  std::deque<Sample> samples_;
  QElapsedTimer timer_;
};