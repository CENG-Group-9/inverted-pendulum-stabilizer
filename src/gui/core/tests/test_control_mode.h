#pragma once

#include <QObject>

namespace pendulum::gui {

class ControlModeTest final : public QObject {
    Q_OBJECT

  private slots:
    void display_name_is_not_empty();
    void display_names_are_distinct();
    void display_name_matches_known_modes();
};

} // namespace pendulum::gui
