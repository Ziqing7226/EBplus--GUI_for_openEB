// gui/config/config_manager.h — JSON config serialization & presets (design §3.7).
//
// Captures the live camera state (Biases, ROI, ESP, Trigger) into a JSON
// document and restores it. Ships built-in presets (High Sensitivity, Low
// Noise, Standard). Validates sensor generation compatibility on load.

#ifndef GUI_CONFIG_CONFIG_MANAGER_H
#define GUI_CONFIG_CONFIG_MANAGER_H

#include <QObject>
#include <QString>
#include <QStringList>
#include <QJsonObject>

namespace gui {

class CameraController;
class AlgoBridge;

class ConfigManager : public QObject {
    Q_OBJECT
public:
    explicit ConfigManager(QObject* parent = nullptr);

    /// @brief Snapshot the current camera state to a JSON object.
    QJsonObject capture(CameraController* controller) const;
    /// @brief Apply a JSON config to the camera.
    bool apply(CameraController* controller, const QJsonObject& obj, QString& err) const;

    bool save_to_file(CameraController* controller, const QString& path, QString& err) const;
    bool load_from_file(CameraController* controller, const QString& path, QString& err) const;

    /// @brief Built-in preset names (localized display strings).
    QStringList preset_names() const;
    /// @brief Apply a built-in preset by index.
    bool apply_preset(CameraController* controller, int index, QString& err) const;

    /// @brief Validate the JSON against the connected sensor.
    QString validate(const QJsonObject& obj, CameraController* controller) const;

    /// @brief Phase 10: capture / apply algorithm parameters (enabled state +
    /// per-algorithm param values). The bridge provides the algorithm registry;
    /// active instances are queried for their current values.
    QJsonObject capture_algo_state(AlgoBridge* bridge) const;
    bool apply_algo_state(AlgoBridge* bridge, const QJsonObject& obj, QString& err,
                          std::map<std::string, std::string>* legacy_roi = nullptr) const;
    bool save_algo_params_to_file(AlgoBridge* bridge, const QString& path, QString& err) const;
    /// When @p legacy_roi is non-null, legacy per-algorithm roi_* entries
    /// (Phase 2.6: the deleted per-backend ROI) are NOT forwarded to
    /// instances; the FIRST algorithm's roi_* values are collected into
    /// @p legacy_roi so the caller can map them onto the unified ROI.
    bool load_algo_params_from_file(AlgoBridge* bridge, const QString& path, QString& err,
                                    std::map<std::string, std::string>* legacy_roi = nullptr) const;

private:
    QJsonObject capture_biases(CameraController* c) const;
    QJsonObject capture_roi(CameraController* c) const;
    QJsonObject capture_esp(CameraController* c) const;
    QJsonObject capture_trigger(CameraController* c) const;

    bool apply_biases(CameraController* c, const QJsonObject& o, QString& err) const;
    bool apply_roi(CameraController* c, const QJsonObject& o, QString& err) const;
    bool apply_esp(CameraController* c, const QJsonObject& o, QString& err) const;
    bool apply_trigger(CameraController* c, const QJsonObject& o, QString& err) const;
};

} // namespace gui

#endif // GUI_CONFIG_CONFIG_MANAGER_H
