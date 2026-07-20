// gui/config/config_manager.cpp

#include "config_manager.h"

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>

#include <map>

#include <metavision/hal/facilities/i_antiflicker_module.h>
#include <metavision/hal/facilities/i_erc_module.h>
#include <metavision/hal/facilities/i_event_trail_filter_module.h>
#include <metavision/hal/facilities/i_ll_biases.h>
#include <metavision/hal/facilities/i_roi.h>
#include <metavision/hal/facilities/i_trigger_in.h>
#include <metavision/hal/facilities/i_trigger_out.h>

#include "app/camera_controller.h"
#include "algo_bridge/algo_bridge.h"

namespace gui {

ConfigManager::ConfigManager(QObject* parent) : QObject(parent) {}

// ---------------------------------------------------------------------------
// Capture
// ---------------------------------------------------------------------------

QJsonObject ConfigManager::capture(CameraController* controller) const {
    QJsonObject root;
    if (!controller) return root;
    const auto& info = controller->sensor_info();
    root["format"] = QStringLiteral("GUI-for-openEB-config");
    root["version"] = 1;
    QJsonObject sensor;
    sensor["integrator"] = info.integrator;
    sensor["generation"] = info.generation_name;
    sensor["width"] = info.width;
    sensor["height"] = info.height;
    root["sensor"] = sensor;
    root["biases"] = capture_biases(controller);
    root["roi"] = capture_roi(controller);
    root["esp"] = capture_esp(controller);
    root["trigger"] = capture_trigger(controller);
    return root;
}

QJsonObject ConfigManager::capture_biases(CameraController* c) const {
    QJsonObject o;
    auto* b = c->biases_facility();
    if (!b) return o;
    try {
        for (const auto& kv : b->get_all_biases()) {
            QJsonObject entry;
            entry["value"] = kv.second;
            Metavision::LL_Bias_Info info;
            try {
                if (b->get_bias_info(kv.first, info)) {
                    entry["desc"] = QString::fromStdString(info.get_description());
                }
            } catch (...) {}
            o[QString::fromStdString(kv.first)] = entry;
        }
    } catch (...) {}
    return o;
}

QJsonObject ConfigManager::capture_roi(CameraController* c) const {
    QJsonObject o;
    auto* r = c->roi_facility();
    if (!r) return o;
    try {
        o["enabled"] = r->is_enabled();
        o["mode"] = static_cast<int>(r->get_mode());
        const auto wins = r->get_windows();
        if (!wins.empty()) {
            // Persist the first window's geometry so the rectangle is
            // restored on apply. Multi-window configs are not common in this
            // GUI (the RoiPanel exposes a single window) so we keep the
            // first one only.
            const auto& w0 = wins.front();
            QJsonObject geom;
            geom["x"] = w0.x;
            geom["y"] = w0.y;
            geom["width"] = w0.width;
            geom["height"] = w0.height;
            o["window"] = geom;
        }
    } catch (...) {}
    return o;
}

QJsonObject ConfigManager::capture_esp(CameraController* c) const {
    QJsonObject o;
    if (auto* af = c->anti_flicker_facility()) {
        QJsonObject a;
        try {
            a["enabled"] = af->is_enabled();
            a["band_low"] = static_cast<int>(af->get_band_low_frequency());
            a["band_high"] = static_cast<int>(af->get_band_high_frequency());
        } catch (...) {}
        o["anti_flicker"] = a;
    }
    if (auto* tf = c->trail_filter_facility()) {
        QJsonObject t;
        try {
            t["enabled"] = tf->is_enabled();
            t["type"] = static_cast<int>(tf->get_type());
            t["threshold"] = static_cast<qint64>(tf->get_threshold());
        } catch (...) {}
        o["trail_filter"] = t;
    }
    if (auto* erc = c->erc_facility()) {
        QJsonObject e;
        try {
            e["enabled"] = erc->is_enabled();
            e["target_rate"] = static_cast<qint64>(erc->get_cd_event_rate());
        } catch (...) {}
        o["erc"] = e;
    }
    return o;
}

QJsonObject ConfigManager::capture_trigger(CameraController* c) const {
    QJsonObject o;
    if (auto* ti = c->trigger_in_facility()) {
        QJsonObject i;
        // I_TriggerIn exposes per-channel state; we persist the Main channel.
        try { i["enabled"] = ti->is_enabled(Metavision::I_TriggerIn::Channel::Main); }
        catch (...) { i["enabled"] = false; }
        o["in"] = i;
    }
    if (auto* to = c->trigger_out_facility()) {
        QJsonObject oo;
        try {
            oo["enabled"] = to->is_enabled();
            oo["period_us"] = static_cast<qint64>(to->get_period());
            oo["duty_cycle"] = to->get_duty_cycle();
        } catch (...) {}
        o["out"] = oo;
    }
    return o;
}

// ---------------------------------------------------------------------------
// Apply
// ---------------------------------------------------------------------------

bool ConfigManager::apply(CameraController* controller, const QJsonObject& obj, QString& err) const {
    if (!controller) {
        err = tr("No camera connected.");
        return false;
    }
    const QString validation = validate(obj, controller);
    if (!validation.isEmpty()) {
        // Surface the validation problem to the caller so the UI can warn
        // the user — previously this was written into `err` but then
        // overwritten by the (successful) apply steps, silently swallowing
        // sensor-mismatch warnings.
        err = validation;
        return false;
    }
    bool ok = true;
    QString sub_err;
    if (obj.contains("biases"))  ok &= apply_biases(controller, obj.value("biases").toObject(), sub_err);
    if (obj.contains("roi"))     ok &= apply_roi(controller, obj.value("roi").toObject(), sub_err);
    if (obj.contains("esp"))     ok &= apply_esp(controller, obj.value("esp").toObject(), sub_err);
    if (obj.contains("trigger")) ok &= apply_trigger(controller, obj.value("trigger").toObject(), sub_err);
    if (!ok && err.isEmpty()) err = sub_err;
    return ok;
}

bool ConfigManager::apply_biases(CameraController* c, const QJsonObject& o, QString& err) const {
    auto* b = c->biases_facility();
    if (!b) { err = tr("Biases not supported."); return false; }
    bool ok = true;
    for (auto it = o.begin(); it != o.end(); ++it) {
        const auto name = it.key().toStdString();
        // Value is persisted as a JSON number; accept both int and string forms.
        const auto val_obj = it.value().toObject().value("value");
        int v = 0;
        bool converted = false;
        if (val_obj.isString()) {
            v = val_obj.toString().toInt(&converted);
        } else if (val_obj.isDouble()) {
            v = val_obj.toInt();
            converted = true;
        }
        if (!converted) { ok = false; continue; }
        try {
            b->set(name, v);
        } catch (...) {
            ok = false;
        }
    }
    return ok;
}

bool ConfigManager::apply_roi(CameraController* c, const QJsonObject& o, QString& /*err*/) const {
    auto* r = c->roi_facility();
    if (!r) return false;
    // Order matters per the I_ROI contract: a window must be set before
    // enable(true) is called. So configure mode + window first, then enable.
    try {
        if (o.contains("mode")) {
            r->set_mode(static_cast<Metavision::I_ROI::Mode>(o.value("mode").toInt()));
        }
        if (o.contains("window")) {
            const auto geom = o.value("window").toObject();
            const int x = geom.value("x").toInt();
            const int y = geom.value("y").toInt();
            const int w = geom.value("width").toInt();
            const int h = geom.value("height").toInt();
            if (w > 0 && h > 0) {
                Metavision::I_ROI::Window win(x, y, w, h);
                r->set_windows({win});
            }
        }
        if (o.contains("enabled")) {
            r->enable(o.value("enabled").toBool());
        }
    } catch (...) { return false; }
    return true;
}

bool ConfigManager::apply_esp(CameraController* c, const QJsonObject& o, QString& /*err*/) const {
    bool ok = true;
    if (o.contains("anti_flicker")) {
        auto* af = c->anti_flicker_facility();
        const auto a = o.value("anti_flicker").toObject();
        if (af) {
            if (a.contains("band_low") && a.contains("band_high")) {
                try {
                    af->set_frequency_band(static_cast<uint32_t>(a.value("band_low").toInt()),
                                           static_cast<uint32_t>(a.value("band_high").toInt()));
                } catch (...) { ok = false; }
            }
            if (a.contains("enabled")) af->enable(a.value("enabled").toBool());
        }
    }
    if (o.contains("trail_filter")) {
        auto* tf = c->trail_filter_facility();
        const auto t = o.value("trail_filter").toObject();
        if (tf) {
            // Apply type/threshold before enabling so the filter is configured
            // with the persisted parameters when it starts processing.
            if (t.contains("type")) {
                try { tf->set_type(static_cast<Metavision::I_EventTrailFilterModule::Type>(
                                       t.value("type").toInt())); }
                catch (...) { ok = false; }
            }
            if (t.contains("threshold")) {
                try { tf->set_threshold(static_cast<uint32_t>(t.value("threshold").toVariant().toLongLong())); }
                catch (...) { ok = false; }
            }
            if (t.contains("enabled")) tf->enable(t.value("enabled").toBool());
        }
    }
    if (o.contains("erc")) {
        auto* erc = c->erc_facility();
        const auto e = o.value("erc").toObject();
        if (erc) {
            if (e.contains("target_rate")) {
                try { erc->set_cd_event_rate(static_cast<uint32_t>(e.value("target_rate").toVariant().toLongLong())); }
                catch (...) { ok = false; }
            }
            if (e.contains("enabled")) erc->enable(e.value("enabled").toBool());
        }
    }
    return ok;
}

bool ConfigManager::apply_trigger(CameraController* c, const QJsonObject& o, QString& /*err*/) const {
    bool ok = true;
    if (o.contains("in")) {
        auto* ti = c->trigger_in_facility();
        const auto i = o.value("in").toObject();
        if (ti) {
            const bool want = i.value("enabled").toBool();
            const auto ch = Metavision::I_TriggerIn::Channel::Main;
            try {
                if (want) ti->enable(ch);
                else      ti->disable(ch);
            } catch (...) { ok = false; }
        }
    }
    if (o.contains("out")) {
        auto* to = c->trigger_out_facility();
        const auto oo = o.value("out").toObject();
        if (to) {
            if (oo.contains("period_us")) {
                try { to->set_period(static_cast<uint32_t>(oo.value("period_us").toVariant().toLongLong())); }
                catch (...) { ok = false; }
            }
            if (oo.contains("duty_cycle")) {
                try { to->set_duty_cycle(oo.value("duty_cycle").toDouble()); }
                catch (...) { ok = false; }
            }
            if (oo.contains("enabled")) {
                try {
                    if (oo.value("enabled").toBool()) to->enable();
                    else                              to->disable();
                } catch (...) { ok = false; }
            }
        }
    }
    return ok;
}

// ---------------------------------------------------------------------------
// File I/O
// ---------------------------------------------------------------------------

bool ConfigManager::save_to_file(CameraController* controller, const QString& path, QString& err) const {
    const QJsonObject obj = capture(controller);
    QJsonDocument doc(obj);
    QFile f(path);
    if (!f.open(QIODevice::WriteOnly)) {
        err = tr("Cannot open file for writing:\n%1").arg(path);
        return false;
    }
    const QByteArray data = doc.toJson(QJsonDocument::Indented);
    if (f.write(data) != data.size() || !f.flush()) {
        err = tr("Failed to write config file:\n%1").arg(path);
        return false;
    }
    return true;
}

bool ConfigManager::load_from_file(CameraController* controller, const QString& path, QString& err) const {
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) {
        err = tr("Cannot open file for reading:\n%1").arg(path);
        return false;
    }
    QJsonParseError pe;
    auto doc = QJsonDocument::fromJson(f.readAll(), &pe);
    if (pe.error != QJsonParseError::NoError) {
        err = pe.errorString();
        return false;
    }
    return apply(controller, doc.object(), err);
}

// ---------------------------------------------------------------------------
// Presets
// ---------------------------------------------------------------------------

QStringList ConfigManager::preset_names() const {
    return {tr("Standard"), tr("High Sensitivity"), tr("Low Noise")};
}

bool ConfigManager::apply_preset(CameraController* controller, int index, QString& err) const {
    if (!controller) {
        err = tr("No camera connected.");
        return false;
    }
    auto* b = controller->biases_facility();
    if (!b) {
        err = tr("Biases not supported by this sensor.");
        return false;
    }
    // Presets are heuristic guidance: standard = defaults, high-sensitivity =
    // lower diff thresholds, low-noise = higher diff thresholds.
    QJsonObject obj = capture(controller);
    QJsonObject biases = obj.value("biases").toObject();
    int missing = 0;
    auto adjust = [&biases, &missing](const QString& key, int delta) {
        if (!biases.contains(key)) { ++missing; return; }
        QJsonObject e = biases.value(key).toObject();
        // Value is stored as a JSON int (capture_biases writes kv.second).
        int v = e.value("value").toInt();
        e["value"] = v + delta;
        biases[key] = e;
    };
    switch (index) {
        case 0: break; // Standard = no change
        case 1: adjust("bias_diff_on", -10); adjust("bias_diff_off", -10); break;
        case 2: adjust("bias_diff_on", +15); adjust("bias_diff_off", +15); break;
        default: err = tr("Unknown preset."); return false;
    }
    if (missing > 0) {
        err = tr("Sensor does not expose bias_diff_on/bias_diff_off; preset has no effect.");
        return false;
    }
    obj["biases"] = biases;
    return apply(controller, obj, err);
}

QString ConfigManager::validate(const QJsonObject& obj, CameraController* controller) const {
    if (obj.value("format").toString() != "GUI-for-openEB-config") {
        return tr("Not a GUI-for-openEB config file.");
    }
    const auto sensor = obj.value("sensor").toObject();
    const QString gen = sensor.value("generation").toString();
    const QString live = controller->sensor_info().generation_name;
    if (!gen.isEmpty() && !live.isEmpty() && gen != live) {
        return tr("Sensor mismatch: config for '%1', connected '%2'.").arg(gen, live);
    }
    return {};
}

// ---------------------------------------------------------------------------
// Phase 10: algorithm parameter save/load
// ---------------------------------------------------------------------------

QJsonObject ConfigManager::capture_algo_state(AlgoBridge* bridge) const {
    QJsonObject root;
    root["format"] = QStringLiteral("GUI-for-openEB-algo-params");
    root["version"] = 1;
    if (!bridge) return root;
    QJsonObject algos;
    for (const auto& info : bridge->list_algos()) {
        QJsonObject entry;
        entry["category"] = QString::fromStdString(info.category);
        QJsonObject params;
        // Query the live instance (if any) for current values; fall back to
        // the factory default when no instance exists yet.
        auto live = bridge->find_live(info.name);
        for (const auto& p : info.params) {
            std::string val = p.default_value;
            if (live) {
                const auto cur = live->get_param(p.key);
                if (!cur.empty()) val = cur;
            }
            params[QString::fromStdString(p.key)] = QString::fromStdString(val);
        }
        entry["params"] = params;
        if (live) entry["enabled"] = live->is_enabled();
        algos[QString::fromStdString(info.name)] = entry;
    }
    root["algorithms"] = algos;
    return root;
}

bool ConfigManager::apply_algo_state(AlgoBridge* bridge, const QJsonObject& obj, QString& err) const {
    if (!bridge) { err = tr("No algorithm bridge."); return false; }
    if (obj.value("format").toString() != "GUI-for-openEB-algo-params") {
        err = tr("Not an algorithm parameter file.");
        return false;
    }
    // Apply parameter values to live instances. Instances not yet created by
    // AlgorithmsPanel are cached via bridge->cache_algo_params() so they are
    // replayed when the instance is eventually created (N1). Without this,
    // loading a config before enabling an algorithm would silently discard
    // all saved parameters.
    const auto algos = obj.value("algorithms").toObject();
    bool ok = true;
    QStringList unknown_algos;
    for (auto it = algos.begin(); it != algos.end(); ++it) {
        const auto name = it.key().toStdString();
        if (!bridge->find(name)) {
            // Unknown algorithm — skip but flag failure with a descriptive
            // message so the user knows which entries were rejected (BUG-R1).
            ok = false;
            unknown_algos << it.key();
            continue;
        }
        const auto entry = it.value().toObject();
        const auto params = entry.value("params").toObject();
        auto live = bridge->find_live(name);
        if (live) {
            for (auto pit = params.begin(); pit != params.end(); ++pit) {
                live->set_param(pit.key().toStdString(), pit.value().toString().toStdString());
            }
            if (entry.contains("enabled")) {
                if (entry.value("enabled").toBool()) {
                    // Enforce the algorithm mutex (audit §5-E4): enabling one
                    // algorithm from a config must first disable every other
                    // live instance, so two enabled=true entries can never
                    // run simultaneously. Note: this path cannot sync the
                    // sidebar checkbox (ConfigManager has no panel access) —
                    // the panel sync is handled by the MainWindow/panel
                    // integration layer.
                    for (auto& other : bridge->list_live()) {
                        if (other && other->info().name != name) {
                            other->set_enabled(false);
                        }
                    }
                    live->set_enabled(true);
                } else {
                    live->set_enabled(false);
                }
            }
        } else {
            // No live instance — cache the params so create() can replay
            // them when the algorithm is later enabled (N1).
            std::map<std::string, std::string> cached;
            for (auto pit = params.begin(); pit != params.end(); ++pit) {
                cached[pit.key().toStdString()] = pit.value().toString().toStdString();
            }
            bridge->cache_algo_params(name, cached);
        }
    }
    if (!unknown_algos.isEmpty()) {
        err = tr("Unknown algorithm(s) in config: %1").arg(unknown_algos.join(", "));
    }
    return ok;
}

bool ConfigManager::save_algo_params_to_file(AlgoBridge* bridge, const QString& path, QString& err) const {
    const QJsonObject obj = capture_algo_state(bridge);
    QJsonDocument doc(obj);
    QFile f(path);
    if (!f.open(QIODevice::WriteOnly)) {
        err = tr("Cannot open file for writing:\n%1").arg(path);
        return false;
    }
    const QByteArray data = doc.toJson(QJsonDocument::Indented);
    if (f.write(data) != data.size() || !f.flush()) {
        err = tr("Failed to write algorithm parameter file:\n%1").arg(path);
        return false;
    }
    return true;
}

bool ConfigManager::load_algo_params_from_file(AlgoBridge* bridge, const QString& path, QString& err) const {
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) {
        err = tr("Cannot open file for reading:\n%1").arg(path);
        return false;
    }
    QJsonParseError pe;
    auto doc = QJsonDocument::fromJson(f.readAll(), &pe);
    if (pe.error != QJsonParseError::NoError) {
        err = pe.errorString();
        return false;
    }
    return apply_algo_state(bridge, doc.object(), err);
}

} // namespace gui
