// gui/display/event_display_widget.cpp

#include "event_display_widget.h"

#include <QMouseEvent>
#include <QPainter>

#include <algorithm>

namespace gui {

namespace {
constexpr const char* kVertSrc = R"GLSL(#version 330 core
out vec2 vUV;
void main() {
    float x = (gl_VertexID == 1 || gl_VertexID == 3) ? 1.0 : -1.0;
    float y = (gl_VertexID == 2 || gl_VertexID == 3) ? 1.0 : -1.0;
    gl_Position = vec4(x, y, 0.0, 1.0);
    vUV = vec2(x * 0.5 + 0.5, 1.0 - (y * 0.5 + 0.5));
}
)GLSL";

constexpr const char* kFragSrc = R"GLSL(#version 330 core
in vec2 vUV;
out vec4 fragColor;
uniform sampler2D tex;
void main() {
    fragColor = texture(tex, vUV);
}
)GLSL";
} // namespace

EventDisplayWidget::EventDisplayWidget(QWidget* parent) : QOpenGLWidget(parent) {
    setMinimumSize(320, 240);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    setAttribute(Qt::WA_PaintOnScreen, false);
    setUpdateBehavior(QOpenGLWidget::NoPartialUpdate);

    overlay_label_ = new QLabel(this);
    overlay_label_->setText(
        "<div style='text-align:center;'>"
        "<h2 style='color:#e8e8e8;margin-bottom:8px;'>No camera connected</h2>"
        "<p style='color:#bbb;font-size:13px;'>Connect a camera or open an event file to begin.</p>"
        "<p style='color:#888;font-size:12px;margin-top:12px;'>"
        "&nbsp;&nbsp;File → Open File...&nbsp;&nbsp;|&nbsp;&nbsp;Camera → Connect First Available"
        "</p>"
        "</div>");
    overlay_label_->setAlignment(Qt::AlignCenter);
    overlay_label_->setStyleSheet(
        "QLabel { background: #1a1a1a; color: #ddd; }");
    overlay_label_->setAttribute(Qt::WA_TransparentForMouseEvents, true);
    overlay_label_->setAutoFillBackground(true);
    setMouseTracking(false);
    setCursor(Qt::ArrowCursor);
}

EventDisplayWidget::~EventDisplayWidget() {
    makeCurrent();
    texture_.reset();
    vao_.reset();
    program_.reset();
    doneCurrent();
}

void EventDisplayWidget::initializeGL() {
    initializeOpenGLFunctions();

    // When this widget is reparented (e.g. its containing QDockWidget is
    // dragged out to become a floating window), Qt destroys the old OpenGL
    // context and creates a new one. We must free the GPU resources that
    // belong to the old context BEFORE it is destroyed, otherwise the
    // unique_ptr destructors would run later with the new (or no) context
    // current, crashing inside the GL driver. UniqueConnection ensures we
    // don't accumulate duplicate connections across multiple reparents.
    connect(context(), &QOpenGLContext::aboutToBeDestroyed,
            this, &EventDisplayWidget::cleanup_gl, Qt::UniqueConnection);

    // If we were reparented, initializeGL() is called again for the new
    // context. Discard any stale resource wrappers from the old context —
    // their underlying GL objects are already gone (freed in cleanup_gl).
    // Resetting before recreating prevents the unique_ptr assignment from
    // trying to delete objects on the wrong context.
    texture_.reset();
    vao_.reset();
    program_.reset();

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

    program_ = std::make_unique<QOpenGLShaderProgram>();
    if (!program_->addShaderFromSourceCode(QOpenGLShader::Vertex, kVertSrc) ||
        !program_->addShaderFromSourceCode(QOpenGLShader::Fragment, kFragSrc) ||
        !program_->link()) {
        // Shader failure should not happen on a sane GL driver; fall back to
        // clearing only. The overlay label still informs the user.
        program_.reset();
        return;
    }
    tex_loc_ = program_->uniformLocation("tex");

    vao_ = std::make_unique<QOpenGLVertexArrayObject>();
    vao_->create();
    vao_->bind();
    vao_->release();
}

void EventDisplayWidget::resizeGL(int w, int h) {
    glViewport(0, 0, w, h);
    if (overlay_label_) {
        overlay_label_->setGeometry(0, 0, w, h);
    }
}

void EventDisplayWidget::cleanup_gl() {
    // Called by QOpenGLContext::aboutToBeDestroyed. The context is still
    // valid at this point, so makeCurrent() succeeds and the GL delete
    // calls inside the unique_ptr destructors target the correct context.
    makeCurrent();
    texture_.reset();
    vao_.reset();
    program_.reset();
    doneCurrent();
}

void EventDisplayWidget::paintGL() {
    const int widget_w = width() * devicePixelRatioF();
    const int widget_h = height() * devicePixelRatioF();
    // Defensive: a zero-sized widget would divide by zero in the letterbox
    // math below (unreachable while setMinimumSize holds — audit §六-D1).
    if (widget_w <= 0 || widget_h <= 0) return;

    glClear(GL_COLOR_BUFFER_BIT);

    if (!program_ || frame_.isNull()) {
        // Even without a frame, paint the in-progress drag rectangle if any.
        if (dragging_ || roi_overlay_enabled_) {
            QPainter p(this);
            draw_overlay(p);
        }
        return;
    }

    // (Re)upload the texture if a new frame arrived.
    if (frame_dirty_ || !texture_) {
        // Skip the deep copy if the frame is already in the target format
        // (the common path: FramePipeline/FileFrameGenerator both emit
        // Format_RGB888). convertToFormat allocates a new buffer even when
        // the format matches, which is a per-frame waste.
        QImage img = (frame_.format() == QImage::Format_RGB888)
                         ? frame_
                         : frame_.convertToFormat(QImage::Format_RGB888);
        if (img.isNull()) {
            return;
        }
        if (!texture_) {
            texture_ = std::make_unique<QOpenGLTexture>(img);
            texture_->setMinificationFilter(QOpenGLTexture::Linear);
            texture_->setMagnificationFilter(QOpenGLTexture::Linear);
            texture_->setWrapMode(QOpenGLTexture::ClampToEdge);
        } else if (texture_->width() != img.width() || texture_->height() != img.height()) {
            texture_->destroy();
            texture_->create();
            texture_->setSize(img.width(), img.height());
            texture_->setFormat(QOpenGLTexture::RGB8_UNorm);
            texture_->setMinificationFilter(QOpenGLTexture::Linear);
            texture_->setMagnificationFilter(QOpenGLTexture::Linear);
            texture_->setWrapMode(QOpenGLTexture::ClampToEdge);
            texture_->allocateStorage(QOpenGLTexture::RGB, QOpenGLTexture::UInt8);
            texture_->setData(QOpenGLTexture::RGB, QOpenGLTexture::UInt8, img.constBits());
        } else {
            texture_->setData(QOpenGLTexture::RGB, QOpenGLTexture::UInt8, img.constBits());
        }
        frame_dirty_ = false;
    }

    draw_letterboxed(widget_w, widget_h, texture_->width(), texture_->height());

    // Paint 2D overlays on top of the GL frame using a QPainter. Qt handles
    // the GL/native composition automatically when used inside paintGL.
    if (dragging_ || roi_overlay_enabled_) {
        QPainter p(this);
        draw_overlay(p);
    }
}

void EventDisplayWidget::set_frame(const QImage& frame) {
    if (frame.isNull()) {
        return;
    }
    frame_ = frame;
    frame_dirty_ = true;
    if (overlay_label_) {
        overlay_label_->hide();
    }
    update();
}

void EventDisplayWidget::clear() {
    frame_ = QImage();
    frame_dirty_ = false;
    roi_overlay_enabled_ = false;
    dragging_ = false;
    if (overlay_label_) {
        overlay_label_->show();
    }
    update();
}

QImage EventDisplayWidget::current_frame() const {
    return frame_;
}

void EventDisplayWidget::draw_letterboxed(int widget_w, int widget_h,
                                          int img_w, int img_h) {
    if (img_w <= 0 || img_h <= 0 || !texture_ || !program_) {
        return;
    }
    const float img_aspect = static_cast<float>(img_w) / img_h;
    const float win_aspect = static_cast<float>(widget_w) / widget_h;
    int vp_w, vp_h, vp_x, vp_y;
    if (img_aspect > win_aspect) {
        vp_w = widget_w;
        vp_h = static_cast<int>(widget_w / img_aspect);
        vp_x = 0;
        vp_y = (widget_h - vp_h) / 2;
    } else {
        vp_h = widget_h;
        vp_w = static_cast<int>(widget_h * img_aspect);
        vp_x = (widget_w - vp_w) / 2;
        vp_y = 0;
    }
    glViewport(vp_x, vp_y, vp_w, vp_h);

    program_->bind();
    if (texture_) {
        texture_->bind(0);
    }
    program_->setUniformValue(tex_loc_, 0);

    if (vao_) {
        vao_->bind();
    }
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    if (vao_) {
        vao_->release();
    }
    if (texture_) {
        texture_->release();
    }
    program_->release();
}

// ---------------------------------------------------------------------------
// ROI drag + overlay (Phase 2)
// ---------------------------------------------------------------------------

void EventDisplayWidget::set_roi_drag_mode(bool on) {
    roi_drag_mode_ = on;
    if (on) {
        setCursor(Qt::CrossCursor);
    } else {
        setCursor(Qt::ArrowCursor);
        dragging_ = false;
        update();
    }
}

void EventDisplayWidget::set_roi_overlay(int x, int y, int w, int h, bool enabled) {
    roi_overlay_sensor_ = QRect(x, y, w, h);
    roi_overlay_enabled_ = enabled;
    update();
}

bool EventDisplayWidget::compute_logical_letterbox(QRect& out) const {
    if (!texture_ || texture_->width() <= 0 || texture_->height() <= 0) {
        return false;
    }
    const int img_w = texture_->width();
    const int img_h = texture_->height();
    const int win_w = width();
    const int win_h = height();
    if (win_w <= 0 || win_h <= 0) return false;

    const float img_aspect = static_cast<float>(img_w) / img_h;
    const float win_aspect = static_cast<float>(win_w) / win_h;
    int vp_w, vp_h, vp_x, vp_y;
    if (img_aspect > win_aspect) {
        vp_w = win_w;
        vp_h = static_cast<int>(win_w / img_aspect);
        vp_x = 0;
        vp_y = (win_h - vp_h) / 2;
    } else {
        vp_h = win_h;
        vp_w = static_cast<int>(win_h * img_aspect);
        vp_x = (win_w - vp_w) / 2;
        vp_y = 0;
    }
    out = QRect(vp_x, vp_y, vp_w, vp_h);
    return true;
}

bool EventDisplayWidget::widget_to_sensor(const QPoint& widget_pos, QPoint& sensor_pos) const {
    QRect vp;
    if (!compute_logical_letterbox(vp)) return false;
    if (vp.width() == 0 || vp.height() == 0) return false;
    if (!vp.contains(widget_pos)) {
        // Allow points slightly outside to avoid clipping the edges.
    }
    const int img_w = texture_->width();
    const int img_h = texture_->height();
    const float sx = static_cast<float>(widget_pos.x() - vp.x()) / vp.width()  * img_w;
    const float sy = static_cast<float>(widget_pos.y() - vp.y()) / vp.height() * img_h;
    const int ix = static_cast<int>(sx);
    const int iy = static_cast<int>(sy);
    // Clamp to [0, img_w] / [0, img_h] (NOT img_w-1): this mapping is used
    // for ROI drag corners, where the end corner is exclusive — clamping to
    // img_w-1 made a full-width drag produce w=img_w-1, permanently
    // excluding the rightmost/bottom-most pixel row (audit §六-D2).
    sensor_pos = QPoint(std::clamp(ix, 0, img_w), std::clamp(iy, 0, img_h));
    return true;
}

void EventDisplayWidget::draw_overlay(QPainter& painter) {
    painter.setRenderHint(QPainter::Antialiasing, false);
    QPen pen(QColor(255, 220, 0, 220));
    pen.setWidth(2);
    pen.setStyle(Qt::SolidLine);
    painter.setPen(pen);
    painter.setBrush(QColor(255, 220, 0, 40));

    QRect vp;
    const bool have_vp = compute_logical_letterbox(vp);

    // Persistent overlay (sensor → widget coords).
    if (roi_overlay_enabled_ && !roi_overlay_sensor_.isEmpty() && have_vp) {
        QRect sensor = roi_overlay_sensor_;
        const float sx = static_cast<float>(sensor.x()) / texture_->width()  * vp.width()  + vp.x();
        const float sy = static_cast<float>(sensor.y()) / texture_->height() * vp.height() + vp.y();
        const float sw = static_cast<float>(sensor.width())  / texture_->width()  * vp.width();
        const float sh = static_cast<float>(sensor.height()) / texture_->height() * vp.height();
        painter.drawRect(QRectF(sx, sy, sw, sh));
    }

    // In-progress drag rectangle (already in widget coords).
    if (dragging_) {
        QRect r = QRect(drag_start_widget_, drag_curr_widget_).normalized();
        painter.drawRect(r);
    }
}

void EventDisplayWidget::mousePressEvent(QMouseEvent* event) {
    if (!roi_drag_mode_ || event->button() != Qt::LeftButton) {
        return;
    }
    dragging_ = true;
    drag_start_widget_ = event->pos();
    drag_curr_widget_ = event->pos();
    update();
}

void EventDisplayWidget::mouseMoveEvent(QMouseEvent* event) {
    if (!dragging_) return;
    drag_curr_widget_ = event->pos();
    update();
}

void EventDisplayWidget::mouseReleaseEvent(QMouseEvent* event) {
    if (!dragging_ || event->button() != Qt::LeftButton) return;
    dragging_ = false;
    drag_curr_widget_ = event->pos();

    QPoint s_start, s_end;
    if (!widget_to_sensor(drag_start_widget_, s_start) ||
        !widget_to_sensor(drag_curr_widget_,  s_end)) {
        update();
        return;
    }
    if (s_start.x() > s_end.x()) std::swap(s_start.rx(), s_end.rx());
    if (s_start.y() > s_end.y()) std::swap(s_start.ry(), s_end.ry());
    const int w = s_end.x() - s_start.x();
    const int h = s_end.y() - s_start.y();
    if (w > 0 && h > 0) {
        emit roi_dragged(s_start.x(), s_start.y(), w, h);
    }
    update();
}

} // namespace gui
