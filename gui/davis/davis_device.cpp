// gui/davis/davis_device.cpp — see davis_device.h.
// Ported from dv-processing 2.0.4 io/camera/{usb_device.hpp, davis.hpp}
// (Apache-2.0), events-only feature set, C++17 + libusb-1.0.
//
// Register addresses and the init sequence mirror the reference exactly; the
// deviations are: APS frames/IMU/trigger streams stay disabled (events-only),
// no debug-endpoint transfers, no automatic exposure, timestamps rebased to 0.

#include "davis_device.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <stdexcept>
#include <thread>

namespace gui::davis {
namespace {

constexpr std::uint16_t VID_INIVATION = 0x152A;
constexpr std::uint16_t PID_DAVIS_FX2 = 0x841B;
constexpr std::uint16_t PID_DAVIS_FX3 = 0x841A;
constexpr std::uint8_t USB_DEVICE_TYPE_FX2 = 0;
constexpr std::uint8_t USB_DEVICE_TYPE_FX3_GEN2 = 5;
constexpr std::uint8_t FX2_FIRMWARE_REQUIRED = 4;
constexpr std::uint8_t FX3_FIRMWARE_REQUIRED = 6;
constexpr std::uint8_t LOGIC_REQUIRED = 18;
constexpr std::uint8_t LOGIC_MINIMUM_PATCH = 1;

constexpr std::uint8_t VENDOR_REQUEST_LOG_LEVEL = 0xB1;
constexpr std::uint8_t VENDOR_REQUEST_SPI_CONFIG = 0xBF;
constexpr std::uint8_t VENDOR_REQUEST_SPI_CONFIG_MULTIPLE = 0xC2;
constexpr std::uint8_t VENDOR_REQUEST_DATA_CLEANUP = 0xC6;
constexpr std::uint8_t DATA_ENDPOINT = 0x82;
constexpr std::uint32_t DATA_TRANSFERS_NUMBER = 32;
constexpr std::uint32_t DATA_TRANSFERS_SIZE = 8 * 1024;
constexpr int SENSOR_CHIP_DAVIS240A = 0;
constexpr int SENSOR_CHIP_DAVIS240B = 1;
constexpr int SENSOR_CHIP_DAVIS240C = 2;
constexpr int SENSOR_CHIP_DAVIS346 = 5;
constexpr int SENSOR_CHIP_DAVIS640 = 6;
constexpr int SENSOR_CHIP_CDAVIS = 7;

// Register addresses (dv-processing davis.hpp constants).
constexpr std::uint8_t MODULE_MULTIPLEXER = 0;
constexpr std::uint8_t MODULE_DVS = 1;
constexpr std::uint8_t MODULE_APS = 2;
constexpr std::uint8_t MODULE_IMU = 3;
constexpr std::uint8_t MODULE_EXTERNAL_INPUT = 4;
constexpr std::uint8_t MODULE_BIAS = 5;
constexpr std::uint8_t MODULE_CHIP = 5;
constexpr std::uint8_t MODULE_SYSINFO = 6;
constexpr std::uint8_t MODULE_USB = 9;
constexpr std::uint16_t MUX_RUN = 0;
constexpr std::uint16_t MUX_TIMESTAMP_RUN = 1;
constexpr std::uint16_t MUX_TIMESTAMP_RESET = 2;
constexpr std::uint16_t MUX_RUN_CHIP = 3;
constexpr std::uint16_t MUX_DROP_EXTINPUT_ON_TRANSFER_STALL = 4;
constexpr std::uint16_t MUX_DROP_DVS_ON_TRANSFER_STALL = 5;
constexpr std::uint16_t DVS_SIZE_COLUMNS = 0;
constexpr std::uint16_t DVS_SIZE_ROWS = 1;
constexpr std::uint16_t DVS_ORIENTATION_INFO = 2;
constexpr std::uint16_t DVS_RUN = 3;
constexpr std::uint16_t DVS_WAIT_ON_TRANSFER_STALL = 4;
constexpr std::uint16_t DVS_EXTERNAL_AER_CONTROL = 5;
constexpr std::uint16_t DVS_HAS_BACKGROUND_ACTIVITY_FILTER = 30;
constexpr std::uint16_t DVS_FILTER_BACKGROUND_ACTIVITY = 31;
constexpr std::uint16_t DVS_FILTER_BACKGROUND_ACTIVITY_TIME = 32;
constexpr std::uint16_t DVS_FILTER_REFRACTORY_PERIOD = 33;
constexpr std::uint16_t DVS_FILTER_REFRACTORY_PERIOD_TIME = 34;
constexpr std::uint16_t DVS_HAS_SKIP_FILTER = 50;
constexpr std::uint16_t DVS_FILTER_SKIP_EVENTS = 51;
constexpr std::uint16_t DVS_HAS_POLARITY_FILTER = 60;
constexpr std::uint16_t DVS_FILTER_POLARITY_FLATTEN = 61;
constexpr std::uint16_t DVS_HAS_ROI_FILTER = 40;
constexpr std::uint16_t DVS_FILTER_ROI_START_COLUMN = 41;
constexpr std::uint16_t DVS_FILTER_ROI_START_ROW = 42;
constexpr std::uint16_t DVS_FILTER_ROI_END_COLUMN = 43;
constexpr std::uint16_t DVS_FILTER_ROI_END_ROW = 44;
constexpr std::uint16_t APS_SIZE_COLUMNS = 0;
constexpr std::uint16_t APS_SIZE_ROWS = 1;
constexpr std::uint16_t APS_ORIENTATION_INFO = 2;
constexpr std::uint16_t APS_RUN = 4;
constexpr std::uint16_t APS_WAIT_ON_TRANSFER_STALL = 5;
constexpr std::uint16_t APS_HAS_GLOBAL_SHUTTER = 6;
constexpr std::uint16_t APS_GLOBAL_SHUTTER = 7;
constexpr std::uint16_t APS_START_COLUMN_0 = 8;
constexpr std::uint16_t APS_START_ROW_0 = 9;
constexpr std::uint16_t APS_END_COLUMN_0 = 10;
constexpr std::uint16_t APS_END_ROW_0 = 11;
constexpr std::uint16_t APS_EXPOSURE = 12;
constexpr std::uint16_t APS_FRAME_INTERVAL = 13;
constexpr std::uint16_t IMU_TYPE = 0;
constexpr std::uint16_t IMU_ORIENTATION_INFO = 1;
constexpr std::uint16_t IMU_RUN_ACCELEROMETER = 2;
constexpr std::uint16_t IMU_RUN_GYROSCOPE = 3;
constexpr std::uint16_t IMU_RUN_TEMPERATURE = 4;
constexpr std::uint16_t IMU_SAMPLE_RATE_DIVIDER = 5;
constexpr std::uint16_t IMU_ACCEL_DLPF = 6;
constexpr std::uint16_t IMU_ACCEL_FULL_SCALE = 7;
constexpr std::uint16_t IMU_GYRO_DLPF = 9;
constexpr std::uint16_t IMU_GYRO_FULL_SCALE = 10;
constexpr std::uint16_t EXTINPUT_RUN_DETECTOR = 0;
constexpr std::uint16_t EXTINPUT_DETECT_RISING_EDGES = 1;
constexpr std::uint16_t EXTINPUT_DETECT_FALLING_EDGES = 2;
constexpr std::uint16_t EXTINPUT_DETECT_PULSES = 3;
constexpr std::uint16_t EXTINPUT_GENERATE_PULSE_INTERVAL = 13;
constexpr std::uint16_t EXTINPUT_GENERATE_PULSE_LENGTH = 14;
constexpr std::uint16_t EXTINPUT_GENERATE_INJECT_ON_RISING_EDGE = 15;
constexpr std::uint16_t EXTINPUT_GENERATE_INJECT_ON_FALLING_EDGE = 16;
constexpr std::uint16_t SYSINFO_LOGIC_VERSION = 0;
constexpr std::uint16_t SYSINFO_CHIP_IDENTIFIER = 1;
constexpr std::uint16_t SYSINFO_DEVICE_IS_MASTER = 2;
constexpr std::uint16_t SYSINFO_LOGIC_CLOCK = 3;
constexpr std::uint16_t SYSINFO_ADC_CLOCK = 4;
constexpr std::uint16_t SYSINFO_USB_CLOCK = 5;
constexpr std::uint16_t SYSINFO_CLOCK_DEVIATION = 6;
constexpr std::uint16_t SYSINFO_LOGIC_PATCH = 7;
constexpr std::uint16_t USB_RUN = 0;
constexpr std::uint16_t USB_EARLY_PACKET_DELAY = 1;
constexpr std::uint16_t DAVIS346_CHIP_GLOBAL_SHUTTER = 142;
constexpr std::uint16_t DAVIS346_CHIP_DIGITALMUX0 = 128;
constexpr std::uint16_t DAVIS346_CHIP_DIGITALMUX1 = 129;
constexpr std::uint16_t DAVIS346_CHIP_DIGITALMUX2 = 130;
constexpr std::uint16_t DAVIS346_CHIP_DIGITALMUX3 = 131;
constexpr std::uint16_t DAVIS346_CHIP_ANALOGMUX0 = 132;
constexpr std::uint16_t DAVIS346_CHIP_ANALOGMUX1 = 133;
constexpr std::uint16_t DAVIS346_CHIP_ANALOGMUX2 = 134;
constexpr std::uint16_t DAVIS346_CHIP_BIASMUX0 = 135;
constexpr std::uint16_t DAVIS346_CHIP_RESETCALIBNEURON = 136;
constexpr std::uint16_t DAVIS346_CHIP_TYPENCALIBNEURON = 137;
constexpr std::uint16_t DAVIS346_CHIP_RESETTESTPIXEL = 138;
constexpr std::uint16_t DAVIS346_CHIP_AERNAROW = 140;
constexpr std::uint16_t DAVIS346_CHIP_USEAOUT = 141;
constexpr std::uint16_t DAVIS346_CHIP_SELECTGRAYCOUNTER = 143;
constexpr std::uint16_t DAVIS346_CHIP_TESTADC = 144;

std::runtime_error usb_error(const std::string& msg, int code) {
    return std::runtime_error(msg + " (libusb: " + libusb_error_name(code) + ")");
}

void store_be32(std::uint8_t* dst, std::uint32_t value) {
    dst[0] = static_cast<std::uint8_t>(value >> 24);
    dst[1] = static_cast<std::uint8_t>(value >> 16);
    dst[2] = static_cast<std::uint8_t>(value >> 8);
    dst[3] = static_cast<std::uint8_t>(value);
}

std::uint32_t load_be32(const std::uint8_t* src) {
    return (static_cast<std::uint32_t>(src[0]) << 24) | (static_cast<std::uint32_t>(src[1]) << 16) |
           (static_cast<std::uint32_t>(src[2]) << 8) | static_cast<std::uint32_t>(src[3]);
}

} // namespace

// ---------------------------------------------------------------------------

std::vector<DeviceDescriptor> find_devices() {
    std::vector<DeviceDescriptor> found;
    libusb_context* ctx = nullptr;
    if (libusb_init(&ctx) != LIBUSB_SUCCESS) {
        return found;
    }

    libusb_device** list = nullptr;
    const ssize_t count = libusb_get_device_list(ctx, &list);
    if (count >= 0) {
        for (ssize_t i = 0; i < count; ++i) {
            libusb_device_descriptor desc{};
            if (libusb_get_device_descriptor(list[i], &desc) != LIBUSB_SUCCESS) {
                continue;
            }
            if (desc.idVendor != VID_INIVATION) {
                continue;
            }
            const bool is_fx2 = (desc.idProduct == PID_DAVIS_FX2);
            const bool is_fx3 = (desc.idProduct == PID_DAVIS_FX3);
            if (!is_fx2 && !is_fx3) {
                continue;
            }
            const auto device_type = static_cast<std::uint8_t>((desc.bcdDevice >> 8) & 0xFF);
            if (is_fx3 && device_type == USB_DEVICE_TYPE_FX3_GEN2) {
                continue; // Gen-2 FX3 boards use the nextgen protocol.
            }

            DeviceDescriptor info;
            info.vid = desc.idVendor;
            info.pid = desc.idProduct;
            info.bus = libusb_get_bus_number(list[i]);
            info.addr = libusb_get_device_address(list[i]);
            info.firmware = static_cast<std::uint8_t>(desc.bcdDevice & 0xFF);

            libusb_device_handle* handle = nullptr;
            if (libusb_open(list[i], &handle) == LIBUSB_SUCCESS) {
                std::string serial(9, '\0');
                auto got = libusb_get_string_descriptor_ascii(handle, desc.iSerialNumber,
                    reinterpret_cast<unsigned char*>(serial.data()), static_cast<int>(serial.size()));
                if (got > 8) {
                    got = 8;
                }
                if (got > 0) {
                    info.serial = serial.substr(0, static_cast<std::size_t>(got));
                }
                libusb_close(handle);
            }
            if (info.serial.empty()) {
                // Session-local repeatable serial (reference behavior).
                char tmp[8];
                std::snprintf(tmp, sizeof(tmp), "TMP%05d", (info.bus << 8) | info.addr);
                info.serial = tmp;
            }
            found.push_back(std::move(info));
        }
        libusb_free_device_list(list, 1);
    }

    libusb_exit(ctx);
    return found;
}

// ---------------------------------------------------------------------------

Device::Device(const DeviceDescriptor& descriptor)
    : pid_(descriptor.pid), firmware_(descriptor.firmware), serial_(descriptor.serial),
      parser_(0, 0, false), biases_([this](std::uint16_t addr, std::uint16_t word) {
          spi_config_send(MODULE_BIAS, addr, word);
      }) {
    // Discovery needs the descriptor's bus/address to re-locate the device in
    // a fresh libusb context (per-device contexts, like the reference).
    libusb_context* ctx = nullptr;
    if (libusb_init(&ctx) != LIBUSB_SUCCESS) {
        throw std::runtime_error("DAVIS: failed to initialize libusb.");
    }
    context_ = ctx;

    libusb_device** list = nullptr;
    const ssize_t count = libusb_get_device_list(context_, &list);
    if (count < 0) {
        libusb_exit(context_);
        context_ = nullptr;
        throw std::runtime_error("DAVIS: failed to list USB devices.");
    }

    for (ssize_t i = 0; i < count && handle_ == nullptr; ++i) {
        libusb_device_descriptor desc{};
        if (libusb_get_device_descriptor(list[i], &desc) != LIBUSB_SUCCESS) continue;
        if (desc.idVendor != descriptor.vid || desc.idProduct != descriptor.pid) continue;
        if (libusb_get_bus_number(list[i]) != descriptor.bus) continue;
        if (libusb_get_device_address(list[i]) != descriptor.addr) continue;

        libusb_device_handle* handle = nullptr;
        if (libusb_open(list[i], &handle) != LIBUSB_SUCCESS) {
            libusb_free_device_list(list, 1);
            libusb_exit(context_);
            context_ = nullptr;
            throw std::runtime_error(
                "DAVIS: failed to open USB device — check permissions (udev rules) "
                "and that no other program is using the camera.");
        }
        handle_ = handle;
    }
    libusb_free_device_list(list, 1);

    if (handle_ == nullptr) {
        libusb_exit(context_);
        context_ = nullptr;
        throw std::runtime_error("DAVIS: device disappeared during open.");
    }

    int active_config = 0;
    if (libusb_get_configuration(handle_, &active_config) != LIBUSB_SUCCESS) {
        teardown_usb();
        throw std::runtime_error("DAVIS: failed to get USB configuration.");
    }
    if (active_config != 1 && libusb_set_configuration(handle_, 1) != LIBUSB_SUCCESS) {
        teardown_usb();
        throw std::runtime_error("DAVIS: failed to set USB configuration 1.");
    }
    if (libusb_claim_interface(handle_, 0) != LIBUSB_SUCCESS) {
        teardown_usb();
        throw std::runtime_error("DAVIS: failed to claim USB interface 0 — the camera may be in use.");
    }

    usb_thread_start();
    usb_cleanup_buffers();

    try {
        configure_idle();
    } catch (...) {
        usb_thread_stop();
        libusb_release_interface(handle_, 0);
        teardown_usb();
        throw;
    }
}

Device::~Device() {
    try {
        stop();
    } catch (...) {
    }
    if (usb_thread_run_.load()) {
        usb_thread_stop();
    }
    if (handle_ != nullptr) {
        libusb_release_interface(handle_, 0);
    }
    teardown_usb();
}

void Device::teardown_usb() {
    if (handle_ != nullptr) {
        libusb_close(handle_);
        handle_ = nullptr;
    }
    if (context_ != nullptr) {
        libusb_exit(context_);
        context_ = nullptr;
    }
}

void Device::set_event_sink(EventSink sink) {
    sink_ = std::move(sink);
}

void Device::set_gone_callback(GoneCallback callback) {
    gone_callback_ = std::move(callback);
}

// --- USB primitives ---------------------------------------------------------

void Device::usb_control_out(std::uint8_t request, std::uint16_t value, std::uint16_t index,
                             const std::uint8_t* data, std::size_t size) {
    std::atomic<int> completed{-1};
    libusb_transfer* transfer = libusb_alloc_transfer(0);
    if (transfer == nullptr) throw std::runtime_error("DAVIS: out of memory (control transfer).");

    // Setup + data must be contiguous for libusb control transfers.
    std::vector<std::uint8_t> buffer(sizeof(libusb_control_setup) + size, 0);
    libusb_fill_control_setup(buffer.data(), LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR |
                                                  LIBUSB_RECIPIENT_DEVICE,
        request, value, index, static_cast<std::uint16_t>(size));
    if (size > 0 && data != nullptr) {
        std::memcpy(buffer.data() + sizeof(libusb_control_setup), data, size);
    }

    libusb_fill_control_transfer(transfer, handle_, buffer.data(),
        [](libusb_transfer* t) {
            static_cast<std::atomic_int*>(t->user_data)->store(t->status);
        },
        &completed, 0);

    if (libusb_submit_transfer(transfer) != LIBUSB_SUCCESS) {
        libusb_free_transfer(transfer);
        throw std::runtime_error("DAVIS: failed to submit control transfer OUT.");
    }
    while (completed.load() < 0) {
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    libusb_free_transfer(transfer);
    if (completed.load() != LIBUSB_TRANSFER_COMPLETED) {
        throw std::runtime_error("DAVIS: control transfer OUT failed.");
    }
}

void Device::usb_control_in(std::uint8_t request, std::uint16_t value, std::uint16_t index,
                            std::uint8_t* data, std::size_t size) {
    std::atomic<int> completed{-1};
    libusb_transfer* transfer = libusb_alloc_transfer(0);
    if (transfer == nullptr) throw std::runtime_error("DAVIS: out of memory (control transfer).");

    std::vector<std::uint8_t> buffer(sizeof(libusb_control_setup) + size, 0);
    libusb_fill_control_setup(buffer.data(), LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR |
                                                  LIBUSB_RECIPIENT_DEVICE,
        request, value, index, static_cast<std::uint16_t>(size));

    libusb_fill_control_transfer(transfer, handle_, buffer.data(),
        [](libusb_transfer* t) {
            static_cast<std::atomic_int*>(t->user_data)->store(t->status);
        },
        &completed, 0);

    if (libusb_submit_transfer(transfer) != LIBUSB_SUCCESS) {
        libusb_free_transfer(transfer);
        throw std::runtime_error("DAVIS: failed to submit control transfer IN.");
    }
    while (completed.load() < 0) {
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    const bool ok = (completed.load() == LIBUSB_TRANSFER_COMPLETED);
    if (ok && data != nullptr && size > 0) {
        std::memcpy(data, buffer.data() + sizeof(libusb_control_setup), size);
    }
    libusb_free_transfer(transfer);
    if (!ok) {
        throw std::runtime_error("DAVIS: control transfer IN failed.");
    }
}

void Device::spi_config_send(std::uint8_t module, std::uint16_t param, std::uint32_t value) {
    std::uint8_t be[4];
    store_be32(be, value);
    usb_control_out(VENDOR_REQUEST_SPI_CONFIG, module, param, be, 4);
}

std::uint32_t Device::spi_config_receive(std::uint8_t module, std::uint16_t param) {
    std::uint8_t be[4] = {0, 0, 0, 0};
    usb_control_in(VENDOR_REQUEST_SPI_CONFIG, module, param, be, 4);
    return load_be32(be);
}

void Device::spi_config_send_multiple(const std::vector<std::array<std::uint8_t, 6>>& configs) {
    if (configs.empty()) return;
    // Entries: [module][param][value big-endian u32] — 6 bytes each.
    usb_control_out(VENDOR_REQUEST_SPI_CONFIG_MULTIPLE,
        static_cast<std::uint16_t>(configs.size()), 0, configs.front().data(), configs.size() * 6);
}

void Device::usb_cleanup_buffers() {
    try {
        usb_control_out(VENDOR_REQUEST_DATA_CLEANUP, 0, 0, nullptr, 0);
    } catch (const std::exception&) {
        libusb_clear_halt(handle_, data_endpoint_);
    }
}

// --- USB thread + data transfers ---------------------------------------------

void Device::usb_thread_start() {
    std::scoped_lock lock(usb_ops_lock_);
    if (usb_thread_run_.load()) throw std::runtime_error("DAVIS: USB thread already running.");
    usb_thread_ = std::thread([this]() {
        usb_thread_run_.store(true);
        timeval timeout = {0, 10000}; // 10 ms
        while (usb_thread_run_.load()) {
            if (libusb_handle_events_timeout_completed(context_, &timeout, nullptr) != LIBUSB_SUCCESS) {
                // Transient event-handling failures are tolerated (reference behavior).
            }
        }
    });
    while (!usb_thread_run_.load()) {
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
}

void Device::usb_thread_stop() {
    std::scoped_lock lock(usb_ops_lock_);
    if (!usb_thread_run_.load()) return;
    usb_thread_run_.store(false);
    if (usb_thread_.joinable()) usb_thread_.join();
}

void LIBUSB_CALL Device::usb_data_transfer_cb(libusb_transfer* transfer) {
    auto* self = static_cast<Device*>(transfer->user_data);

    if ((transfer->status == LIBUSB_TRANSFER_COMPLETED ||
            transfer->status == LIBUSB_TRANSFER_CANCELLED) &&
        transfer->actual_length > 0) {
        self->parser_.parse(transfer->buffer, static_cast<std::size_t>(transfer->actual_length),
            self->sink_);
    }

    if (transfer->status == LIBUSB_TRANSFER_COMPLETED) {
        if (libusb_submit_transfer(transfer) == LIBUSB_SUCCESS) {
            return;
        }
    }

    // Unrecoverable (unplugged or stopping).
    {
        std::scoped_lock lock(self->data_transfers_lock_);
        self->data_transfers_active_--;
        const bool failed_not_cancelled = (transfer->status != LIBUSB_TRANSFER_CANCELLED);
        if (self->data_transfers_active_ == 0 && failed_not_cancelled) {
            if (self->gone_callback_) self->gone_callback_();
        }
    }
}

void Device::usb_data_transfers_start() {
    std::scoped_lock lock(usb_ops_lock_);
    std::scoped_lock data_lock(data_transfers_lock_);
    if (data_transfers_active_ > 0) return;
    for (std::uint32_t i = 0; i < DATA_TRANSFERS_NUMBER; ++i) {
        libusb_transfer* transfer = libusb_alloc_transfer(0);
        if (transfer == nullptr) break;
        auto* buffer = static_cast<std::uint8_t*>(malloc(DATA_TRANSFERS_SIZE));
        if (buffer == nullptr) {
            libusb_free_transfer(transfer);
            break;
        }
        transfer->buffer = buffer;
        transfer->length = static_cast<int>(DATA_TRANSFERS_SIZE);
        transfer->dev_handle = handle_;
        transfer->endpoint = data_endpoint_;
        transfer->type = LIBUSB_TRANSFER_TYPE_BULK;
        transfer->callback = &usb_data_transfer_cb;
        transfer->user_data = this;
        transfer->timeout = 0;
        transfer->flags = LIBUSB_TRANSFER_FREE_BUFFER;
        if (libusb_submit_transfer(transfer) != LIBUSB_SUCCESS) {
            free(buffer);
            libusb_free_transfer(transfer);
            continue;
        }
        data_transfers_.push_back(transfer);
    }
    data_transfers_active_ = static_cast<std::uint32_t>(data_transfers_.size());
    if (data_transfers_.empty()) {
        throw std::runtime_error("DAVIS: unable to allocate any USB data transfers.");
    }
}

void Device::usb_data_transfers_stop() {
    // A single cancel pass is NOT enough while the camera streams: a transfer
    // that completed just before the cancel request reaches it is re-submitted
    // by its callback and keeps receiving data. Keep cancelling (releasing the
    // locks so the USB-thread callbacks can run) until every transfer is gone
    // — same loop as the reference usbCancelAndDeallocateDataTransfersNLCK.
    std::scoped_lock ops_lock(usb_ops_lock_);
    while (true) {
        {
            std::scoped_lock data_lock(data_transfers_lock_);
            if (data_transfers_active_ == 0 || data_transfers_.empty()) break;
            for (auto* transfer : data_transfers_) {
                libusb_cancel_transfer(transfer);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::scoped_lock data_lock(data_transfers_lock_);
    for (auto* transfer : data_transfers_) {
        libusb_free_transfer(transfer);
    }
    data_transfers_.clear();
    data_transfers_active_ = 0;
}

// --- DAVIS configuration -------------------------------------------------------

void Device::configure_idle() {
    // Verify firmware/logic version (reference hard-fails on mismatch).
    {
        const bool is_fx2 = (pid_ == PID_DAVIS_FX2);
        const auto required_fw = is_fx2 ? FX2_FIRMWARE_REQUIRED : FX3_FIRMWARE_REQUIRED;
        if (firmware_ != required_fw) {
            throw std::runtime_error("DAVIS: unsupported USB firmware version " +
                                     std::to_string(firmware_) + " (expected " +
                                     std::to_string(required_fw) + "). Update the camera with Flashy.");
        }
    }
    const auto logic_version = spi_config_receive(MODULE_SYSINFO, SYSINFO_LOGIC_VERSION);
    if (logic_version != LOGIC_REQUIRED) {
        throw std::runtime_error("DAVIS: unsupported FPGA logic version " +
                                 std::to_string(logic_version) + " (expected " +
                                 std::to_string(LOGIC_REQUIRED) + "). Update the camera with Flashy.");
    }
    const auto logic_patch = spi_config_receive(MODULE_SYSINFO, SYSINFO_LOGIC_PATCH);
    if (logic_patch < LOGIC_MINIMUM_PATCH) {
        throw std::runtime_error("DAVIS: FPGA logic patch " + std::to_string(logic_patch) +
                                 " is too old (minimum " + std::to_string(LOGIC_MINIMUM_PATCH) + ").");
    }

    const auto chip_id = static_cast<int>(spi_config_receive(MODULE_SYSINFO, SYSINFO_CHIP_IDENTIFIER));
    if (chip_id != SENSOR_CHIP_DAVIS346 && chip_id != SENSOR_CHIP_DAVIS640) {
        throw std::runtime_error("DAVIS: unsupported sensor model (chip id " + std::to_string(chip_id) +
                                 "); only DAVIS346/640 are supported by this integration.");
    }
    model_name_ = (chip_id == SENSOR_CHIP_DAVIS346) ? "DAVIS346" : "DAVIS640";

    // Clocks (Hz) scaled by the deviation factor (per-mille).
    const auto logic_clock = static_cast<double>(spi_config_receive(MODULE_SYSINFO, SYSINFO_LOGIC_CLOCK));
    const auto usb_clock = static_cast<double>(spi_config_receive(MODULE_SYSINFO, SYSINFO_USB_CLOCK));
    const auto deviation = static_cast<double>(spi_config_receive(MODULE_SYSINFO, SYSINFO_CLOCK_DEVIATION));
    logic_clock_ = static_cast<float>(logic_clock * (deviation / 1000.0));
    usb_clock_ = static_cast<float>(usb_clock * (deviation / 1000.0));

    // Resolutions + orientation.
    const auto columns = static_cast<int>(spi_config_receive(MODULE_DVS, DVS_SIZE_COLUMNS));
    const auto rows = static_cast<int>(spi_config_receive(MODULE_DVS, DVS_SIZE_ROWS));
    const auto dvs_orientation = spi_config_receive(MODULE_DVS, DVS_ORIENTATION_INFO);
    width_ = columns;
    height_ = rows;
    const bool invert_xy = (dvs_orientation & 0x04) != 0;
    if (invert_xy) {
        std::swap(width_, height_);
    }
    // The parser range-checks in DEVICE coordinates (unswapped) and emits
    // user-orientation coordinates.
    parser_.reset();
    parser_ = Parser(columns, rows, invert_xy);

    (void)spi_config_receive(MODULE_IMU, IMU_TYPE); // reported by the reference; unused here

    // Shut the device down into a known idle state before configuring.
    spi_config_send(MODULE_DVS, DVS_RUN, false);
    spi_config_send(MODULE_IMU, IMU_RUN_ACCELEROMETER, false);
    spi_config_send(MODULE_IMU, IMU_RUN_GYROSCOPE, false);
    spi_config_send(MODULE_IMU, IMU_RUN_TEMPERATURE, false);
    spi_config_send(MODULE_APS, APS_RUN, false);
    spi_config_send(MODULE_EXTERNAL_INPUT, EXTINPUT_RUN_DETECTOR, false);
    spi_config_send(MODULE_MULTIPLEXER, MUX_RUN, false);
    spi_config_send(MODULE_MULTIPLEXER, MUX_TIMESTAMP_RUN, false);
    spi_config_send(MODULE_USB, USB_RUN, false);
    spi_config_send(MODULE_MULTIPLEXER, MUX_RUN_CHIP, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    usb_cleanup_buffers();

    // DVS configuration.
    spi_config_send(MODULE_DVS, DVS_WAIT_ON_TRANSFER_STALL, false);
    spi_config_send(MODULE_DVS, DVS_EXTERNAL_AER_CONTROL, false);
    spi_config_send(MODULE_DVS, DVS_FILTER_BACKGROUND_ACTIVITY, 0);
    spi_config_send(MODULE_DVS, DVS_FILTER_BACKGROUND_ACTIVITY_TIME, 0);
    spi_config_send(MODULE_DVS, DVS_FILTER_REFRACTORY_PERIOD, 0);
    spi_config_send(MODULE_DVS, DVS_FILTER_REFRACTORY_PERIOD_TIME, 0);
    spi_config_send(MODULE_DVS, DVS_FILTER_POLARITY_FLATTEN, false);
    // ROI filter: full sensor.
    {
        const bool run_state = false;
        spi_config_send(MODULE_DVS, DVS_RUN, run_state);
        spi_config_send(MODULE_DVS, DVS_FILTER_ROI_START_COLUMN, 0);
        spi_config_send(MODULE_DVS, DVS_FILTER_ROI_START_ROW, 0);
        spi_config_send(MODULE_DVS, DVS_FILTER_ROI_END_COLUMN, static_cast<std::uint32_t>(width_ - 1));
        spi_config_send(MODULE_DVS, DVS_FILTER_ROI_END_ROW, static_cast<std::uint32_t>(height_ - 1));
        spi_config_send(MODULE_DVS, DVS_RUN, run_state);
    }

    // APS configuration (harmless for the events-only stream, kept for parity
    // with the reference init).
    spi_config_send(MODULE_APS, APS_WAIT_ON_TRANSFER_STALL, true);
    spi_config_send(MODULE_APS, APS_GLOBAL_SHUTTER, true);
    spi_config_send(MODULE_CHIP, DAVIS346_CHIP_GLOBAL_SHUTTER, true);
    spi_config_send(MODULE_APS, APS_RUN, false);
    spi_config_send(MODULE_APS, APS_START_COLUMN_0, 0);
    spi_config_send(MODULE_APS, APS_START_ROW_0, 0);
    spi_config_send(MODULE_APS, APS_END_COLUMN_0, static_cast<std::uint32_t>(width_ - 1));
    spi_config_send(MODULE_APS, APS_END_ROW_0, static_cast<std::uint32_t>(height_ - 1));
    spi_config_send(MODULE_APS, APS_RUN, false);
    // 20 ms exposure + free-run interval, in ADC-clock ticks.
    spi_config_send(MODULE_APS, APS_EXPOSURE,
        static_cast<std::uint32_t>(std::llround(20000.0F * logic_clock_)));
    spi_config_send(MODULE_APS, APS_FRAME_INTERVAL, 0);

    // Chip configuration.
    spi_config_send(MODULE_CHIP, DAVIS346_CHIP_DIGITALMUX0, 0);
    spi_config_send(MODULE_CHIP, DAVIS346_CHIP_DIGITALMUX1, 0);
    spi_config_send(MODULE_CHIP, DAVIS346_CHIP_DIGITALMUX2, 0);
    spi_config_send(MODULE_CHIP, DAVIS346_CHIP_DIGITALMUX3, 0);
    spi_config_send(MODULE_CHIP, DAVIS346_CHIP_ANALOGMUX0, 0);
    spi_config_send(MODULE_CHIP, DAVIS346_CHIP_ANALOGMUX1, 0);
    spi_config_send(MODULE_CHIP, DAVIS346_CHIP_ANALOGMUX2, 0);
    spi_config_send(MODULE_CHIP, DAVIS346_CHIP_BIASMUX0, 0);
    spi_config_send(MODULE_CHIP, DAVIS346_CHIP_RESETCALIBNEURON, true);
    spi_config_send(MODULE_CHIP, DAVIS346_CHIP_TYPENCALIBNEURON, false);
    spi_config_send(MODULE_CHIP, DAVIS346_CHIP_RESETTESTPIXEL, true);
    spi_config_send(MODULE_CHIP, DAVIS346_CHIP_AERNAROW, false);
    spi_config_send(MODULE_CHIP, DAVIS346_CHIP_USEAOUT, false);
    spi_config_send(MODULE_CHIP, DAVIS346_CHIP_SELECTGRAYCOUNTER, 1);
    spi_config_send(MODULE_CHIP, DAVIS346_CHIP_TESTADC, false);

    // Multiplexer stall-drop + IMU defaults (IMU stays off).
    spi_config_send(MODULE_MULTIPLEXER, MUX_DROP_EXTINPUT_ON_TRANSFER_STALL, true);
    spi_config_send(MODULE_MULTIPLEXER, MUX_DROP_DVS_ON_TRANSFER_STALL, true);
    spi_config_send(MODULE_IMU, IMU_SAMPLE_RATE_DIVIDER, 0);
    spi_config_send(MODULE_IMU, IMU_ACCEL_DLPF, 1);
    spi_config_send(MODULE_IMU, IMU_GYRO_DLPF, 1);
    spi_config_send(MODULE_IMU, IMU_ACCEL_FULL_SCALE, 1); // ±4 g
    spi_config_send(MODULE_IMU, IMU_GYRO_FULL_SCALE, 1);  // ±500 °/s

    // External input detector/generator defaults.
    spi_config_send(MODULE_EXTERNAL_INPUT, EXTINPUT_DETECT_RISING_EDGES, false);
    spi_config_send(MODULE_EXTERNAL_INPUT, EXTINPUT_DETECT_FALLING_EDGES, false);
    spi_config_send(MODULE_EXTERNAL_INPUT, EXTINPUT_DETECT_PULSES, false);
    const auto gen_high = static_cast<std::uint32_t>(std::llround(5.0F * logic_clock_));
    spi_config_send(MODULE_EXTERNAL_INPUT, EXTINPUT_GENERATE_PULSE_LENGTH, gen_high);
    spi_config_send(MODULE_EXTERNAL_INPUT, EXTINPUT_GENERATE_PULSE_INTERVAL, gen_high * 2);
    spi_config_send(MODULE_EXTERNAL_INPUT, EXTINPUT_GENERATE_INJECT_ON_RISING_EDGE, false);
    spi_config_send(MODULE_EXTERNAL_INPUT, EXTINPUT_GENERATE_INJECT_ON_FALLING_EDGE, false);

    // USB early-packet delay: 1 ms in USB-clock ticks.
    spi_config_send(MODULE_USB, USB_EARLY_PACKET_DELAY,
        static_cast<std::uint32_t>(std::llround(1000.0F * usb_clock_)));

    // Default biases (reference power-up table) + shifted-source biases.
    biases_.apply_defaults();
    const auto ssp_word = encode_shifted_source(1, 33); // reference defaults
    spi_config_send(MODULE_BIAS, DAVIS346_BIAS_SSP, ssp_word);
    spi_config_send(MODULE_BIAS, DAVIS346_BIAS_SSN, ssp_word);
}

void Device::send_timestamp_reset() {
    // Two SPI writes (reset high → low) batched in one multiple request.
    std::array<std::uint8_t, 6> high = {MODULE_MULTIPLEXER};
    std::array<std::uint8_t, 6> low = {MODULE_MULTIPLEXER};
    high[1] = static_cast<std::uint8_t>(MUX_TIMESTAMP_RESET);
    store_be32(high.data() + 2, 1);
    low[1] = static_cast<std::uint8_t>(MUX_TIMESTAMP_RESET);
    store_be32(low.data() + 2, 0);
    spi_config_send_multiple({high, low});
}

bool Device::wait_for_timestamp_reset() {
    for (int waited_ms = 0; waited_ms < 1000; waited_ms += 10) {
        if (parser_.time_initialized()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
}

void Device::start() {
    if (streaming_.load()) return;
    usb_data_transfers_start();
    try {
        spi_config_send(MODULE_MULTIPLEXER, MUX_RUN_CHIP, true);
        spi_config_send(MODULE_USB, USB_RUN, true);
        spi_config_send(MODULE_MULTIPLEXER, MUX_TIMESTAMP_RUN, true);
        spi_config_send(MODULE_MULTIPLEXER, MUX_RUN, true);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        // Events only — frames/IMU/trigger streams stay disabled.
        spi_config_send(MODULE_DVS, DVS_RUN, true);
        send_timestamp_reset();
        if (!wait_for_timestamp_reset()) {
            throw std::runtime_error("DAVIS: no timestamp reset received — stream did not start.");
        }
    } catch (...) {
        // Leave the run switches OFF so a retry starts from the idle state
        // (and the FPGA stops producing into a drained endpoint).
        try {
            spi_config_send(MODULE_DVS, DVS_RUN, false);
            spi_config_send(MODULE_MULTIPLEXER, MUX_RUN, false);
            spi_config_send(MODULE_MULTIPLEXER, MUX_TIMESTAMP_RUN, false);
            spi_config_send(MODULE_USB, USB_RUN, false);
            spi_config_send(MODULE_MULTIPLEXER, MUX_RUN_CHIP, false);
        } catch (...) {
        }
        usb_data_transfers_stop();
        throw;
    }
    streaming_.store(true);
}

void Device::stop() {
    if (!streaming_.exchange(false)) return;
    try {
        spi_config_send(MODULE_DVS, DVS_RUN, false);
        spi_config_send(MODULE_MULTIPLEXER, MUX_RUN, false);
        spi_config_send(MODULE_MULTIPLEXER, MUX_TIMESTAMP_RUN, false);
        spi_config_send(MODULE_USB, USB_RUN, false);
        spi_config_send(MODULE_MULTIPLEXER, MUX_RUN_CHIP, false);
    } catch (...) {
        // Device may already be gone — still tear the transfers down.
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    usb_data_transfers_stop();
    parser_.reset();
}

} // namespace gui::davis
