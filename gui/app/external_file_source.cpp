// gui/app/external_file_source.cpp

#include "external_file_source.h"

#include "aedat4_file_source.h"
#include "alpdata_file_source.h"

#include <algorithm>
#include <cctype>

namespace gui {

ExternalFileSource::~ExternalFileSource() = default;

namespace {

std::string lower_extension(const std::string& path) {
    const auto dot = path.rfind('.');
    if (dot == std::string::npos) return {};
    std::string ext = path.substr(dot);
    std::transform(ext.begin(), ext.end(), ext.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return ext;
}

} // namespace

bool is_external_file_extension(const std::string& path) {
    const std::string ext = lower_extension(path);
    return ext == ".aedat4" || ext == ".alpdata";
}

std::unique_ptr<ExternalFileSource> try_open_external_file(const std::string& path) {
    const std::string ext = lower_extension(path);
    if (ext == ".aedat4") {
        return std::make_unique<Aedat4FileSource>(path);
    }
    if (ext == ".alpdata") {
        return std::make_unique<AlpdataFileSource>(path);
    }
    return nullptr;
}

} // namespace gui
