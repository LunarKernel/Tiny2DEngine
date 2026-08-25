#include "experiment_file.h"

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <stdexcept>
#include <string>

namespace tiny2d::sandbox {
namespace {

void Require(bool condition, const char* message) {
  if (!condition) {
    throw std::invalid_argument(message);
  }
}

bool HasDuplicateKey(
    const std::vector<std::pair<std::string, std::string>>& entries) {
  for (std::size_t i = 0; i < entries.size(); ++i) {
    for (std::size_t j = i + 1; j < entries.size(); ++j) {
      if (entries[i].first == entries[j].first) {
        return true;
      }
    }
  }
  return false;
}

void FailParse(std::string* error, const std::string& message) {
  if (error != nullptr) {
    *error = message;
  }
}

// Splits "<key>: <value>" after the given prefix; empty value is
// allowed, an empty key or a missing ": " separator is not.
bool SplitKeyValue(const std::string& line, std::size_t prefix_length,
                   std::string* key, std::string* value) {
  const std::size_t separator = line.find(": ", prefix_length);
  if (separator == std::string::npos || separator == prefix_length) {
    return false;
  }
  *key = line.substr(prefix_length, separator - prefix_length);
  *value = line.substr(separator + 2);
  return true;
}

}  // namespace

std::string WriteExperiment(const ExperimentFile& file) {
  Require(!file.model_id.empty(), "Experiment files need a model id.");
  for (const auto& list : {file.parameters, file.checkpoints}) {
    for (const auto& [key, value] : list) {
      static_cast<void>(value);
      Require(!key.empty(), "Experiment keys must not be empty.");
    }
    Require(!HasDuplicateKey(list),
            "Experiment keys must be unique within their list.");
  }

  std::string text;
  text += "# tiny2d-exp " + std::to_string(kExperimentFormatVersion) + "\n";
  text += "# product_version: " + file.product_version + "\n";
  text += "# model: " + file.model_id + "\n";
  for (const auto& [key, value] : file.parameters) {
    text += "param ";
    text += key;
    text += ": ";
    text += value;
    text += '\n';
  }
  for (const auto& [key, value] : file.checkpoints) {
    text += "checkpoint ";
    text += key;
    text += ": ";
    text += value;
    text += '\n';
  }
  return text;
}

bool ParseExperiment(const std::string& text, ExperimentFile* out,
                     std::string* error) {
  if (out == nullptr) {
    FailParse(error, "Experiment parse target is null.");
    return false;
  }

  std::vector<std::string> lines;
  std::size_t start = 0;
  while (start < text.size()) {
    std::size_t end = text.find('\n', start);
    if (end == std::string::npos) {
      end = text.size();
    }
    std::string line = text.substr(start, end - start);
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    lines.push_back(std::move(line));
    start = end + 1;
  }
  while (!lines.empty() && lines.back().empty()) {
    lines.pop_back();
  }

  const std::string version_prefix = "# tiny2d-exp ";
  const std::string expected_version =
      version_prefix + std::to_string(kExperimentFormatVersion);
  if (lines.size() < 3 || lines[0] != expected_version) {
    FailParse(error, lines.empty() || lines[0].rfind(version_prefix, 0) != 0
                         ? "Not a tiny2d-exp file."
                         : "Unsupported tiny2d-exp format version.");
    return false;
  }
  const std::string product_prefix = "# product_version: ";
  if (lines[1].rfind(product_prefix, 0) != 0) {
    FailParse(error, "Missing product_version header.");
    return false;
  }
  const std::string model_prefix = "# model: ";
  if (lines[2].rfind(model_prefix, 0) != 0 ||
      lines[2].size() == model_prefix.size()) {
    FailParse(error, "Missing model header.");
    return false;
  }

  ExperimentFile parsed;
  parsed.product_version = lines[1].substr(product_prefix.size());
  parsed.model_id = lines[2].substr(model_prefix.size());

  const std::string param_prefix = "param ";
  const std::string checkpoint_prefix = "checkpoint ";
  for (std::size_t i = 3; i < lines.size(); ++i) {
    const std::string& line = lines[i];
    std::string key;
    std::string value;
    if (line.rfind(param_prefix, 0) == 0) {
      if (!SplitKeyValue(line, param_prefix.size(), &key, &value)) {
        FailParse(error, "Malformed param line.");
        return false;
      }
      parsed.parameters.emplace_back(std::move(key), std::move(value));
    } else if (line.rfind(checkpoint_prefix, 0) == 0) {
      if (!SplitKeyValue(line, checkpoint_prefix.size(), &key, &value)) {
        FailParse(error, "Malformed checkpoint line.");
        return false;
      }
      parsed.checkpoints.emplace_back(std::move(key), std::move(value));
    } else {
      FailParse(error, "Unknown line in experiment file.");
      return false;
    }
  }
  if (HasDuplicateKey(parsed.parameters) ||
      HasDuplicateKey(parsed.checkpoints)) {
    FailParse(error, "Duplicate key in experiment file.");
    return false;
  }

  *out = std::move(parsed);
  return true;
}

bool ParseExperimentFloat(const std::string& text, float* out) {
  if (text.empty() || out == nullptr) {
    return false;
  }
  char* end = nullptr;
  const float value = std::strtof(text.c_str(), &end);
  if (end != text.c_str() + text.size() || !std::isfinite(value)) {
    return false;
  }
  *out = value;
  return true;
}

bool ParseExperimentDouble(const std::string& text, double* out) {
  if (text.empty() || out == nullptr) {
    return false;
  }
  char* end = nullptr;
  const double value = std::strtod(text.c_str(), &end);
  if (end != text.c_str() + text.size() || !std::isfinite(value)) {
    return false;
  }
  *out = value;
  return true;
}

bool ParseExperimentInt(const std::string& text, int* out) {
  if (text.empty() || out == nullptr) {
    return false;
  }
  char* end = nullptr;
  const long value = std::strtol(text.c_str(), &end, 10);
  if (end != text.c_str() + text.size() || value < INT_MIN || value > INT_MAX) {
    return false;
  }
  *out = static_cast<int>(value);
  return true;
}

std::string MakeExperimentFileName(const std::string& slug,
                                   const std::string& timestamp, int attempt) {
  Require(!slug.empty() && !timestamp.empty(),
          "Experiment file names need a slug and a timestamp.");
  Require(slug.find_first_of("/\\") == std::string::npos &&
              timestamp.find_first_of("/\\") == std::string::npos,
          "Experiment file name parts must not contain path separators.");
  Require(attempt >= 1, "Experiment file name attempts start at 1.");
  std::string name = slug + "_" + timestamp;
  if (attempt > 1) {
    name += "_" + std::to_string(attempt);
  }
  return name + ".exp";
}

}  // namespace tiny2d::sandbox
