#include "csv_export.h"

#include <cstdio>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <utility>

namespace tiny2d::sandbox {
namespace {

void Require(bool condition, const char* message) {
  if (!condition) {
    throw std::invalid_argument(message);
  }
}

// Comment-line values must stay on one line; CR and LF become spaces.
std::string FlattenMetadataValue(const std::string& value) {
  std::string flattened = value;
  for (char& character : flattened) {
    if (character == '\n' || character == '\r') {
      character = ' ';
    }
  }
  return flattened;
}

}  // namespace

std::string CsvFloat(float value) {
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.9g", static_cast<double>(value));
  return buffer;
}

std::string CsvDouble(double value) {
  char buffer[40];
  std::snprintf(buffer, sizeof(buffer), "%.17g", value);
  return buffer;
}

std::string EscapeCsvField(const std::string& field) {
  const bool needs_quotes =
      field.find_first_of(",\"\r\n") != std::string::npos ||
      (!field.empty() && (field.front() == ' ' || field.back() == ' '));
  if (!needs_quotes) {
    return field;
  }
  std::string escaped = "\"";
  for (const char character : field) {
    if (character == '"') {
      escaped += "\"\"";
    } else {
      escaped += character;
    }
  }
  escaped += '"';
  return escaped;
}

std::string BuildCsv(const CsvMetadata& metadata,
                     const std::vector<std::string>& columns,
                     const std::vector<std::vector<std::string>>& rows) {
  Require(!metadata.model_id.empty(), "CSV metadata needs a model id.");
  Require(!columns.empty(), "CSV export needs at least one column.");
  for (const std::vector<std::string>& row : rows) {
    Require(row.size() == columns.size(),
            "Every CSV row must match the column count.");
  }

  std::string csv;
  csv += "# tiny2d-csv " + std::to_string(kCsvFormatVersion) + "\n";
  csv +=
      "# product_version: " + FlattenMetadataValue(metadata.product_version) +
      "\n";
  csv += "# model: " + FlattenMetadataValue(metadata.model_id) + "\n";
  for (const auto& [name, value] : metadata.parameters) {
    csv += "# param " + FlattenMetadataValue(name) + ": " +
           FlattenMetadataValue(value) + "\n";
  }
  csv += "# status: " + FlattenMetadataValue(metadata.status) + "\n";

  for (std::size_t i = 0; i < columns.size(); ++i) {
    csv += (i == 0 ? "" : ",") + EscapeCsvField(columns[i]);
  }
  csv += '\n';
  for (const std::vector<std::string>& row : rows) {
    for (std::size_t i = 0; i < row.size(); ++i) {
      csv += (i == 0 ? "" : ",") + EscapeCsvField(row[i]);
    }
    csv += '\n';
  }
  return csv;
}

std::string MakeCsvFileName(const std::string& slug,
                            const std::string& timestamp, int attempt) {
  Require(!slug.empty() && !timestamp.empty(),
          "CSV file names need a slug and a timestamp.");
  Require(slug.find_first_of("/\\") == std::string::npos &&
              timestamp.find_first_of("/\\") == std::string::npos,
          "CSV file name parts must not contain path separators.");
  Require(attempt >= 1, "CSV file name attempts start at 1.");
  std::string name = slug + "_" + timestamp;
  if (attempt > 1) {
    name += "_" + std::to_string(attempt);
  }
  return name + ".csv";
}

bool ReadTextFile(const std::string& path, std::string* content,
                  std::string* error) {
  if (content == nullptr) {
    if (error != nullptr) {
      *error = "Read target is null.";
    }
    return false;
  }
  std::ifstream stream(path, std::ios::binary);
  if (!stream.is_open()) {
    if (error != nullptr) {
      *error = "Could not open '" + path + "' for reading.";
    }
    return false;
  }
  std::string text((std::istreambuf_iterator<char>(stream)),
                   std::istreambuf_iterator<char>());
  if (stream.bad()) {
    if (error != nullptr) {
      *error = "Reading '" + path + "' failed.";
    }
    return false;
  }
  *content = std::move(text);
  return true;
}

bool WriteTextFile(const std::string& path, const std::string& content,
                   std::string* error) {
  std::ofstream stream(path, std::ios::binary | std::ios::trunc);
  if (!stream.is_open()) {
    if (error != nullptr) {
      *error = "Could not open '" + path + "' for writing.";
    }
    return false;
  }
  stream.write(content.data(), static_cast<std::streamsize>(content.size()));
  stream.flush();
  if (!stream.good()) {
    if (error != nullptr) {
      *error = "Writing '" + path + "' failed.";
    }
    return false;
  }
  return true;
}

}  // namespace tiny2d::sandbox
