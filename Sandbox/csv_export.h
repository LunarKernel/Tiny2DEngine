#ifndef TINY2DENGINE_SANDBOX_CSV_EXPORT_H_
#define TINY2DENGINE_SANDBOX_CSV_EXPORT_H_

#include <string>
#include <utility>
#include <vector>

namespace tiny2d::sandbox {

// Format version written as the first line of every export. Bump only
// with a documented format change.
inline constexpr int kCsvFormatVersion = 1;

// Metadata block written as '#'-prefixed comment lines ahead of the CSV
// header row: format version, product version, model identifier, one
// line per input parameter, and the caller's status summary (the run
// state at export time). Values are flattened to one line; commas,
// quotes, and spaces need no escaping inside comment lines.
struct CsvMetadata {
  std::string model_id;
  std::string product_version;
  std::vector<std::pair<std::string, std::string>> parameters;
  std::string status;
};

// Round-trip numeric formatting: 9 significant digits reproduce every
// float bit pattern, 17 reproduce every double. Values must be finite
// for faithful re-analysis; non-finite values format per printf.
std::string CsvFloat(float value);
std::string CsvDouble(double value);

// RFC-4180 field escaping: a field containing a comma, double quote,
// CR, LF, or a leading or trailing space is wrapped in double quotes
// with embedded quotes doubled; other fields pass through unchanged.
std::string EscapeCsvField(const std::string& field);

// Builds the complete export: metadata comment lines, the header row,
// then one row per sample. Every cell and column name goes through
// EscapeCsvField. Throws std::invalid_argument (producing no partial
// output) when model_id or columns is empty or any row's width differs
// from the column count. Deterministic: equal inputs give equal bytes.
std::string BuildCsv(const CsvMetadata& metadata,
                     const std::vector<std::string>& columns,
                     const std::vector<std::vector<std::string>>& rows);

// "<slug>_<timestamp>.csv" for attempt 1, "<slug>_<timestamp>_<n>.csv"
// for collision attempts n >= 2 (the caller probes existence and
// increments). Throws std::invalid_argument when slug or timestamp is
// empty or contains a path separator, or when attempt < 1.
std::string MakeCsvFileName(const std::string& slug,
                            const std::string& timestamp, int attempt);

// Writes content to path, replacing any existing file. Returns false
// and fills error (when non-null) on failure; never throws.
bool WriteTextFile(const std::string& path, const std::string& content,
                   std::string* error);

// Reads path's entire contents into content. Returns false and fills
// error (when non-null) on failure; never throws.
bool ReadTextFile(const std::string& path, std::string* content,
                  std::string* error);

}  // namespace tiny2d::sandbox

#endif  // TINY2DENGINE_SANDBOX_CSV_EXPORT_H_
