#ifndef TINY2DENGINE_SANDBOX_EXPERIMENT_FILE_H_
#define TINY2DENGINE_SANDBOX_EXPERIMENT_FILE_H_

#include <string>
#include <utility>
#include <vector>

namespace tiny2d::sandbox {

// Format version written as the first line of every experiment file.
// The format version gates loading; the product version never does.
inline constexpr int kExperimentFormatVersion = 1;

// One tiny2d-exp document: three '#' header lines (format version,
// product version, model id) followed by bare `param <name>: <value>`
// and `checkpoint <name>: <value>` lines in order. Unlike tiny2d-csv,
// the param and checkpoint lines carry no '#' prefix - the whole file
// is data. Values use the established round-trip formats (9
// significant digits for floats, 17 for doubles).
struct ExperimentFile {
  std::string model_id;
  std::string product_version;
  std::vector<std::pair<std::string, std::string>> parameters;
  std::vector<std::pair<std::string, std::string>> checkpoints;
};

// Emits the document above, deterministically (equal inputs give
// equal bytes). Throws std::invalid_argument (producing no output)
// on an empty model id, an empty key, or a duplicate key within
// either list.
std::string WriteExperiment(const ExperimentFile& file);

// Strict, failure-atomic parse: false leaves out untouched and fills
// error (when non-null) on a missing or malformed header, an
// unsupported format version, an unknown or malformed line, a
// duplicate key, or an empty key. Order within each list is the file
// order.
bool ParseExperiment(const std::string& text, ExperimentFile* out,
                     std::string* error);

// Strict full-consumption value parsers for experiment loading:
// false on empty text, any unconsumed trailing character, or (for
// the floating-point variants) a non-finite result.
bool ParseExperimentFloat(const std::string& text, float* out);
bool ParseExperimentDouble(const std::string& text, double* out);
bool ParseExperimentInt(const std::string& text, int* out);

// "<slug>_<timestamp>.exp" for attempt 1, "<slug>_<timestamp>_<n>.exp"
// for collision attempts n >= 2, with exactly the same rejection
// rules as the CSV name builder (empty parts, path separators,
// attempt < 1 all throw std::invalid_argument).
std::string MakeExperimentFileName(const std::string& slug,
                                   const std::string& timestamp, int attempt);

}  // namespace tiny2d::sandbox

#endif  // TINY2DENGINE_SANDBOX_EXPERIMENT_FILE_H_
