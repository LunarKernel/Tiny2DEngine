#include "csv_export.h"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "test_support.h"

namespace {

using tiny2d::sandbox::BuildCsv;
using tiny2d::sandbox::CsvDouble;
using tiny2d::sandbox::CsvFloat;
using tiny2d::sandbox::CsvMetadata;
using tiny2d::sandbox::EscapeCsvField;
using tiny2d::sandbox::MakeCsvFileName;
using tiny2d::sandbox::WriteTextFile;

void TestMetadataLinesAndDeterminism() {
  CsvMetadata metadata;
  metadata.model_id = "V20 StackLab";
  metadata.product_version = "9.9.9-test";
  metadata.parameters = {{"box_count", "10"}, {"box_edge_m", "1"}};
  metadata.status = "RUNNING t=1.5 s\ncriteria 5/5";

  const std::vector<std::string> columns = {"time_s", "value_m"};
  const std::vector<std::vector<std::string>> rows = {{"0", "1.5"},
                                                      {"0.25", "-2"}};
  const std::string csv = BuildCsv(metadata, columns, rows);
  const std::string expected =
      "# tiny2d-csv 1\n"
      "# product_version: 9.9.9-test\n"
      "# model: V20 StackLab\n"
      "# param box_count: 10\n"
      "# param box_edge_m: 1\n"
      "# status: RUNNING t=1.5 s criteria 5/5\n"
      "time_s,value_m\n"
      "0,1.5\n"
      "0.25,-2\n";
  CHECK(csv == expected);
  // Deterministic: equal inputs give byte-equal output.
  CHECK(BuildCsv(metadata, columns, rows) == csv);
}

void TestFieldEscaping() {
  CHECK(EscapeCsvField("plain") == "plain");
  CHECK(EscapeCsvField("") == "");
  CHECK(EscapeCsvField("a,b") == "\"a,b\"");
  CHECK(EscapeCsvField("say \"hi\"") == "\"say \"\"hi\"\"\"");
  CHECK(EscapeCsvField("line\nbreak") == "\"line\nbreak\"");
  CHECK(EscapeCsvField(" leading") == "\" leading\"");
  CHECK(EscapeCsvField("trailing ") == "\"trailing \"");
  CHECK(EscapeCsvField("inner space") == "inner space");

  // Escaped cells survive inside a built document.
  CsvMetadata metadata;
  metadata.model_id = "escape";
  const std::string csv =
      BuildCsv(metadata, {"a,b"}, {{"cell \"quoted\", with comma"}});
  CHECK(csv.find("\"a,b\"") != std::string::npos);
  CHECK(csv.find("\"cell \"\"quoted\"\", with comma\"") != std::string::npos);
}

void TestNumericRoundTrip() {
  const float float_values[] = {0.0f,
                                -0.0f,
                                0.1f,
                                1.0f / 3.0f,
                                0.60000002384185791f,
                                -123.456f,
                                1.17549435e-38f,
                                3.40282347e+38f};
  for (const float value : float_values) {
    const std::string text = CsvFloat(value);
    const float parsed = std::strtof(text.c_str(), nullptr);
    CHECK(parsed == value);
  }
  const double double_values[] = {0.0,
                                  0.1,
                                  1.0 / 3.0,
                                  3.14159265358979312,
                                  -9.81,
                                  2.2250738585072014e-308,
                                  1.7976931348623157e+308};
  for (const double value : double_values) {
    const std::string text = CsvDouble(value);
    const double parsed = std::strtod(text.c_str(), nullptr);
    CHECK(parsed == value);
  }
}

void TestBuildRejectsBadShapes() {
  const auto expect_reject =
      [](const CsvMetadata& metadata, const std::vector<std::string>& columns,
         const std::vector<std::vector<std::string>>& rows) {
        bool threw = false;
        try {
          BuildCsv(metadata, columns, rows);
        } catch (const std::invalid_argument&) {
          threw = true;
        }
        CHECK(threw);
      };

  CsvMetadata valid;
  valid.model_id = "model";
  CsvMetadata no_model;
  expect_reject(no_model, {"time_s"}, {});
  expect_reject(valid, {}, {});
  expect_reject(valid, {"time_s"}, {{"0", "extra"}});
  expect_reject(valid, {"a", "b"}, {{"only-one"}});
}

void TestMakeCsvFileName() {
  CHECK(MakeCsvFileName("stack_lab", "20260815_010203", 1) ==
        "stack_lab_20260815_010203.csv");
  CHECK(MakeCsvFileName("chaos_lab", "20260815_010203", 2) ==
        "chaos_lab_20260815_010203_2.csv");
  CHECK(MakeCsvFileName("chaos_lab", "20260815_010203", 13) ==
        "chaos_lab_20260815_010203_13.csv");

  const auto expect_reject = [](const std::string& slug,
                                const std::string& timestamp, int attempt) {
    bool threw = false;
    try {
      MakeCsvFileName(slug, timestamp, attempt);
    } catch (const std::invalid_argument&) {
      threw = true;
    }
    CHECK(threw);
  };
  expect_reject("", "20260815_010203", 1);
  expect_reject("stack_lab", "", 1);
  expect_reject("stack_lab", "20260815_010203", 0);
  expect_reject("bad/slug", "20260815_010203", 1);
  expect_reject("stack_lab", "..\\up", 1);
}

void TestWriteTextFile() {
  namespace fs = std::filesystem;
  const fs::path path =
      fs::temp_directory_path() / "tiny2d_csv_export_test_roundtrip.csv";
  const std::string content = "# tiny2d-csv 1\ntime_s\n0\n";
  std::string error;
  CHECK(WriteTextFile(path.string(), content, &error));
  CHECK(error.empty());
  std::ifstream stream(path, std::ios::binary);
  const std::string read_back((std::istreambuf_iterator<char>(stream)),
                              std::istreambuf_iterator<char>());
  stream.close();
  CHECK(read_back == content);
  fs::remove(path);

  const fs::path bad_path =
      fs::temp_directory_path() / "tiny2d_csv_export_missing_dir" / "file.csv";
  error.clear();
  CHECK(!WriteTextFile(bad_path.string(), content, &error));
  CHECK(!error.empty());
}

struct NamedTest {
  const char* name;
  void (*function)();
};

}  // namespace

int main() {
  std::cout << std::unitbuf;
  const NamedTest tests[] = {
      {"metadata lines and determinism", TestMetadataLinesAndDeterminism},
      {"field escaping", TestFieldEscaping},
      {"numeric round trip", TestNumericRoundTrip},
      {"build rejects bad shapes", TestBuildRejectsBadShapes},
      {"file name builder", TestMakeCsvFileName},
      {"write text file", TestWriteTextFile},
  };
  for (const NamedTest& test : tests) {
    test.function();
    std::cout << "[PASS] " << test.name << '\n';
  }
  std::cout << sizeof(tests) / sizeof(tests[0]) << " tests, "
            << tiny2d::test::CheckCount() << " checks passed\n";
  return 0;
}
