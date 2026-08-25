#include "experiment_file.h"

#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "test_support.h"

namespace {

using tiny2d::sandbox::ExperimentFile;
using tiny2d::sandbox::MakeExperimentFileName;
using tiny2d::sandbox::ParseExperiment;
using tiny2d::sandbox::WriteExperiment;

ExperimentFile MakeSample() {
  ExperimentFile file;
  file.model_id = "V20 StackLab";
  file.product_version = "2.0.0";
  file.parameters = {{"box_count", "10"}, {"box_edge_m", "1"}};
  file.checkpoints = {{"time_s", "2.0020833334419876"}, {"box0_x_m", "50"}};
  return file;
}

void TestWriteFormatAndDeterminism() {
  const std::string text = WriteExperiment(MakeSample());
  const std::string expected =
      "# tiny2d-exp 1\n"
      "# product_version: 2.0.0\n"
      "# model: V20 StackLab\n"
      "param box_count: 10\n"
      "param box_edge_m: 1\n"
      "checkpoint time_s: 2.0020833334419876\n"
      "checkpoint box0_x_m: 50\n";
  CHECK(text == expected);
  CHECK(WriteExperiment(MakeSample()) == text);
}

void TestRoundTrip() {
  const ExperimentFile original = MakeSample();
  ExperimentFile parsed;
  std::string error;
  CHECK(ParseExperiment(WriteExperiment(original), &parsed, &error));
  CHECK(error.empty());
  CHECK(parsed.model_id == original.model_id);
  CHECK(parsed.product_version == original.product_version);
  CHECK(parsed.parameters == original.parameters);
  CHECK(parsed.checkpoints == original.checkpoints);
}

void TestWriteRejections() {
  const auto expect_throw = [](const ExperimentFile& file) {
    bool threw = false;
    try {
      WriteExperiment(file);
    } catch (const std::invalid_argument&) {
      threw = true;
    }
    CHECK(threw);
  };
  ExperimentFile no_model = MakeSample();
  no_model.model_id.clear();
  expect_throw(no_model);
  ExperimentFile empty_key = MakeSample();
  empty_key.parameters.push_back({"", "1"});
  expect_throw(empty_key);
  ExperimentFile duplicate = MakeSample();
  duplicate.checkpoints.push_back({"time_s", "3"});
  expect_throw(duplicate);
}

void TestParseRejections() {
  const auto expect_reject = [](const std::string& text,
                                const std::string& expected_error) {
    ExperimentFile sentinel;
    sentinel.model_id = "untouched";
    std::string error;
    CHECK(!ParseExperiment(text, &sentinel, &error));
    CHECK(error == expected_error);
    // Failure atomicity: the output is untouched.
    CHECK(sentinel.model_id == "untouched");
  };

  expect_reject("", "Not a tiny2d-exp file.");
  expect_reject("garbage\n", "Not a tiny2d-exp file.");
  expect_reject("# tiny2d-exp 2\n# product_version: x\n# model: m\n",
                "Unsupported tiny2d-exp format version.");
  expect_reject("# tiny2d-exp 1\n# model: m\n# product_version: x\n",
                "Missing product_version header.");
  expect_reject("# tiny2d-exp 1\n# product_version: x\n# model: \n",
                "Missing model header.");
  const std::string header =
      "# tiny2d-exp 1\n# product_version: 2.0.0\n# model: m\n";
  expect_reject(header + "bogus line\n", "Unknown line in experiment file.");
  expect_reject(header + "param broken\n", "Malformed param line.");
  expect_reject(header + "checkpoint : 5\n", "Malformed checkpoint line.");
  expect_reject(header + "param a: 1\nparam a: 2\n",
                "Duplicate key in experiment file.");
  expect_reject(header + "param a: 1\n\ncheckpoint t: 2\n",
                "Unknown line in experiment file.");
  CHECK(!ParseExperiment(header, nullptr, nullptr));
}

void TestParseToleratesLineEndingsAndTrailingBlank() {
  // CRLF and a trailing blank line both parse identically to LF.
  const std::string crlf =
      "# tiny2d-exp 1\r\n# product_version: 2.0.0\r\n# model: m\r\n"
      "param a: 1\r\n\r\n";
  ExperimentFile parsed;
  std::string error;
  CHECK(ParseExperiment(crlf, &parsed, &error));
  CHECK(parsed.model_id == "m");
  CHECK(parsed.parameters.size() == 1);
  CHECK(parsed.parameters[0] ==
        std::make_pair(std::string("a"), std::string("1")));
}

void TestFileNameBuilder() {
  CHECK(MakeExperimentFileName("stack_lab", "20260816_010203", 1) ==
        "stack_lab_20260816_010203.exp");
  CHECK(MakeExperimentFileName("chaos_lab", "20260816_010203", 3) ==
        "chaos_lab_20260816_010203_3.exp");
  const auto expect_throw = [](const std::string& slug,
                               const std::string& timestamp, int attempt) {
    bool threw = false;
    try {
      MakeExperimentFileName(slug, timestamp, attempt);
    } catch (const std::invalid_argument&) {
      threw = true;
    }
    CHECK(threw);
  };
  expect_throw("", "20260816_010203", 1);
  expect_throw("stack_lab", "", 1);
  expect_throw("stack_lab", "20260816_010203", 0);
  expect_throw("bad/slug", "20260816_010203", 1);
  expect_throw("stack_lab", "..\\up", 1);
}

struct NamedTest {
  const char* name;
  void (*function)();
};

}  // namespace

int main() {
  std::cout << std::unitbuf;
  const NamedTest tests[] = {
      {"write format and determinism", TestWriteFormatAndDeterminism},
      {"round trip", TestRoundTrip},
      {"write rejections", TestWriteRejections},
      {"parse rejections", TestParseRejections},
      {"line endings and trailing blank",
       TestParseToleratesLineEndingsAndTrailingBlank},
      {"file name builder", TestFileNameBuilder},
  };
  for (const NamedTest& test : tests) {
    test.function();
    std::cout << "[PASS] " << test.name << '\n';
  }
  std::cout << sizeof(tests) / sizeof(tests[0]) << " tests, "
            << tiny2d::test::CheckCount() << " checks passed\n";
  return 0;
}
