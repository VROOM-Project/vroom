// main.cpp
#include <aws/core/Aws.h>
#include <aws/core/auth/AWSCredentialsProvider.h>
#include <aws/core/client/ClientConfiguration.h>
#include <aws/core/platform/Environment.h>
#include <aws/core/utils/HashingUtils.h>
#include <aws/core/utils/json/JsonSerializer.h>
#include <aws/core/utils/logging/ConsoleLogSystem.h>
#include <aws/core/utils/logging/LogLevel.h>
#include <aws/core/utils/logging/LogMacros.h>
#include <aws/lambda-runtime/runtime.h>
#include <aws/s3/S3Client.h>
#include <aws/s3/model/GetObjectRequest.h>
#include <iostream>
#include <memory>
using namespace aws::lambda_runtime;

// vroom includes
#include "problems/vrp.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include "structures/cl_args.h"
#include "utils/exception.h"
#include "utils/helpers.h"
#include "utils/input_parser.h"
#include "utils/output_json.h"
#include "utils/version.h"

/*
Vroom Input and Output.
*/
std::string runVroom(const std::string& vroomInputAsString,
                     const int timeout,
                     const int threads,
                     const int explorationLimit) {
  /*
  vroomInputAsString: {.....}     // contains vroom input
  timeout: int              			// stop solving after 'timout' seconds
  threads: int                    // number of threads to run solve.
  explorationLevel: int           // exploration level used by vroom
  */
  // Defaults
  bool useGeometry = false; // Support for geometry needs to be added. Will be
                            // added in future if needed.

  vroom::Input problemInstance = vroom::Input();
  vroom::io::parse(problemInstance, vroomInputAsString, useGeometry);

  vroom::Solution sol =
    problemInstance.solve(explorationLimit, threads, timeout);

  auto json_output = vroom::io::to_json(sol, useGeometry);

  rapidjson::StringBuffer buffer;
  buffer.Clear();
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  json_output.Accept(writer);

  return std::string(buffer.GetString()); // free() automatically called.
}

/*
AWS lambda functioning.
*/

// Function and constants declarations
std::string download_and_encode_file(Aws::S3::S3Client const& client,
                                     Aws::String const& bucket,
                                     Aws::String const& key,
                                     Aws::String& encoded_output);

std::string encode(Aws::String const& filename, Aws::String& output);
char const TAG[] = "LAMBDA_ALLOC";
static const char* ALLOCATION_TAG = "vroomLambda";

// Lambda handler
static invocation_response my_handler(invocation_request const& req) {
  /*
    payload: {
            "vroom_input": obj,	// contains vroom input
            "timout": int, 			// stop solving after 'timout' seconds
            "threads": int,     // number of threads to run solve.
            "exploration": int,	// exploration level used by vroom
            "use_s3_link": bool,
            "s3_bucket": string,
            "file_key": string,
    }
    */
  using namespace Aws::Utils::Json;
  JsonValue json(req.payload);
  if (!json.WasParseSuccessful()) {
    return invocation_response::failure("Failed to parse input JSON",
                                        "InvalidJSON");
  }

  auto requestPayload = json.View();
  auto timeout = (requestPayload.ValueExists("timeout"))
                   ? requestPayload.GetInteger("timeout") * 60
                   : 10 * 60; // 10 minutes
  auto threads = (requestPayload.ValueExists("threads"))
                   ? requestPayload.GetInteger("threads")
                   : 2;
  auto exploration = (requestPayload.ValueExists("exploration"))
                       ? requestPayload.GetInteger("exploration")
                       : 2;
  auto useS3 = (requestPayload.ValueExists("use_s3_link"))
                 ? requestPayload.GetBool("use_s3_link")
                 : false;

  std::string vroomInput;
  if (useS3) {
    if (!requestPayload.ValueExists("s3_bucket") ||
        !requestPayload.ValueExists("file_key") ||
        !requestPayload.GetObject("s3_bucket").IsString() ||
        !requestPayload.GetObject("file_key").IsString()) {
      return invocation_response::
        failure("Missing input value s3_bucket or file_key", "InvalidJSON");
    }
    // auto bucket = requestPayload.GetString("s3_bucket");
    // auto key = requestPayload.GetString("file_key");

    // AWS_LOGSTREAM_INFO(TAG,
    //                    "Attempting to download file from s3://" << bucket <<
    //                    "/"
    //                                                             << key);

    // Aws::String base64_encoded_file;
    // auto err =
    //   download_and_encode_file(client, bucket, key, base64_encoded_file);
    // if (!err.empty()) {
    //   return invocation_response::failure(err, "DownloadFailure");
    // }
    // // TODO: how to get from S3 testing.
    // return invocation_response::success(base64_encoded_file,
    //                                     "application/base64");
  } else {
    if (!requestPayload.ValueExists("vroom_input") ||
        !requestPayload.GetObject("vroom_input").IsObject()) {
      return invocation_response::
        failure("Some problem with the vroomInput, not a json object",
                "InvalidJSON");
    }
    vroomInput = requestPayload.GetObject("vroom_input").WriteCompact();
  }

  // Run vroom
  std::string solution = runVroom(vroomInput, timeout, threads, exploration);

  return invocation_response::success(solution, "application/json");
}

std::function<std::shared_ptr<Aws::Utils::Logging::LogSystemInterface>()>
GetConsoleLoggerFactory() {
  return [] {
    return Aws::MakeShared<
      Aws::Utils::Logging::ConsoleLogSystem>("console_logger",
                                             Aws::Utils::Logging::LogLevel::
                                               Trace);
  };
}

int main() {
  // // S3 usage
  // using namespace Aws;
  // SDKOptions options;
  // options.loggingOptions.logLevel = Aws::Utils::Logging::LogLevel::Trace;
  // options.loggingOptions.logger_create_fn = GetConsoleLoggerFactory();
  // InitAPI(options);
  // {
  //   Client::ClientConfiguration config;
  //   config.region = Aws::Environment::GetEnv("AWS_REGION");
  //   config.caFile = "/etc/pki/tls/certs/ca-bundle.crt";

  //   auto credentialsProvider =
  //     Aws::MakeShared<Aws::Auth::EnvironmentAWSCredentialsProvider>(TAG);
  //   S3::S3Client client(credentialsProvider,
  //                       Aws::MakeShared<S3EndpointProvider>(ALLOCATION_TAG),
  //                       config);
  //   auto handler_fn =
  //     [&client](aws::lambda_runtime::invocation_request const& req) {
  //       return my_handler(req, client);
  //     };
  //   run_handler(handler_fn);
  // }
  // ShutdownAPI(options);
  run_handler(my_handler);
  return 0;
}

std::string encode(Aws::IOStream& stream, Aws::String& output) {
  Aws::Vector<unsigned char> bits;
  bits.reserve(stream.tellp());
  stream.seekg(0, stream.beg);

  char streamBuffer[1024 * 4];
  while (stream.good()) {
    stream.read(streamBuffer, sizeof(streamBuffer));
    auto bytesRead = stream.gcount();

    if (bytesRead > 0) {
      bits.insert(bits.end(),
                  (unsigned char*)streamBuffer,
                  (unsigned char*)streamBuffer + bytesRead);
    }
  }
  Aws::Utils::ByteBuffer bb(bits.data(), bits.size());
  output = Aws::Utils::HashingUtils::Base64Encode(bb);
  return {};
}

std::string download_and_encode_file(Aws::S3::S3Client const& client,
                                     Aws::String const& bucket,
                                     Aws::String const& key,
                                     Aws::String& encoded_output) {
  using namespace Aws;

  S3::Model::GetObjectRequest request;
  request.WithBucket(bucket).WithKey(key);

  auto outcome = client.GetObject(request);
  if (outcome.IsSuccess()) {
    AWS_LOGSTREAM_INFO(TAG, "Download completed!");
    auto& s = outcome.GetResult().GetBody();
    return encode(s, encoded_output);
  } else {
    AWS_LOGSTREAM_ERROR(TAG, "Failed with error: " << outcome.GetError());
    return outcome.GetError().GetMessage();
  }
}