// main.cpp
#include <aws/lambda-runtime/runtime.h>
using namespace aws::lambda_runtime;



// We will mimic the main function of the vroom.cpp
/* 
Input: {
   "vroom_input": json string,
   "limit": stop solving after 'limit' seconds.
   "threads": threads
}
*/

invocation_response my_handler(invocation_request const& request)
{
   run_example_with_custom_matrix();
   return invocation_response::success("Hello, World!", "application/json");
}

int main()
{
   run_handler(my_handler);
   return 0;
}