#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cstring>

int main() {
    // Command to run the Python script
    const char* command = "python3 ./server.py";  // Change this to your script path

    // Open a pipe to the command
    FILE* pipe = popen(command, "r");
    if (!pipe) {
        std::cerr << "Failed to run Python script" << std::endl;
        return 1;
    }

    // Buffer to store the output
    char buffer[128];
    std::string result = "";

    // Read the output line by line
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        result += buffer;

    }
    size_t pos = result.find(":");
    result = result.substr(pos + 1);
    size_t pos_comma = result.find(",");
    double x = 0;
    double y = 0;
    x = std::stod(result.substr(0, pos_comma));
    y = std::stod(result.substr(pos_comma + 1));
    // Close the pipe
    fclose(pipe);

    // Print the captured output
    //std::cout << "y from Python script:\n" << y << std::endl;
        std::cout << "x from Python script : " << x << "\n";
        std::cout << "x from Python script : " << y << "\n";
        std::cout << "x+y from Python script : " << x+y << "\n";


    return 0;
}
