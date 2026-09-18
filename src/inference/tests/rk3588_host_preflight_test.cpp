#include "robot/rk3588_host_preflight.hpp"

#include <cerrno>
#include <csignal>
#include <cstring>
#include <iostream>
#include <spawn.h>
#include <stdexcept>
#include <sys/wait.h>

namespace {

void expect(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

constexpr pid_t kChild = 123;
int spawn_error = 0;
int wait_error = 0;
int child_status = 0;
int interruptions = 0;
int wait_calls = 0;

void test_exit_status()
{
    std::string error = "previous error";
    expect(inference::verify_rk3588_host_layout(error) && error.empty(),
           "successful live check must clear the old error");
    for (const int code : {1, 7, 127}) {
        child_status = code << 8;
        expect(!inference::verify_rk3588_host_layout(error) &&
                   error.find("exit_status=" + std::to_string(code)) != std::string::npos,
               "application ignored or lost the script failure status");
    }
    child_status = SIGTERM;
    expect(!inference::verify_rk3588_host_layout(error) &&
               error.find("signal=" + std::to_string(SIGTERM)) != std::string::npos,
           "signaled preflight must fail");
}

void test_process_errors()
{
    std::string error;
    for (const int code : {ENOENT, EACCES}) {
        spawn_error = code;
        wait_calls = 0;
        expect(!inference::verify_rk3588_host_layout(error) && wait_calls == 0 &&
                   error.find(std::strerror(code)) != std::string::npos,
               "unavailable deployment script must fail without waiting");
    }
    spawn_error = 0;
    child_status = 0;
    wait_error = ECHILD;
    expect(!inference::verify_rk3588_host_layout(error) &&
               error.find("cannot wait") != std::string::npos,
           "wait failure must fail preflight");
    wait_error = 0;
    interruptions = 1;
    wait_calls = 0;
    expect(inference::verify_rk3588_host_layout(error) && error.empty() && wait_calls == 2,
           "interrupted wait must retry the same live check");
}

}  // namespace

// The application boundary is tested without invoking an installed system script.
extern "C" int __wrap_posix_spawn(pid_t* child, const char* path,
    const posix_spawn_file_actions_t* actions, const posix_spawnattr_t* attributes,
    char* const arguments[], char* const environment[])
{
    expect(std::strcmp(path, "/usr/local/sbin/robot-rt-setup") == 0 &&
               std::strcmp(arguments[0], path) == 0 &&
               std::strcmp(arguments[1], "check") == 0 && arguments[2] == nullptr &&
               actions == nullptr && attributes == nullptr && environment != nullptr,
           "preflight must invoke only the fixed deployment live check");
    *child = kChild;
    return spawn_error;
}

extern "C" pid_t __wrap_waitpid(pid_t child, int* status, int options)
{
    expect(child == kChild && options == 0, "preflight must wait for its own child");
    ++wait_calls;
    if (interruptions > 0) {
        --interruptions;
        errno = EINTR;
        return -1;
    }
    if (wait_error != 0) {
        errno = wait_error;
        return -1;
    }
    *status = child_status;
    return child;
}

int main()
{
    try {
        test_exit_status();
        test_process_errors();
        std::cout << "rk3588_host_preflight_test passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
