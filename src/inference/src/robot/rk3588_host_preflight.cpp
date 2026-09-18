#include "robot/rk3588_host_preflight.hpp"

#include <cerrno>
#include <cstring>
#include <spawn.h>
#include <sys/wait.h>

extern char** environ;

namespace inference {

bool verify_rk3588_host_layout(std::string& error)
{
    error.clear();
    char executable[] = "/usr/local/sbin/robot-rt-setup";
    char check[] = "check";
    char* arguments[] = {executable, check, nullptr};
    pid_t child;
    const int spawn_error = ::posix_spawn(
        &child, executable, nullptr, nullptr, arguments, environ);
    if (spawn_error != 0) {
        error = std::string("cannot execute /usr/local/sbin/robot-rt-setup check: ") +
            std::strerror(spawn_error);
        return false;
    }

    int status;
    pid_t waited;
    do {
        waited = ::waitpid(child, &status, 0);
    } while (waited == -1 && errno == EINTR);
    if (waited == -1) {
        error = std::string("cannot wait for robot-rt-setup check: ") + std::strerror(errno);
        return false;
    }
    if (WIFEXITED(status)) {
        if (WEXITSTATUS(status) == 0) {
            return true;
        }
        error = "robot-rt-setup check failed: exit_status=" + std::to_string(WEXITSTATUS(status));
    } else if (WIFSIGNALED(status)) {
        error = "robot-rt-setup check terminated: signal=" + std::to_string(WTERMSIG(status));
    } else {
        error = "robot-rt-setup check did not exit normally";
    }
    return false;
}

}  // namespace inference
