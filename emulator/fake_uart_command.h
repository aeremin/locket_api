#ifndef LOCKET_API_FAKE_UART_COMMAND_H
#define LOCKET_API_FAKE_UART_COMMAND_H

#include <cstdint>
#include <optional>
#include <string>
#include <sstream>

#include "api/uart_command.h"

class FakeUartCommand: public UartCommand {
public:
    explicit FakeUartCommand(const std::string& command_line);

    bool NameIs(const char* expected_name) const override;
    std::optional<int32_t> GetNext() override;

private:
    std::string command;
    std::stringstream stream;
};

#endif //LOCKET_API_FAKE_UART_COMMAND_H
