#include "fake_uart_command.h"

#include <iostream>
#include <limits>

FakeUartCommand::FakeUartCommand(const std::string& command_line): stream(command_line) {
    stream >> command;
}

bool FakeUartCommand::NameIs(const char* expected_name) const {
    return command == expected_name;
}

std::optional<int32_t> FakeUartCommand::GetNext() {
    int32_t value = std::numeric_limits<int32_t>::min();
    stream >> value;
    if (value == std::numeric_limits<int32_t>::min()) return std::nullopt;
    return value;
}


