#pragma once

#include <sstream>

std::string create_date() {
  time_t t = time(nullptr);
  const tm* localTime = localtime(&t);
  std::stringstream s;
  s << "20" << localTime->tm_year - 100 << "_";
  s << localTime->tm_mon + 1 << "_";
  s << localTime->tm_mday << "_";
  s << localTime->tm_hour << "_";
  s << localTime->tm_min << "_";
  s << localTime->tm_sec;
  return (s.str());
}
