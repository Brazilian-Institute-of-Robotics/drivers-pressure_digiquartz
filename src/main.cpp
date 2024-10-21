// Copyright (c) 2023, SENAI Cimatec
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdexcept>
#include <string>
#include <iostream>
#include "pressure_pkg/pressure_driver.hpp"

int main(int argc, char ** argv)
{
  std::ignore = argc;
  std::ignore = argv;

  // Inicializa o driver de pressão
  pressure_pkg::PressureDriver pressure_driver;

  double pressure_value{0.0};

  while (true) {
    try {
      pressure_value = pressure_driver.getPressure();
      std::cout << "Pressure: " << pressure_value << std::endl;
    } catch (const std::exception & error) {
      std::cout << error.what() << std::endl;
    }
  }

  return 0;
}
