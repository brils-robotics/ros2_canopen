// Copyright (c) 2022, StoglRobotics
// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Lovro Ivanov lovro.ivanov@gmail.com
 * \date    2022-06-29
 *
 */
//----------------------------------------------------------------------

#include <gmock/gmock.h>

#include <memory>
#include <string>

#include "hardware_interface/resource_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_control_test_assets/components_urdfs.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

namespace
{
// The single-argument ResourceManager(urdf) constructor used in older ros2_control no longer
// exists - the installed hardware_interface requires an explicit clock and logger.
std::unique_ptr<hardware_interface::ResourceManager> makeResourceManager(const std::string & urdf)
{
  return std::make_unique<hardware_interface::ResourceManager>(
    urdf, std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME),
    rclcpp::get_logger("test_canopen_system"));
}
}  // namespace

class TestCanopenSystem : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // TODO(anyone): Extend this description to your robot
    canopen_system_2dof_ =
      R"(
        <ros2_control name="CanopenSystem2dof" type="system">
          <hardware>
            <plugin>canopen_ros2_control/CanopenSystem</plugin>
          </hardware>
          <joint name="joint1">
            <command_interface name="position"/>
            <state_interface name="position"/>
            <param name="initial_position">1.57</param>
          </joint>
          <joint name="joint2">
            <command_interface name="position"/>
            <state_interface name="position"/>
            <param name="initial_position">0.7854</param>
          </joint>
        </ros2_control>
    )";

    // No joints - keeps this focused on gpio export and avoids having to match
    // CanopenSystem's per-joint rpdo/tpdo/nmt interface names for an unrelated fixture.
    canopen_system_pdo_gate_only_ =
      R"(
        <ros2_control name="CanopenSystemPdoGate" type="system">
          <hardware>
            <plugin>canopen_ros2_control/CanopenSystem</plugin>
          </hardware>
          <gpio name="canopen_pdo_gate">
            <command_interface name="enable"/>
          </gpio>
        </ros2_control>
    )";
  }

  std::string canopen_system_2dof_;
  std::string canopen_system_pdo_gate_only_;
};

TEST_F(TestCanopenSystem, load_canopen_system_2dof)
{
  auto urdf = ros2_control_test_assets::urdf_head + canopen_system_2dof_ +
              ros2_control_test_assets::urdf_tail;
  ASSERT_NO_THROW(makeResourceManager(urdf));
}

// Without the 'canopen_pdo_gate' gpio declared, no gate command interface is exported -
// this is the pre-existing behavior and must not regress for hardware that omits the gpio.
TEST_F(TestCanopenSystem, pdo_gate_not_exported_without_gpio)
{
  auto urdf = ros2_control_test_assets::urdf_head + canopen_system_2dof_ +
              ros2_control_test_assets::urdf_tail;
  auto rm = makeResourceManager(urdf);
  EXPECT_FALSE(rm->command_interface_exists("canopen_pdo_gate/enable"));
}

// With the gpio declared, the command interface must be exported so a controller can claim it.
TEST_F(TestCanopenSystem, pdo_gate_exported_with_gpio)
{
  auto urdf = ros2_control_test_assets::urdf_head + canopen_system_pdo_gate_only_ +
              ros2_control_test_assets::urdf_tail;
  auto rm = makeResourceManager(urdf);
  EXPECT_TRUE(rm->command_interface_exists("canopen_pdo_gate/enable"));
}
