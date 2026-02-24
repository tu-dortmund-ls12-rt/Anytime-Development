// Copyright 2025 Anytime System
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "anytime_core/anytime_waitable.hpp"

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <functional>
#include <memory>

class RclcppEnvironment : public ::testing::Environment
{
public:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

testing::Environment * const rclcpp_env =
  testing::AddGlobalTestEnvironment(new RclcppEnvironment);

TEST(AnytimeWaitable, ConstructWithCallback)
{
  int call_count = 0;
  auto waitable = std::make_shared<anytime_core::AnytimeWaitable>(
    [&call_count]() { call_count++; });

  // execute() should invoke the callback
  std::shared_ptr<void> data = nullptr;
  waitable->execute(data);
  EXPECT_EQ(call_count, 1);

  waitable->execute(data);
  EXPECT_EQ(call_count, 2);
}

TEST(AnytimeWaitable, ConstructWithoutCallback)
{
  auto waitable = std::make_shared<anytime_core::AnytimeWaitable>();

  // execute() should not crash with no callback
  std::shared_ptr<void> data = nullptr;
  EXPECT_NO_THROW(waitable->execute(data));
}

TEST(AnytimeWaitable, TriggerDoesNotCrash)
{
  auto waitable = std::make_shared<anytime_core::AnytimeWaitable>();

  // trigger() fires the guard condition — should not crash
  EXPECT_NO_THROW(waitable->trigger());
  EXPECT_NO_THROW(waitable->trigger());
}

TEST(AnytimeWaitable, GetNumberOfReadyGuardConditions)
{
  auto waitable = std::make_shared<anytime_core::AnytimeWaitable>();

  // Should always return 1 (one guard condition exists)
  EXPECT_EQ(waitable->get_number_of_ready_guard_conditions(), 1u);
}

TEST(AnytimeWaitable, SetAndClearOnReadyCallback)
{
  auto waitable = std::make_shared<anytime_core::AnytimeWaitable>();

  // Setting and clearing on_ready_callback should not crash
  EXPECT_NO_THROW(
    waitable->set_on_ready_callback([](size_t, int) {}));
  EXPECT_NO_THROW(waitable->clear_on_ready_callback());
}

TEST(AnytimeWaitable, TakeDataReturnsNull)
{
  auto waitable = std::make_shared<anytime_core::AnytimeWaitable>();

  // take_data() should return nullptr
  EXPECT_EQ(waitable->take_data(), nullptr);
}

TEST(AnytimeWaitable, TakeDataByEntityIdReturnsNull)
{
  auto waitable = std::make_shared<anytime_core::AnytimeWaitable>();

  // take_data_by_entity_id() should return nullptr
  EXPECT_EQ(waitable->take_data_by_entity_id(0), nullptr);
  EXPECT_EQ(waitable->take_data_by_entity_id(42), nullptr);
}
