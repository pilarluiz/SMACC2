#pragma once

#include <optional>
#include <smacc2/smacc_client.hpp>
#include <smacc2/smacc_signal.hpp>
#include "rclcpp_action/rclcpp_action.hpp"

namespace smacc2
{
namespace client_bases
{
template <typename TAction>
class SmaccActionServerClient : public smacc2::ISmaccClient
{
  using GoalHandleTAction = rclcpp_action::ServerGoalHandle<TAction>;

public:
  std::optional<std::string> actionName_;
  SmaccActionServerClient() { initialized_ = false; }
  SmaccActionServerClient(std::string action_name)
  {
    actionName_ = action_name;
    initialized_ = false;
  }

  virtual ~SmaccActionServerClient() {}

  smacc2::SmaccSignal<rclcpp_action::GoalResponse(
    const rclcpp_action::GoalUUID,
    std::shared_ptr<const typename TAction::Goal>)>
    onHandleGoal_;

  smacc2::SmaccSignal<rclcpp_action::CancelResponse(
    const typename std::shared_ptr<GoalHandleTAction>)>
    onHandleCancel_;

  smacc2::SmaccSignal<void(
    const typename std::shared_ptr<GoalHandleTAction>)>
    onHandleAccepted_;

  template <typename T>
  boost::signals2::connection onHandleGoal(
    rclcpp_action::GoalResponse (T::*callback)(
      const rclcpp_action::GoalUUID,
      std::shared_ptr<const typename TAction::Goal>),
    T* object)
  {
    return this->getStateMachine()->createSignalConnection(
      onHandleGoal_, callback, object);
  }

  template <typename T>
  boost::signals2::connection onHandleCancel(
    rclcpp_action::CancelResponse (T::*callback)(
      const typename std::shared_ptr<GoalHandleTAction>),
    T* object)
  {
    return this->getStateMachine()->createSignalConnection(
      onHandleCancel_, callback, object);
  }

  template <typename T>
  boost::signals2::connection onHandleAccepted(
    void (T::*callback)(
      const typename std::shared_ptr<GoalHandleTAction>),
    T* object)
  {
    return this->getStateMachine()->createSignalConnection(
      onHandleAccepted_, callback, object);
  }

  void onInitialize() override
  {
    if (!initialized_)
    {
      if (!actionName_)
      {
        RCLCPP_ERROR_STREAM(
          getLogger(),
          "[" << this->getName() << "] action server with no action name set. Skipping.");
      }
      else
      {
        RCLCPP_INFO_STREAM(
          getLogger(), "[" << this->getName() << "] Client Action: " << *actionName_);

        server_ = rclcpp_action::create_server<TAction>(
          getNode(),
          *actionName_,
          std::bind(&SmaccActionServerClient::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
          std::bind(&SmaccActionServerClient::handleCancel, this, std::placeholders::_1),
          std::bind(&SmaccActionServerClient::handleAccepted, this, std::placeholders::_1)
        );

        this->initialized_ = true;
      }
    }
  }

private:
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const typename TAction::Goal> goal)
  {
    return *(onHandleGoal_(uuid, goal));
  }

  rclcpp_action::CancelResponse handleCancel(
    const typename std::shared_ptr<GoalHandleTAction> goal_handle)
  {
    return *(onHandleCancel_(goal_handle));
  }

  void handleAccepted(const typename std::shared_ptr<GoalHandleTAction> goal_handle)
  {
    onHandleAccepted_(goal_handle);
  }

  typename rclcpp_action::Server<TAction>::SharedPtr server_;
  bool initialized_;
};
}  // namespace client_bases
}  // namespace smacc2
