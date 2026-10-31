#include <iostream>
#include <type_traits>
#include <nav2_core/controller.hpp>
#include <nav2_core/goal_checker.hpp>
#include <nav2_msgs/action/follow_path.hpp>
template<class> struct Args;
template<class R, class C, class... A> struct Args<R(C::*)(A...)> {
  static constexpr size_t count = sizeof...(A);
};
template<class T, class = void> struct HasNewPath : std::false_type {};
template<class T> struct HasNewPath<T, std::void_t<decltype(&T::newPathReceived)>> : std::true_type {};
template<class T, class = void> struct HasHandlerId : std::false_type {};
template<class T> struct HasHandlerId<T, std::void_t<decltype(std::declval<T>().path_handler_id)>> : std::true_type {};
template<class T, class = void> struct HasXY : std::false_type {};
template<class T> struct HasXY<T, std::void_t<decltype(&T::isGoalXYReached)>> : std::true_type {};
int main() {
  std::cout << "{\"controller_compute_arguments\":"
    << Args<decltype(&nav2_core::Controller::computeVelocityCommands)>::count
    << ",\"goal_checker_arguments\":"
    << Args<decltype(&nav2_core::GoalChecker::isGoalReached)>::count
    << ",\"goal_tolerance_arguments\":"
    << Args<decltype(&nav2_core::GoalChecker::getTolerances)>::count
    << ",\"xy_reached_method\":" << HasXY<nav2_core::GoalChecker>::value
    << ",\"new_path_received\":" << HasNewPath<nav2_core::Controller>::value
    << ",\"installed_followpath_has_handler_id\":"
    << HasHandlerId<nav2_msgs::action::FollowPath::Goal>::value << "}\n";
}
