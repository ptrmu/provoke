#include <cstdio>

#include "provoke/provoke_node.hpp"

#include "provoke_node_impl.hpp"

#include "sm_manager.hpp"

namespace provoke
{
  ProvokeNode::ProvokeNode()
    : Node("provoke_node"), impl_(std::make_unique<ProvokeNodeImpl>(*this))
  {
  }

  ProvokeNode::~ProvokeNode()
  {}
}

int main(int argc, char **argv)
{
  (void) argc;
  (void) argv;

  auto sm_manager = std::make_unique<provoke::ProvokeNode>();

  printf("hello world provoke package\n");
  return 0;
}
