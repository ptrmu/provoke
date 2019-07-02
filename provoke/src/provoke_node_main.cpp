
#include <cstdio>

#include "provoke/provoke_node.hpp"


int main(int argc, char **argv)
{
  (void) argc;
  (void) argv;

  auto provoke_node = provoke::node_factory();

  printf("hello world provoke package\n");
  return 0;
}

