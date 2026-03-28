#ifndef TOOLS__EXITER_HPP
#define TOOLS__EXITER_HPP

namespace tools
{
class Exiter
{
public:
  Exiter();

  bool exit() const;
  void set_exit();
};

}  // namespace tools

#endif  // TOOLS__EXITER_HPP