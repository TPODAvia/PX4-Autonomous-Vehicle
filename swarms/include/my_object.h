#ifndef MY_OBJECT_H
#define MY_OBJECT_H

#include <string>

class MyObject
{
public:
  MyObject(const std::string& name, double x, double y, double theta);

  std::string getName() const;
  double getX() const;
  double getY() const;
  double getTheta() const;

private:
  std::string name_;
  double x_;
  double y_;
  double theta_;
};

#endif // MY_OBJECT_H