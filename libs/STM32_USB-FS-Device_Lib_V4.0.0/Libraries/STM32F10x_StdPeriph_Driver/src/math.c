#include "math.h"
double sqrt(double x)
{
 register double ret;
 __asm__(
  "fsqrt"
  : "=t" (ret)
  : "0" (x)
  );

 return ret;
}//计算x的平方根


double atan2 (double x, double y)
{
 register double ret;
 __asm__(
  "fpatan\n\t"
  "fld %%st(0)"
  : "=t" (ret)
  : "0" (y), "u" (x)
  );

 return ret;
}//求x / y的反正切值。

double asin(double x)
{
 return atan2 (x, sqrt (1.0 - x * x));
}//求x的反正弦值。
