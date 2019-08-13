#include "../user/ib.h"

int main() {
  char *buffer;
  unsigned lpw = 9000;
  unsigned lgw = 4500;
  unsigned opw = 560;
  unsigned ogw = 1680;
  unsigned zpw = 560;
  unsigned zgw = 560;
  unsigned utp = 560;
  unsigned f = 38000;
  unsigned dc_n = 50;
  unsigned dc_m = 100;
  char *binary = "11001101001100100000111111110000";

  sendIR(lpw, lgw, opw, ogw, zpw, zgw, utp, f, dc_n, dc_m, binary);
  return 0;
}