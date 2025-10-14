#include <stdio.h>

#define COMP_X   0
#define COMP_Y   1
#define COMP_Z   2
#define COMP_PSI 3
#define COMP_N   4 
struct TrajStep {
  float t;
  float Y0[COMP_N];
  float Y1[COMP_N];
  float Y2[COMP_N];
};

void read_traj(char* filename) {
  FILE * f;
  f = fopen(filename, "rb+");
  struct TrajStep foo;
  while (fread(&foo, sizeof(foo), 1, f)) {
    printf("t:%f x:%f %f %f\n", foo.t, foo.Y0[COMP_X], foo.Y1[COMP_X], foo.Y2[COMP_X]);
  }
  fclose (f);
}

int main(int argc, char** argv) {
  read_traj("outputs/001_circle_with_intro_slow.bin");
  return 0;
}
