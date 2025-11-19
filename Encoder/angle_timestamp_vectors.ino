#include <vector>
#include <sys/time.h>
#include <iostream>

std::vector<int> angles;
std::vector<int> times;

int64_t get_time() {
    struct timeval tod;
    gettimeofday(&tod, NULL);
    int64_t t = (int64_t)tod.tv_sec * 1000LL + (int64_t)tod.tv_usec / 1000LL;//convert timeval to ms
    return t;
}

void removeN(std::vector<int>& angles, std::vector<int>& times, int n){
  angles.erase(angles.begin(), angles.begin() + n);
  times.erase(times.begin(), times.begin() + n);
}

int get_angle(){
  return 0;
}
void setup() {

}

void loop() {
  int cA = get_angle();
  int cT = get_time();
  angles.push_back(cA);
  times.push_back(cT);
  std::cout << "angles: " << cA <<"\n" << "times: " << cT;
}
