#include <iostream>
#include <lipp.h>

#include "omp.h"

using namespace std;

int main() {
  lipp_prob::LIPP<int, int> lipp;

  int key_num = 99;
   pair<int, int> *keys = new pair<int, int>[100];
  //omp_set_num_threads(4);

// #pragma omp parallel for schedule(static, 1)
  for (int i = 0; i < key_num; i++) {
     keys[i] = {i, i};
    // printf("Thread %d insert(%d)\n", omp_get_thread_num(), i);
    //std::cout<<i<<std::endl;
    //lipp.insert(i, i);
    // mix write with read
    // if (i>5)
    // printf("Thread %d, read %d\n", omp_get_thread_num(), lipp.at(i-5,
    // false));
  }
  keys[99]={1000000000,1000000000};
  // printf("bulk loading\n");
   lipp.bulk_load(keys, 100);
  std::pair <int, int> *result = new std::pair <int, int>[200];
  int ret=lipp.range_query_len(result, 457, 5);
  for(int i=0;i<ret;i++){
    std::cout<<"scan "<<result[i].first<<" "<<result[i].second<<std::endl;
  }

  printf("start\n");

//#pragma omp parallel for schedule(static, 1)
  for (int i = 0; i < key_num; i++) {
    int val;
    lipp.at(i,val);
    if (val != i)
      printf("wrong payload at %d\n", i);
  }

  cout << "exists(1) = " << (lipp.exists(1) ? "true" : "false") << endl;
  cout << "exists(100) = " << (lipp.exists(100) ? "true" : "false") << endl;
  cout << "exists(4000) = " << (lipp.exists(4000) ? "true" : "false") << endl;

  // show tree structure
  // lipp.show();

  return 0;
}
