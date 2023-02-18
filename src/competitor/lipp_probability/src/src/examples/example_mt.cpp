#include <iostream>
#include <lipp.h>

#include "omp.h"

using namespace std;

int main() {
  lipp_prob::LIPP<int, int> lipp;

  int key_num = 10;
   pair<int, int> *keys = new pair<int, int>[key_num];
  omp_set_num_threads(1);

// #pragma omp parallel for schedule(static, 1)
  /*for (int i = 0; i < key_num; i++) {
     keys[i] = {i, i};
  }*/
  // printf("bulk loading\n");
  keys[0]={1,1};
  keys[1]={10,10};
  keys[2]={100,100};
  keys[3]={1000,1000};
  keys[4]={10000,10000};
  keys[5]={10000,10000};
   lipp.bulk_load(keys, 5);
  /*std::pair <int, int> *result = new std::pair <int, int>[200];
  int ret=lipp.range_query_len(result, 457, 5);
  for(int i=0;i<ret;i++){
    std::cout<<"scan "<<result[i].first<<" "<<result[i].second<<std::endl;
  }*/

  printf("start\n");

#pragma omp parallel for schedule(static, 1)
  for (int i = 0; i < 1; i++) {
    lipp.insert(99,99);
  }
  lipp.show();
#pragma omp parallel for schedule(static, 1)
  for (int i = 0; i < 1; i++) {
    lipp.insert(0,0);
  }
  lipp.show();
#pragma omp parallel for schedule(static, 1)
  for (int i = 0; i < 1; i++) {
    lipp.insert(9999999,9999999);
  }
#pragma omp parallel for schedule(static, 1)
  for (int i = 0; i < 1; i++) {
    lipp.insert(88,88);
  }
  lipp.show();
  //insert(0,0);
  //insert(9999999,9999999);


/*  cout << "exists(1) = " << (lipp.exists(1) ? "true" : "false") << endl;
  cout << "exists(100) = " << (lipp.exists(100) ? "true" : "false") << endl;
  cout << "exists(4000) = " << (lipp.exists(4000) ? "true" : "false") << endl;*/

  // show tree structure
  // lipp.show();

  return 0;
}
