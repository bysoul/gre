#!/bin/bash
cd ./build
array=(covid osm libio genome wise book fb)

for(( i=0;i<${#array[@]};i++))
do
  ./microbench --keys_file=../data/datasets/"${array[i]}" --keys_file_type=binary --read=0 --insert=1 --operations_num=200000000 --table_size=-1 --init_table_ratio=0.5 --output_path=./t.txt --thread_num=48 --memory=1 --index=lipp_prob
  ./microbench --keys_file=../data/datasets/"${array[i]}" --keys_file_type=binary --read=0 --insert=1 --operations_num=200000000 --table_size=-1 --init_table_ratio=0.5 --output_path=./t.txt --thread_num=48 --memory=1 --index=lipp_prob_t
done