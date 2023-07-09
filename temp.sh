#!/bin/bash
cd ./build
thread_num=(1 4 8)
#thread_num=(1 4 8 16 24 36 48 60 70)
#index=(masstree artolc alexol lipp_prob lippol finedex)
index=(lipp_prob)
#data=(covid osm libio fb)
data=(osm)

for(( x=0;x<${#data[@]};x++))
do
  for(( y=0;y<${#index[@]};y++))
  do
    for(( i=0;i<${#thread_num[@]};i++))
    do
      #./microbench --keys_file=../data/datasets/datasets/"${data[x]}" --keys_file_type=binary --read=0 --insert=1 --operations_num=200000000 --table_size=-1 --init_table_ratio=0.5 --output_path=./"${data[x]}"wo.txt --thread_num="${thread_num[i]}" --memory=1 --index="${index[y]}"
      ./microbench --keys_file=../data/datasets/"${data[x]}" --keys_file_type=binary --read=1 --insert=0 --operations_num=200000000 --table_size=-1 --init_table_ratio=1 --output_path=./out.txt --thread_num="${thread_num[i]}" --memory=1 --index="${index[y]}"

      #./microbench --keys_file=../data/datasets/"${data[x]}" --keys_file_type=binary --read=0 --insert=0 --scan=1 --operations_num=200000000 --table_size=-1 --init_table_ratio=1 --output_path=./scan/"${data[x]}".txt --thread_num="${thread_num[i]}" --memory=1 --index="${index[y]}"
      #./microbench --keys_file=../data/"${data[x]}" --keys_file_type=binary --read=0 --insert=1 --operations_num=200000000 --table_size=-1 --init_table_ratio=0.5 --output_path=./alexol.txt --thread_num="${thread_num[i]}" --memory=1 --index="${index[y]}"

#      ./microbench --keys_file=../data/datasets/"${data[x]}" --keys_file_type=binary --read=1 --insert=0 --operations_num=200000000 --table_size=-1 --init_table_ratio=1 --output_path=./alex_new/"${data[x]}"ro.txt --thread_num="${thread_num[i]}" --memory=1 --index="${index[y]}"
#      rm core.*
#      ./microbench --keys_file=../data/datasets/"${data[x]}" --keys_file_type=binary --read=0.8 --insert=0.2 --operations_num=200000000 --table_size=-1 --init_table_ratio=0.5 --output_path=./alex_new/"${data[x]}"8020.txt --thread_num="${thread_num[i]}" --memory=1 --index="${index[y]}"
#      rm core.*
#      ./microbench --keys_file=../data/datasets/"${data[x]}" --keys_file_type=binary --read=0.5 --insert=0.5 --operations_num=200000000 --table_size=-1 --init_table_ratio=0.5 --output_path=./alex_new/"${data[x]}"5050.txt --thread_num="${thread_num[i]}" --memory=1 --index="${index[y]}"
#      rm core.*
#      ./microbench --keys_file=../data/datasets/"${data[x]}" --keys_file_type=binary --read=0 --insert=1 --operations_num=200000000 --table_size=-1 --init_table_ratio=0.5 --output_path=./alex_new/"${data[x]}"wo.txt --thread_num="${thread_num[i]}" --memory=1 --index="${index[y]}"
#      rm core.*
    done
  done
done