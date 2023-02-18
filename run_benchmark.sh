#!/bin/bash

#trap ctrl_c INT
#
#function ctrl_c() {
#    echo "Trapped CTRL_C"
#    kill -KILL $PID
#}

cd ./build

#rm -r result

mkdir final

echo "covid begin"
printf "covid begin\n">>process.txt

array=(1 4 8 16 24 36 48)
index=("lipp_prob" "lippol" "xindex")
data=(covid osm libio genome)

./microbench --keys_file=../data/datasets/covid --keys_file_type=binary --read=1 --insert=0 --operations_num=200000000 --table_size=-1 --init_table_ratio=1 --output_path=./final/covidro.txt --thread_num=1,4,8,16,24,36,48 --memory=1 --index=alexol,artolc,masstree,finedex
for(( j=0;j<${#index[@]};j++))
do
  for(( i=0;i<${#array[@]};i++))
  do
    ./microbench --keys_file=../data/datasets/covid --keys_file_type=binary --read=1 --insert=0 --operations_num=200000000 --table_size=-1 --init_table_ratio=1 --output_path=./final/covidro.txt --thread_num="${array[i]}" --memory=1 --index="${index[j]}"
  done
done


./microbench --keys_file=../data/datasets/covid --keys_file_type=binary --read=0.8 --insert=0.2 --operations_num=200000000 --table_size=-1 --init_table_ratio=0.5 --output_path=./final/covid8020.txt --thread_num=1,4,8,16,24,36,48 --memory=1 --index=alexol,artolc,masstree,finedex
for(( j=0;j<${#index[@]};j++))
do
  for(( i=0;i<${#array[@]};i++))
  do
    ./microbench --keys_file=../data/datasets/covid --keys_file_type=binary --read=0.8 --insert=0.2 --operations_num=200000000 --table_size=-1 --init_table_ratio=0.5 --output_path=./final/covid8020.txt --thread_num="${array[i]}" --memory=1 --index="${index[j]}"
  done
done


./microbench --keys_file=../data/datasets/covid --keys_file_type=binary --read=0.5 --insert=0.5 --operations_num=200000000 --table_size=-1 --init_table_ratio=0.5 --output_path=./final/covid5050.txt --thread_num=1,4,8,16,24,36,48  --memory=1 --index=alexol,artolc,masstree,finedex
for(( j=0;j<${#index[@]};j++))
do
  for(( i=0;i<${#array[@]};i++))
  do
    ./microbench --keys_file=../data/datasets/covid --keys_file_type=binary --read=0.5 --insert=0.5 --operations_num=200000000 --table_size=-1 --init_table_ratio=0.5 --output_path=./final/covid5050.txt --thread_num="${array[i]}"  --memory=1 --index="${index[j]}"
  done
done

./microbench --keys_file=../data/datasets/covid --keys_file_type=binary --read=0 --insert=1 --operations_num=200000000 --table_size=-1 --init_table_ratio=0.5 --output_path=./final/covidwo.txt --thread_num=1,4,8,16,24,36,48  --memory=1 --index=alexol,artolc,masstree,finedex
for(( j=0;j<${#index[@]};j++))
do
  for(( i=0;i<${#array[@]};i++))
  do
    ./microbench --keys_file=../data/datasets/covid --keys_file_type=binary --read=0 --insert=1 --operations_num=200000000 --table_size=-1 --init_table_ratio=0.5 --output_path=./final/covidwo.txt --thread_num="${array[i]}" --memory=1 --index="${index[j]}"
  done
done

echo "covid end"
printf "covid end\n">>process.txt

echo "genome begin"
printf "genome begin\n">>process.txt

./microbench --keys_file=../data/datasets/genome --keys_file_type=binary --read=1 --insert=0 --operations_num=200000000 --table_size=-1 --init_table_ratio=1 --output_path=./final/genomero.txt --thread_num=1,4,8,16,24,36,48 --memory=1 --index=alexol,artolc,masstree,finedex
for(( j=0;j<${#index[@]};j++))
do
  for(( i=0;i<${#array[@]};i++))
  do
    ./microbench --keys_file=../data/datasets/genome --keys_file_type=binary --read=1 --insert=0 --operations_num=200000000 --table_size=-1 --init_table_ratio=1 --output_path=./final/genomero.txt --thread_num="${array[i]}" --memory=1 --index="${index[j]}"
  done
done


./microbench --keys_file=../data/datasets/genome --keys_file_type=binary --read=0.8 --insert=0.2 --operations_num=200000000 --table_size=-1 --init_table_ratio=0.5 --output_path=./final/genome8020.txt --thread_num=1,4,8,16,24,36,48 --memory=1 --index=alexol,artolc,masstree,finedex
for(( j=0;j<${#index[@]};j++))
do
  for(( i=0;i<${#array[@]};i++))
  do
    ./microbench --keys_file=../data/datasets/genome --keys_file_type=binary --read=1 --insert=0 --operations_num=200000000 --table_size=-1 --init_table_ratio=1 --output_path=./final/genomero.txt --thread_num="${array[i]}" --memory=1 --index="${index[j]}"
  done
done


./microbench --keys_file=../data/datasets/genome --keys_file_type=binary --read=0.5 --insert=0.5 --operations_num=200000000 --table_size=-1 --init_table_ratio=0.5 --output_path=./final/genome5050.txt --thread_num=1,4,8,16,24,36,48  --memory=1 --index=alexol,artolc,masstree,finedex
for(( i=0;i<${#array[@]};i++))
do
  ./microbench --keys_file=../data/datasets/genome --keys_file_type=binary --read=0.5 --insert=0.5 --operations_num=200000000 --table_size=-1 --init_table_ratio=0.5 --output_path=./final/genome5050.txt --thread_num="${array[i]}"  --memory=1 --index=lipp_prob
done
for(( i=0;i<${#array[@]};i++))
do
  ./microbench --keys_file=../data/datasets/genome --keys_file_type=binary --read=0.5 --insert=0.5 --operations_num=200000000 --table_size=-1 --init_table_ratio=0.5 --output_path=./final/genome5050.txt --thread_num="${array[i]}"  --memory=1 --index=lippol
done


./microbench --keys_file=../data/datasets/genome --keys_file_type=binary --read=0 --insert=1 --operations_num=200000000 --table_size=-1 --init_table_ratio=0.5 --output_path=./final/genomewo.txt --thread_num=1,4,8,16,24,36,48  --memory=1 --index=alexol,artolc,masstree,finedex
for(( i=0;i<${#array[@]};i++))
do
  ./microbench --keys_file=../data/datasets/genome --keys_file_type=binary --read=0 --insert=1 --operations_num=200000000 --table_size=-1 --init_table_ratio=0.5 --output_path=./final/genomewo.txt --thread_num="${array[i]}"  --memory=1 --index=lipp_prob
done
for(( i=0;i<${#array[@]};i++))
do
  ./microbench --keys_file=../data/datasets/genome --keys_file_type=binary --read=0 --insert=1 --operations_num=200000000 --table_size=-1 --init_table_ratio=0.5 --output_path=./final/genomewo.txt --thread_num="${array[i]}"  --memory=1 --index=lippol
done


echo "genome end"
printf "genome end\n">>process.txt

echo "libio begin"
printf "libio begin\n">>process.txt

./microbench --keys_file=../data/datasets/libio --keys_file_type=binary --read=1 --insert=0 --operations_num=200000000 --table_size=-1 --init_table_ratio=1 --output_path=./final/libioro.txt --thread_num=1,4,8,16,24,36,48 --memory=1 --index=alexol,artolc,masstree,finedex
for(( i=0;i<${#array[@]};i++))
do
  ./microbench --keys_file=../data/datasets/libio --keys_file_type=binary --read=1 --insert=0 --operations_num=200000000 --table_size=-1 --init_table_ratio=1 --output_path=./final/libioro.txt --thread_num="${array[i]}" --memory=1 --index=lipp_prob
done
for(( i=0;i<${#array[@]};i++))
do
  ./microbench --keys_file=../data/datasets/libio --keys_file_type=binary --read=1 --insert=0 --operations_num=200000000 --table_size=-1 --init_table_ratio=1 --output_path=./final/libioro.txt --thread_num="${array[i]}" --memory=1 --index=lippol
done


./microbench --keys_file=../data/datasets/libio --keys_file_type=binary --read=0.8 --insert=0.2 --operations_num=200000000 --table_size=-1 --init_table_ratio=0.5 --output_path=./final/libio8020.txt --thread_num=1,4,8,16,24,36,48  --memory=1 --index=alexol,artolc,masstree,finedex
for(( i=0;i<${#array[@]};i++))
do
  ./microbench --keys_file=../data/datasets/libio --keys_file_type=binary --read=0.8 --insert=0.2 --operations_num=200000000 --table_size=-1 --init_table_ratio=0.5 --output_path=./final/libio8020.txt --thread_num="${array[i]}"  --memory=1 --index=lipp_prob
done
for(( i=0;i<${#array[@]};i++))
do
  ./microbench --keys_file=../data/datasets/libio --keys_file_type=binary --read=0.8 --insert=0.2 --operations_num=200000000 --table_size=-1 --init_table_ratio=0.5 --output_path=./final/libio8020.txt --thread_num="${array[i]}"  --memory=1 --index=lippol
done


./microbench --keys_file=../data/datasets/libio --keys_file_type=binary --read=0.5 --insert=0.5 --operations_num=200000000 --table_size=-1 --init_table_ratio=0.5 --output_path=./final/libio5050.txt --thread_num=1,4,8,16,24,36,48  --memory=1 --index=alexol,artolc,masstree,finedex
for(( i=0;i<${#array[@]};i++))
do
  ./microbench --keys_file=../data/datasets/libio --keys_file_type=binary --read=0.5 --insert=0.5 --operations_num=200000000 --table_size=-1 --init_table_ratio=0.5 --output_path=./final/libio5050.txt --thread_num="${array[i]}"  --memory=1 --index=lipp_prob
done
for(( i=0;i<${#array[@]};i++))
do
  ./microbench --keys_file=../data/datasets/libio --keys_file_type=binary --read=0.5 --insert=0.5 --operations_num=200000000 --table_size=-1 --init_table_ratio=0.5 --output_path=./final/libio5050.txt --thread_num="${array[i]}"  --memory=1 --index=lippol
done


./microbench --keys_file=../data/datasets/libio --keys_file_type=binary --read=0 --insert=1 --operations_num=200000000 --table_size=-1 --init_table_ratio=0.5 --output_path=./final/libiowo.txt --thread_num=1,4,8,16,24,36,48  --memory=1 --index=alexol,artolc,masstree,finedex
for(( i=0;i<${#array[@]};i++))
do
  ./microbench --keys_file=../data/datasets/libio --keys_file_type=binary --read=0 --insert=1 --operations_num=200000000 --table_size=-1 --init_table_ratio=0.5 --output_path=./final/libiowo.txt --thread_num="${array[i]}"  --memory=1 --index=lipp_prob
done
for(( i=0;i<${#array[@]};i++))
do
  ./microbench --keys_file=../data/datasets/libio --keys_file_type=binary --read=0 --insert=1 --operations_num=200000000 --table_size=-1 --init_table_ratio=0.5 --output_path=./final/libiowo.txt --thread_num="${array[i]}"  --memory=1 --index=lippol
done


echo "libio end"
printf "libio end\n">>process.txt

echo "osm begin"
printf "osm begin\n">>process.txt

./microbench --keys_file=../data/datasets/osm --keys_file_type=binary --read=1 --insert=0 --operations_num=200000000 --table_size=-1 --init_table_ratio=1 --output_path=./final/osmro.txt --thread_num=1,4,8,16,24,36,48 --memory=1 --index=alexol,artolc,masstree,finedex
for(( i=0;i<${#array[@]};i++))
do
  ./microbench --keys_file=../data/datasets/osm --keys_file_type=binary --read=1 --insert=0 --operations_num=200000000 --table_size=-1 --init_table_ratio=1 --output_path=./final/osmro.txt --thread_num="${array[i]}" --memory=1 --index=lipp_prob
done
for(( i=0;i<${#array[@]};i++))
do
  ./microbench --keys_file=../data/datasets/osm --keys_file_type=binary --read=1 --insert=0 --operations_num=200000000 --table_size=-1 --init_table_ratio=1 --output_path=./final/osmro.txt --thread_num="${array[i]}" --memory=1 --index=lippol
done


./microbench --keys_file=../data/datasets/osm --keys_file_type=binary --read=0.8 --insert=0.2 --operations_num=200000000 --table_size=-1 --init_table_ratio=0.5 --output_path=./final/osm8020.txt --thread_num=1,4,8,16,24,36,48 --memory=1 --index=alexol,artolc,masstree,finedex
for(( i=0;i<${#array[@]};i++))
do
  ./microbench --keys_file=../data/datasets/osm --keys_file_type=binary --read=0.8 --insert=0.2 --operations_num=200000000 --table_size=-1 --init_table_ratio=0.5 --output_path=./final/osm8020.txt --thread_num="${array[i]}" --memory=1 --index=lipp_prob
done
for(( i=0;i<${#array[@]};i++))
do
  ./microbench --keys_file=../data/datasets/osm --keys_file_type=binary --read=0.8 --insert=0.2 --operations_num=200000000 --table_size=-1 --init_table_ratio=0.5 --output_path=./final/osm8020.txt --thread_num="${array[i]}" --memory=1 --index=lippol
done


./microbench --keys_file=../data/datasets/osm --keys_file_type=binary --read=0.5 --insert=0.5 --operations_num=200000000 --table_size=-1 --init_table_ratio=0.5 --output_path=./final/osm5050.txt --thread_num=1,4,8,16,24,36,48  --memory=1 --index=alexol,artolc,masstree,finedex
for(( i=0;i<${#array[@]};i++))
do
  ./microbench --keys_file=../data/datasets/osm --keys_file_type=binary --read=0.5 --insert=0.5 --operations_num=200000000 --table_size=-1 --init_table_ratio=0.5 --output_path=./final/osm5050.txt --thread_num="${array[i]}"  --memory=1 --index=lipp_prob
done
for(( i=0;i<${#array[@]};i++))
do
  ./microbench --keys_file=../data/datasets/osm --keys_file_type=binary --read=0.5 --insert=0.5 --operations_num=200000000 --table_size=-1 --init_table_ratio=0.5 --output_path=./final/osm5050.txt --thread_num="${array[i]}"  --memory=1 --index=lippol
done


./microbench --keys_file=../data/datasets/osm --keys_file_type=binary --read=0 --insert=1 --operations_num=200000000 --table_size=-1 --init_table_ratio=0.5 --output_path=./final/osmwo.txt --thread_num=1,4,8,16,24,36,48  --memory=1 --index=alexol,artolc,masstree,finedex
for(( i=0;i<${#array[@]};i++))
do
  ./microbench --keys_file=../data/datasets/osm --keys_file_type=binary --read=0 --insert=1 --operations_num=200000000 --table_size=-1 --init_table_ratio=0.5 --output_path=./final/osmwo.txt --thread_num="${array[i]}"   --memory=1 --index=lipp_prob
done
for(( i=0;i<${#array[@]};i++))
do
  ./microbench --keys_file=../data/datasets/osm --keys_file_type=binary --read=0 --insert=1 --operations_num=200000000 --table_size=-1 --init_table_ratio=0.5 --output_path=./final/osmwo.txt --thread_num="${array[i]}"   --memory=1 --index=lippol
done


echo "osm end"
printf "osm end\n">>process.txt