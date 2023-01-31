#!/bin/bash

trap ctrl_c INT

function ctrl_c() {
    echo "Trapped CTRL_C"
    kill -KILL $PID
}

cd ./build

#rm -r result
rm process.txt
#mkdir result
touch process.txt

echo "covid begin"
printf "covid begin\n">>process.txt

./microbench --keys_file=../data/datasets/covid --keys_file_type=binary --read=0.8 --insert=0.2 --operations_num=200000000 --table_size=-1 --init_table_ratio=0.5 --output_path=./result/covid8020.txt --thread_num=1,2,4,8,16,24,36,48 --memory=1 --index=lippol,lipp_prob,alexol,artolc &
PID=$!
echo "PID=" $PID
wait $PID

./microbench --keys_file=../data/datasets/covid --keys_file_type=binary --read=0.5 --insert=0.5 --operations_num=200000000 --table_size=-1 --init_table_ratio=0.5 --output_path=./result/covid5050.txt --thread_num=1,2,4,8,16,24,36,48  --memory=1 --index=lippol,lipp_prob,alexol,artolc &
PID=$!
echo "PID=" $PID
wait $PID

./microbench --keys_file=../data/datasets/covid --keys_file_type=binary --read=0 --insert=1 --operations_num=200000000 --table_size=-1 --init_table_ratio=0.5 --output_path=./result/covidwo.txt --thread_num=1,2,4,8,16,24,36,48  --memory=1 --index=lippol,lipp_prob,alexol,artolc &
PID=$!
echo "PID=" $PID
wait $PID

echo "covid end"
printf "covid end\n">>process.txt

echo "genome begin"
printf "genome begin\n">>process.txt

./microbench --keys_file=../data/datasets/genome --keys_file_type=binary --read=0.8 --insert=0.2 --operations_num=200000000 --table_size=-1 --init_table_ratio=0.5 --output_path=./result/genome8020.txt --thread_num=1,2,4,8,16,24,36,48 --memory=1 --index=lippol,lipp_prob,alexol,artolc &
PID=$!
echo "PID=" $PID
wait $PID

./microbench --keys_file=../data/datasets/genome --keys_file_type=binary --read=0.5 --insert=0.5 --operations_num=200000000 --table_size=-1 --init_table_ratio=0.5 --output_path=./result/genome5050.txt --thread_num=1,2,4,8,16,24,36,48  --memory=1 --index=lippol,lipp_prob,alexol,artolc &
PID=$!
echo "PID=" $PID
wait $PID

./microbench --keys_file=../data/datasets/genome --keys_file_type=binary --read=0 --insert=1 --operations_num=200000000 --table_size=-1 --init_table_ratio=0.5 --output_path=./result/genomewo.txt --thread_num=1,2,4,8,16,24,36,48  --memory=1 --index=lippol,lipp_prob,alexol,artolc &
PID=$!
echo "PID=" $PID
wait $PID

echo "genome end"
printf "genome end\n">>process.txt

echo "libio begin"
printf "libio begin\n">>process.txt

./microbench --keys_file=../data/datasets/libio --keys_file_type=binary --read=0.8 --insert=0.2 --operations_num=200000000 --table_size=-1 --init_table_ratio=0.5 --output_path=./result/libio8020.txt --thread_num=1,2,4,8,16,24,36,48  --memory=1 --index=lippol,lipp_prob,alexol,artolc &
PID=$!
echo "PID=" $PID
wait $PID

./microbench --keys_file=../data/datasets/libio --keys_file_type=binary --read=0.5 --insert=0.5 --operations_num=200000000 --table_size=-1 --init_table_ratio=0.5 --output_path=./result/libio5050.txt --thread_num=1,2,4,8,16,24,36,48  --memory=1 --index=lippol,lipp_prob,alexol,artolc &
PID=$!
echo "PID=" $PID
wait $PID

./microbench --keys_file=../data/datasets/libio --keys_file_type=binary --read=0 --insert=1 --operations_num=200000000 --table_size=-1 --init_table_ratio=0.5 --output_path=./result/libiowo.txt --thread_num=1,2,4,8,16,24,36,48  --memory=1 --index=lippol,lipp_prob,alexol,artolc &
PID=$!
echo "PID=" $PID
wait $PID

echo "libio end"
printf "libio end\n">>process.txt

echo "osm begin"
printf "osm begin\n">>process.txt

./microbench --keys_file=../data/datasets/osm --keys_file_type=binary --read=0.8 --insert=0.2 --operations_num=200000000 --table_size=-1 --init_table_ratio=0.5 --output_path=./result/osm8020.txt --thread_num=1,2,4,8,16,24,36,48 --memory=1 --index=lippol,lipp_prob,alexol,artolc &
PID=$!
echo "PID=" $PID
wait $PID

./microbench --keys_file=../data/datasets/osm --keys_file_type=binary --read=0.5 --insert=0.5 --operations_num=200000000 --table_size=-1 --init_table_ratio=0.5 --output_path=./result/osm5050.txt --thread_num=1,2,4,8,16,24,36,48  --memory=1 --index=lippol,lipp_prob,alexol,artolc &
PID=$!
echo "PID=" $PID
wait $PID

./microbench --keys_file=../data/datasets/osm --keys_file_type=binary --read=0 --insert=1 --operations_num=200000000 --table_size=-1 --init_table_ratio=0.5 --output_path=./result/osmwo.txt --thread_num=1,2,4,8,16,24,36,48  --memory=1 --index=lippol,lipp_prob,alexol,artolc &
PID=$!
echo "PID=" $PID
wait $PID

echo "osm end"
printf "osm end\n">>process.txt