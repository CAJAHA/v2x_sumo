for infVeh in 0 20 40
do
    for run in 1 2 3 4 5 6 7 8 9 10
    do
        c="v2x_communication_example --infVehNum=${infVeh} --run=${run}"
        echo $c
        ./waf --run "$c"
    done
done